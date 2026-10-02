#include "control_board_hardware_interface/control_board_hardware_interface.hpp"

#include <fcntl.h>
#include <sched.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace control_board_hardware_interface {

const char *RED_ANSI = "\033[1;31m";
const char *YELLOW_ANSI = "\033[1;33m";
const char *RESET_ANSI = "\033[0m";

namespace {

std::string to_lower_copy(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

bool parse_bool_param(const std::string &value, bool default_value) {
  const std::string lowered = to_lower_copy(value);
  if (lowered == "true" || lowered == "1" || lowered == "yes" || lowered == "on") {
    return true;
  }
  if (lowered == "false" || lowered == "0" || lowered == "no" || lowered == "off") {
    return false;
  }
  return default_value;
}

}  // namespace

// diagnostics for detecting frozen joint_states
static std::vector<float> g_prev_hw_positions;
static int g_no_update_count = 0;
static int g_read_count = 0;

ControlBoardHardwareInterface::~ControlBoardHardwareInterface() {
  // Release lock if we have it
  if (lock_fd_ != -1) {
    flock(lock_fd_, LOCK_UN);
    close(lock_fd_);
    unlink("/tmp/control_board_hardware.lock");  // Delete the lock file
  }

  // Deactivate everything when ctrl-c is pressed
  on_deactivate(rclcpp_lifecycle::State());
}

hardware_interface::CallbackReturn ControlBoardHardwareInterface::on_init(
    const hardware_interface::HardwareInfo &info) {
  lock_fd_ = open("/tmp/control_board_hardware.lock", O_CREAT, 0666);
  if (lock_fd_ == -1) {
    RCLCPP_ERROR(rclcpp::get_logger("ControlBoardHardwareInterface"), "Failed to open lock file");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (flock(lock_fd_, LOCK_EX | LOCK_NB) == -1) {
    RCLCPP_ERROR(rclcpp::get_logger("ControlBoardHardwareInterface"),
                 "Another instance is already running");
    close(lock_fd_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_state_positions_.resize(info_.joints.size(), 0.0);
  hw_state_velocities_.resize(info_.joints.size(), 0.0);
  hw_state_efforts_.resize(info_.joints.size(), 0.0);

  hw_command_positions_.resize(info_.joints.size(), 0.0);
  hw_command_velocities_.resize(info_.joints.size(), 0.0);
  hw_command_efforts_.resize(info_.joints.size(), 0.0);
  hw_command_kps_.resize(info_.joints.size(), 0.0);
  hw_command_kds_.resize(info_.joints.size(), 0.0);

  for (const hardware_interface::ComponentInfo &joint : info_.joints) {
    // Set params for each joint
    hw_actuator_can_channels_.push_back(std::stoi(joint.parameters.at("can_channel")));
    hw_actuator_can_ids_.push_back(std::stoi(joint.parameters.at("can_id")));

    // Set limits for each joint
    hw_actuator_position_mins_.push_back(std::stod(joint.parameters.at("position_min")));
    hw_actuator_position_maxs_.push_back(std::stod(joint.parameters.at("position_max")));
    hw_actuator_velocity_maxs_.push_back(std::stod(joint.parameters.at("velocity_max")));
    hw_actuator_effort_maxs_.push_back(std::stod(joint.parameters.at("effort_max")));
    hw_actuator_kp_maxs_.push_back(std::stod(joint.parameters.at("kp_max")));
    hw_actuator_kd_maxs_.push_back(std::stod(joint.parameters.at("kd_max")));

    // Homing parameters
    hw_actuator_homing_stages_.push_back(std::stoi(joint.parameters.at("homing_stage")));
    hw_actuator_homing_velocities_.push_back(std::stod(joint.parameters.at("homing_velocity")));
    hw_actuator_homing_kps_.push_back(std::stod(joint.parameters.at("homing_kp")));
    hw_actuator_homing_kds_.push_back(std::stod(joint.parameters.at("homing_kd")));
    const auto hold_kp = joint.parameters.find("hold_kp");
    const auto hold_kd = joint.parameters.find("hold_kd");
    hw_actuator_hold_kps_.push_back(hold_kp == joint.parameters.end()
                                        ? hw_actuator_homing_kps_.back()
                                        : std::stod(hold_kp->second));
    hw_actuator_hold_kds_.push_back(hold_kd == joint.parameters.end()
                                        ? hw_actuator_homing_kds_.back()
                                        : std::stod(hold_kd->second));
    hw_actuator_homed_positions_.push_back(std::stod(joint.parameters.at("homed_position")));
    hw_actuator_post_homing_positions_.push_back(
        std::stod(joint.parameters.at("post_homing_position")));
    hw_actuator_zero_positions_.push_back(0.0);
    hw_actuator_homing_torque_thresholds_.push_back(
        std::stod(joint.parameters.at("homing_torque_threshold")));
    hw_actuator_is_homed_.push_back(false);
  }

  if (apply_motor_map() != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.hardware_parameters.count("use_imu") &&
      (info_.hardware_parameters.at("use_imu") == "false" ||
       info_.hardware_parameters.at("use_imu") == "False")) {
    RCLCPP_WARN(rclcpp::get_logger("ControlBoardHardwareInterface"), "%sIMU not enabled%s",
                YELLOW_ANSI, RESET_ANSI);
    imu_manager_.enabled = false;
  } else {
    imu_manager_.enabled = true;

    // Set up IMU parameters
    double imu_roll = std::stod(info_.sensors[0].parameters.at("roll"));
    double imu_pitch = std::stod(info_.sensors[0].parameters.at("pitch"));
    double imu_yaw = std::stod(info_.sensors[0].parameters.at("yaw"));
    imu_manager_.set_imu_offset(imu_roll, imu_pitch, imu_yaw);

    // Set up the IMU
    // TODO: make micros_between_reports a parameter
    imu_manager_.begin(/*micros_between_reports=*/10000);
  }

  if (info_.hardware_parameters.count("imu_gyro_filter_alpha")) {
    imu_gyro_filter_alpha_ =
        std::stod(info_.hardware_parameters.at("imu_gyro_filter_alpha"));
  }
  if (!std::isfinite(imu_gyro_filter_alpha_) || imu_gyro_filter_alpha_ <= 0.0 ||
      imu_gyro_filter_alpha_ > 1.0) {
    RCLCPP_ERROR(rclcpp::get_logger("ControlBoardHardwareInterface"),
                 "imu_gyro_filter_alpha must be in (0, 1], got %.3f",
                 imu_gyro_filter_alpha_);
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"),
              "IMU gyro low-pass filter enabled: alpha=%.2f", imu_gyro_filter_alpha_);

  // Set up SPI
  init_spi();
  spi_command_ = get_spi_command();
  spi_data_ = get_spi_data();

  parse_partial_hardware_params();

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
ControlBoardHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Add joint state interfaces
  for (auto i = 0u; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_state_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_state_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_state_efforts_[i]));
  }

  // Add IMU state interfaces
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "orientation.x",
                                                                   &hw_state_imu_orientation_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "orientation.y",
                                                                   &hw_state_imu_orientation_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "orientation.z",
                                                                   &hw_state_imu_orientation_[2]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "orientation.w",
                                                                   &hw_state_imu_orientation_[3]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "angular_velocity.x", &hw_state_imu_angular_velocity_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "angular_velocity.y", &hw_state_imu_angular_velocity_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "angular_velocity.z", &hw_state_imu_angular_velocity_[2]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "linear_acceleration.x", &hw_state_imu_linear_acceleration_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "linear_acceleration.y", &hw_state_imu_linear_acceleration_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "linear_acceleration.z", &hw_state_imu_linear_acceleration_[2]));

  // TODO: Add once int64 type is supported
  //   state_interfaces.emplace_back(
  //       hardware_interface::StateInterface("imu_sensor", "imu_packet_timestamp",
  //       &imu_packet_timestamp_micros_));
  //   state_interfaces.emplace_back(hardware_interface::StateInterface(
  //       "imu_sensor", "imu_measurement_timestamp", &imu_measurement_timestamp_micros_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "time_since_measurement_seconds", &imu_time_since_measurement_seconds_));

  return state_interfaces;
}

hardware_interface::CallbackReturn ControlBoardHardwareInterface::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // reset values always when configuring hardware
  for (uint i = 0; i < hw_state_positions_.size(); i++) {
    hw_state_positions_[i] = 0.0;
    hw_state_velocities_[i] = 0.0;
    hw_state_efforts_[i] = 0.0;
    hw_command_positions_[i] = 0.0;
    hw_command_velocities_[i] = 0.0;
    hw_command_efforts_[i] = 0.0;
    hw_command_kps_[i] = 0.0;
    hw_command_kds_[i] = 0.0;
  }
  imu_gyro_filter_initialized_ = false;

  RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface>
ControlBoardHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_command_positions_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_command_velocities_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_command_efforts_[i]));
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(info_.joints[i].name, "kp", &hw_command_kps_[i]));
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(info_.joints[i].name, "kd", &hw_command_kds_[i]));
  }

  return command_interfaces;
}

bool contains_nan(const Eigen::Quaternionf &q) {
  return std::isnan(q.x()) || std::isnan(q.y()) || std::isnan(q.z()) || std::isnan(q.w());
}

hardware_interface::CallbackReturn ControlBoardHardwareInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // TODO: figure out how to disable if IMU is enabled but not reading properly
  //   if (imu_manager_.enabled && !sample_imu_multiple_attempts(*imu_, /*max_samples=*/500,
  //   /*delay_ms=*/10)) {
  //     RCLCPP_ERROR(rclcpp::get_logger("ControlBoardHardwareInterface"),
  //                  "%sFailed to get a good IMU sample within 500 attempts. Deactivating "
  //                  "motors%s",
  //                  RED_ANSI, RESET_ANSI);
  //     deactivate_motors();
  //     return hardware_interface::CallbackReturn::ERROR;
  //   }

  // Enable actuators on the selected CAN channels only. Repeat to ensure receipt.
  configure_inactive_joints();
  apply_spi_flags();
  for (int i = 0; i < 10; i++) {
    apply_spi_flags();
    spi_driver_run();
  }
  copy_actuator_states();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Homing (skipped for inactive channels and when torque is disabled)
  do_homing();
  start_revive_listener();
  publish_homing_status(homing_seq_ > 0 ? "done" : "none", "rear_first", -1, 0.0);

  RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

void ControlBoardHardwareInterface::deactivate_motors() {
  // Один кадр flags=0 STM превращает в Exit и сразу следом шлёт прошлый PD
  // (kp хоуминга). Без нулевого kp на следующих кадрах hip остаётся держать
  // момент после остановки процесса.
  torque_enable_ = false;
  for (auto i = 0u; i < hw_command_kps_.size(); ++i) {
    hw_command_kps_[i] = 0.0;
    hw_command_kds_[i] = 0.0;
    hw_command_efforts_[i] = 0.0;
    hw_command_velocities_[i] = 0.0;
  }
  std::memset(spi_command_, 0, sizeof(*spi_command_));
  for (int i = 0; i < 30; ++i) {
    spi_driver_run();
  }
}

hardware_interface::CallbackReturn ControlBoardHardwareInterface::on_error(
    [[maybe_unused]] const rclcpp_lifecycle::State &previous_state) {
  deactivate_motors();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ControlBoardHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  stop_revive_listener();
  deactivate_motors();
  RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool ControlBoardHardwareInterface::hw_states_contains_nan() {
  return contains_nan(hw_state_positions_) || contains_nan(hw_state_velocities_) ||
         contains_nan(hw_state_efforts_) || contains_nan(hw_state_imu_orientation_) ||
         contains_nan(hw_state_imu_angular_velocity_) ||
         contains_nan(hw_state_imu_linear_acceleration_);
}

hardware_interface::return_type ControlBoardHardwareInterface::read(
    [[maybe_unused]] const rclcpp::Time &time, [[maybe_unused]] const rclcpp::Duration &period) {
  // Write to and read from the actuators
  if (rehome_pending_.exchange(false)) {
    // Блокирует цикл контроллеров на время хоуминга, как и при запуске сервиса.
    rehome(rehome_mode_.load(), rehome_delay_s_.load());
  }
  apply_spi_flags();
  spi_driver_run();
  const int revive_left = revive_exit_frames_.load();
  if (revive_left > 0) {
    revive_exit_frames_.store(revive_left - 1);
    if (revive_left == 1) {
      RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"),
                  "Моторы: Enter отправлен, хоуминг не запускался");
    }
  }
  copy_actuator_states();

  // Diagnostics: detect frozen joint states
  g_read_count++;
  if (g_prev_hw_positions.size() != hw_state_positions_.size()) {
    g_prev_hw_positions.assign(hw_state_positions_.begin(), hw_state_positions_.end());
  } else {
    bool any_active = false;
    bool any_change = false;
    for (size_t i = 0; i < hw_state_positions_.size(); ++i) {
      if (!is_joint_active(i)) {
        continue;
      }
      any_active = true;
      if (std::fabs(hw_state_positions_[i] - g_prev_hw_positions[i]) > 1e-6) {
        any_change = true;
        break;
      }
    }
    if (!any_active) {
      any_change = true;
    }
    if (!any_change) {
      g_no_update_count++;
      if ((g_no_update_count % 50) == 0) {
        RCLCPP_WARN(rclcpp::get_logger("ControlBoardHardwareInterface"),
                    "No joint state updates detected for %d consecutive reads (read_count=%d)",
                    g_no_update_count, g_read_count);
      }
    } else {
      if (g_no_update_count > 0) {
        RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"),
                    "Joint states resumed updating after %d frozen reads", g_no_update_count);
      }
      g_no_update_count = 0;
    }
    // copy current positions into prev
    for (size_t i = 0; i < hw_state_positions_.size(); ++i) {
      g_prev_hw_positions[i] = hw_state_positions_[i];
    }
  }

  // Print spi driver status occasionally
  if ((g_read_count % 100) == 0) {
    if (spi_data_) {
      RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"),
                  "SPI driver status: 0x%08x (read_count=%d)", spi_data_->spi_driver_status,
                  g_read_count);
    }
  }

  // Print joint position
  // RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"), "Joint 0: %f",
  // spi_data_->q_abad[1]);

  tf2::Quaternion corrected_quat = tf2::Quaternion::getIdentity();
  tf2::Vector3 angular_velocity(0, 0, 0);
  tf2::Vector3 linear_acceleration(0, 0, 0);

  if (imu_manager_.enabled) {
    // TODO: put this functionality into the imu_manager
    BNO055::Output imu_output = imu_manager_.get_imu_data();
    std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();

    imu_packet_timestamp_micros_ = std::chrono::duration_cast<std::chrono::microseconds>(
                                       imu_output.packet_timestamp.time_since_epoch())
                                       .count();
    imu_measurement_timestamp_micros_ = std::chrono::duration_cast<std::chrono::microseconds>(
                                            imu_output.measurement_timestamp.time_since_epoch())
                                            .count();
    imu_time_since_measurement_micros_ = std::chrono::duration_cast<std::chrono::microseconds>(
                                             now - imu_output.measurement_timestamp)
                                             .count();
    imu_time_since_measurement_seconds_ = imu_time_since_measurement_micros_ / 1e6;

    // Print imu timestamps and age
    // RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"),
    //             "Packet: %ld us\tmesurement: %ldus\tage as of read(): %ldus = %fs",
    //             imu_packet_timestamp_micros_, imu_measurement_timestamp_micros_,
    //             imu_time_since_measurement_micros_, imu_time_since_measurement_seconds_);

    // Only publish orientation/gyro from a fresh sensor report.
    // Stale last-known pose was shown as "live" while BNO was dead (control
    // packets used to refresh the timestamp without new rotation vector).
    constexpr double kMaxFreshAgeSec = 0.25;
    if (imu_time_since_measurement_seconds_ >= 0.0 &&
        imu_time_since_measurement_seconds_ < kMaxFreshAgeSec) {
      tf2::Quaternion imu_quat(imu_output.quat.x(), imu_output.quat.y(), imu_output.quat.z(),
                               imu_output.quat.w());
      corrected_quat = imu_quat * imu_manager_.offset_quaternion.inverse();
      if (corrected_quat.length2() > 1e-12) {
        corrected_quat.normalize();
      }

      tf2::Vector3 imu_ang_vel(imu_output.gyro.x(), imu_output.gyro.y(), imu_output.gyro.z());
      angular_velocity = imu_manager_.offset_rotation_matrix * imu_ang_vel;

      tf2::Vector3 imu_acc(imu_output.acc.x(), imu_output.acc.y(), imu_output.acc.z());
      linear_acceleration = imu_manager_.offset_rotation_matrix * imu_acc;
    }
  }

  // Updating the state interfaces with corrected values
  hw_state_imu_orientation_[0] = corrected_quat.x();
  hw_state_imu_orientation_[1] = corrected_quat.y();
  hw_state_imu_orientation_[2] = corrected_quat.z();
  hw_state_imu_orientation_[3] = corrected_quat.w();

  const std::array<double, 3> raw_angular_velocity = {
      angular_velocity.x(), angular_velocity.y(), angular_velocity.z()};
  if (!imu_gyro_filter_initialized_) {
    // Start from the first sample so enabling the filter never creates a
    // startup transient. Only gyro is filtered; quaternion stays unfiltered.
    filtered_imu_angular_velocity_ = raw_angular_velocity;
    imu_gyro_filter_initialized_ = true;
  } else {
    for (auto i = 0u; i < raw_angular_velocity.size(); ++i) {
      filtered_imu_angular_velocity_[i] =
          imu_gyro_filter_alpha_ * raw_angular_velocity[i] +
          (1.0 - imu_gyro_filter_alpha_) * filtered_imu_angular_velocity_[i];
    }
  }
  hw_state_imu_angular_velocity_ = filtered_imu_angular_velocity_;

  hw_state_imu_linear_acceleration_[0] = linear_acceleration.x();
  hw_state_imu_linear_acceleration_[1] = linear_acceleration.y();
  hw_state_imu_linear_acceleration_[2] = linear_acceleration.z();

  // Check if any NaNs in hardware state arrays. Catches IMU issues.
  if (hw_states_contains_nan()) {
    RCLCPP_ERROR(rclcpp::get_logger("ControlBoardHardwareInterface"),
                 "HW state array contained NaN. Deactivating motors");
    std::stringstream ss;
    ss << hw_state_imu_orientation_ << '\n'
       << hw_state_imu_angular_velocity_ << '\n'
       << hw_state_imu_linear_acceleration_ << '\n'
       << hw_state_positions_ << '\n'
       << hw_state_velocities_ << '\n'
       << hw_state_efforts_ << '\n';
    RCLCPP_ERROR(rclcpp::get_logger("ControlBoardHardwareInterface"), "%s", ss.str().c_str());
    deactivate_motors();
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ControlBoardHardwareInterface::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  // Keep selected motor drivers enabled every control cycle (flags gate torque in spine firmware).
  apply_spi_flags();
  copy_actuator_commands(true);
  log_joint_commands();
  return hardware_interface::return_type::OK;
}

void ControlBoardHardwareInterface::rehome(int mode, double delay_s) {
  if (!torque_enable_) {
    RCLCPP_WARN(rclcpp::get_logger("ControlBoardHardwareInterface"),
                "Повторный хоуминг не запущен: момент отключён (torque_enable=false)");
    return;
  }
  // Ноль прошлого хоуминга больше не годится: суставы могли сместиться, а хоуминг
  // работает в сырых координатах энкодера. Отправленный в SPI кадр остаётся прежним.
  for (auto i = 0u; i < hw_state_positions_.size(); i++) {
    if (!is_joint_active(i)) {
      continue;
    }
    hw_actuator_is_homed_[i] = false;
    hw_actuator_zero_positions_[i] = 0.0;
  }
  copy_actuator_states();
  do_homing(mode, delay_s);
}

void ControlBoardHardwareInterface::publish_homing_status(const std::string &state,
                                                          const std::string &mode, int stage,
                                                          double delay_s) {
  if (!homing_status_pub_) {
    return;
  }
  std::ostringstream ss;
  ss << "{\"seq\":" << homing_seq_ << ",\"state\":\"" << state << "\",\"mode\":\"" << mode
     << "\",\"stage\":" << stage << ",\"delay\":" << delay_s << ",\"skipped\":[";
  bool first = true;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    if (!is_joint_active(i) || i >= hw_actuator_homing_skipped_.size() ||
        !hw_actuator_homing_skipped_[i]) {
      continue;
    }
    ss << (first ? "" : ",") << "\"" << info_.joints[i].name << "\"";
    first = false;
  }
  ss << "]}";
  std_msgs::msg::String msg;
  msg.data = ss.str();
  homing_status_pub_->publish(msg);
}

void ControlBoardHardwareInterface::do_homing(int mode, double delay_s) {
  bool any_active = false;
  for (auto i = 0u; i < hw_state_positions_.size(); i++) {
    if (is_joint_active(i)) {
      any_active = true;
      break;
    }
  }

  if (!torque_enable_) {
    RCLCPP_WARN(rclcpp::get_logger("ControlBoardHardwareInterface"),
                "%sTorque disabled; skipping homing. Commands will be logged only.%s", YELLOW_ANSI,
                RESET_ANSI);
    return;
  }

  if (!any_active) {
    RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"),
                "No active CAN channels; skipping homing");
    return;
  }

  const bool all_at_once = (mode == 1);
  const std::string mode_name = all_at_once ? "all" : "rear_first";
  RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"),
              "Homing actuators... (режим %s, пауза между задними и передними %.1f с)",
              mode_name.c_str(), all_at_once ? 0.0 : delay_s);
  homing_seq_++;

  // Стадии 0-2 — задние ноги, 3-5 — передние; внутри ноги колено → отведение → бедро.
  // «Все сразу» сворачивает передние на те же три стадии.
  std::vector<int> stages = hw_actuator_homing_stages_;
  if (all_at_once) {
    for (auto &s : stages) {
      s %= 3;
    }
  }
  bool rear_stage_active = false;
  for (auto i = 0u; i < stages.size(); i++) {
    if (is_joint_active(i) && stages[i] < 3) {
      rear_stage_active = true;
    }
  }
  const bool use_pause = !all_at_once && delay_s > 0.0 && rear_stage_active;
  bool pause_started = false;
  bool pause_done = false;
  std::chrono::steady_clock::time_point pause_until;
  int last_published_stage = -2;

  int dt_ms = 10;

  apply_spi_flags();
  std::this_thread::sleep_for(std::chrono::milliseconds(dt_ms));
  copy_actuator_commands();
  spi_driver_run();
  copy_actuator_states();
  std::this_thread::sleep_for(std::chrono::milliseconds(dt_ms));

  // Set the initial commands
  for (auto i = 0u; i < hw_state_positions_.size(); i++) {
    if (!is_joint_active(i)) {
      continue;
    }
    hw_command_positions_[i] = hw_state_positions_[i];
    hw_command_velocities_[i] = 0.0;
    hw_command_kps_[i] = 0.0;
    hw_command_kds_[i] = 0.0;
  }

  apply_spi_flags();
  std::this_thread::sleep_for(std::chrono::milliseconds(dt_ms));
  copy_actuator_commands();
  spi_driver_run();
  copy_actuator_states();
  std::this_thread::sleep_for(std::chrono::milliseconds(dt_ms));

  // Keep track of filtered torques
  std::vector<double> filtered_torques;
  filtered_torques.resize(hw_state_positions_.size(), 0.0);
  std::vector<double> homing_start_pos = hw_state_positions_;
  hw_actuator_homing_skipped_.assign(hw_state_positions_.size(), 0);
  std::vector<uint8_t> &homing_skipped = hw_actuator_homing_skipped_;
  publish_homing_status("running", mode_name, -1, delay_s);
  // Размах fb за хоуминг: сустав, стартовавший в упоре, после отъезда наберёт ход,
  // а зажатый или с мёртвым энкодером — нет.
  std::vector<double> travel_min = hw_state_positions_;
  std::vector<double> travel_max = hw_state_positions_;
  std::vector<uint8_t> backed_off(hw_state_positions_.size(), 0);
  std::vector<int> backoff_ticks(hw_state_positions_.size(), 0);
  constexpr double kMinHomingTravel = 0.10;
  constexpr double kBackoff = 0.30;
  const auto homing_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(90);
  double alpha = 0.5;

  // Loop until all actuators are homed
  bool all_homed = false;
  bool all_returned = false;
  while (!all_homed) {
    if (std::chrono::steady_clock::now() > homing_deadline) {
      for (auto i = 0u; i < hw_state_positions_.size(); i++) {
        if (!is_joint_active(i) || hw_actuator_is_homed_[i]) {
          continue;
        }
        homing_skipped[i] = 1;
        hw_actuator_is_homed_[i] = true;
        hw_command_kps_[i] = 0.0;
        hw_command_kds_[i] = 0.0;
        hw_command_velocities_[i] = 0.0;
        RCLCPP_WARN(rclcpp::get_logger("ControlBoardHardwareInterface"),
                    "Homing timeout: skipping actuator %d (%s) — нет упора за 90 с, момент снят", i,
                    info_.joints[i].name.c_str());
      }
      break;
    }
    // Set the homing stage to that of the lowest-stage unhomed actuator
    int homing_stage = 999;
    for (auto i = 0u; i < stages.size(); i++) {
      if (!is_joint_active(i)) {
        continue;
      }
      if (!hw_actuator_is_homed_[i] && (stages[i] < homing_stage)) {
        homing_stage = stages[i];
      }
    }
    // Пауза, когда задние ноги готовы, а передние ещё не начинали: задние держат упор.
    bool paused = false;
    if (use_pause && !pause_done && homing_stage >= 3 && homing_stage != 999) {
      const auto now = std::chrono::steady_clock::now();
      if (!pause_started) {
        pause_started = true;
        pause_until = now + std::chrono::milliseconds(static_cast<int>(delay_s * 1000.0));
        RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"),
                    "Задние ноги в нуле, пауза %.1f с перед передними", delay_s);
        publish_homing_status("running", mode_name, homing_stage, delay_s);
      }
      if (now < pause_until) {
        paused = true;
      } else {
        pause_done = true;
      }
    }
    if (homing_stage != last_published_stage && !paused) {
      last_published_stage = homing_stage;
      if (homing_stage != 999) {
        publish_homing_status("running", mode_name, homing_stage, delay_s);
      }
    }
    // Check if all actuators are homed
    all_homed = true;
    for (auto i = 0u; i < hw_state_positions_.size(); i++) {
      if (!is_joint_active(i)) {
        continue;
      }
      if (homing_stage < stages[i] || (paused && !hw_actuator_is_homed_[i])) {
        hw_command_positions_[i] = hw_state_positions_[i];
        hw_command_velocities_[i] = 0.0;
        hw_command_kps_[i] = 0.0;
        hw_command_kds_[i] = 0.0;
        all_homed = false;
        continue;
      } else if (homing_skipped[i]) {
        hw_command_kps_[i] = 0.0;
        hw_command_kds_[i] = 0.0;
        hw_command_velocities_[i] = 0.0;
      } else {
        hw_command_kps_[i] = hw_actuator_homing_kps_[i];
        hw_command_kds_[i] = hw_actuator_homing_kds_[i];
      }
      if (!hw_actuator_is_homed_[i]) {
        travel_min[i] = std::min(travel_min[i], hw_state_positions_[i]);
        travel_max[i] = std::max(travel_max[i], hw_state_positions_[i]);
        if (backoff_ticks[i] > 0) {
          backoff_ticks[i]--;
          all_homed = false;
          continue;
        }
        filtered_torques[i] =
            (1.0 - alpha) * filtered_torques[i] +
            alpha * ((hw_command_positions_[i] - hw_state_positions_[i]) * hw_command_kps_[i] +
                     (hw_command_velocities_[i] - hw_state_velocities_[i]) * hw_command_kds_[i]);
        if (std::abs(filtered_torques[i]) >= hw_actuator_homing_torque_thresholds_[i]) {
          const double traveled = travel_max[i] - travel_min[i];
          const bool encoder_alive = std::abs(homing_start_pos[i]) > 1e-3;
          if (traveled < kMinHomingTravel && encoder_alive && !backed_off[i]) {
            backed_off[i] = 1;
            backoff_ticks[i] = 500 / dt_ms;
            filtered_torques[i] = 0.0;
            const double dir = hw_actuator_homing_velocities_[i] > 0.0 ? 1.0 : -1.0;
            hw_command_positions_[i] = hw_state_positions_[i] - dir * kBackoff;
            hw_command_velocities_[i] = 0.0;
            all_homed = false;
            RCLCPP_WARN(rclcpp::get_logger("ControlBoardHardwareInterface"),
                        "Actuator %d (%s) стартовал в упоре (Δ=%.4f): отъезжаю на %.2f рад и "
                        "захожу снова",
                        i, info_.joints[i].name.c_str(), traveled, kBackoff);
          } else if (traveled < kMinHomingTravel) {
            homing_skipped[i] = 1;
            hw_actuator_is_homed_[i] = true;
            hw_command_kps_[i] = 0.0;
            hw_command_kds_[i] = 0.0;
            hw_command_velocities_[i] = 0.0;
            RCLCPP_WARN(rclcpp::get_logger("ControlBoardHardwareInterface"),
                        "Skipping homing for actuator %d (%s): fb не едет (Δ=%.4f) — ложный упор, "
                        "момент снят",
                        i, info_.joints[i].name.c_str(), traveled);
          } else {
            hw_actuator_zero_positions_[i] = hw_state_positions_[i] - hw_actuator_homed_positions_[i];
            hw_command_positions_[i] = hw_actuator_homed_positions_[i];
            hw_command_velocities_[i] = 0.0;
            hw_actuator_is_homed_[i] = true;
            RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"),
                        "Homed actuator %d (%s)", i, info_.joints[i].name.c_str());
          }
        } else {
          hw_command_positions_[i] += hw_actuator_homing_velocities_[i] * dt_ms * 0.001;
          hw_command_velocities_[i] = hw_actuator_homing_velocities_[i];
          all_homed = false;
        }
      }
    }
    apply_spi_flags();
    copy_actuator_commands();
    spi_driver_run();
    copy_actuator_states();

    // Sleep for dt
    std::this_thread::sleep_for(std::chrono::milliseconds(dt_ms));
  }
  while (!all_returned) {
    all_returned = true;
    for (auto i = 0u; i < hw_state_positions_.size(); i++) {
      if (!is_joint_active(i)) {
        continue;
      }
      if (homing_skipped[i]) {
        hw_command_kps_[i] = 0.0;
        hw_command_kds_[i] = 0.0;
        hw_command_velocities_[i] = 0.0;
        continue;
      }
      if (hw_command_positions_[i] < hw_actuator_post_homing_positions_[i] - 0.01) {
        hw_command_positions_[i] += std::abs(hw_actuator_homing_velocities_[i]) * dt_ms * 0.001;
        hw_command_velocities_[i] = std::abs(hw_actuator_homing_velocities_[i]);
        all_returned = false;
      } else if (hw_command_positions_[i] > hw_actuator_post_homing_positions_[i] + 0.01) {
        hw_command_positions_[i] -= std::abs(hw_actuator_homing_velocities_[i]) * dt_ms * 0.001;
        hw_command_velocities_[i] = -std::abs(hw_actuator_homing_velocities_[i]);
        all_returned = false;
      } else {
        hw_command_positions_[i] = hw_actuator_post_homing_positions_[i];
        hw_command_velocities_[i] = 0.0;
      }
    }
    apply_spi_flags();
    copy_actuator_commands();
    spi_driver_run();
    copy_actuator_states();

    // Sleep for dt
    std::this_thread::sleep_for(std::chrono::milliseconds(dt_ms));
  }
  for (auto i = 0u; i < hw_state_positions_.size(); i++) {
    if (!is_joint_active(i) || homing_skipped[i]) {
      continue;
    }
    hw_command_kps_[i] = hw_actuator_hold_kps_[i];
    hw_command_kds_[i] = hw_actuator_hold_kds_[i];
  }
  RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"),
              "HOMING_RESULT name CAN id start end delta status hold");
  for (auto i = 0u; i < hw_state_positions_.size(); i++) {
    const double delta = hw_state_positions_[i] - homing_start_pos[i];
    const bool moved = std::abs(delta) >= kMinHomingTravel;
    const bool holding = hw_command_kps_[i] > 0.0;
    const char *status = "ВЫКЛ";
    if (is_joint_active(i)) {
      status = moved ? "ЕХАЛ" : "СТОИТ";
    }
    const char *hold = holding ? "ДЕРЖИТ" : "ОТПУЩЕН";
    RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"),
                "HOMING_RESULT %s CAN%d id%d start=%+.3f end=%+.3f d=%+.3f %s %s",
                info_.joints[i].name.c_str(), hw_actuator_can_channels_[i],
                hw_actuator_can_ids_[i], homing_start_pos[i], hw_state_positions_[i], delta,
                status, hold);
    if (homing_skipped[i]) {
      RCLCPP_ERROR(rclcpp::get_logger("ControlBoardHardwareInterface"),
                   "HOMING_SKIPPED %s: ноль не найден, момента не будет до перезапуска. "
                   "Походка на этом суставе работать не будет.",
                   info_.joints[i].name.c_str());
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"), "Finished homing!");
  publish_homing_status("done", mode_name, -1, delay_s);
}

void ControlBoardHardwareInterface::copy_actuator_commands(bool use_position_limits) {
  const bool partial_mode = (partial_motors_ != "all");

  // Zero commands on disabled CAN channels so missing motors never receive PD/FF.
  for (int ch = 0; ch < 4; ++ch) {
    if (channel_enabled_[ch]) {
      continue;
    }
    spi_command_->q_des_abad[ch] = 0.0f;
    spi_command_->q_des_hip[ch] = 0.0f;
    spi_command_->q_des_knee[ch] = 0.0f;
    spi_command_->qd_des_abad[ch] = 0.0f;
    spi_command_->qd_des_hip[ch] = 0.0f;
    spi_command_->qd_des_knee[ch] = 0.0f;
    spi_command_->kp_abad[ch] = 0.0f;
    spi_command_->kp_hip[ch] = 0.0f;
    spi_command_->kp_knee[ch] = 0.0f;
    spi_command_->kd_abad[ch] = 0.0f;
    spi_command_->kd_hip[ch] = 0.0f;
    spi_command_->kd_knee[ch] = 0.0f;
    spi_command_->tau_abad_ff[ch] = 0.0f;
    spi_command_->tau_hip_ff[ch] = 0.0f;
    spi_command_->tau_knee_ff[ch] = 0.0f;
  }

  // Iterate through the joints
  for (auto i = 0u; i < hw_state_positions_.size(); i++) {
    if (!is_joint_active(i)) {
      continue;
    }

    const double vel_limit =
        partial_mode ? std::min(hw_actuator_velocity_maxs_[i], partial_velocity_max_)
                     : hw_actuator_velocity_maxs_[i];
    const double eff_limit =
        partial_mode ? std::min(hw_actuator_effort_maxs_[i], partial_effort_max_)
                     : hw_actuator_effort_maxs_[i];
    const double kp_limit =
        partial_mode ? hw_actuator_kp_maxs_[i] * partial_kp_scale_ : hw_actuator_kp_maxs_[i];

    double cmd_pos = hw_command_positions_[i];
    double cmd_vel = std::clamp(hw_command_velocities_[i], -vel_limit, vel_limit);
    double cmd_eff = std::clamp(hw_command_efforts_[i], -eff_limit, eff_limit);
    double cmd_kp = std::clamp(hw_command_kps_[i], 0.0, kp_limit);
    double cmd_kd = std::clamp(hw_command_kds_[i], 0.0, hw_actuator_kd_maxs_[i]);

    const bool no_zero = i < hw_actuator_homing_skipped_.size() && hw_actuator_homing_skipped_[i];
    if (!torque_enable_ || no_zero) {
      cmd_kp = 0.0;
      cmd_kd = 0.0;
      cmd_eff = 0.0;
    }

    if (use_position_limits && cmd_kp > 0.0) {
      if (cmd_pos < hw_actuator_position_mins_[i]) {
        cmd_pos = hw_actuator_position_mins_[i];
        cmd_vel = std::clamp(cmd_vel, 0.0, vel_limit);
        cmd_eff = std::clamp(cmd_eff, 0.0, eff_limit);
      } else if (cmd_pos > hw_actuator_position_maxs_[i]) {
        cmd_pos = hw_actuator_position_maxs_[i];
        cmd_vel = std::clamp(cmd_vel, -vel_limit, 0.0);
        cmd_eff = std::clamp(cmd_eff, -eff_limit, 0.0);
      }
      cmd_pos = std::clamp(cmd_pos, hw_actuator_position_mins_[i], hw_actuator_position_maxs_[i]);
    }

    cmd_pos += hw_actuator_zero_positions_[i];

    uint can_channel = hw_actuator_can_channels_[i] - 1;
    // ID 1: abad, ID 2: hip, ID 3: knee (not corresponding to the actual joint names, just used
    // to make the Cheetah code send to the CAN IDs we want)
    switch (hw_actuator_can_ids_[i]) {
      case 1:
        spi_command_->q_des_abad[can_channel] = cmd_pos;
        spi_command_->qd_des_abad[can_channel] = cmd_vel;
        spi_command_->kp_abad[can_channel] = cmd_kp;
        spi_command_->kd_abad[can_channel] = cmd_kd;
        spi_command_->tau_abad_ff[can_channel] = cmd_eff;
        break;
      case 2:
        spi_command_->q_des_hip[can_channel] = cmd_pos;
        spi_command_->qd_des_hip[can_channel] = cmd_vel;
        spi_command_->kp_hip[can_channel] = cmd_kp;
        spi_command_->kd_hip[can_channel] = cmd_kd;
        spi_command_->tau_hip_ff[can_channel] = cmd_eff;
        break;
      case 3:
        spi_command_->q_des_knee[can_channel] = cmd_pos;
        spi_command_->qd_des_knee[can_channel] = cmd_vel;
        spi_command_->kp_knee[can_channel] = cmd_kp;
        spi_command_->kd_knee[can_channel] = cmd_kd;
        spi_command_->tau_knee_ff[can_channel] = cmd_eff;
        break;
    }
  }
}

void ControlBoardHardwareInterface::copy_actuator_states() {
  // Iterate through the joints
  for (auto i = 0u; i < hw_state_positions_.size(); i++) {
    if (!is_joint_active(i)) {
      // Missing motors: follow the commanded trajectory so the 12-DoF policy keeps running.
      hw_state_positions_[i] = hw_command_positions_[i];
      hw_state_velocities_[i] = hw_command_velocities_[i];
      hw_state_efforts_[i] = 0.0;
      continue;
    }

    float state_pos = hw_state_positions_[i];
    float state_vel = hw_state_velocities_[i];

    uint can_channel = hw_actuator_can_channels_[i] - 1;
    // ID 1: abad, ID 2: hip, ID 3: knee (not corresponding to the actual joint names, just used
    // to make the Cheetah code send to the CAN IDs we want)
    switch (hw_actuator_can_ids_[i]) {
      case 1:
        state_pos = spi_data_->q_abad[can_channel];
        state_vel = spi_data_->qd_abad[can_channel];
        break;
      case 2:
        state_pos = spi_data_->q_hip[can_channel];
        state_vel = spi_data_->qd_hip[can_channel];
        break;
      case 3:
        state_pos = spi_data_->q_knee[can_channel];
        state_vel = spi_data_->qd_knee[can_channel];
        break;
    }
    hw_state_positions_[i] = state_pos - hw_actuator_zero_positions_[i];
    hw_state_velocities_[i] = state_vel;

    // Estimate actuator efforts based on motor driver PD control
    hw_state_efforts_[i] =
        (hw_command_positions_[i] - hw_state_positions_[i]) * hw_command_kps_[i] +
        (hw_command_velocities_[i] - hw_state_velocities_[i]) * hw_command_kds_[i];
  }
}

hardware_interface::CallbackReturn ControlBoardHardwareInterface::apply_motor_map() {
  const auto param = info_.hardware_parameters.find("motor_map_file");
  if (param == info_.hardware_parameters.end() || param->second.empty()) {
    RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"),
                "motor_map_file не задан, CAN id берутся из URDF");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  const std::string &path = param->second;
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("ControlBoardHardwareInterface"),
                 "Не читается motor map %s: %s", path.c_str(), e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!root["joints"] || !root["joints"].IsMap()) {
    RCLCPP_ERROR(rclcpp::get_logger("ControlBoardHardwareInterface"),
                 "В %s нет словаря joints", path.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  const bool swap_front_back =
      root["swap_front_back"] ? root["swap_front_back"].as<bool>() : false;
  const bool swap_left_right =
      root["swap_left_right"] ? root["swap_left_right"].as<bool>() : false;

  // Имя сустава → (can_channel, can_id). Swap меняет адреса целых ног.
  std::map<std::string, std::pair<int, int>> assignments;
  const YAML::Node joints_yaml = root["joints"];
  for (auto it = joints_yaml.begin(); it != joints_yaml.end(); ++it) {
    const std::string name = it->first.as<std::string>();
    const YAML::Node entry = it->second;
    if (!entry["can_channel"] || !entry["can_id"]) {
      RCLCPP_ERROR(rclcpp::get_logger("ControlBoardHardwareInterface"),
                   "%s: нужны can_channel и can_id", name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    int channel = 0;
    int can_id = 0;
    try {
      channel = entry["can_channel"].as<int>();
      can_id = entry["can_id"].as<int>();
    } catch (const std::exception &e) {
      RCLCPP_ERROR(rclcpp::get_logger("ControlBoardHardwareInterface"),
                   "%s: can_channel/can_id не число (%s)", name.c_str(), e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
    assignments[name] = {channel, can_id};
  }

  auto exchange = [&assignments](const std::string &a, const std::string &b) {
    std::swap(assignments.at(a), assignments.at(b));
  };
  if (swap_front_back) {
    for (const char *side : {"l", "r"}) {
      for (const char *joint : {"1", "2", "3"}) {
        exchange(std::string("leg_front_") + side + "_" + joint,
                 std::string("leg_back_") + side + "_" + joint);
      }
    }
  }
  if (swap_left_right) {
    for (const char *fb : {"front", "back"}) {
      for (const char *joint : {"1", "2", "3"}) {
        exchange(std::string("leg_") + fb + "_l_" + joint,
                 std::string("leg_") + fb + "_r_" + joint);
      }
    }
  }
  if (swap_front_back || swap_left_right) {
    RCLCPP_WARN(rclcpp::get_logger("ControlBoardHardwareInterface"),
                "%smotor_map swap_front_back=%s swap_left_right=%s%s", YELLOW_ANSI,
                swap_front_back ? "true" : "false", swap_left_right ? "true" : "false",
                RESET_ANSI);
  }

  std::set<std::string> urdf_names;
  for (const auto &joint : info_.joints) {
    urdf_names.insert(joint.name);
  }
  if (assignments.size() != urdf_names.size()) {
    RCLCPP_ERROR(rclcpp::get_logger("ControlBoardHardwareInterface"),
                 "В motor map %zu суставов, в URDF %zu", assignments.size(), urdf_names.size());
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (const auto &entry : assignments) {
    if (!urdf_names.count(entry.first)) {
      RCLCPP_ERROR(rclcpp::get_logger("ControlBoardHardwareInterface"),
                   "В motor map неизвестный сустав %s", entry.first.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  std::set<std::pair<int, int>> used;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const std::string &name = info_.joints[i].name;
    const auto found = assignments.find(name);
    if (found == assignments.end()) {
      RCLCPP_ERROR(rclcpp::get_logger("ControlBoardHardwareInterface"),
                   "%s: нет в motor map", name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    const int channel = found->second.first;
    const int can_id = found->second.second;
    if (channel < 1 || channel > 4 || can_id < 1 || can_id > 3) {
      RCLCPP_ERROR(rclcpp::get_logger("ControlBoardHardwareInterface"),
                   "%s: CAN %d id %d вне диапазона (канал 1..4, id 1..3)", name.c_str(), channel,
                   can_id);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (!used.insert({channel, can_id}).second) {
      RCLCPP_ERROR(rclcpp::get_logger("ControlBoardHardwareInterface"),
                   "%s: CAN %d id %d уже назначен другому суставу", name.c_str(), channel, can_id);
      return hardware_interface::CallbackReturn::ERROR;
    }
    const int previous_channel = hw_actuator_can_channels_[i];
    const int previous_id = hw_actuator_can_ids_[i];
    hw_actuator_can_channels_[i] = channel;
    hw_actuator_can_ids_[i] = can_id;
    if (previous_channel != channel || previous_id != can_id) {
      RCLCPP_WARN(rclcpp::get_logger("ControlBoardHardwareInterface"),
                  "%s%s: CAN %d id %d → CAN %d id %d%s", YELLOW_ANSI, name.c_str(),
                  previous_channel, previous_id, channel, can_id, RESET_ANSI);
    } else {
      RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"), "%s: CAN %d id %d",
                  name.c_str(), channel, can_id);
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

void ControlBoardHardwareInterface::parse_partial_hardware_params() {
  if (info_.hardware_parameters.count("partial_motors")) {
    partial_motors_ = to_lower_copy(info_.hardware_parameters.at("partial_motors"));
  } else {
    partial_motors_ = "all";
  }

  if (info_.hardware_parameters.count("torque_enable")) {
    torque_enable_ = parse_bool_param(info_.hardware_parameters.at("torque_enable"), true);
  } else {
    torque_enable_ = true;
  }

  channel_enabled_[0] = true;
  channel_enabled_[1] = true;
  channel_enabled_[2] = true;
  channel_enabled_[3] = true;

  if (partial_motors_ == "front") {
    channel_enabled_[0] = true;   // CAN 1: front left
    channel_enabled_[1] = true;   // CAN 2: front right
    channel_enabled_[2] = false;  // CAN 3: back left
    channel_enabled_[3] = false;  // CAN 4: back right
  } else if (partial_motors_ == "rear" || partial_motors_ == "back") {
    channel_enabled_[0] = false;
    channel_enabled_[1] = false;
    channel_enabled_[2] = true;
    channel_enabled_[3] = true;
  } else if (partial_motors_ == "none" || partial_motors_ == "off" || partial_motors_ == "fake") {
    channel_enabled_[0] = false;
    channel_enabled_[1] = false;
    channel_enabled_[2] = false;
    channel_enabled_[3] = false;
  } else if (partial_motors_ != "all") {
    RCLCPP_WARN(rclcpp::get_logger("ControlBoardHardwareInterface"),
                "Unknown partial_motors='%s', defaulting to all", partial_motors_.c_str());
    partial_motors_ = "all";
  }

  log_commands_ = (partial_motors_ != "all") || !torque_enable_;

  if (info_.hardware_parameters.count("partial_velocity_max")) {
    partial_velocity_max_ = std::stod(info_.hardware_parameters.at("partial_velocity_max"));
  }
  if (info_.hardware_parameters.count("partial_effort_max")) {
    partial_effort_max_ = std::stod(info_.hardware_parameters.at("partial_effort_max"));
  }
  if (info_.hardware_parameters.count("partial_kp_scale")) {
    partial_kp_scale_ = std::stod(info_.hardware_parameters.at("partial_kp_scale"));
  }

  RCLCPP_WARN(rclcpp::get_logger("ControlBoardHardwareInterface"),
              "%spartial_motors=%s flags=[%d, %d, %d, %d] torque_enable=%s "
              "(front=CAN1/2, rear=CAN3/4)%s",
              YELLOW_ANSI, partial_motors_.c_str(), channel_enabled_[0] && torque_enable_,
              channel_enabled_[1] && torque_enable_, channel_enabled_[2] && torque_enable_,
              channel_enabled_[3] && torque_enable_, torque_enable_ ? "true" : "false", RESET_ANSI);

  if (partial_motors_ != "all" && torque_enable_) {
    RCLCPP_WARN(rclcpp::get_logger("ControlBoardHardwareInterface"),
                "%sPartial torque is ON. Policy commands assume four legs; keep the robot "
                "supported and start with torque_enable:=False to log commands first. "
                "Partial limits: vel<=%.2f rad/s, effort<=%.2f Nm, kp_scale=%.2f%s",
                RED_ANSI, partial_velocity_max_, partial_effort_max_, partial_kp_scale_,
                RESET_ANSI);
  }

  if (log_commands_) {
    const char *bags_dir = "/home/pi/bags";
    std::string log_path;
    std::time_t now = std::time(nullptr);
    std::tm local_tm{};
    localtime_r(&now, &local_tm);
    char stamp[32];
    std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &local_tm);

    if (access(bags_dir, W_OK) == 0) {
      log_path = std::string(bags_dir) + "/partial_motors_commands_" + stamp + ".csv";
    } else {
      log_path = std::string("/tmp/partial_motors_commands_") + stamp + ".csv";
    }

    command_log_.open(log_path, std::ios::out);
    if (command_log_.is_open()) {
      command_log_ << "t_sec";
      for (const auto &joint : info_.joints) {
        command_log_ << "," << joint.name << "_pos," << joint.name << "_vel," << joint.name
                     << "_eff," << joint.name << "_kp," << joint.name << "_kd," << joint.name
                     << "_active";
      }
      command_log_ << "\n";
      RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"),
                  "Logging 12-joint commands to %s", log_path.c_str());
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("ControlBoardHardwareInterface"),
                   "Failed to open command log file %s", log_path.c_str());
      log_commands_ = false;
    }
  }
}

void ControlBoardHardwareInterface::apply_spi_flags() {
  const bool exiting = revive_exit_frames_.load() > 0;
  for (int ch = 0; ch < 4; ++ch) {
    spi_command_->flags[ch] = (!exiting && channel_enabled_[ch] && torque_enable_) ? 1 : 0;
  }
}

void ControlBoardHardwareInterface::start_revive_listener() {
  if (revive_node_) {
    return;
  }
  revive_node_ = std::make_shared<rclcpp::Node>("motor_revive");
  revive_sub_ = revive_node_->create_subscription<std_msgs::msg::Empty>(
      "/motor_revive", rclcpp::QoS(1), [this](std_msgs::msg::Empty::ConstSharedPtr) {
        // 8 кадров ~15 мс: хватает, чтобы спина увидела спад флага даже если один SPI-кадр битый.
        revive_exit_frames_.store(8);
        RCLCPP_WARN(rclcpp::get_logger("ControlBoardHardwareInterface"),
                    "Оживление: Exit всем моторам, затем Enter. Хоуминга нет, момент пропадёт на долю секунды");
      });
  imu_power_pub_ = revive_node_->create_publisher<std_msgs::msg::Bool>(
      "/imu_power_state", rclcpp::QoS(1).reliable().transient_local());
  imu_power_sub_ = revive_node_->create_subscription<std_msgs::msg::Bool>(
      "/imu_power", rclcpp::QoS(1), [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        imu_manager_.request_power(msg->data);
        std_msgs::msg::Bool st;
        st.data = msg->data;
        imu_power_pub_->publish(st);
      });
  {
    std_msgs::msg::Bool st;
    st.data = imu_manager_.power_wanted();
    imu_power_pub_->publish(st);
  }
  // transient_local: страница пульта, открытая после хоуминга, сразу получает последний статус.
  homing_status_pub_ = revive_node_->create_publisher<std_msgs::msg::String>(
      "/motor_homing/status", rclcpp::QoS(1).reliable().transient_local());
  rehome_sub_ = revive_node_->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/motor_rehome", rclcpp::QoS(1),
      [this](std_msgs::msg::Float32MultiArray::ConstSharedPtr msg) {
        if (msg->data.size() != 2) {
          RCLCPP_ERROR(rclcpp::get_logger("ControlBoardHardwareInterface"),
                       "/motor_rehome: нужно [режим, пауза_с], пришло %zu чисел", msg->data.size());
          return;
        }
        const int mode = static_cast<int>(std::lround(msg->data[0]));
        const double delay_s = msg->data[1];
        if ((mode != 0 && mode != 1) || !(delay_s >= 0.0 && delay_s <= 30.0)) {
          RCLCPP_ERROR(rclcpp::get_logger("ControlBoardHardwareInterface"),
                       "/motor_rehome: режим %d (0 задние→передние, 1 все сразу), пауза %.1f "
                       "(0..30 с) — отклонено",
                       mode, delay_s);
          return;
        }
        if (rehome_pending_.load()) {
          RCLCPP_WARN(rclcpp::get_logger("ControlBoardHardwareInterface"),
                      "/motor_rehome: прошлый запрос ещё не выполнен, новый отклонён");
          return;
        }
        rehome_mode_.store(mode);
        rehome_delay_s_.store(delay_s);
        rehome_pending_.store(true);
        RCLCPP_WARN(rclcpp::get_logger("ControlBoardHardwareInterface"),
                    "Повторный хоуминг по запросу: режим %s, пауза %.1f с. Моторы обмякнут, "
                    "робот должен висеть на подставке или лежать",
                    mode == 1 ? "все сразу" : "сначала задние", delay_s);
      });
  revive_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  revive_executor_->add_node(revive_node_);
  revive_thread_ = std::thread([this]() { revive_executor_->spin(); });
}

void ControlBoardHardwareInterface::stop_revive_listener() {
  if (revive_executor_) {
    revive_executor_->cancel();
  }
  if (revive_thread_.joinable()) {
    revive_thread_.join();
  }
  if (revive_executor_ && revive_node_) {
    revive_executor_->remove_node(revive_node_);
  }
  revive_sub_.reset();
  imu_power_sub_.reset();
  imu_power_pub_.reset();
  rehome_sub_.reset();
  homing_status_pub_.reset();
  revive_executor_.reset();
  revive_node_.reset();
}

bool ControlBoardHardwareInterface::is_joint_active(std::size_t joint_index) const {
  if (joint_index >= hw_actuator_can_channels_.size()) {
    return false;
  }
  const int can_channel = hw_actuator_can_channels_[joint_index] - 1;
  if (can_channel < 0 || can_channel > 3) {
    return false;
  }
  return channel_enabled_[can_channel];
}

void ControlBoardHardwareInterface::configure_inactive_joints() {
  for (auto i = 0u; i < hw_state_positions_.size(); i++) {
    if (is_joint_active(i)) {
      continue;
    }
    hw_actuator_is_homed_[i] = true;
    hw_actuator_zero_positions_[i] = 0.0;
    hw_state_positions_[i] = hw_actuator_post_homing_positions_[i];
    hw_state_velocities_[i] = 0.0;
    hw_state_efforts_[i] = 0.0;
    hw_command_positions_[i] = hw_actuator_post_homing_positions_[i];
    hw_command_velocities_[i] = 0.0;
    hw_command_efforts_[i] = 0.0;
    hw_command_kps_[i] = 0.0;
    hw_command_kds_[i] = 0.0;
    RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"),
                "Skipping homing for inactive joint %s (CAN %d id %d); using virtual state",
                info_.joints[i].name.c_str(), hw_actuator_can_channels_[i],
                hw_actuator_can_ids_[i]);
  }
}

void ControlBoardHardwareInterface::log_joint_commands() {
  if (!log_commands_) {
    return;
  }

  command_log_count_++;
  const double t_sec = static_cast<double>(command_log_count_) / 520.0;

  if (command_log_.is_open()) {
    command_log_ << std::fixed << std::setprecision(6) << t_sec;
    for (auto i = 0u; i < hw_command_positions_.size(); i++) {
      command_log_ << "," << hw_command_positions_[i] << "," << hw_command_velocities_[i] << ","
                   << hw_command_efforts_[i] << "," << hw_command_kps_[i] << ","
                   << hw_command_kds_[i] << "," << (is_joint_active(i) ? 1 : 0);
    }
    command_log_ << "\n";
    if ((command_log_count_ % 50) == 0) {
      command_log_.flush();
    }
  }

  if ((command_log_count_ % 520) == 0) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(3) << "joint commands:";
    for (auto i = 0u; i < hw_command_positions_.size(); i++) {
      ss << " " << info_.joints[i].name << "=" << hw_command_positions_[i];
    }
    RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"), "%s", ss.str().c_str());
  }
}

}  // namespace control_board_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(control_board_hardware_interface::ControlBoardHardwareInterface,
                       hardware_interface::SystemInterface)
