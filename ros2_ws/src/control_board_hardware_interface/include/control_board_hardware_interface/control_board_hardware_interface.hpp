#pragma once

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <atomic>
#include <fstream>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "imu_manager.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rt/rt_spi.h"

#include <cstddef>

// Overload the << operator for vector and array
template <typename T>
std::ostream &operator<<(std::ostream &os, const std::vector<T> &container) {
  for (const auto &e : container) {
    os << e << " ";
  }
  return os;
}
template <typename T, std::size_t N>
std::ostream &operator<<(std::ostream &os, const std::array<T, N> &container) {
  for (const auto &e : container) {
    os << e << " ";
  }
  return os;
}

namespace control_board_hardware_interface {

template <typename T>
bool contains_nan(const T &container) {
  return std::any_of(container.begin(), container.end(),
                     [](double val) { return std::isnan(val); });
}

class ControlBoardHardwareInterface : public hardware_interface::SystemInterface {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ControlBoardHardwareInterface)

  virtual ~ControlBoardHardwareInterface();

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State &previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::CallbackReturn on_error(
      const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time &time,
                                       const rclcpp::Duration &period) override;

  hardware_interface::return_type write(const rclcpp::Time &time,
                                        const rclcpp::Duration &period) override;

 private:
  void deactivate_motors();
  void copy_actuator_commands(bool use_position_limits = false);
  void copy_actuator_states();
  // mode 0: сначала задние ноги (стадии 0-2), потом передние (3-5), между ними пауза delay_s.
  // mode 1: все ноги сразу, порядок колено → отведение → бедро сохраняется.
  void do_homing(int mode = 0, double delay_s = 0.0);
  void rehome(int mode, double delay_s);
  void publish_homing_status(const std::string &state, const std::string &mode, int stage,
                             double delay_s);
  bool hw_states_contains_nan();
  void parse_partial_hardware_params();
  hardware_interface::CallbackReturn apply_motor_map();
  void apply_spi_flags();
  void start_revive_listener();
  void stop_revive_listener();
  bool is_joint_active(std::size_t joint_index) const;
  void configure_inactive_joints();
  void log_joint_commands();

  int lock_fd_ = -1;  // File descriptor for lock file

  // Partial-hardware mode: enable a subset of CAN channels (front/rear/none)
  std::string partial_motors_ = "all";
  bool torque_enable_ = true;
  // Кадры с flags=0. Спина на спаде шлёт Exit всем моторам платы, на следующем
  // кадре с 1 — Enter. Ноль энкодера не пишется, хоуминг не запускается.
  std::atomic<int> revive_exit_frames_{0};
  rclcpp::Node::SharedPtr revive_node_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr revive_sub_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr revive_executor_;
  std::thread revive_thread_;
  // Повторный хоуминг по /motor_rehome [mode, delay_s]: подписка принимает, read() выполняет.
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr rehome_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr homing_status_pub_;
  std::atomic<bool> rehome_pending_{false};
  std::atomic<int> rehome_mode_{0};
  std::atomic<double> rehome_delay_s_{0.0};
  int homing_seq_ = 0;
  bool log_commands_ = false;
  bool channel_enabled_[4] = {true, true, true, true};
  double partial_velocity_max_ = 3.0;
  double partial_effort_max_ = 1.0;
  double partial_kp_scale_ = 0.5;
  std::ofstream command_log_;
  int command_log_count_ = 0;

  IMUManager imu_manager_;

  // TODO: Refactor the horrible c-style spi code
  spi_command_t *spi_command_;
  spi_data_t *spi_data_;

  // IMU state
  std::array<double, 4> hw_state_imu_orientation_;          // x, y, z, w
  std::array<double, 3> hw_state_imu_angular_velocity_;     // x, y, z
  std::array<double, 3> hw_state_imu_linear_acceleration_;  // x, y, z
  // One-pole low-pass filter for the gyro only. Orientation remains the raw,
  // normalized quaternion from the IMU, so filtering cannot accumulate drift.
  double imu_gyro_filter_alpha_ = 0.2;
  std::array<double, 3> filtered_imu_angular_velocity_{};
  bool imu_gyro_filter_initialized_ = false;
  double imu_roll_, imu_pitch_, imu_yaw_;

  // Actuator CAN config
  std::vector<int> hw_actuator_can_channels_;
  std::vector<int> hw_actuator_can_ids_;

  // Actuator homing
  std::vector<int> hw_actuator_homing_stages_;
  std::vector<double> hw_actuator_homing_velocities_;
  std::vector<double> hw_actuator_homing_kps_;
  std::vector<double> hw_actuator_homing_kds_;
  // kp/kd после хоуминга. По умолчанию равны homing_*; hold_kp поднимает удержание,
  // не меняя порог, на котором ищется упор.
  std::vector<double> hw_actuator_hold_kps_;
  std::vector<double> hw_actuator_hold_kds_;
  std::vector<double> hw_actuator_homed_positions_;
  std::vector<double> hw_actuator_post_homing_positions_;
  std::vector<double> hw_actuator_zero_positions_;
  std::vector<double> hw_actuator_homing_torque_thresholds_;
  std::vector<bool> hw_actuator_is_homed_;
  // Сустав без калибровки нуля: момент на него не подаётся до перезапуска.
  std::vector<uint8_t> hw_actuator_homing_skipped_;

  // Actuator limits
  std::vector<double> hw_actuator_position_mins_;
  std::vector<double> hw_actuator_position_maxs_;
  std::vector<double> hw_actuator_velocity_maxs_;
  std::vector<double> hw_actuator_effort_maxs_;
  std::vector<double> hw_actuator_kp_maxs_;
  std::vector<double> hw_actuator_kd_maxs_;

  // Actuator states
  std::vector<double> hw_state_positions_;
  std::vector<double> hw_state_velocities_;
  std::vector<double> hw_state_efforts_;

  // Actuator commands
  std::vector<double> hw_command_positions_;
  std::vector<double> hw_command_velocities_;
  std::vector<double> hw_command_efforts_;
  std::vector<double> hw_command_kps_;
  std::vector<double> hw_command_kds_;

  int64_t imu_packet_timestamp_micros_;
  int64_t imu_measurement_timestamp_micros_;
  int64_t imu_time_since_measurement_micros_;
  double imu_time_since_measurement_seconds_;
};

}  // namespace control_board_hardware_interface
