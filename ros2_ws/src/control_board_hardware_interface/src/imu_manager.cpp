#include "control_board_hardware_interface/imu_manager.hpp"

#include <chrono>

// set rotation matrix based on roll, pitch, yaw
void IMUManager::set_imu_offset(double roll, double pitch, double yaw) {
  offset_quaternion.setRPY(roll, pitch, yaw);
  offset_rotation_matrix = tf2::Matrix3x3(offset_quaternion);
}

bool IMUManager::contains_nan(const Eigen::Quaternionf &q) {
  return std::isnan(q.x()) || std::isnan(q.y()) || std::isnan(q.z()) || std::isnan(q.w());
}

std::optional<std::pair<BNO055::Output, uint32_t>> IMUManager::sample_imu_multiple_attempts(
    int max_samples, int delay_ms) {
  // Record start time
  auto start = std::chrono::high_resolution_clock::now();

  (void)max_samples;
  (void)delay_ms;
  BNO055::Output output = imu_->sample();
  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  return std::make_pair(output, duration.count());
}

void IMUManager::poll_imu() {
  auto maybe_output = sample_imu_multiple_attempts(/*max_samples=*/5, /*delay_ms=*/1);
  if (maybe_output) {
    // lock and copy imu data
    std::lock_guard<std::mutex> lock(imu_mutex_);
    imu_data = maybe_output->first;
    imu_sampling_time_us = maybe_output->second;
  }
}

BNO055::Output IMUManager::get_imu_data() {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    return imu_data;
}

void IMUManager::reopen() {
  RCLCPP_WARN(rclcpp::get_logger("ControlBoardHardwareInterface"),
              "IMU не обновляет положение 1 с, открываю I2C заново");
  imu_.reset();
  imu_ = std::make_unique<BNO055>(IMU_I2C_DEVICE_NUMBER, micros_between_reports_);
}

void IMUManager::imu_read_thread_fn() {
  auto last_packet_at = std::chrono::steady_clock::now();
  auto last_stamp = std::chrono::high_resolution_clock::time_point{};
  while (!stop_imu_read_thread_) {
    poll_imu();
    const auto stamp = get_imu_data().packet_timestamp;
    const auto now = std::chrono::steady_clock::now();
    if (stamp != last_stamp) {
      last_stamp = stamp;
      last_packet_at = now;
    } else if (now - last_packet_at >= std::chrono::seconds(1)) {
      reopen();
      last_packet_at = std::chrono::steady_clock::now();
      last_stamp = {};
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

void IMUManager::begin(uint32_t micros_between_reports) {
  micros_between_reports_ = micros_between_reports;
  imu_ = std::make_unique<BNO055>(IMU_I2C_DEVICE_NUMBER, micros_between_reports);

  stop_imu_read_thread_ = false;
  imu_read_thread_ = std::make_unique<std::thread>(&IMUManager::imu_read_thread_fn, this);
}

void IMUManager::stop() {
  stop_imu_read_thread_ = true;
  if (imu_read_thread_ && imu_read_thread_->joinable()) {
    imu_read_thread_->join();
  }
}
