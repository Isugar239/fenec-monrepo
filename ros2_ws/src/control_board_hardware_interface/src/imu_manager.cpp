#include "control_board_hardware_interface/imu_manager.hpp"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

#include <gpiod.h>
#include <libgen.h>
#include <limits.h>
#include <sys/wait.h>
#include <unistd.h>

namespace {

constexpr int kImuPowerOffMs = 800;
constexpr int kImuPowerOnSettleMs = 1000;

int run_cmd(const std::string &cmd) {
  const int status = std::system(cmd.c_str());
  if (status == -1) {
    return -1;
  }
  if (WIFEXITED(status)) {
    return WEXITSTATUS(status);
  }
  return -1;
}

bool write_sysfs(const std::string &path, const std::string &value) {
  // unbind/bind требуют root. sudo -n: без TTY, с чистым PATH.
  const std::string cmd = "sudo -n env PATH=/usr/bin:/bin HOME=/home/pi "
                          "sh -c \"printf '%s' '" +
                          value + "' > '" + path + "'\" 2>/tmp/imu_i2c_sysfs.err";
  const int rc = run_cmd(cmd);
  if (rc != 0) {
    std::string err;
    FILE *f = std::fopen("/tmp/imu_i2c_sysfs.err", "r");
    if (f) {
      char buf[256] = {0};
      if (std::fgets(buf, sizeof(buf), f) != nullptr) {
        err = buf;
        while (!err.empty() && (err.back() == '\n' || err.back() == '\r')) {
          err.pop_back();
        }
      }
      std::fclose(f);
    }
    RCLCPP_ERROR(rclcpp::get_logger("ControlBoardHardwareInterface"),
                 "IMU: sysfs write rc=%d path=%s value=%s err=%s", rc, path.c_str(),
                 value.c_str(), err.c_str());
    return false;
  }
  return true;
}

// Отключить линии SDA/SCL от контроллера, прокрутить SCL, STOP, вернуть I2C alt.
void bitbang_i2c_bus_clear() {
  run_cmd(
      "pinctrl set 2 ip pu; "
      "pinctrl set 3 op dh; "
      "i=0; while [ $i -lt 9 ]; do "
      "  pinctrl set 3 dl; usleep 20; pinctrl set 3 dh; usleep 20; "
      "  i=$((i+1)); "
      "done; "
      "pinctrl set 2 op dl; usleep 20; "
      "pinctrl set 3 dh; usleep 20; "
      "pinctrl set 2 dh; usleep 20; "
      "pinctrl set 2 a3 pu; "
      "pinctrl set 3 a3 pu");
}

// Пока VCC БНО снят, SDA/SCL через подтяжки держат ~1.4 V на чипе
// (подпитка через ESD-диоды). Сажаем шину в 0 на время off.
void i2c_hold_low() {
  run_cmd("pinctrl set 2 op dl; pinctrl set 3 op dl");
}

void i2c_restore_alt() {
  run_cmd("pinctrl set 2 a3 pu; pinctrl set 3 a3 pu");
}

}  // namespace

void IMUManager::set_imu_offset(double roll, double pitch, double yaw) {
  offset_quaternion.setRPY(roll, pitch, yaw);
  offset_rotation_matrix = tf2::Matrix3x3(offset_quaternion);
}

bool IMUManager::contains_nan(const Eigen::Quaternionf &q) {
  return std::isnan(q.x()) || std::isnan(q.y()) || std::isnan(q.z()) || std::isnan(q.w());
}

std::optional<std::pair<BNO055::Output, uint32_t>> IMUManager::sample_imu_multiple_attempts(
    int max_samples, int delay_ms) {
  auto start = std::chrono::high_resolution_clock::now();

  (void)max_samples;
  (void)delay_ms;
  auto output = imu_->sample();
  if (!output) {
    return std::nullopt;
  }
  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  return std::make_pair(*output, duration.count());
}

void IMUManager::invalidate_imu_data() {
  std::lock_guard<std::mutex> lock(imu_mutex_);
  imu_data.quat = Eigen::Quaternionf::Identity();
  imu_data.gyro.setZero();
  imu_data.acc.setZero();
  imu_data.packet_timestamp = {};
  imu_data.measurement_timestamp = {};
}

void IMUManager::poll_imu() {
  auto maybe_output = sample_imu_multiple_attempts(/*max_samples=*/5, /*delay_ms=*/1);
  if (maybe_output) {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    imu_data = maybe_output->first;
    imu_sampling_time_us = maybe_output->second;
  }
}

BNO055::Output IMUManager::get_imu_data() {
  std::lock_guard<std::mutex> lock(imu_mutex_);
  return imu_data;
}

bool IMUManager::open_power_gpio() {
  close_power_gpio();
  pwr_line_ = gpiod_line_find(IMU_PWR_GPIO_NAME);
  if (pwr_line_ == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("ControlBoardHardwareInterface"),
                 "IMU: не найден %s — power-cycle через MOSFET недоступен", IMU_PWR_GPIO_NAME);
    return false;
  }
  // default 0 = LOW = питание ВКЛ (active-low P-MOS)
  if (gpiod_line_request_output(pwr_line_, "bno086_pwr", 0) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ControlBoardHardwareInterface"),
                 "IMU: не удалось занять %s как output: %s", IMU_PWR_GPIO_NAME, strerror(errno));
    gpiod_line_close_chip(pwr_line_);
    pwr_line_ = nullptr;
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"),
              "IMU: питание на %s (LOW=вкл, HIGH=выкл)", IMU_PWR_GPIO_NAME);
  return true;
}

void IMUManager::close_power_gpio() {
  if (pwr_line_ == nullptr) {
    return;
  }
  gpiod_line_release(pwr_line_);
  gpiod_line_close_chip(pwr_line_);
  pwr_line_ = nullptr;
}

void IMUManager::set_imu_power(bool on) {
  if (pwr_line_ == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("ControlBoardHardwareInterface"),
                 "IMU: set_imu_power(%d) без GPIO — MOSFET не управляется", static_cast<int>(on));
    return;
  }
  // active-low: on → 0, off → 1
  const int value = on ? 0 : 1;
  if (gpiod_line_set_value(pwr_line_, value) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ControlBoardHardwareInterface"),
                 "IMU: gpiod_line_set_value(%d) на %s: %s", value, IMU_PWR_GPIO_NAME,
                 strerror(errno));
  }
}

void IMUManager::power_cycle_imu() {
  RCLCPP_WARN(rclcpp::get_logger("ControlBoardHardwareInterface"),
              "IMU: power-cycle VCC через %s + SDA/SCL=0 на время off (%d ms / settle %d ms)",
              IMU_PWR_GPIO_NAME, kImuPowerOffMs, kImuPowerOnSettleMs);
  // Сначала шину в 0, потом VCC off — иначе подтяжки держат ~1.4 V на чипе.
  i2c_hold_low();
  set_imu_power(false);
  std::this_thread::sleep_for(std::chrono::milliseconds(kImuPowerOffMs));
  set_imu_power(true);
  i2c_restore_alt();
  std::this_thread::sleep_for(std::chrono::milliseconds(kImuPowerOnSettleMs));
}

void IMUManager::request_power(bool on) {
  power_wanted_.store(on);
  power_apply_pending_.store(true);
  RCLCPP_WARN(rclcpp::get_logger("ControlBoardHardwareInterface"),
              "IMU: запрос питания %s (GPIO17)", on ? "ВКЛ" : "ВЫКЛ");
}

void IMUManager::apply_power_if_pending() {
  if (!power_apply_pending_.exchange(false)) {
    return;
  }
  const bool on = power_wanted_.load();
  if (!on) {
    imu_.reset();
    i2c_hold_low();
    set_imu_power(false);
    invalidate_imu_data();
    RCLCPP_WARN(rclcpp::get_logger("ControlBoardHardwareInterface"),
                "IMU: питание снято (GPIO17 HIGH, SDA/SCL=0 — без подпитки)");
    return;
  }
  // Холодный старт: bus low + VCC off → on → I2C alt → settle → init.
  imu_.reset();
  i2c_hold_low();
  set_imu_power(false);
  std::this_thread::sleep_for(std::chrono::milliseconds(kImuPowerOffMs));
  set_imu_power(true);
  i2c_restore_alt();
  std::this_thread::sleep_for(std::chrono::milliseconds(kImuPowerOnSettleMs));
  imu_ = std::make_unique<BNO055>(IMU_I2C_DEVICE_NUMBER, micros_between_reports_);
  RCLCPP_WARN(rclcpp::get_logger("ControlBoardHardwareInterface"),
              "IMU: питание подано, драйвер переоткрыт");
}

void IMUManager::reopen() {
  if (!power_wanted_.load()) {
    return;
  }
  RCLCPP_WARN(rclcpp::get_logger("ControlBoardHardwareInterface"),
              "IMU не обновляет положение 5 с: power-cycle VCC + soft-reset (без I2C unbind)");
  invalidate_imu_data();
  imu_.reset();
  power_cycle_imu();
  imu_ = std::make_unique<BNO055>(IMU_I2C_DEVICE_NUMBER, micros_between_reports_);
}

void IMUManager::imu_read_thread_fn() {
  auto last_packet_at = std::chrono::steady_clock::now();
  auto last_stamp = std::chrono::high_resolution_clock::time_point{};
  while (!stop_imu_read_thread_) {
    apply_power_if_pending();
    if (!power_wanted_.load() || !imu_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      continue;
    }
    poll_imu();
    const auto stamp = get_imu_data().packet_timestamp;
    const auto now = std::chrono::steady_clock::now();
    if (stamp != last_stamp) {
      last_stamp = stamp;
      last_packet_at = now;
    } else if (now - last_packet_at >= std::chrono::seconds(5)) {
      reopen();
      last_packet_at = std::chrono::steady_clock::now();
      last_stamp = {};
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

bool IMUManager::cycle_i2c_bus() {
  const std::string adapter_link =
      "/sys/bus/i2c/devices/i2c-" + std::to_string(IMU_I2C_DEVICE_NUMBER);

  char real_adapter[PATH_MAX];
  if (realpath(adapter_link.c_str(), real_adapter) == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("ControlBoardHardwareInterface"),
                 "IMU: нет %s — unbind I2C невозможен: %s", adapter_link.c_str(),
                 strerror(errno));
    return false;
  }

  char path_for_parent[PATH_MAX];
  std::strncpy(path_for_parent, real_adapter, PATH_MAX - 1);
  path_for_parent[PATH_MAX - 1] = '\0';
  char *platform_dir = dirname(path_for_parent);

  char path_for_basename[PATH_MAX];
  std::strncpy(path_for_basename, platform_dir, PATH_MAX - 1);
  path_for_basename[PATH_MAX - 1] = '\0';
  const std::string platform_device = basename(path_for_basename);

  const std::string driver_sys = std::string(platform_dir) + "/driver";
  char driver_link[PATH_MAX];
  const ssize_t n = readlink(driver_sys.c_str(), driver_link, sizeof(driver_link) - 1);
  if (n < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ControlBoardHardwareInterface"),
                 "IMU: нет driver у %s: %s", platform_device.c_str(), strerror(errno));
    return false;
  }
  driver_link[n] = '\0';

  char driver_link_copy[PATH_MAX];
  std::strncpy(driver_link_copy, driver_link, PATH_MAX - 1);
  driver_link_copy[PATH_MAX - 1] = '\0';
  const std::string driver_name = basename(driver_link_copy);

  const std::string unbind_path =
      "/sys/bus/platform/drivers/" + driver_name + "/unbind";
  const std::string bind_path = "/sys/bus/platform/drivers/" + driver_name + "/bind";

  RCLCPP_WARN(rclcpp::get_logger("ControlBoardHardwareInterface"),
              "IMU: отключаю I2C-%d (%s / %s), bitbang clear, включаю обратно",
              IMU_I2C_DEVICE_NUMBER, platform_device.c_str(), driver_name.c_str());

  if (!write_sysfs(unbind_path, platform_device)) {
    RCLCPP_ERROR(rclcpp::get_logger("ControlBoardHardwareInterface"),
                 "IMU: unbind %s не удался — шина останется залипшей до ребута",
                 platform_device.c_str());
    return false;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  bitbang_i2c_bus_clear();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  if (!write_sysfs(bind_path, platform_device)) {
    RCLCPP_ERROR(rclcpp::get_logger("ControlBoardHardwareInterface"),
                 "IMU: bind %s не удался — /dev/i2c-%d может пропасть",
                 platform_device.c_str(), IMU_I2C_DEVICE_NUMBER);
    return false;
  }

  const std::string dev = "/dev/i2c-" + std::to_string(IMU_I2C_DEVICE_NUMBER);
  for (int i = 0; i < 50; ++i) {
    if (access(dev.c_str(), R_OK | W_OK) == 0) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  RCLCPP_ERROR(rclcpp::get_logger("ControlBoardHardwareInterface"),
               "IMU: после bind нет %s", dev.c_str());
  return false;
}

void IMUManager::begin(uint32_t micros_between_reports) {
  micros_between_reports_ = micros_between_reports;

  if (!open_power_gpio()) {
    RCLCPP_ERROR(rclcpp::get_logger("ControlBoardHardwareInterface"),
                 "IMU: без %s залипший BNO не оживёт без ребута — припаяй MOSFET на VCC",
                 IMU_PWR_GPIO_NAME);
  } else {
    set_imu_power(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(kImuPowerOnSettleMs));
  }

  imu_ = std::make_unique<BNO055>(IMU_I2C_DEVICE_NUMBER, micros_between_reports);

  stop_imu_read_thread_ = false;
  imu_read_thread_ = std::make_unique<std::thread>(&IMUManager::imu_read_thread_fn, this);
}

void IMUManager::stop() {
  stop_imu_read_thread_ = true;
  if (imu_read_thread_ && imu_read_thread_->joinable()) {
    imu_read_thread_->join();
  }
  imu_.reset();
  // GPIO не отпускаем: иначе 100k на базе PNP выключит BNO «по умолчанию».
}
