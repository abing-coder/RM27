#include "dm_imu.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <thread>
#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace io
{
namespace
{
constexpr uint8_t kHead0 = 0x55;
constexpr uint8_t kHead1 = 0xAA;
constexpr uint8_t kEnd = 0x0A;
constexpr uint8_t kRegAccel = 0x01;
constexpr uint8_t kRegGyro = 0x02;
constexpr uint8_t kRegEuler = 0x03;
constexpr uint8_t kRegQuat = 0x04;

int frame_length(uint8_t reg)
{
  // 55 AA ID REG + payload + CRC16 + 0A
  // 加速度/角速度/欧拉角 payload 12 字节，共 19；四元数 payload 16 字节，共 23。
  if (reg == kRegAccel || reg == kRegGyro || reg == kRegEuler) return 19;
  if (reg == kRegQuat) return 23;
  return 0;
}

constexpr std::array<uint16_t, 256> make_crc16_table()
{
  // 附录四的表：多项式 0x1021，非反射。
  std::array<uint16_t, 256> table{};
  for (int i = 0; i < 256; ++i) {
    uint16_t crc = static_cast<uint16_t>(i << 8);
    for (int bit = 0; bit < 8; ++bit) {
      if (crc & 0x8000)
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
      else
        crc = static_cast<uint16_t>(crc << 1);
    }
    table[static_cast<size_t>(i)] = crc;
  }
  return table;
}
}  // namespace

uint16_t DM_IMU::crc16(const uint8_t * data, uint16_t len)
{
  // 附录四 Get_CRC16。表虽是 CCITT，但固件写的是 crc << 1，不能改成标准的 << 8。
  // 实测 CRC 为小端，紧挨帧尾 0x0A。
  static constexpr auto table = make_crc16_table();
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; ++i) {
    const uint8_t index = static_cast<uint8_t>((crc >> 8) ^ data[i]);
    crc = static_cast<uint16_t>((crc << 1) ^ table[index]);
  }
  return crc;
}

DM_IMU::DM_IMU(const std::string & port) : port_(port), queue_(5000)
{
  init_serial();
  rec_thread_ = std::thread(&DM_IMU::get_imu_data_thread, this);
  queue_.pop(data_ahead_);
  queue_.pop(data_behind_);
  tools::logger()->info("[DM_IMU] initialized on {}", port_);
}

DM_IMU::~DM_IMU()
{
  stop_thread_ = true;
  if (rec_thread_.joinable()) rec_thread_.join();
  if (serial_.isOpen()) serial_.close();
}

void DM_IMU::init_serial()
{
  try {
    serial_.setPort(port_);
    // USB 虚拟串口波特率由模块忽略，这里与常见上位机保持一致。
    serial_.setBaudrate(921600);
    serial_.setFlowcontrol(serial::flowcontrol_none);
    serial_.setParity(serial::parity_none);
    serial_.setStopbits(serial::stopbits_one);
    serial_.setBytesize(serial::eightbits);
    serial::Timeout time_out = serial::Timeout::simpleTimeout(20);
    serial_.setTimeout(time_out);
    serial_.open();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    tools::logger()->info("[DM_IMU] serial port opened");
  } catch (const serial::IOException & e) {
    throw std::runtime_error(
      "[DM_IMU] failed to open " + port_ + " (" + e.what() +
      "). 确认 Type-C 已插入，且当前用户在 dialout 组。");
  }
}

void DM_IMU::get_imu_data_thread()
{
  auto last_wait_log = std::chrono::steady_clock::now();
  while (!stop_thread_) {
    if (!serial_.isOpen()) {
      tools::logger()->warn("[DM_IMU] serial port closed");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    try {
      uint8_t tmp[256];
      size_t n = serial_.read(tmp, sizeof(tmp));
      if (n > 0) {
        rx_.insert(rx_.end(), tmp, tmp + n);
        parse_buffer();
      } else if (!have_quaternion_ && rx_.empty()) {
        auto now = std::chrono::steady_clock::now();
        if (now - last_wait_log > std::chrono::seconds(2)) {
          tools::logger()->warn(
            "[DM_IMU] 还没有收到数据。若灯是绿灯呼吸，请在设置模式把输出接口设为 USB(00) 并保存。");
          last_wait_log = now;
        }
      }
    } catch (const std::exception & e) {
      tools::logger()->warn("[DM_IMU] read error: {}", e.what());
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (rx_.size() > 4096) rx_.erase(rx_.begin(), rx_.end() - 64);
  }
}

void DM_IMU::parse_buffer()
{
  while (try_consume_frame()) {
  }
}

bool DM_IMU::try_consume_frame()
{
  auto head = std::find(rx_.begin(), rx_.end(), kHead0);
  if (head == rx_.end()) {
    rx_.clear();
    return false;
  }
  if (head != rx_.begin()) rx_.erase(rx_.begin(), head);
  if (rx_.size() < 4) return false;
  if (rx_[1] != kHead1) {
    rx_.erase(rx_.begin());
    return true;
  }

  const int len = frame_length(rx_[3]);
  if (len == 0) {
    rx_.erase(rx_.begin());
    return true;
  }
  if (static_cast<int>(rx_.size()) < len) return false;
  if (rx_[len - 1] != kEnd) {
    rx_.erase(rx_.begin());
    return true;
  }

  const uint16_t calc = crc16(rx_.data(), static_cast<uint16_t>(len - 3));
  const uint16_t got = static_cast<uint16_t>(rx_[len - 3] | (rx_[len - 2] << 8));
  if (calc != got) {
    static int mismatches = 0;
    if (mismatches++ % 200 == 0) {
      tools::logger()->warn(
        "[DM_IMU] CRC 不匹配 reg={:#04x} calc={:#06x} got={:#06x}", rx_[3], calc, got);
    }
    rx_.erase(rx_.begin());
    return true;
  }

  const uint8_t reg = rx_[3];
  const uint8_t * payload = rx_.data() + 4;
  if (reg == kRegQuat) {
    float w = 0, x = 0, y = 0, z = 0;
    std::memcpy(&w, payload + 0, 4);
    std::memcpy(&x, payload + 4, 4);
    std::memcpy(&y, payload + 8, 4);
    std::memcpy(&z, payload + 12, 4);
    Eigen::Quaterniond q(w, x, y, z);
    const double n2 = q.squaredNorm();
    if (n2 > 0.8 && n2 < 1.2) push_attitude(q.normalized(), true);
  } else if (reg == kRegEuler && !have_quaternion_) {
    float roll = 0, pitch = 0, yaw = 0;
    std::memcpy(&roll, payload + 0, 4);
    std::memcpy(&pitch, payload + 4, 4);
    std::memcpy(&yaw, payload + 8, 4);
    constexpr double deg2rad = M_PI / 180.0;
    Eigen::Quaterniond q =
      Eigen::AngleAxisd(yaw * deg2rad, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch * deg2rad, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(roll * deg2rad, Eigen::Vector3d::UnitX());
    push_attitude(q.normalized(), false);
  }

  rx_.erase(rx_.begin(), rx_.begin() + len);
  return true;
}

void DM_IMU::push_attitude(const Eigen::Quaterniond & q, bool from_quaternion)
{
  if (from_quaternion && !have_quaternion_) {
    have_quaternion_ = true;
    tools::logger()->info("[DM_IMU] 使用模块四元数作为云台姿态");
  } else if (!from_quaternion && !have_quaternion_) {
    static bool warned = false;
    if (!warned) {
      warned = true;
      tools::logger()->warn("[DM_IMU] 尚未收到四元数帧，暂时用欧拉角重建。请打开四元数输出。");
    }
  }

  queue_.push({q, std::chrono::steady_clock::now()});
}

Eigen::Quaterniond DM_IMU::imu_at(std::chrono::steady_clock::time_point timestamp)
{
  if (data_behind_.timestamp < timestamp) data_ahead_ = data_behind_;

  while (true) {
    queue_.pop(data_behind_);
    if (data_behind_.timestamp > timestamp) break;
    data_ahead_ = data_behind_;
  }

  Eigen::Quaterniond q_a = data_ahead_.q.normalized();
  Eigen::Quaterniond q_b = data_behind_.q.normalized();
  auto t_a = data_ahead_.timestamp;
  auto t_b = data_behind_.timestamp;
  auto t_c = timestamp;
  std::chrono::duration<double> t_ab = t_b - t_a;
  std::chrono::duration<double> t_ac = t_c - t_a;

  auto k = t_ac / t_ab;
  return q_a.slerp(k, q_b).normalized();
}

}  // namespace io
