#ifndef IO__Dm_Imu_HPP
#define IO__Dm_Imu_HPP

#include <serial/serial.h>

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

#include "tools/thread_safe_queue.hpp"

namespace io
{

// Type-C 虚拟串口上的达妙 DM-IMU-L1。姿态优先使用模块 EKF 四元数（寄存器 0x04）。
class DM_IMU
{
public:
  explicit DM_IMU(const std::string & port = "/dev/ttyACM0");
  ~DM_IMU();

  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);

private:
  struct IMUData
  {
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point timestamp;
  };

  void init_serial();
  void get_imu_data_thread();
  void parse_buffer();
  bool try_consume_frame();
  void push_attitude(const Eigen::Quaterniond & q, bool from_quaternion);

  static uint16_t crc16_ccitt(const uint8_t * data, uint16_t len);
  static uint16_t crc16_shift1(const uint8_t * data, uint16_t len);

  std::string port_;
  serial::Serial serial_;
  std::thread rec_thread_;
  std::vector<uint8_t> rx_;

  tools::ThreadSafeQueue<IMUData> queue_;
  IMUData data_ahead_, data_behind_;

  std::atomic<bool> stop_thread_{false};
  bool have_quaternion_{false};
  int crc_kind_{0};  // 0 未锁定，1 附录四的 crc<<1，2 标准 CCITT
};

}  // namespace io

#endif
