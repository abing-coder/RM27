

#include "cboard.hpp"

#include "io/damiao_can.hpp"
#include "io/socketcan.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io
{
CBoard::CBoard(const std::string & config_path)
: mode(Mode::idle),
  shoot_mode(ShootMode::left_shoot),
  bullet_speed(0),
  queue_(5000),
  can_(nullptr),
  can_disabled_(false)
{
  std::string can_interface = read_yaml(config_path);
  if (can_disabled_) {
    auto t = std::chrono::steady_clock::now();
    data_ahead_ = {Eigen::Quaterniond::Identity(), t};
    data_behind_ = {Eigen::Quaterniond::Identity(), t};
    tools::logger()->info("[Cboard] Opened (CAN disabled, using identity pose).");
    return;
  }
  auto rx = std::bind(&CBoard::callback, this, std::placeholders::_1);
  if (can_interface.rfind("damiao", 0) == 0) {
    can_ = std::make_unique<DamiaoCAN>(can_interface, rx);
  } else {
    can_ = std::make_unique<SocketCAN>(can_interface, rx);
  }
  if (can_send_only_) {
    auto t = std::chrono::steady_clock::now();
    data_ahead_ = {Eigen::Quaterniond::Identity(), t};
    data_behind_ = {Eigen::Quaterniond::Identity(), t};
    tools::logger()->info("[Cboard] Opened (send-only, no feedback).");
    return;
  }
  tools::logger()->info("[Cboard] Waiting for q...");
  queue_.pop(data_ahead_);
  queue_.pop(data_behind_);
  tools::logger()->info("[Cboard] Opened.");
}

Eigen::Quaterniond CBoard::imu_at(std::chrono::steady_clock::time_point timestamp)
{
  if (can_disabled_ || can_send_only_) return data_ahead_.q;

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

  // 四元数插值
  auto k = t_ac / t_ab;
  Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();

  return q_c;
}

// 与达妙 can_handler 协议一致：8 字节 = yaw(int16 大端) + pitch(int16 大端) + flag(有无装甲板 int16) + shoot(火控 int16)
void CBoard::send(Command command) const
{
  // 若配置了 can_send_hz_ > 0，则按该频率限流发送
  if (can_send_hz_ > 0.0) {
    auto now = std::chrono::steady_clock::now();
    auto period = std::chrono::duration<double>(1.0 / can_send_hz_);
    if (last_send_inited_) {
      auto dt = now - last_send_time_;
      if (dt < period) {
        return;  // 距离上次发送间隔未到，丢弃本次指令
      }
    }
    last_send_time_ = now;
    last_send_inited_ = true;
  }

  int16_t yaw_i = static_cast<int16_t>(command.yaw*100);
  int16_t pitch_i = static_cast<int16_t>(command.pitch*100);
  int16_t flag_i = command.control ? 1 : 0;    // 有无装甲板：有=1，无=0
  int16_t shoot_i = command.shoot ? 1 : 0;     // 火控：开火=1，不开火=0
  std::cout << "yaw_i: " << command.yaw << "   " << "pitch_i: " << command.pitch << std::endl;
  std::cout << "flag_i: " << flag_i << "   " << "shoot_i: " << shoot_i << std::endl;

  can_frame frame;
  frame.can_id = send_canid_;
  frame.can_dlc = 8;
  frame.data[0] = (yaw_i >> 8) & 0xFF;
  frame.data[1] = yaw_i & 0xFF;
  frame.data[2] = (pitch_i >> 8) & 0xFF;
  frame.data[3] = pitch_i & 0xFF;
  frame.data[4] = (flag_i >> 8) & 0xFF;
  frame.data[5] = flag_i & 0xFF;
  frame.data[6] = (shoot_i >> 8) & 0xFF;
  frame.data[7] = shoot_i & 0xFF;

  if (can_disabled_) return;
  try {
    can_->write(&frame);
  } catch (const std::exception & e) {
    tools::logger()->warn("{}", e.what());
  }
}

void CBoard::callback(const can_frame & frame)
{
  auto timestamp = std::chrono::steady_clock::now();

  if (frame.can_id == quaternion_canid_) {
    auto x = (int16_t)(frame.data[0] << 8 | frame.data[1]) / 1e4;
    auto y = (int16_t)(frame.data[2] << 8 | frame.data[3]) / 1e4;
    auto z = (int16_t)(frame.data[4] << 8 | frame.data[5]) / 1e4;
    auto w = (int16_t)(frame.data[6] << 8 | frame.data[7]) / 1e4;

    if (std::abs(x * x + y * y + z * z + w * w - 1) > 1e-2) {
      tools::logger()->warn("Invalid q: {} {} {} {}", w, x, y, z);
      return;
    }

    queue_.push({{w, x, y, z}, timestamp});
  }

  else if (frame.can_id == bullet_speed_canid_) {
    bullet_speed = (int16_t)(frame.data[0] << 8 | frame.data[1]) / 1e2;
    mode = Mode(frame.data[2]);
    shoot_mode = ShootMode(frame.data[3]);
    ft_angle = (int16_t)(frame.data[4] << 8 | frame.data[5]) / 1e4;

    // 限制日志输出频率为1Hz
    static auto last_log_time = std::chrono::steady_clock::time_point::min();
    auto now = std::chrono::steady_clock::now();

    if (bullet_speed > 0 && tools::delta_time(now, last_log_time) >= 1.0) {
      tools::logger()->info(
        "[CBoard] Bullet speed: {:.2f} m/s, Mode: {}, Shoot mode: {}, FT angle: {:.2f} rad",
        bullet_speed, MODES[mode], SHOOT_MODES[shoot_mode], ft_angle);
      last_log_time = now;
    }
  }
}

// 实现方式有待改进
std::string CBoard::read_yaml(const std::string & config_path)
{
  auto yaml = tools::load(config_path);

  can_disabled_ = yaml["can_disabled"] ? yaml["can_disabled"].as<bool>() : false;
  can_send_only_ = yaml["can_send_only"] ? yaml["can_send_only"].as<bool>() : false;
  // can_send_hz: 0 或缺失=按调用频率发送；>0 时按该值限流发送
  can_send_hz_ = yaml["can_send_hz"] ? yaml["can_send_hz"].as<double>() : 0.0;
  quaternion_canid_ = tools::read<int>(yaml, "quaternion_canid");
  bullet_speed_canid_ = tools::read<int>(yaml, "bullet_speed_canid");
  send_canid_ = tools::read<int>(yaml, "send_canid");

  if (can_disabled_) return std::string();

  if (!yaml["can_interface"]) {
    throw std::runtime_error("Missing 'can_interface' in YAML configuration.");
  }
  return yaml["can_interface"].as<std::string>();
}

}  // namespace io