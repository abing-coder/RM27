#include "shooter.hpp"

#include <cmath>

#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
Shooter::Shooter(const std::string & config_path) : last_command_{false, false, 0, 0}
{
  auto yaml = YAML::LoadFile(config_path);
  // 配置文件中的 first_tolerance/second_tolerance 以“度”为单位
  first_tolerance_ = yaml["first_tolerance"].as<double>();
  second_tolerance_ = yaml["second_tolerance"].as<double>();
  small_gyro_fire_yaw_thresh_ =
    yaml["small_gyro_fire_yaw_thresh"] ? yaml["small_gyro_fire_yaw_thresh"].as<double>() : 1.0;
  fire_window_yaw_thresh_deg_ =
    yaml["fire_window_yaw_thresh_deg"] ? yaml["fire_window_yaw_thresh_deg"].as<double>() : 1.5;
  fire_window_pitch_thresh_deg_ =
    yaml["fire_window_pitch_thresh_deg"] ? yaml["fire_window_pitch_thresh_deg"].as<double>() : 2.0;
  fire_ready_consecutive_frames_ =
    yaml["fire_ready_consecutive_frames"] ? yaml["fire_ready_consecutive_frames"].as<int>() : 2;
  fire_cooldown_ms_ = yaml["fire_cooldown_ms"] ? yaml["fire_cooldown_ms"].as<int>() : 60;
  judge_distance_ = yaml["judge_distance"].as<double>();
  auto_fire_ = yaml["auto_fire"].as<bool>();
  use_gimbal_feedback_ = yaml["use_gimbal_feedback"] ? yaml["use_gimbal_feedback"].as<bool>() : true;
}

void Shooter::reset_debug_state()
{
  debug_small_gyro_yaw_error_deg = std::nullopt;
  debug_small_gyro_fire_window = false;
  debug_small_gyro_checked = false;
  debug_hit_yaw_error_deg = std::nullopt;
  debug_hit_pitch_error_deg = std::nullopt;
  debug_hit_window_ok = false;
  debug_fire_ready_streak = fire_ready_streak_;
}

bool Shooter::shoot_small_gyro(const auto_aim::Aimer & aimer, const auto_aim::Target & target)
{
  if (!aimer.debug_small_gyro_active || !aimer.debug_aim_point.valid) return false;
  if (aimer.debug_small_gyro_armor_id < 0 || aimer.debug_total_predict_time <= 0) return false;

  debug_small_gyro_checked = true;

  auto predicted_target = target;
  predicted_target.predict(aimer.debug_total_predict_time);
  auto predicted_armors = predicted_target.armor_xyza_list();
  if (aimer.debug_small_gyro_armor_id >= static_cast<int>(predicted_armors.size())) return false;

  const auto & predicted_armor = predicted_armors[aimer.debug_small_gyro_armor_id];
  const double predicted_yaw = std::atan2(predicted_armor[1], predicted_armor[0]);
  const double aim_yaw = std::atan2(aimer.debug_aim_point.xyza[1], aimer.debug_aim_point.xyza[0]);
  const double yaw_error_deg =
    std::abs(tools::limit_rad(predicted_yaw - aim_yaw)) * 180.0 / 3.141592653589793;
  debug_small_gyro_yaw_error_deg = yaw_error_deg;
  debug_small_gyro_fire_window = yaw_error_deg < small_gyro_fire_yaw_thresh_;
  
  return debug_small_gyro_fire_window;
}

bool Shooter::hit_window_ok(const auto_aim::Aimer & aimer, const auto_aim::Target & target)
{
  // 用 “子弹到达装甲板那一刻” 的预测装甲板角度，判断是否进入命中窗口
  if (!aimer.debug_aim_point.valid) return false;
  if (aimer.debug_total_predict_time <= 0 || aimer.debug_fly_time <= 0) return false;

  auto predicted_target = target;
  predicted_target.predict(aimer.debug_total_predict_time + aimer.debug_fly_time);
  auto predicted_armors = predicted_target.armor_xyza_list();
  if (predicted_armors.empty()) return false;

  const double aim_yaw = std::atan2(aimer.debug_aim_point.xyza[1], aimer.debug_aim_point.xyza[0]);
  const double aim_pitch = std::atan2(
    aimer.debug_aim_point.xyza[2],
    std::sqrt(tools::square(aimer.debug_aim_point.xyza[0]) + tools::square(aimer.debug_aim_point.xyza[1])));

  // 选取“预测时刻最接近枪口方向”的那块装甲板
  double best_yaw_err = 1e9;
  double best_pitch_err = 1e9;
  for (const auto & armor : predicted_armors) {
    const double yaw = std::atan2(armor[1], armor[0]);
    const double pitch = std::atan2(armor[2], std::sqrt(tools::square(armor[0]) + tools::square(armor[1])));
    const double yaw_err = std::abs(tools::limit_rad(yaw - aim_yaw));
    const double pitch_err = std::abs(tools::limit_rad(pitch - aim_pitch));
    if (yaw_err < best_yaw_err) {
      best_yaw_err = yaw_err;
      best_pitch_err = pitch_err;
    }
  }

  const double rad2deg = 180.0 / 3.141592653589793;
  debug_hit_yaw_error_deg = best_yaw_err * rad2deg;
  debug_hit_pitch_error_deg = best_pitch_err * rad2deg;
  debug_hit_window_ok =
    (debug_hit_yaw_error_deg.value() < fire_window_yaw_thresh_deg_) &&
    (debug_hit_pitch_error_deg.value() < fire_window_pitch_thresh_deg_);
  return debug_hit_window_ok;
}

bool Shooter::shoot(
  const io::Command & command, const auto_aim::Aimer & aimer,
  const std::list<auto_aim::Target> & targets, const Eigen::Vector3d & gimbal_pos)
{
  reset_debug_state();
  if (!command.control || targets.empty() || !auto_fire_) return false;

  // 目标位置（m）
  auto target_x = targets.front().ekf_x()[0];
  auto target_y = targets.front().ekf_x()[2];
  // 距离越远容差越大（单位：度）
  auto tolerance = std::sqrt(tools::square(target_x) + tools::square(target_y)) > judge_distance_
                     ? second_tolerance_
                     : first_tolerance_;

  // 将云台姿态从弧度转为角度（deg）
  const double rad2deg = 180.0 / 3.141592653589793;
  double gimbal_yaw_deg = gimbal_pos[0] * rad2deg;

  // 此处所有比较都在“度”空间：Command 中的 yaw/pitch 也按“度”理解
  bool command_stable =
    std::abs(last_command_.yaw - command.yaw) < tolerance * 2 &&
    std::abs(last_command_.pitch - command.pitch) < tolerance * 2;
  bool gimbal_ok =
    !use_gimbal_feedback_ || (std::abs(gimbal_yaw_deg - last_command_.yaw) < tolerance);
  // 无云台反馈时：仅当当前 delta（瞄准误差）足够小才开火，即认为云台已跟上目标
  bool delta_small =
    use_gimbal_feedback_ || (std::abs(command.yaw) < tolerance && std::abs(command.pitch) < tolerance);

  bool fire_ready = false;
  if (aimer.debug_small_gyro_active) {
    // 小陀螺模式下走专用火控：核心判据是预测装甲板是否进入枪口 yaw 窗口。
    fire_ready = aimer.debug_aim_point.valid && gimbal_ok && shoot_small_gyro(aimer, targets.front());
  } else {
    // 普通目标：必须满足“预测命中窗口”，避免装甲板旋转但未进入枪线时浪费弹药
    bool window_ok = hit_window_ok(aimer, targets.front());
    fire_ready = command_stable && gimbal_ok && delta_small && window_ok;
  }

  // 额外抑制：需要连续若干帧满足 fire_ready，且两次开火之间有冷却时间
  auto now = std::chrono::steady_clock::now();
  if (fire_ready) {
    fire_ready_streak_++;
  } else {
    fire_ready_streak_ = 0;
  }
  debug_fire_ready_streak = fire_ready_streak_;

  bool cooldown_ok = true;
  if (last_fire_time_ != std::chrono::steady_clock::time_point::min()) {
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_fire_time_).count();
    cooldown_ok = ms >= fire_cooldown_ms_;
  }

  if (fire_ready && fire_ready_streak_ >= fire_ready_consecutive_frames_ && cooldown_ok) {
    last_fire_time_ = now;
    last_command_ = command;
    return true;
  }

  last_command_ = command;
  return false;
}

}  // namespace auto_aim
