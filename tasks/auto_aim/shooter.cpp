#include "shooter.hpp"

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
  judge_distance_ = yaml["judge_distance"].as<double>();
  auto_fire_ = yaml["auto_fire"].as<bool>();
  use_gimbal_feedback_ = yaml["use_gimbal_feedback"] ? yaml["use_gimbal_feedback"].as<bool>() : true;
}

bool Shooter::shoot(
  const io::Command & command, const auto_aim::Aimer & aimer,
  const std::list<auto_aim::Target> & targets, const Eigen::Vector3d & gimbal_pos)
{
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
  if (command_stable && gimbal_ok && delta_small && aimer.debug_aim_point.valid) {
    last_command_ = command;
    return true;
  }

  last_command_ = command;
  return false;
}

}  // namespace auto_aim