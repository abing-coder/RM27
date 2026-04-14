#ifndef AUTO_AIM__SHOOTER_HPP
#define AUTO_AIM__SHOOTER_HPP

#include <optional>
#include <string>

#include "io/command.hpp"
#include "tasks/auto_aim/aimer.hpp"

namespace auto_aim
{
class Shooter
{
public:
  std::optional<double> debug_small_gyro_yaw_error_deg = std::nullopt;
  bool debug_small_gyro_fire_window = false;
  bool debug_small_gyro_checked = false;

  Shooter(const std::string & config_path);

  bool shoot(
    const io::Command & command, const auto_aim::Aimer & aimer,
    const std::list<auto_aim::Target> & targets, const Eigen::Vector3d & gimbal_pos);

  void reset_debug_state();

private:
  io::Command last_command_;
  double judge_distance_;
  double first_tolerance_;
  double second_tolerance_;
  double small_gyro_fire_yaw_thresh_;
  bool auto_fire_;
  bool use_gimbal_feedback_;  // false 时无云台反馈(无IMU)，仅凭指令稳定判定开火

  bool shoot_small_gyro(const auto_aim::Aimer & aimer, const auto_aim::Target & target);
};
}  // namespace auto_aim

#endif  // AUTO_AIM__SHOOTER_HPP
