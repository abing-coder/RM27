#ifndef AUTO_AIM__AIMER_HPP
#define AUTO_AIM__AIMER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>
#include <optional>

#include "io/cboard.hpp"
#include "io/command.hpp"
#include "target.hpp"
#include "tools/barrel_ballistics.hpp"

namespace auto_aim
{

struct AimPoint
{
  bool valid;
  Eigen::Vector4d xyza;
};

class Aimer
{
public:
  AimPoint debug_aim_point{false, Eigen::Vector4d::Zero()};
  tools::BarrelSolution debug_ballistics;
  explicit Aimer(const std::string & config_path);
  const Eigen::Vector3d & barrel_translation() const;
  Eigen::Vector3d barrel_position(const Eigen::Matrix3d & R_gimbal2world) const;
  // 旧接口按 roll=0 求解；实际姿态已知时应使用下面带旋转矩阵的重载。
  io::Command aim(
    std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
    bool to_now = true);

  io::Command aim(
    std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
    const Eigen::Matrix3d & R_gimbal2world, bool to_now = true);

  io::Command aim(
    std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
    io::ShootMode shoot_mode, bool to_now = true);

private:
  double yaw_offset_;
  std::optional<double> left_yaw_offset_, right_yaw_offset_;
  double pitch_offset_;
  double comming_angle_;
  double leaving_angle_;
  double lock_id_ = -1;
  double high_speed_delay_time_;
  double low_speed_delay_time_;
  double decision_speed_;
  double default_bullet_speed_;
  Eigen::Vector3d t_barrel2gimbal_ = Eigen::Vector3d::Zero();

  io::Command aim_impl(
    std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
    const Eigen::Matrix3d & R_gimbal2world, bool to_now, double yaw_offset);

  AimPoint choose_aim_point(const Target & target);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__AIMER_HPP
