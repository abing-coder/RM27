#include "barrel_ballistics.hpp"

#include <cmath>

#include "math_tools.hpp"
#include "trajectory.hpp"

namespace tools
{
BarrelSolution solve_barrel_ballistics(
  const Eigen::Vector3d & target_world, const Eigen::Vector3d & t_barrel2gimbal,
  double roll, double speed, double yaw_offset, double pitch_offset)
{
  if (!target_world.allFinite() || !t_barrel2gimbal.allFinite() || !std::isfinite(roll) ||
      !std::isfinite(speed) || speed <= 0 || !std::isfinite(yaw_offset) ||
      !std::isfinite(pitch_offset)) return {};

  double yaw = std::atan2(target_world.y(), target_world.x()) + yaw_offset;
  double pitch = 0;
  for (int iter = 0; iter < 40; ++iter) {
    const Eigen::Vector3d origin = rotation_matrix({yaw, pitch, roll}) * t_barrel2gimbal;
    const Eigen::Vector3d relative = target_world - origin;
    const double d = relative.head<2>().norm();
    if (d < 1e-6) return {};
    const Trajectory trajectory(speed, d, relative.z());
    if (trajectory.unsolvable || !std::isfinite(trajectory.pitch) ||
        !std::isfinite(trajectory.fly_time) || trajectory.fly_time <= 0) return {};

    const double azimuth = std::atan2(relative.y(), relative.x());
    const double next_yaw = limit_rad(azimuth + yaw_offset);
    const double next_pitch = -(trajectory.pitch + pitch_offset);
    if (std::abs(limit_rad(next_yaw - yaw)) < 1e-9 &&
        std::abs(next_pitch - pitch) < 1e-9) {
      BarrelSolution result;
      result.valid = true;
      result.yaw = next_yaw;
      result.pitch = next_pitch;
      result.fly_time = trajectory.fly_time;
      result.speed = speed;
      result.origin_world = origin;
      const double horizontal_speed = speed * std::cos(trajectory.pitch);
      result.velocity_world = {
        horizontal_speed * std::cos(azimuth), horizontal_speed * std::sin(azimuth),
        speed * std::sin(trajectory.pitch)};
      return result;
    }
    yaw = next_yaw;
    pitch = next_pitch;
  }
  return {};  // 不返回未收敛的发射姿态
}
}  // namespace tools
