#ifndef TOOLS__BARREL_BALLISTICS_HPP
#define TOOLS__BARREL_BALLISTICS_HPP

#include <Eigen/Geometry>

namespace tools
{
struct BarrelSolution
{
  bool valid = false;
  double yaw = 0;    // 云台目标世界角，rad，左转为正
  double pitch = 0;  // 云台目标世界角，rad，抬头为负
  double fly_time = 0;
  double speed = 0;
  Eigen::Vector3d origin_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity_world = Eigen::Vector3d::Zero();  // 补偿前理论弹道的初速度
};

// 枪口采用云台参考轴向。roll 保持当前值；迭代求发射姿态和随姿态变化的发射点。
// 原版角度补偿仍是经验校正，不作为额外的枪口旋转外参。
BarrelSolution solve_barrel_ballistics(
  const Eigen::Vector3d & target_world, const Eigen::Vector3d & t_barrel2gimbal,
  double roll, double speed, double yaw_offset = 0, double pitch_offset = 0);
}  // namespace tools

#endif  // TOOLS__BARREL_BALLISTICS_HPP
