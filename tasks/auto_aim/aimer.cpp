#include "aimer.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
Aimer::Aimer(const std::string & config_path)
: left_yaw_offset_(std::nullopt), right_yaw_offset_(std::nullopt)
{
  auto yaml = YAML::LoadFile(config_path);
  yaw_offset_ = yaml["yaw_offset"].as<double>() / 57.3;        // degree to rad
  pitch_offset_ = yaml["pitch_offset"].as<double>() / 57.3;    // degree to rad
  comming_angle_ = yaml["comming_angle"].as<double>() / 57.3;  // degree to rad
  leaving_angle_ = yaml["leaving_angle"].as<double>() / 57.3;  // degree to rad
  high_speed_delay_time_ = yaml["high_speed_delay_time"].as<double>();
  low_speed_delay_time_ = yaml["low_speed_delay_time"].as<double>();
  decision_speed_ = yaml["decision_speed"].as<double>();
  default_bullet_speed_ = yaml["bullet_speed"] ? yaml["bullet_speed"].as<double>() : 22.0;
  if (yaml["t_barrel2gimbal"]) {
    const auto t = yaml["t_barrel2gimbal"].as<std::vector<double>>();
    if (t.size() != 3) throw std::runtime_error("t_barrel2gimbal 必须包含 3 个数，单位米");
    t_barrel2gimbal_ = {t[0], t[1], t[2]};
    if (!t_barrel2gimbal_.allFinite()) {
      throw std::runtime_error("t_barrel2gimbal 必须是有限数值");
    }
  }
  if (yaml["left_yaw_offset"].IsDefined() && yaml["right_yaw_offset"].IsDefined()) {
    left_yaw_offset_ = yaml["left_yaw_offset"].as<double>() / 57.3;    // degree to rad
    right_yaw_offset_ = yaml["right_yaw_offset"].as<double>() / 57.3;  // degree to rad
    tools::logger()->info("[Aimer] successfully loading shootmode");
  }
}

const Eigen::Vector3d & Aimer::barrel_translation() const { return t_barrel2gimbal_; }

Eigen::Vector3d Aimer::barrel_position(const Eigen::Matrix3d & R_gimbal2world) const
{
  return R_gimbal2world * t_barrel2gimbal_;
}

io::Command Aimer::aim(
  std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
  bool to_now)
{
  return aim_impl(
    std::move(targets), timestamp, bullet_speed, Eigen::Matrix3d::Identity(), to_now, yaw_offset_);
}

io::Command Aimer::aim(
  std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
  const Eigen::Matrix3d & R_gimbal2world, bool to_now)
{
  return aim_impl(
    std::move(targets), timestamp, bullet_speed, R_gimbal2world, to_now, yaw_offset_);
}

io::Command Aimer::aim_impl(
  std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
  const Eigen::Matrix3d & R_gimbal2world, bool to_now, double yaw_offset)
{
  debug_aim_point = {false, Eigen::Vector4d::Zero()};
  debug_ballistics = {};
  if (!R_gimbal2world.allFinite()) return {false, false, 0, 0};
  if (targets.empty()) return {false, false, 0, 0};
  auto target = targets.front();

  double delay_time =
    target.ekf_x()[7] > decision_speed_ ? high_speed_delay_time_ : low_speed_delay_time_;

  if (bullet_speed < 14) bullet_speed = default_bullet_speed_;

  // 考虑detecor和tracker所消耗的时间，此外假设aimer的用时可忽略不计
  auto future = timestamp;
  if (to_now) {
    double dt;
    dt = tools::delta_time(std::chrono::steady_clock::now(), timestamp) + delay_time;
    future += std::chrono::microseconds(int(dt * 1e6));
    target.predict(future);
  }

  else {
    auto dt = 0.005 + delay_time;  //detector-aimer耗时0.005+发弹延时0.1
    // tools::logger()->info("dt is {:.4f} second", dt);
    future += std::chrono::microseconds(int(dt * 1e6));
    target.predict(future);
  }

  auto aim_point0 = choose_aim_point(target);
  debug_aim_point = aim_point0;
  if (!aim_point0.valid) {
    // tools::logger()->debug("Invalid aim_point0.");
    return {false, false, 0, 0};
  }

  // 目标姿态的 yaw/pitch 由弹道求解，roll 沿用当前 IMU 姿态。
  const double roll = tools::eulers(R_gimbal2world, 2, 1, 0)[2];
  auto solve_point = [&](const AimPoint & point) {
    return tools::solve_barrel_ballistics(
      point.xyza.head(3), t_barrel2gimbal_, roll, bullet_speed, yaw_offset, pitch_offset_);
  };
  const auto initial_solution = solve_point(aim_point0);
  if (!initial_solution.valid) {
    debug_aim_point.valid = false;
    return {false, false, 0, 0};
  }

  // 飞行时间来自同一个枪口模型；每次都从发弹时刻的目标副本向后预测。
  double prev_fly_time = initial_solution.fly_time;

  for (int iter = 0; iter < 20; ++iter) {
    // 预测目标在 future + prev_fly_time 时刻的位置
    auto predict_time = future + std::chrono::microseconds(static_cast<int>(prev_fly_time * 1e6));
    auto predicted_target = target;
    predicted_target.predict(predict_time);

    // 计算瞄准点
    auto aim_point = choose_aim_point(predicted_target);
    debug_aim_point = aim_point;
    if (!aim_point.valid) {
      return {false, false, 0, 0};
    }

    const auto solution = solve_point(aim_point);
    if (!solution.valid) {
      debug_aim_point.valid = false;
      return {false, false, 0, 0};
    }

    if (std::abs(solution.fly_time - prev_fly_time) < 1e-5) {
      debug_ballistics = solution;
      return {true, false, solution.yaw, solution.pitch};
    }
    prev_fly_time = solution.fly_time;
  }

  debug_aim_point.valid = false;
  return {false, false, 0, 0};
}

io::Command Aimer::aim(
  std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
  io::ShootMode shoot_mode, bool to_now)
{
  double yaw_offset;
  if (shoot_mode == io::left_shoot && left_yaw_offset_.has_value()) {
    yaw_offset = left_yaw_offset_.value();
  } else if (shoot_mode == io::right_shoot && right_yaw_offset_.has_value()) {
    yaw_offset = right_yaw_offset_.value();
  } else {
    yaw_offset = yaw_offset_;
  }

  return aim_impl(
    std::move(targets), timestamp, bullet_speed, Eigen::Matrix3d::Identity(), to_now, yaw_offset);
}

AimPoint Aimer::choose_aim_point(const Target & target)
{
  Eigen::VectorXd ekf_x = target.ekf_x();
  std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
  auto armor_num = armor_xyza_list.size();
  // 如果装甲板未发生过跳变，则只有当前装甲板的位置已知
  if (!target.jumped) return {true, armor_xyza_list[0]};

  // 整车旋转中心的球坐标yaw
  auto center_yaw = std::atan2(ekf_x[2], ekf_x[0]);

  // 如果delta_angle为0，则该装甲板中心和整车中心的连线在世界坐标系的xy平面过原点
  std::vector<double> delta_angle_list;
  for (int i = 0; i < armor_num; i++) {
    auto delta_angle = tools::limit_rad(armor_xyza_list[i][3] - center_yaw);
    delta_angle_list.emplace_back(delta_angle);
  }

  // 不考虑小陀螺
  if (std::abs(target.ekf_x()[8]) <= 2 && target.name != ArmorName::outpost) {
    // 选择在可射击范围内的装甲板
    std::vector<int> id_list;
    for (int i = 0; i < armor_num; i++) {
      if (std::abs(delta_angle_list[i]) > 60 / 57.3) continue;
      id_list.push_back(i);
    }
    // 绝无可能
    if (id_list.empty()) {
      tools::logger()->warn("Empty id list!");
      return {false, armor_xyza_list[0]};
    }

    // 锁定模式：防止在两个都呈45度的装甲板之间来回切换
    if (id_list.size() > 1) {
      int id0 = id_list[0], id1 = id_list[1];

      // 未处于锁定模式时，选择delta_angle绝对值较小的装甲板，进入锁定模式
      if (lock_id_ != id0 && lock_id_ != id1)
        lock_id_ = (std::abs(delta_angle_list[id0]) < std::abs(delta_angle_list[id1])) ? id0 : id1;

      return {true, armor_xyza_list[lock_id_]};
    }

    // 只有一个装甲板在可射击范围内时，退出锁定模式
    lock_id_ = -1;
    return {true, armor_xyza_list[id_list[0]]};
  }

  double coming_angle, leaving_angle;
  if (target.name == ArmorName::outpost) {
    coming_angle = 70 / 57.3;
    leaving_angle = 30 / 57.3;
  } else {
    coming_angle = comming_angle_;
    leaving_angle = leaving_angle_;
  }

  // 在小陀螺时，一侧的装甲板不断出现，另一侧的装甲板不断消失，显然前者被打中的概率更高
  for (int i = 0; i < armor_num; i++) {
    if (std::abs(delta_angle_list[i]) > coming_angle) continue;
    if (ekf_x[7] > 0 && delta_angle_list[i] < leaving_angle) return {true, armor_xyza_list[i]};
    if (ekf_x[7] < 0 && delta_angle_list[i] > -leaving_angle) return {true, armor_xyza_list[i]};
  }

  return {false, armor_xyza_list[0]};
}

}  // namespace auto_aim
