#include "aimer.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <limits>
#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"

namespace auto_aim
{
namespace
{
double positive_phase(double angle)
{
  auto phase = std::fmod(angle, 2 * CV_PI);
  if (phase < 0) phase += 2 * CV_PI;
  return phase;
}
}  // namespace

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
  small_gyro_speed_thresh_ =
    yaml["small_gyro_speed_thresh"] ? yaml["small_gyro_speed_thresh"].as<double>() : 2.0;
  if (yaml["left_yaw_offset"].IsDefined() && yaml["right_yaw_offset"].IsDefined()) {
    left_yaw_offset_ = yaml["left_yaw_offset"].as<double>() / 57.3;    // degree to rad
    right_yaw_offset_ = yaml["right_yaw_offset"].as<double>() / 57.3;  // degree to rad
    tools::logger()->info("[Aimer] successfully loading shootmode");
  }
}

io::Command Aimer::aim(
  std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
  bool to_now)
{
  debug_aim_point = {false, Eigen::Vector4d::Zero()};
  debug_raw_aim_point = {false, Eigen::Vector4d::Zero()};
  debug_raw_pitch = std::nullopt;
  debug_small_gyro_detected = false;
  debug_small_gyro_active = false;
  debug_small_gyro_armor_id = -1;
  debug_total_predict_time = 0.0;
  debug_fly_time = 0.0;
  if (targets.empty()) return {false, false, 0, 0};
  auto target = targets.front();
  debug_small_gyro_detected = is_small_gyro(target);

  double delay_time =
    std::abs(target.ekf_x()[7]) > decision_speed_ ? high_speed_delay_time_ : low_speed_delay_time_;

  if (bullet_speed < 14) bullet_speed = 20;

  auto raw_aim_point = choose_aim_point(target, false);
  debug_raw_aim_point = raw_aim_point;
  debug_raw_pitch = solve_ballistic_pitch(raw_aim_point, bullet_speed);

  // 弹速打印
  std::cout << "## bullet_speed: " << bullet_speed << std::endl;

  // 考虑detecor和tracker所消耗的时间，此外假设aimer的用时可忽略不计
  auto future = timestamp;
  if (to_now) {
    double dt = tools::delta_time(std::chrono::steady_clock::now(), timestamp) + delay_time;
    future += std::chrono::microseconds(int(dt * 1e6));
    target.predict(future);
  } else {
    auto dt = 0.005 + delay_time;  // detector-aimer耗时0.005 + 发弹延时
    future += std::chrono::microseconds(int(dt * 1e6));
    target.predict(future);
  }

  auto aim_point0 = choose_aim_point(target);
  debug_aim_point = aim_point0;
  debug_total_predict_time = tools::delta_time(future, timestamp);
  if (!aim_point0.valid) {
    return {false, false, 0, 0};
  }

  Eigen::Vector3d xyz0 = aim_point0.xyza.head(3);
  auto d0 = std::sqrt(xyz0[0] * xyz0[0] + xyz0[1] * xyz0[1]);
  tools::Trajectory trajectory0(bullet_speed, d0, xyz0[2]);
  if (trajectory0.unsolvable) {
    tools::logger()->debug(
      "[Aimer] Unsolvable trajectory0: {:.2f} {:.2f} {:.2f}", bullet_speed, d0, xyz0[2]);
    debug_aim_point.valid = false;
    return {false, false, 0, 0};
  }

  double prev_fly_time = trajectory0.fly_time;
  tools::Trajectory current_traj = trajectory0;
  tools::Trajectory current_traj_d0 = trajectory0;

  std::vector<Target> iteration_target(10, target);  // 创建10个目标副本用于迭代预测

  for (int iter = 0; iter < 10; ++iter) {
    // 预测目标在 future + prev_fly_time 时刻的位置
    auto predict_time = future + std::chrono::microseconds(static_cast<int>(prev_fly_time * 1e6));
    iteration_target[iter].predict(predict_time);

    auto aim_point = choose_aim_point(iteration_target[iter]);
    debug_aim_point = aim_point;
    debug_total_predict_time = tools::delta_time(predict_time, timestamp);
    if (!aim_point.valid) {
      return {false, false, 0, 0};
    }

    Eigen::Vector3d xyz = aim_point.xyza.head(3);
    double d = std::sqrt(xyz.x() * xyz.x() + xyz.y() * xyz.y());
    current_traj = tools::Trajectory(bullet_speed, d, xyz.z());
    current_traj_d0 = tools::Trajectory(bullet_speed, 1.0, xyz.z());

    if (current_traj.unsolvable) {
      tools::logger()->debug(
        "[Aimer] Unsolvable trajectory in iter {}: speed={:.2f}, d={:.2f}, z={:.2f}", iter + 1,
        bullet_speed, d, xyz.z());
      debug_aim_point.valid = false;
      return {false, false, 0, 0};
    }

    if (std::abs(current_traj.fly_time - prev_fly_time) < 0.001) break;
    prev_fly_time = current_traj.fly_time;
  }

  debug_fly_time = current_traj.fly_time;
  Eigen::Vector3d final_xyz = debug_aim_point.xyza.head(3);
  double yaw = std::atan2(final_xyz.y(), final_xyz.x()) + yaw_offset_;
  double pitch = current_traj.pitch + pitch_offset_;           // 世界坐标系下pitch向上为负
  double pitch_d0 = -(current_traj_d0.pitch + pitch_offset_);  // 测试 d=0m 的pitch值

  // std::cout << "## pitch: " << pitch << std::endl;
  // std::cout << "## pitch_d0: " << pitch_d0 << std::endl;

  return {true, false, yaw, pitch};
}

std::optional<double> Aimer::solve_ballistic_pitch(
  const AimPoint & aim_point, double bullet_speed) const
{
  if (!aim_point.valid) return std::nullopt;

  return solve_ballistic_pitch(aim_point.xyza.head(3), bullet_speed);
}

std::optional<double> Aimer::solve_ballistic_pitch(
  const Eigen::Vector3d & xyz_in_world, double bullet_speed) const
{
  if (bullet_speed < 14) bullet_speed = 22;

  double d = std::sqrt(xyz_in_world.x() * xyz_in_world.x() + xyz_in_world.y() * xyz_in_world.y());
  tools::Trajectory trajectory(bullet_speed, d, xyz_in_world.z());
  if (trajectory.unsolvable) return std::nullopt;

  return trajectory.pitch + pitch_offset_;
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

  auto command = aim(targets, timestamp, bullet_speed, to_now);
  command.yaw = command.yaw - yaw_offset_ + yaw_offset;

  return command;
}

bool Aimer::is_small_gyro(const Target & target) const
{
  return target.name != ArmorName::outpost && std::abs(target.ekf_x()[7]) >= small_gyro_speed_thresh_ &&
         target.armor_xyza_list().size() >= 2;
}

int Aimer::choose_small_gyro_armor_id(const Target & target) const
{
  const auto ekf_x = target.ekf_x();
  const auto armor_xyza_list = target.armor_xyza_list();
  const double w = ekf_x[7];
  if (armor_xyza_list.empty() || std::abs(w) < 1e-6) return -1;

  const double center_yaw = std::atan2(ekf_x[2], ekf_x[0]);
  double best_phase = std::numeric_limits<double>::max();
  int best_id = -1;

  for (size_t i = 0; i < armor_xyza_list.size(); ++i) {
    const double delta_angle = tools::limit_rad(armor_xyza_list[i][3] - center_yaw);
    const double phase = (w > 0) ? positive_phase(-delta_angle) : positive_phase(delta_angle);
    if (phase < best_phase) {
      best_phase = phase;
      best_id = static_cast<int>(i);
    }
  }

  return best_id;
}

AimPoint Aimer::choose_small_gyro_aim_point(const Target & target)
{
  const auto ekf_x = target.ekf_x();
  const auto armor_xyza_list = target.armor_xyza_list();
  const int selected_id = choose_small_gyro_armor_id(target);
  if (selected_id < 0 || selected_id >= static_cast<int>(armor_xyza_list.size())) {
    return {false, Eigen::Vector4d::Zero()};
  }

  const double center_yaw = std::atan2(ekf_x[2], ekf_x[0]);
  const Eigen::Vector2d center_xy(ekf_x[0], ekf_x[2]);
  const Eigen::Vector2d armor_xy(armor_xyza_list[selected_id][0], armor_xyza_list[selected_id][1]);
  const double orbit_radius = (center_xy - armor_xy).norm();

  Eigen::Vector4d aim_xyza;
  aim_xyza << ekf_x[0] - orbit_radius * std::cos(center_yaw),
    ekf_x[2] - orbit_radius * std::sin(center_yaw), armor_xyza_list[selected_id][2], center_yaw;

  debug_small_gyro_active = true;
  debug_small_gyro_armor_id = selected_id;
  return {true, aim_xyza};
}

AimPoint Aimer::choose_aim_point(const Target & target, bool update_lock)
{
  Eigen::VectorXd ekf_x = target.ekf_x();
  std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
  auto armor_num = armor_xyza_list.size();
  if (armor_xyza_list.empty()) return {false, Eigen::Vector4d::Zero()};

  debug_small_gyro_active = false;
  debug_small_gyro_armor_id = -1;

  // 如果装甲板未发生过跳变，则只有当前装甲板的位置已知
  if (!target.jumped) return {true, armor_xyza_list[0]};

  if (is_small_gyro(target)) {
    auto anti_top_aim_point = choose_small_gyro_aim_point(target);
    if (anti_top_aim_point.valid) return anti_top_aim_point;
  }

  const auto center_yaw = std::atan2(ekf_x[2], ekf_x[0]);

  // 如果delta_angle为0，则该装甲板中心和整车中心的连线在世界坐标系的xy平面过原点
  std::vector<double> delta_angle_list;
  for (int i = 0; i < armor_num; i++) {
    auto delta_angle = tools::limit_rad(armor_xyza_list[i][3] - center_yaw);
    delta_angle_list.emplace_back(delta_angle);
  }

  std::vector<int> id_list;
  for (int i = 0; i < armor_num; i++) {
    if (std::abs(delta_angle_list[i]) > 60 / 57.3) continue;
    id_list.push_back(i);
  }
  if (id_list.empty()) {
    tools::logger()->warn("Empty id list!");
    return {false, armor_xyza_list[0]};
  }

  if (id_list.size() > 1) {
    int id0 = id_list[0], id1 = id_list[1];
    int selected_id = -1;

    // 未处于锁定模式时，选择delta_angle绝对值较小的装甲板，进入锁定模式
    if (lock_id_ == id0 || lock_id_ == id1) {
      selected_id = static_cast<int>(lock_id_);
    } else {
      selected_id = (std::abs(delta_angle_list[id0]) < std::abs(delta_angle_list[id1])) ? id0 : id1;
    }
    if (update_lock) lock_id_ = selected_id;

    return {true, armor_xyza_list[selected_id]};
  }

  if (update_lock) lock_id_ = -1;
  return {true, armor_xyza_list[id_list[0]]};
}

}  // namespace auto_aim
