#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tools/barrel_ballistics.hpp"
#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"

namespace
{
void require(bool condition, const char * message)
{
  if (!condition) throw std::runtime_error(message);
}

Eigen::Vector3d impact(const tools::BarrelSolution & solution)
{
  const double t = solution.fly_time;
  return solution.origin_world + solution.velocity_world * t +
    Eigen::Vector3d(0, 0, -0.5 * 9.7833 * t * t);
}

struct ConfigFile
{
  std::filesystem::path path = std::filesystem::temp_directory_path() /
    ("rm27_barrel_test_" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()) + ".yaml");
  YAML::Node yaml;

  ConfigFile(const std::string & source, const Eigen::Vector3d & translation)
  : yaml(YAML::LoadFile(source))
  {
    yaml["t_barrel2gimbal"] = std::vector<double>{translation.x(), translation.y(), translation.z()};
    yaml["yaw_offset"] = 0;
    yaml["pitch_offset"] = 0;
    yaml["low_speed_delay_time"] = 0;
    yaml["high_speed_delay_time"] = 0;
    yaml["auto_fire"] = true;
    yaml["use_gimbal_feedback"] = true;
    yaml["fire_ready_consecutive_frames"] = 2;
    save();
  }
  void save() const { std::ofstream(path) << yaml; }
  ~ConfigFile()
  {
    std::error_code error;
    std::filesystem::remove(path, error);
  }
};

auto_aim::Armor observation(const Eigen::Vector3d & point)
{
  auto_aim::Armor armor(
    0, 3, 0.99f, {0, 0, 100, 50}, {{0, 0}, {100, 0}, {100, 50}, {0, 50}});
  armor.priority = auto_aim::ArmorPriority::first;
  armor.xyz_in_world = point;
  armor.ypr_in_world = Eigen::Vector3d::Zero();
  armor.ypd_in_world = tools::xyz2ypd(point);
  return armor;
}

void test_geometry()
{
  for (const Eigen::Vector3d & point : std::vector<Eigen::Vector3d>{
         {2, 0, 0}, {5, 2, 1}, {-4, 0.01, -0.4}, {1, -3, 0.5}}) {
    for (const Eigen::Vector3d & translation : std::vector<Eigen::Vector3d>{
           {0, 0, 0}, {0.2, -0.06, 0.1}, {-0.1, 0.08, -0.05}}) {
      for (double roll : {0.0, 0.4, -0.5}) {
        const auto solution = tools::solve_barrel_ballistics(point, translation, roll, 22);
        require(solution.valid, "reachable barrel trajectory rejected");
        require((impact(solution) - point).norm() < 1e-6, "forward projectile missed target");
        const Eigen::Matrix3d pose = tools::rotation_matrix({solution.yaw, solution.pitch, roll});
        require((solution.origin_world - pose * translation).norm() < 1e-8,
          "origin does not follow the firing pose");
        require((solution.velocity_world - pose.col(0) * 22).norm() < 1e-6,
          "launch velocity is not along the barrel axis");
        if (translation.isZero(0)) {
          const tools::Trajectory legacy(22, point.head<2>().norm(), point.z());
          require(std::abs(solution.fly_time - legacy.fly_time) < 1e-10,
            "zero translation changed legacy flight time");
          require(std::abs(solution.pitch + legacy.pitch) < 1e-10,
            "zero translation changed legacy pitch");
        }
      }
    }
  }
  const Eigen::Vector3d calibrated_point(5, 2, 1), translation(0.2, -0.06, 0.1);
  const auto calibrated = tools::solve_barrel_ballistics(
    calibrated_point, translation, 0.4, 22, -1.0 / 57.3, -1.4 / 57.3);
  require(calibrated.valid && (impact(calibrated) - calibrated_point).norm() < 1e-6,
    "empirical offsets broke the theoretical trajectory");
  require((calibrated.origin_world -
    tools::rotation_matrix({calibrated.yaw, calibrated.pitch, 0.4}) * translation).norm() < 1e-8,
    "firing origin ignored the compensated gimbal pose");
  require(!tools::solve_barrel_ballistics({1000, 0, 100}, {0, 0, 0}, 0, 10).valid,
    "unreachable shot accepted");
  require(!tools::solve_barrel_ballistics({0, 0, 0}, {0, 0, 0}, 0, 22).valid,
    "degenerate shot accepted");
  require(!tools::solve_barrel_ballistics({3, 0, 0}, {0, 0, 0}, 0, 0).valid,
    "zero speed accepted");
  require(!tools::solve_barrel_ballistics(
    {3, 0, 0}, {0, 0, 0}, 0, std::numeric_limits<double>::quiet_NaN()).valid,
    "nonfinite speed accepted");
}

void test_aimer_and_fire(const std::string & source)
{
  ConfigFile config(source, {0.2, -0.06, 0.1});
  auto_aim::Aimer aimer(config.path.string());
  const auto timestamp = std::chrono::steady_clock::now();
  const Eigen::Vector3d point(4, 0.5, 0.2);
  auto armor = observation(point);
  auto_aim::Target target(armor, timestamp, 0.2, 4, Eigen::VectorXd::Ones(11));
  const Eigen::Matrix3d current_pose = tools::rotation_matrix({0.3, -0.1, 0.4});
  auto command = aimer.aim({target}, timestamp, 22, current_pose, false);
  require(command.control && aimer.debug_ballistics.valid, "Aimer failed for a static target");
  require((impact(aimer.debug_ballistics) - point).norm() < 1e-6,
    "Aimer used a different origin from its projectile solution");
  require((aimer.barrel_position(current_pose) - current_pose * aimer.barrel_translation()).norm() < 1e-12,
    "current barrel position was not rotated into world");

  // 用连续观测建立横向速度，再独立外推到 Aimer 报告的命中时刻验证拦截。
  for (int i = 1; i <= 20; ++i) {
    target.predict(timestamp + std::chrono::milliseconds(i * 10));
    armor = observation(point + Eigen::Vector3d(0, i * 0.02, 0));
    target.update(armor);
  }
  require(target.ekf_x()[3] > 0.1, "test target did not acquire lateral velocity");
  const auto moving_time = timestamp + std::chrono::milliseconds(200);
  command = aimer.aim({target}, moving_time, 22, current_pose, false);
  require(command.control && aimer.debug_ballistics.valid, "moving target solution failed");
  auto expected = target;
  expected.predict(moving_time + std::chrono::microseconds(
    static_cast<int>((0.005 + aimer.debug_ballistics.fly_time) * 1e6)));
  const auto future_armors = expected.armor_xyza_list();
  double miss = std::numeric_limits<double>::infinity();
  for (const auto & board : future_armors) {
    miss = std::min(miss, (impact(aimer.debug_ballistics) - board.head<3>()).norm());
  }
  require(miss < 1e-3, "projectile did not intercept the moving target");

  auto_aim::Shooter shooter(config.path.string());
  const double rad2deg = 180.0 / std::acos(-1.0);
  io::Command fire = command;
  fire.yaw *= rad2deg;
  fire.pitch *= -rad2deg;
  const Eigen::Vector3d aligned(command.yaw, command.pitch, 0.4);
  auto shoot = [&](Eigen::Vector3d pose) {
    return shooter.shoot(fire, aimer, {target}, pose);
  };
  require(!shoot(aligned + Eigen::Vector3d(0, 0.2, 0)), "pitch misalignment allowed initial fire");
  require(!shoot(aligned), "fire skipped the consecutive-frame requirement");
  require(shoot(aligned), "aligned barrel did not become ready");
  require(!shoot(aligned + Eigen::Vector3d(0, 0.2, 0)), "pitch misalignment allowed fire");
  require(!shoot(aligned), "pitch failure did not reset readiness");
  require(shoot(aligned), "readiness failed to recover");
  require(!shooter.shoot(fire, aimer, {}, aligned), "lost target allowed fire");
  require(!shoot(aligned + Eigen::Vector3d(0, 0.2, 0)), "pitch misalignment allowed reacquired fire");
  require(!shoot(aligned), "lost target did not reset readiness");
  require(shoot(aligned + Eigen::Vector3d(2 * std::acos(-1.0), 0, 0)),
    "yaw wraparound prevented an aligned shot");

  config.yaml["use_gimbal_feedback"] = false;
  config.save();
  auto_aim::Shooter no_feedback(config.path.string());
  const io::Command zero_delta{true, false, 0, 0};
  const io::Command large_delta{true, false, 10, 10};
  require(!no_feedback.shoot(fire, aimer, {target}, Eigen::Vector3d::Zero(), large_delta),
    "large angle error allowed no-feedback fire");
  require(!no_feedback.shoot(fire, aimer, {target}, Eigen::Vector3d::Zero(), zero_delta),
    "no-feedback fire skipped the frame requirement");
  require(no_feedback.shoot(fire, aimer, {target}, Eigen::Vector3d::Zero(), zero_delta),
    "world angle was confused with delta angle");

  aimer.aim({}, moving_time, 22, current_pose, false);
  require(!aimer.debug_aim_point.valid && !aimer.debug_ballistics.valid,
    "lost target retained a stale ballistic solution");
  config.yaml["t_barrel2gimbal"] = std::vector<double>{1, 2};
  config.save();
  bool rejected = false;
  try { auto_aim::Aimer invalid(config.path.string()); }
  catch (const std::exception &) { rejected = true; }
  require(rejected, "malformed translation was accepted");
  config.yaml.remove("t_barrel2gimbal");
  config.save();
  auto_aim::Aimer legacy_config(config.path.string());
  require(legacy_config.barrel_translation().isZero(0), "missing translation did not default to zero");
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    test_geometry();
    test_aimer_and_fire(argc > 1 ? argv[1] : "configs/demo.yaml");
    std::cout << "barrel geometry, interception and fire-control checks passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
