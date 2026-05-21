#ifndef AUTO_AIM__AIMER_HPP
#define AUTO_AIM__AIMER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>
#include <optional>

#include "io/cboard.hpp"
#include "io/command.hpp"
#include "target.hpp"

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
  AimPoint debug_aim_point;
  AimPoint debug_raw_aim_point;
  std::optional<double> debug_raw_pitch;
  bool debug_small_gyro_detected = false;
  bool debug_small_gyro_active = false;
  int debug_small_gyro_armor_id = -1;
  double debug_total_predict_time = 0.0;
  double debug_fly_time = 0.0;  // 子弹飞行时间（秒），用于火控命中窗口判断
  explicit Aimer(const std::string & config_path);
  std::optional<double> solve_ballistic_pitch(
    const Eigen::Vector3d & xyz_in_world, double bullet_speed) const;
  io::Command aim(
    std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
    bool to_now = true);

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
  double small_gyro_speed_thresh_;

  std::optional<double> solve_ballistic_pitch(const AimPoint & aim_point, double bullet_speed) const;
  bool is_small_gyro(const Target & target) const;
  int choose_small_gyro_armor_id(const Target & target) const;
  AimPoint choose_small_gyro_aim_point(const Target & target);
  AimPoint choose_aim_point(const Target & target, bool update_lock = true);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__AIMER_HPP
