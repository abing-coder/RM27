#include <fmt/core.h>

#include <algorithm>
#include <chrono>
#include <optional>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "io/dm_imu/dm_imu.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/multithread/commandgener.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/curve_plot.hpp"
#include "tools/trajectory.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{d display      |      | 显示自瞄可视化窗口 }"
  "{@config-path   | configs/standard3.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  bool show_display = cli.has("display");
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;
  // rviz2 可视化桥：通过 UDP JSON 发给 C++ 桥接节点 build/rviz_bridge（src/rviz_bridge.cpp），
  // 由其转为 MarkerArray/TF 发布；主程序本身不链接 rclcpp，无 ROS 环境也可正常运行
  tools::Plotter rviz_plotter("127.0.0.1", 9871);

  auto config_yaml = YAML::LoadFile(config_path);
  std::string aim_mode =
    config_yaml["aim_mode"] ? config_yaml["aim_mode"].as<std::string>() : "absolute";
  // 枪口偏移：以“度”为单位读入
  double gun_yaw_offset_deg =
    config_yaml["gun_yaw_offset"] ? config_yaml["gun_yaw_offset"].as<double>() : 0.0;
  double gun_pitch_offset_deg =
    config_yaml["gun_pitch_offset"] ? config_yaml["gun_pitch_offset"].as<double>() : 0.0;

  io::CBoard cboard(config_path);
  io::Camera camera(config_path);

  // 姿态只来自 USB 上的 DM-IMU，不使用电控回传的四元数。
  const auto imu_port = config_yaml["dm_imu_port"]
                          ? config_yaml["dm_imu_port"].as<std::string>()
                          : std::string("/dev/ttyACM0");
  io::DM_IMU dm_imu(imu_port);
  if (aim_mode == "delta") {
    tools::logger()->warn(
      "[standard] 姿态已使用 USB IMU 四元数，但 aim_mode=delta 时下发 yaw 仍是像素差值。"
      "要用这路姿态做世界系绝对角，请把 aim_mode 改为 absolute。");
  }

  // 开显示时同时开 YOLO 的 detection 窗口，便于对比是否有检测结果
  auto_aim::YOLO detector(config_path, show_display);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  const bool use_delta_aim = (aim_mode == "delta");  // 无 IMU：yaw 发差值，pitch 可复用弹道

  // rviz2 里的相机与虚拟枪口，都是相对 gimbal 的固定变换
  const auto R_camera2gimbal_data = config_yaml["R_camera2gimbal"].as<std::vector<double>>();
  const auto t_camera2gimbal_data = config_yaml["t_camera2gimbal"].as<std::vector<double>>();
  const Eigen::Quaterniond q_camera2gimbal =
    Eigen::Quaterniond(Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_camera2gimbal_data.data()))
      .normalized();
  const Eigen::Vector3d t_camera2gimbal(t_camera2gimbal_data.data());

  // 虚拟枪口：云台执行下发角后指向瞄准点的方向。云台比理想方向多转了补偿角，所以枪口相对 gimbal 转了 -补偿。
  // 下发 yaw 右转为正、减去 gun_yaw_offset，换成左转为正就是 +gun_yaw_offset；
  // delta 模式的 yaw 来自像素差，不叠加 Aimer 的 yaw_offset。
  constexpr double deg2rad = 3.141592653589793 / 180.0;
  const double yaw_comp_deg =
    (use_delta_aim ? 0.0 : config_yaml["yaw_offset"].as<double>()) + gun_yaw_offset_deg;
  const double pitch_comp_deg = config_yaml["pitch_offset"].as<double>() - gun_pitch_offset_deg;
  // 绕 +Z 为左转；抬头为正的 pitch 对应绕 +Y 的负转角
  const Eigen::Quaterniond q_barrel2gimbal(
    Eigen::AngleAxisd(-yaw_comp_deg * deg2rad, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(pitch_comp_deg * deg2rad, Eigen::Vector3d::UnitY()));

  // 调试窗口的一步预测曲线：上一帧跟踪结果外推到本帧时刻（红） vs 本帧检测（绿），世界系角度
  tools::CurvePlot yaw_plot("yaw", "deg");
  tools::CurvePlot pitch_plot("pitch", "deg");
  std::optional<auto_aim::Target> prev_target;
  const auto t_start = std::chrono::steady_clock::now();
  auto last_loop_time = t_start;
  double fps = 0;
  if (show_display) cv::namedWindow("auto_aim", cv::WINDOW_NORMAL);

  // 预测目标里离检测最近的那块装甲板；没有检测时用上一帧跟踪的那块
  auto nearest_armor_xyz = [](
                             const auto_aim::Target & target,
                             const std::optional<Eigen::Vector3d> & det_xyz) -> Eigen::Vector3d {
    const auto xyza_list = target.armor_xyza_list();
    const int last = std::clamp(target.last_id, 0, static_cast<int>(xyza_list.size()) - 1);
    Eigen::Vector3d best = xyza_list[last].head<3>();
    if (!det_xyz) return best;
    for (const auto & xyza : xyza_list) {
      if ((xyza.head<3>() - *det_xyz).norm() < (best - *det_xyz).norm()) best = xyza.head<3>();
    }
    return best;
  };

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  auto mode = io::Mode::idle;
  auto last_mode = io::Mode::idle;
  int frame_count = 0;

  while (!exiter.exit()) {
    frame_count++;
    camera.read(img, t);
    q = dm_imu.imu_at(t - 1ms);
    mode = cboard.mode;

    if (last_mode != mode) {
      tools::logger()->info("Switch to {}", io::MODES[mode]);
      last_mode = mode;
    }

    // recorder.record(img, q, t);

    solver.set_R_gimbal2world(q);

    Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    auto armors = detector.detect(img, frame_count);
    // 有显示时用与 detection 窗口同一份检测结果做跟踪，保证绿框与 Tracker/Estimator 数据一致
    if (show_display) {
      auto drawn = detector.get_last_drawn_armors();
      if (!drawn.empty()) armors = drawn;
    }

    size_t armors_raw = armors.size();
    auto targets = tracker.track(armors, t);
    size_t armors_after_color = armors.size();

    auto command = aimer.aim(targets, t, cboard.bullet_speed);
    const io::Command aim_world = command;  // Aimer 原始输出（世界系，弧度），调试显示用
    io::Command fire_cmd = command;  // 专门给 Shooter 用的指令（可单独修改 yaw 基准）
    // 统一约定：io::Command 中 yaw/pitch 以“度”为单位
    // delta 模式：yaw 使用预测瞄准点的差值，pitch 使用 detector 裸测量点的弹道预测结果
    if (use_delta_aim) {
      if (command.control && !targets.empty() && aimer.debug_aim_point.valid) {
        const auto & target = targets.front();
        const auto & aim_point = aimer.debug_aim_point;
        auto pts = solver.reproject_armor(
          aim_point.xyza.head(3), aim_point.xyza[3], target.armor_type, target.name);

        bool has_predicted_yaw = false;
        bool has_detect_yaw = false;
        double delta_yaw_pred = 0.0;
        double delta_yaw_det = 0.0;
        std::optional<double> measured_pitch;
        std::optional<auto_aim::Armor> measured_armor;

        auto matched_armor = std::find_if(
          armors.begin(), armors.end(), [&](const auto_aim::Armor & armor) {
            return armor.name == target.name && armor.type == target.armor_type;
          });
        if (matched_armor != armors.end()) {
          measured_armor = *matched_armor;
        } else if (!armors.empty()) {
          measured_armor = armors.front();
        }

        // 1) yaw：使用预测瞄准点（红框）的像素中心
        if (pts.size() >= 4) {
          cv::Point2f predicted_center(
            (pts[0].x + pts[1].x + pts[2].x + pts[3].x) / 4.f,
            (pts[0].y + pts[1].y + pts[2].y + pts[3].y) / 4.f);
          Eigen::Vector2d delta_pred = solver.pixel_to_delta_yaw_pitch(predicted_center);
          delta_yaw_pred = delta_pred[0];
          has_predicted_yaw = true;
        }

        // 2) detector 裸测量点：用于无 IMU 的火控判定和弹道 pitch
        if (measured_armor.has_value()) {
          auto det_armor = measured_armor.value();
          cv::Point2f det_center;
          if (det_armor.points.size() >= 4) {
            det_center = cv::Point2f(
              (det_armor.points[0].x + det_armor.points[1].x + det_armor.points[2].x +
               det_armor.points[3].x) /
                4.f,
              (det_armor.points[0].y + det_armor.points[1].y + det_armor.points[2].y +
               det_armor.points[3].y) /
                4.f);
          } else {
            det_center = det_armor.center;
          }
          Eigen::Vector2d delta_det = solver.pixel_to_delta_yaw_pitch(det_center);
          // yaw 用于 fire_cmd；pitch 用 detector 裸测量点做一次弹道解算
          delta_yaw_det = delta_det[0];
          has_detect_yaw = true;
          solver.solve(det_armor);
          // 与原版 Aimer 相同的弹道公式。原版 pitch 抬头为负，这里取反后抬头为正，和下发约定一致。
          double speed = cboard.bullet_speed < 14 ? 23 : cboard.bullet_speed;
          const auto & xyz = det_armor.xyz_in_world;
          double d = std::sqrt(xyz.x() * xyz.x() + xyz.y() * xyz.y());
          tools::Trajectory traj(speed, d, xyz.z());
          if (!traj.unsolvable) {
            const double pitch_offset = config_yaml["pitch_offset"].as<double>() / 57.3;
            // 弹道角抬头为正，与下发约定一致；原版 Aimer 再取一次负号后抬头为负。
            measured_pitch = traj.pitch + pitch_offset;
          }
        }

        // 无 IMU 的 delta 模式下，仍要求当前帧存在检测结果，避免纯 tracker 预测导致 yaw 抽动
        if (has_predicted_yaw && has_detect_yaw && measured_pitch.has_value()) {
          command.control = true;
          // 发送角度（度）：差值(弧度)转度，再减枪口偏移(已是度)；下位机接收的就是“度”
          const double rad2deg = 180.0 / 3.141592653589793;
          command.yaw = ((delta_yaw_pred * rad2deg) - gun_yaw_offset_deg);
          command.pitch = (measured_pitch.value() * rad2deg) - gun_pitch_offset_deg;
          // 火控判断：yaw 也以检测框为基准
          if (has_detect_yaw) {
            fire_cmd.yaw = ((delta_yaw_det * rad2deg) - gun_yaw_offset_deg);
          } else {
            fire_cmd.yaw = command.yaw;
          }
          fire_cmd.pitch = command.pitch;
        } else {
          command.control = false;
          command.shoot = false;
          command.yaw = 0;
          command.pitch = 0;
        }
      } else {
        command.control = false;
        command.shoot = false;
        command.yaw = 0;
        command.pitch = 0;
      }
    } else {
      // 原版 Aimer：yaw 左转为正，pitch 抬头为负。电控约定 yaw 右转为正、pitch 抬头为正。
      const double rad2deg = 180.0 / 3.141592653589793;
      command.pitch = -command.pitch;
      const double gimbal_pitch_up = -ypr[1];  // 绕 +Y 正转是低头
      // Shooter 用 IMU 云台 yaw 判断是否跟上，这里给它云台应到达的世界系角度
      fire_cmd.yaw = aim_world.yaw * rad2deg + gun_yaw_offset_deg;
      fire_cmd.pitch = command.pitch * rad2deg - gun_pitch_offset_deg;
      if (command.control) {
        command.yaw = -tools::limit_rad(command.yaw - ypr[0]) * rad2deg - gun_yaw_offset_deg;
        command.pitch = (command.pitch - gimbal_pitch_up) * rad2deg - gun_pitch_offset_deg;
      } else {
        command.yaw = 0;
        command.pitch = 0;
      }
    }
    // 火控 shoot：control=1 且存在目标时，由 Shooter 判定是否开火
    if (command.control && !targets.empty()) {
      command.shoot = shooter.shoot(fire_cmd, aimer, targets, ypr);
    } else {
      shooter.reset_debug_state();
      command.shoot = false;
    }
    cboard.send(command);

    // 发布整车估计与火控状态给 rviz2 桥接节点（无人监听时 UDP 直接丢弃，开销可忽略）
    {
      nlohmann::json rviz_json;
      rviz_json["targets"] = nlohmann::json::array();
      for (const auto & target : targets) {
        Eigen::VectorXd x = target.ekf_x();
        if (x.size() < 11) continue;
        nlohmann::json tj;
        tj["name"] = auto_aim::ARMOR_NAMES[target.name];
        tj["armor_type"] = auto_aim::ARMOR_TYPES[target.armor_type];
        tj["center"] = {x[0], x[2], x[4]};
        tj["radius"] = x[8];
        auto armors_json = nlohmann::json::array();
        for (const Eigen::Vector4d & xyza : target.armor_xyza_list())
          armors_json.push_back({xyza[0], xyza[1], xyza[2], xyza[3]});
        tj["armors"] = armors_json;
        rviz_json["targets"].push_back(tj);
      }

      // 火控选中的瞄准点：control 有效时标红，shoot 开火时标橙
      const auto & aim_point = aimer.debug_aim_point;
      if (!targets.empty() && aim_point.valid) {
        const auto & target = targets.front();
        rviz_json["aim"] = {
          {"xyza", {aim_point.xyza[0], aim_point.xyza[1], aim_point.xyza[2], aim_point.xyza[3]}},
          {"armor_type", auto_aim::ARMOR_TYPES[target.armor_type]},
          {"control", command.control},
          {"shoot", command.shoot}};
      }

      // 自身位姿与解算器使用同一旋转：IMU 四元数经 R_gimbal2imubody 得到云台姿态。
      Eigen::Quaterniond q_gimbal(solver.R_gimbal2world());
      rviz_json["self"] = {
        {"q", {q_gimbal.x(), q_gimbal.y(), q_gimbal.z(), q_gimbal.w()}},
        {"ypr", {ypr[0], ypr[1], ypr[2]}}};
      rviz_json["camera"] = {
        {"t", {t_camera2gimbal.x(), t_camera2gimbal.y(), t_camera2gimbal.z()}},
        {"q", {q_camera2gimbal.x(), q_camera2gimbal.y(), q_camera2gimbal.z(), q_camera2gimbal.w()}}};
      rviz_json["barrel"] = {
        {"q", {q_barrel2gimbal.x(), q_barrel2gimbal.y(), q_barrel2gimbal.z(), q_barrel2gimbal.w()}}};

      rviz_plotter.plot(rviz_json);
    }

    if (show_display) {
      const auto & draw_armors = detector.get_last_drawn_armors();
      cv::Mat vis = img.clone();
      const double rad2deg = 180.0 / 3.141592653589793;

      const auto now = std::chrono::steady_clock::now();
      const double loop_dt = tools::delta_time(now, last_loop_time);
      last_loop_time = now;
      if (loop_dt > 0) fps = fps > 0 ? 0.9 * fps + 0.1 / loop_dt : 1.0 / loop_dt;

      // 一步预测：上一帧的跟踪结果外推到本帧时刻，对比本帧检测解算出的同一块装甲板
      std::optional<Eigen::Vector3d> det_ypd, pred_ypd;
      if (!targets.empty()) {
        const auto & target = targets.front();
        std::optional<Eigen::Vector3d> det_xyz;
        for (auto armor : armors) {
          if (armor.name != target.name || armor.type != target.armor_type) continue;
          solver.solve(armor);
          det_xyz = armor.xyz_in_world;
          det_ypd = armor.ypd_in_world;
          break;
        }
        if (prev_target && prev_target->name == target.name) {
          auto predicted = *prev_target;
          predicted.predict(t);
          pred_ypd = tools::xyz2ypd(nearest_armor_xyz(predicted, det_xyz));
        }
      }
      prev_target = targets.empty() ? std::nullopt : std::make_optional(targets.front());

      auto angle_deg = [&](const std::optional<Eigen::Vector3d> & ypd, int i) -> std::optional<double> {
        if (!ypd) return std::nullopt;
        return (*ypd)[i] * rad2deg;
      };
      const double t_plot = tools::delta_time(t, t_start);
      yaw_plot.push(t_plot, angle_deg(pred_ypd, 0), angle_deg(det_ypd, 0));
      pitch_plot.push(t_plot, angle_deg(pred_ypd, 1), angle_deg(det_ypd, 1));

      // ----- 画面下方的调试栏：左侧文字，右侧 yaw / pitch 曲线 -----
      constexpr int strip_h = 420;
      constexpr int text_w = 600;
      cv::Mat strip(strip_h, vis.cols, vis.type(), cv::Scalar(15, 15, 15));
      int text_y = 34;
      auto line = [&](const std::string & text, const cv::Scalar & color, double scale = 0.65) {
        tools::draw_text(strip, text, {12, text_y}, color, scale, scale > 0.7 ? 2 : 1);
        text_y += static_cast<int>(46 * scale);
      };
      auto opt_deg = [](const std::optional<double> & v) {
        return v ? fmt::format("{:+.2f}", *v) : std::string("--");
      };
      const cv::Scalar white{255, 255, 255}, gray{150, 150, 150}, yellow{0, 255, 255},
        green{0, 255, 0}, orange{0, 165, 255};

      line(
        fmt::format("frame {}  fps {:.0f}  {}  {}", frame_count, fps, io::MODES[mode], aim_mode),
        yellow);

      // 下发给电控的差值（度）：yaw 右转为正，pitch 抬头为正
      const cv::Scalar send_color = command.shoot ? orange : (command.control ? green : gray);
      line(fmt::format("SEND d_yaw {:+.2f}  d_pitch {:+.2f}", command.yaw, command.pitch), send_color, 0.85);
      line(fmt::format("     control {}  shoot {}", command.control ? 1 : 0, command.shoot ? 1 : 0), send_color, 0.85);

      // IMU 云台姿态（世界系，pitch 抬头为正）
      line(
        fmt::format(
          "IMU   yaw {:+.2f}  pitch {:+.2f}  roll {:+.2f}", ypr[0] * rad2deg, -ypr[1] * rad2deg,
          ypr[2] * rad2deg),
        white);

      line(
        fmt::format(
          "Track [{}]  armors {}/{}  targets {}", tracker.state(), armors_raw, armors_after_color,
          targets.size()),
        white);
      if (!targets.empty()) {
        const auto & target = targets.front();
        const Eigen::VectorXd x = target.ekf_x();
        line(
          fmt::format(
            "Target {} {}  id {}  dist {:.2f} m", auto_aim::ARMOR_NAMES[target.name],
            auto_aim::ARMOR_TYPES[target.armor_type], target.last_id, std::hypot(x[0], x[2], x[4])),
          white);
        line(fmt::format("  w {:+.2f} rad/s  r {:.2f}", x[7], x[8]), white);
      } else {
        line("Target --", gray);
      }

      if (aim_world.control) {
        line(
          fmt::format(
            "Aim   yaw {:+.2f}  pitch {:+.2f} (world)", aim_world.yaw * rad2deg,
            aim_world.pitch * rad2deg),
          white);
        const bool default_speed = cboard.bullet_speed < 14;
        line(
          fmt::format("  v {:.1f}{}", default_speed ? 23.0 : cboard.bullet_speed, default_speed ? " default" : ""),
          default_speed ? orange : white);
      }

      line(
        fmt::format("Fire  streak {}", shooter.debug_fire_ready_streak),
        command.shoot ? green : white);

      if (pred_ypd && det_ypd) {
        line(
          fmt::format(
            "Pred  err yaw {:+.2f}  pitch {:+.2f}",
            tools::limit_rad((*pred_ypd)[0] - (*det_ypd)[0]) * rad2deg,
            ((*pred_ypd)[1] - (*det_ypd)[1]) * rad2deg),
          white);
      }

      // 有绿框但跟踪器不触发：enemy_color 过滤掉了所有检测
      if (armors_raw > 0 && armors_after_color == 0 && !draw_armors.empty()) {
        const auto & first = draw_armors.front();
        const std::string det_color = first.color <= auto_aim::Color::purple
                                        ? auto_aim::COLORS[static_cast<size_t>(first.color)]
                                        : "?";
        line(fmt::format(">> set enemy_color to \"{}\"", det_color), orange);
      } else if (draw_armors.empty()) {
        line("no det: check enemy_color / min_confidence", orange);
      }

      const int plot_w = vis.cols - text_w;
      yaw_plot.draw(strip, {text_w, 0, plot_w, strip_h / 2});
      pitch_plot.draw(strip, {text_w, strip_h / 2, plot_w, strip_h / 2});

      // 检测框：用与 detection 窗口同一份 armors 画，保证两边一致
      int vis_w = vis.cols, vis_h = vis.rows;
      for (const auto & armor : draw_armors) {
        if (armor.points.size() >= 4) {
          std::vector<cv::Point> pts;
          for (const auto & p : armor.points) {
            int px = std::clamp(cvRound(p.x), 0, vis_w - 1);
            int py = std::clamp(cvRound(p.y), 0, vis_h - 1);
            pts.push_back(cv::Point(px, py));
          }
          if (pts.size() >= 4) {
            std::vector<std::vector<cv::Point>> contours = {pts};
            cv::polylines(vis, contours, true, cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
          }
        }
      }
      // 四个装甲板（绿点，与 auto_aim_test 一致）、瞄准点（红点）
      if (!targets.empty()) {
        const auto & target = targets.front();
        Eigen::VectorXd x = target.ekf_x();

        if (x.size() >= 11) {
          auto draw_orbit = [&](double radius, double z_world, const cv::Scalar & color) {
            if (radius <= 1e-3) return;

            std::vector<cv::Point> segment;
            for (int i = 0; i <= 72; ++i) {
              double theta = (2.0 * CV_PI * i) / 72.0;
              cv::Point3f world_point(
                static_cast<float>(x[0] + radius * std::cos(theta)),
                static_cast<float>(x[2] + radius * std::sin(theta)),
                static_cast<float>(z_world));
              auto pixels = solver.world2pixel({world_point});
              if (!pixels.empty()) {
                cv::Point pixel(cvRound(pixels[0].x), cvRound(pixels[0].y));
                if (
                  pixel.x >= 0 && pixel.x < vis.cols && pixel.y >= 0 && pixel.y < vis.rows)
                {
                  segment.push_back(pixel);
                  continue;
                }
              }

              if (segment.size() >= 2) {
                std::vector<std::vector<cv::Point>> contours = {segment};
                cv::polylines(vis, contours, false, color, 1, cv::LINE_AA);
              }
              segment.clear();
            }

            if (segment.size() >= 2) {
              std::vector<std::vector<cv::Point>> contours = {segment};
              cv::polylines(vis, contours, false, color, 1, cv::LINE_AA);
            }
          };

          draw_orbit(x[8], x[4], cv::Scalar(0, 255, 255));
          if (target.armor_xyza_list().size() == 4 && (std::abs(x[9]) > 1e-3 || std::abs(x[10]) > 1e-3)) {
            draw_orbit(x[8] + x[9], x[4] + x[10], cv::Scalar(0, 165, 255));
          }
        }

        for (const Eigen::Vector4d & xyza : target.armor_xyza_list()) {
          auto image_points =
            solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
          tools::draw_points(vis, image_points, {0, 255, 0});
        }
        auto aim_point = aimer.debug_aim_point;
        if (aim_point.valid) {
          auto image_points = solver.reproject_armor(
            aim_point.xyza.head(3), aim_point.xyza[3], target.armor_type, target.name);
          const bool fire_ready = command.shoot;
          const cv::Scalar fire_color = fire_ready ? cv::Scalar(0, 165, 255) : cv::Scalar(0, 0, 255);
          tools::draw_points(vis, image_points, fire_color, fire_ready ? 4 : 2);
        }

        if (x.size() >= 5) {
          std::vector<cv::Point3f> center_world = {
            cv::Point3f(static_cast<float>(x[0]), static_cast<float>(x[2]), static_cast<float>(x[4]))};
          auto center_pixels = solver.world2pixel(center_world);
          if (!center_pixels.empty()) {
            cv::Point center_point(cvRound(center_pixels[0].x), cvRound(center_pixels[0].y));
            if (
              center_point.x >= 0 && center_point.x < vis.cols && center_point.y >= 0 &&
              center_point.y < vis.rows)
            {
              cv::circle(vis, center_point, 8, cv::Scalar(255, 255, 0), 5, cv::LINE_AA);
              tools::draw_point(vis, center_point, {255, 255, 0}, 5);
              // tools::draw_text(vis, "center", center_point + cv::Point(10, -10), {255, 255, 0}, 0.55, 2);
            }
          }
        }
      }
      cv::Mat canvas;
      cv::vconcat(vis, strip, canvas);
      if (frame_count == 1) cv::resizeWindow("auto_aim", 900, 900 * canvas.rows / canvas.cols);
      cv::imshow("auto_aim", canvas);
      if (cv::waitKey(1) == 'q') break;
    }
  }

  return 0;
}

/*
    make -C build -j$(nproc)
    ./build/standard configs/demo.yaml -d
*/
