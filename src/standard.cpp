#include <fmt/core.h>

#include <algorithm>
#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "io/camera.hpp"
#include "io/cboard.hpp"
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

  // 开显示时同时开 YOLO 的 detection 窗口，便于对比是否有检测结果
  auto_aim::YOLO detector(config_path, show_display);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  const bool use_delta_aim = (aim_mode == "delta");  // 无 IMU/无弹道：仅发 yaw/pitch 差值

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  auto mode = io::Mode::idle;
  auto last_mode = io::Mode::idle;
  int frame_count = 0;

  while (!exiter.exit()) {
    frame_count++;
    camera.read(img, t);
    q = cboard.imu_at(t - 1ms);
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
    io::Command fire_cmd = command;  // 专门给 Shooter 用的指令（可单独修改 yaw 基准）
    // 统一约定：io::Command 中 yaw/pitch 以“度”为单位
    // delta 模式：yaw 使用预测瞄准点的差值，pitch 使用当前检测装甲板的差值
    if (use_delta_aim) {
      if (!targets.empty() && aimer.debug_aim_point.valid) {
        const auto & target = targets.front();
        const auto & aim_point = aimer.debug_aim_point;
        auto pts = solver.reproject_armor(
          aim_point.xyza.head(3), aim_point.xyza[3], target.armor_type, target.name);

        bool has_predicted_yaw = false;
        bool has_detect_pitch = false;
        bool has_detect_yaw = false;
        double delta_yaw_pred = 0.0;
        double delta_pitch_det = 0.0;
        double delta_yaw_det = 0.0;

        // 1) yaw：使用预测瞄准点（红框）的像素中心
        if (pts.size() >= 4) {
          cv::Point2f predicted_center(
            (pts[0].x + pts[1].x + pts[2].x + pts[3].x) / 4.f,
            (pts[0].y + pts[1].y + pts[2].y + pts[3].y) / 4.f);
          Eigen::Vector2d delta_pred = solver.pixel_to_delta_yaw_pitch(predicted_center);
          delta_yaw_pred = delta_pred[0];
          has_predicted_yaw = true;
        }

        // 2) pitch：使用当前检测装甲板（绿框）的像素中心
        if (!armors.empty()) {
          const auto & det_armor = armors.front();
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
          // 检测框：同时提供 yaw / pitch，用于：
          // - pitch：通信发送
          // - yaw：火控判断（fire_cmd）
          delta_yaw_det = delta_det[0];
          delta_pitch_det = delta_det[1];
          has_detect_pitch = true;
          has_detect_yaw = true;
        }

        if (has_predicted_yaw && has_detect_pitch) {
          command.control = true;
          // 发送角度（度）：差值(弧度)转度，再减枪口偏移(已是度)；下位机接收的就是“度”
          const double rad2deg = 180.0 / 3.141592653589793;
          command.yaw = ((delta_yaw_pred * rad2deg) - gun_yaw_offset_deg);
          command.pitch = ((delta_pitch_det * rad2deg) - gun_pitch_offset_deg);
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
      // 绝对模式：Aimer 输出为弧度，这里统一转为“度”再发送/用于火控
      const double rad2deg = 180.0 / 3.141592653589793;
      command.yaw = command.yaw * rad2deg - gun_yaw_offset_deg;
      command.pitch = command.pitch * rad2deg - gun_pitch_offset_deg;
      fire_cmd = command;
    }
    // 火控 shoot：control=1 且存在目标时，由 Shooter 判定是否开火
    if (command.control && !targets.empty()) {
      command.shoot = shooter.shoot(fire_cmd, aimer, targets, ypr);
    } else {
      command.shoot = false;
    }
    cboard.send(command);

    if (show_display) {
      const auto & draw_armors = detector.get_last_drawn_armors();
      cv::Mat vis = img.clone();

      int line_h = 28;
      int y0 = 30;
      const int panel_w = 480;
      const int panel_h = 580;
      // 半透明黑色背景，保证 Tracker/Estimator 文字在任何画面下都清晰可见
      cv::Mat vis_roi = vis(cv::Rect(0, 0, panel_w, panel_h));
      cv::Mat panel(panel_h, panel_w, vis.type(), cv::Scalar(0, 0, 0));
      cv::Mat blended;
      cv::addWeighted(panel, 0.5, vis_roi, 0.5, 0, blended);
      blended.copyTo(vis_roi);

      // 帧号（用于确认画面在更新）
      tools::draw_text(vis, fmt::format("frame: {}", frame_count), {10, y0}, {0, 255, 255}, 0.7);
      y0 += line_h;
      // ----- 跟踪器 -----
      tools::draw_text(vis, "Tracker:", {10, y0}, {0, 255, 255}, 0.7);
      y0 += line_h;
      tools::draw_text(vis, fmt::format("  state: [{}]", tracker.state()), {10, y0}, {255, 255, 255}, 0.6);
      y0 += line_h;
      tools::draw_text(vis, fmt::format("  armors: {} (enemy: {})  targets: {}", armors_raw, armors_after_color, targets.size()), {10, y0}, {255, 255, 255}, 0.6);
      y0 += line_h;
      // 有绿框但跟踪器不触发：enemy_color 过滤掉了所有检测，给出可操作提示
      if (armors_raw > 0 && armors_after_color == 0 && !draw_armors.empty()) {
        const auto & first = draw_armors.front();
        std::string det_color = (first.color <= auto_aim::Color::purple) ? auto_aim::COLORS[static_cast<size_t>(first.color)] : "?";
        tools::draw_text(vis, fmt::format("  >> config enemy_color to \"{}\" (now filtered to 0)", det_color), {10, y0}, {0, 200, 255}, 0.55);
        y0 += line_h;
      }
      // ----- 整车估计器 (EKF) -----
      tools::draw_text(vis, "Estimator:", {10, y0}, {0, 255, 255}, 0.7);
      y0 += line_h;
      if (!targets.empty()) {
        const auto & target = targets.front();
        tools::draw_text(vis, fmt::format("  armor_id: {}", target.last_id), {10, y0}, {255, 255, 255}, 0.6);
        y0 += line_h;
        Eigen::VectorXd x = target.ekf_x();
        if (x.size() >= 11) {
          tools::draw_text(vis, fmt::format("  xyz: ({:.2f},{:.2f},{:.2f})", x[0], x[2], x[4]), {10, y0}, {200, 200, 255}, 0.6);
          y0 += line_h;
          tools::draw_text(vis, fmt::format("  vxy: ({:.2f},{:.2f}) w:{:.2f}", x[1], x[3], x[7]), {10, y0}, {200, 200, 255}, 0.6);
          y0 += line_h;
          tools::draw_text(vis, fmt::format("  r:{:.2f} a:{:.1f}deg", x[8], x[6] * 57.3), {10, y0}, {200, 200, 255}, 0.6);
          y0 += line_h;
        }
      } else {
        tools::draw_text(vis, "  (no target)", {10, y0}, {150, 150, 150}, 0.6);
        y0 += line_h;
      }
      // ----- 火控：control=aim 成功；shoot=control 且 Shooter 判定可开火（见下方说明） -----
      tools::draw_text(vis, "Fire control:", {10, y0}, {0, 255, 255}, 0.7);
      y0 += line_h;
      tools::draw_text(vis, fmt::format("  bullet_speed: {:.1f}  mode: {}", cboard.bullet_speed, io::MODES[mode]), {10, y0}, {255, 255, 255}, 0.6);
      y0 += line_h;
      tools::draw_text(vis, fmt::format("  control:{} shoot:{}", command.control ? 1 : 0, command.shoot ? 1 : 0), {10, y0}, {255, 255, 255}, 0.6);
      y0 += line_h;
      tools::draw_text(
        vis, fmt::format("  cmd yaw:{:.2f}  pitch:{:.2f}", command.yaw, command.pitch),
        {10, y0},
        targets.empty() ? cv::Scalar(150, 150, 150) : cv::Scalar(255, 255, 255), 0.6);
      if (targets.empty() && armors_raw > 0) {
        tools::draw_text(vis, "  (no target -> fire control idle)", {10, y0 + line_h}, {0, 200, 255}, 0.55);
        y0 += line_h;
      }
      y0 += line_h;
      if (draw_armors.empty()) {
        tools::draw_text(
          vis, "no det: check enemy_color/min_confidence or aim at armor", {10, y0 + line_h},
          {0, 165, 255}, 0.55);
      }

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
        for (const Eigen::Vector4d & xyza : target.armor_xyza_list()) {
          auto image_points =
            solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
          tools::draw_points(vis, image_points, {0, 255, 0});
        }
        auto aim_point = aimer.debug_aim_point;
        if (aim_point.valid) {
          auto image_points = solver.reproject_armor(
            aim_point.xyza.head(3), aim_point.xyza[3], target.armor_type, target.name);
          tools::draw_points(vis, image_points, {0, 0, 255});
        }
      }
      cv::namedWindow("auto_aim", cv::WINDOW_NORMAL);
      cv::resizeWindow("auto_aim", 800, 600);
      cv::imshow("auto_aim", vis);
      if (cv::waitKey(1) == 'q') break;
    }
  }

  return 0;
}

/*
    make -C build -j$(nproc)
    ./build/standard configs/demo.yaml -d
*/
