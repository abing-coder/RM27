#include <fmt/core.h>

#include <algorithm>
#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

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

  io::CBoard cboard(config_path);
  io::Camera camera(config_path);

  // 开显示时同时开 YOLO 的 detection 窗口，便于对比是否有检测结果
  auto_aim::YOLO detector(config_path, show_display);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

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
    // 火控触发条件：
    //   control=1：Aimer::aim 成功 → 有 target、瞄准点 valid、弹道可解，否则 0。
    //   shoot=1：在 control=1 基础上，Shooter::shoot 为 true → config auto_fire: true，
    //            且指令变化小、云台跟上（|cmd.yaw - gimbal.yaw| < tolerance），否则 0。
    if (command.control && !targets.empty()) {
      command.shoot = shooter.shoot(command, aimer, targets, ypr);
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
      tools::draw_text(vis, fmt::format("  cmd yaw:{:.2f} pitch:{:.2f}", command.yaw * 57.3, command.pitch * 57.3), {10, y0}, targets.empty() ? cv::Scalar(150, 150, 150) : cv::Scalar(255, 255, 255), 0.6);
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
