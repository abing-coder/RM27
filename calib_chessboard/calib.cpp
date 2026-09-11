#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>  // 必须在 opencv2/core/eigen.hpp 上面
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <optional>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;

const std::string keys =
  "{help h usage ? |                                | 输出命令行参数说明}"
  "{@config-path   | calib_chessboard/config.yaml   | yaml 配置文件路径 }";

// ---------------------------------------------------------------------------
struct Config
{
  std::string path;

  bool do_capture = true;
  bool do_intrinsics = true;
  bool do_handeye = true;
  bool use_imu = false;

  int cols = 8;
  int rows = 5;
  float square_mm = 25.f;
  std::string detector = "sb";

  std::string data_dir = "calib_chessboard/data";
  std::string output_yaml = "calib_chessboard/output/result.yaml";
  std::string export_to;

  int min_frames = 15;
  double preview_scale = 0.5;
  bool require_board = true;
  bool auto_save = false;
  double auto_save_interval_s = 1.5;
  double auto_min_corner_shift_px = 40.0;
  bool show_each = true;

  bool fix_k3 = true;
  std::string handeye_method = "tsai";

  std::string camera_name = "opencv";
  int opencv_device = 0;
  int opencv_width = 1280;
  int opencv_height = 720;

  std::vector<double> R_gimbal2imubody = {1, 0, 0, 0, 1, 0, 0, 0, 1};
};

template <typename T>
T yaml_get(const YAML::Node & n, const char * key, T fallback)
{
  return n && n[key] ? n[key].as<T>() : fallback;
}

Config load_config(const std::string & path)
{
  auto yaml = YAML::LoadFile(path);
  Config c;
  c.path = path;

  auto steps = yaml["steps"];
  c.do_capture = yaml_get(steps, "capture", true);
  c.do_intrinsics = yaml_get(steps, "intrinsics", true);
  c.do_handeye = yaml_get(steps, "handeye", true);
  c.use_imu = yaml_get(yaml, "use_imu", false);

  auto board = yaml["chessboard"];
  c.cols = yaml_get(board, "inner_corners_cols", 8);
  c.rows = yaml_get(board, "inner_corners_rows", 5);
  c.square_mm = yaml_get(board, "square_size_mm", 25.0);
  c.detector = yaml_get(board, "detector", std::string("sb"));

  c.data_dir = yaml_get(yaml, "data_dir", std::string("calib_chessboard/data"));
  c.output_yaml = yaml_get(yaml, "output_yaml", std::string("calib_chessboard/output/result.yaml"));
  c.export_to = yaml_get(yaml, "export_to", std::string(""));

  auto cap = yaml["capture"];
  c.min_frames = yaml_get(cap, "min_frames", 15);
  c.preview_scale = yaml_get(cap, "preview_scale", 0.5);
  c.require_board = yaml_get(cap, "require_board", true);
  c.auto_save = yaml_get(cap, "auto_save", false);
  c.auto_save_interval_s = yaml_get(cap, "auto_save_interval_s", 1.5);
  c.auto_min_corner_shift_px = yaml_get(cap, "auto_min_corner_shift_px", 40.0);
  c.show_each = yaml_get(cap, "show_each", true);

  auto calib = yaml["calib"];
  c.fix_k3 = yaml_get(calib, "fix_k3", true);
  c.handeye_method = yaml_get(calib, "handeye_method", std::string("tsai"));

  c.camera_name = yaml_get(yaml, "camera_name", std::string("opencv"));
  c.opencv_device = yaml_get(yaml, "opencv_device", 0);
  c.opencv_width = yaml_get(yaml, "opencv_width", 1280);
  c.opencv_height = yaml_get(yaml, "opencv_height", 720);

  if (yaml["R_gimbal2imubody"])
    c.R_gimbal2imubody = yaml["R_gimbal2imubody"].as<std::vector<double>>();
  if (c.R_gimbal2imubody.size() != 9)
    throw std::runtime_error("R_gimbal2imubody 必须是 9 个数（行优先 3x3）");

  if (yaml["can_disabled"] && yaml["can_disabled"].as<bool>()) c.use_imu = false;

  return c;
}

cv::Size pattern_size(const Config & c) { return {c.cols, c.rows}; }

std::vector<cv::Point3f> object_points(const Config & c)
{
  std::vector<cv::Point3f> pts;
  pts.reserve(c.cols * c.rows);
  for (int i = 0; i < c.rows; ++i)
    for (int j = 0; j < c.cols; ++j)
      pts.emplace_back(j * c.square_mm, i * c.square_mm, 0.f);
  return pts;
}

cv::HandEyeCalibrationMethod parse_handeye_method(const std::string & s)
{
  if (s == "park") return cv::CALIB_HAND_EYE_PARK;
  if (s == "horaud") return cv::CALIB_HAND_EYE_HORAUD;
  if (s == "andreff") return cv::CALIB_HAND_EYE_ANDREFF;
  if (s == "daniilidis") return cv::CALIB_HAND_EYE_DANIILIDIS;
  return cv::CALIB_HAND_EYE_TSAI;
}

bool detect_chessboard(
  const cv::Mat & img, const Config & c, std::vector<cv::Point2f> & corners)
{
  cv::Mat gray;
  if (img.channels() == 3)
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
  else
    gray = img;

  const auto ps = pattern_size(c);
  corners.clear();
  bool ok = false;

  if (c.detector != "classic") {
    ok = cv::findChessboardCornersSB(
      gray, ps, corners, cv::CALIB_CB_EXHAUSTIVE | cv::CALIB_CB_ACCURACY);
  }
  if (!ok) {
    ok = cv::findChessboardCorners(
      gray, ps, corners,
      cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);
    if (ok) {
      cv::cornerSubPix(
        gray, corners, {11, 11}, {-1, -1},
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 40, 0.01));
    }
  }
  return ok && static_cast<int>(corners.size()) == c.cols * c.rows;
}

const char * kCalibWin = "chessboard calib";

enum class StepState { Wait, Run, Done, Skip, Fail };

// 与 ROS camera_calibration 相同：X/Y/Size/Skew ∈ [0,1]
using RosParam = std::array<double, 4>;
const RosParam kParamRange{0.7, 0.7, 0.4, 0.5};
const char * kParamName[] = {"X", "Y", "Size", "Skew"};
const char * kParamHow[] = {
  "LEFT <-> RIGHT", "UP <-> DOWN", "NEAR <-> FAR", "TILT the board"};

double ros_param_dist(const RosParam & a, const RosParam & b)
{
  return std::abs(a[0] - b[0]) + std::abs(a[1] - b[1]) + std::abs(a[2] - b[2]) +
         std::abs(a[3] - b[3]);
}

bool ros_params(
  const std::vector<cv::Point2f> & corners, const cv::Size & img, int cols, int rows, RosParam & p)
{
  const int n = cols * rows;
  if (static_cast<int>(corners.size()) != n || img.width < 2 || img.height < 2) return false;

  const cv::Point2f ul = corners[0];
  const cv::Point2f ur = corners[cols - 1];
  const cv::Point2f dr = corners[n - 1];
  const cv::Point2f dl = corners[n - cols];

  auto ang = [](cv::Point2f a, cv::Point2f b, cv::Point2f c) {
    const cv::Point2f ab = a - b, cb = c - b;
    const double nab = cv::norm(ab), ncb = cv::norm(cb);
    if (nab < 1e-6 || ncb < 1e-6) return 0.0;
    const double cosv = std::clamp(ab.dot(cb) / (nab * ncb), -1.0, 1.0);
    return std::acos(cosv);
  };
  const double skew = std::min(1.0, 2.0 * std::abs(CV_PI / 2.0 - ang(ul, ur, dr)));

  const cv::Point2f a = ur - ul, b = dr - ur, c = dl - dr;
  const cv::Point2f pp = b + c, q = a + b;
  const double area = std::abs(pp.x * q.y - pp.y * q.x) / 2.0;

  double mx = 0, my = 0;
  for (const auto & pt : corners) {
    mx += pt.x;
    my += pt.y;
  }
  mx /= n;
  my /= n;

  const double border = std::sqrt(std::max(area, 1.0));
  const double denx = std::max(1.0, img.width - border);
  const double deny = std::max(1.0, img.height - border);
  p[0] = std::clamp((mx - border / 2.0) / denx, 0.0, 1.0);
  p[1] = std::clamp((my - border / 2.0) / deny, 0.0, 1.0);
  p[2] = std::sqrt(area / std::max(1.0, static_cast<double>(img.area())));
  p[3] = skew;
  return true;
}

struct RosSampler
{
  std::vector<RosParam> db;
  const double min_dist = 0.2;

  bool add_if_new(const RosParam & p, bool force)
  {
    if (!force && !db.empty()) {
      double best = 1e9;
      for (const auto & q : db) best = std::min(best, ros_param_dist(p, q));
      if (best <= min_dist) return false;
    }
    db.push_back(p);
    return true;
  }

  void stats(RosParam & lo, RosParam & hi, RosParam & prog, bool & goodenough) const
  {
    lo = {0, 0, 0, 0};
    hi = {0, 0, 0, 0};
    prog = {0, 0, 0, 0};
    goodenough = false;
    if (db.empty()) return;
    lo = db[0];
    hi = db[0];
    for (const auto & p : db) {
      for (int i = 0; i < 4; ++i) {
        lo[i] = std::min(lo[i], p[i]);
        hi[i] = std::max(hi[i], p[i]);
      }
    }
    lo[2] = 0;
    lo[3] = 0;
    bool all = true;
    for (int i = 0; i < 4; ++i) {
      prog[i] = std::min(1.0, (hi[i] - lo[i]) / kParamRange[i]);
      if (prog[i] < 1.0) all = false;
    }
    goodenough = all || static_cast<int>(db.size()) >= 40;
  }
};

std::string next_action(
  bool found, const RosSampler & sp, const RosParam & lo, const RosParam & hi, const RosParam & prog,
  bool goodenough)
{
  if (goodenough) return "ALL GREEN  press C once (do not hold)";
  if (!found) return "Show the chessboard until corners light up";
  if (sp.db.empty()) return "Hold still: first sample will be auto-saved";

  int worst = 0;
  for (int i = 1; i < 4; ++i)
    if (prog[i] < prog[worst]) worst = i;

  if (worst == 0) {
    if (hi[0] < 0.55) return "NEXT: move board RIGHT  -->";
    if (lo[0] > 0.45) return "NEXT: move board LEFT   <--";
    return "NEXT: move board LEFT and RIGHT more";
  }
  if (worst == 1) {
    if (hi[1] < 0.55) return "NEXT: move board DOWN";
    if (lo[1] > 0.45) return "NEXT: move board UP";
    return "NEXT: move board UP and DOWN more";
  }
  if (worst == 2) {
    if (hi[2] < 0.25) return "NEXT: move CLOSER  (board bigger)";
    return "NEXT: move FARTHER (board smaller)";
  }
  return "NEXT: TILT the board (left / right / rotate)";
}

struct ProgressBoard
{
  StepState capture = StepState::Wait;
  StepState intra = StepState::Wait;
  StepState hand = StepState::Wait;
  StepState save = StepState::Wait;
  int saved = 0;
  int need = 15;
  int intra_i = 0;
  int intra_n = 0;
  int intra_ok = 0;
  bool found = false;
  bool imu = false;
  bool goodenough = false;
  bool live_ok = false;
  int flash = 0;
  double brightness = 0;
  RosParam live{}, lo{}, hi{}, prog{};
  std::vector<RosParam> samples;
  std::string hint;
  std::optional<Eigen::Vector3d> ypr_deg;

  void pull(const RosSampler & sp)
  {
    samples = sp.db;
    saved = static_cast<int>(sp.db.size());
    sp.stats(lo, hi, prog, goodenough);
  }

  cv::Mat render(int height) const;
  void show(const cv::Mat * camera = nullptr) const;
};

cv::Mat to_bgr(const cv::Mat & img)
{
  if (img.empty()) return img;
  cv::Mat bgr;
  if (img.channels() == 1)
    cv::cvtColor(img, bgr, cv::COLOR_GRAY2BGR);
  else if (img.channels() == 4)
    cv::cvtColor(img, bgr, cv::COLOR_BGRA2BGR);
  else
    bgr = img.clone();
  return bgr;
}

double image_brightness(const cv::Mat & img)
{
  cv::Mat gray;
  if (img.channels() == 3)
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
  else if (img.channels() == 4)
    cv::cvtColor(img, gray, cv::COLOR_BGRA2GRAY);
  else
    gray = img;
  return cv::mean(gray)[0];
}

void put(
  cv::Mat & img, const std::string & text, cv::Point org, cv::Scalar color, double scale = 0.55,
  int thick = 1)
{
  cv::putText(img, text, org, cv::FONT_HERSHEY_SIMPLEX, scale, color, thick, cv::LINE_AA);
}

void draw_bar(cv::Mat & img, cv::Rect r, double frac, cv::Scalar fill)
{
  frac = std::clamp(frac, 0.0, 1.0);
  cv::rectangle(img, r, {50, 50, 55}, -1);
  cv::rectangle(img, {r.x, r.y, std::max(0, static_cast<int>(r.width * frac)), r.height}, fill, -1);
  cv::rectangle(img, r, {160, 160, 160}, 1);
}

cv::Scalar state_color(StepState s)
{
  switch (s) {
    case StepState::Run:
      return {0, 200, 255};
    case StepState::Done:
      return {40, 200, 80};
    case StepState::Skip:
      return {140, 140, 140};
    case StepState::Fail:
      return {40, 40, 220};
    default:
      return {90, 90, 90};
  }
}

const char * state_text(StepState s)
{
  switch (s) {
    case StepState::Run:
      return "RUNNING";
    case StepState::Done:
      return "DONE";
    case StepState::Skip:
      return "SKIP";
    case StepState::Fail:
      return "FAIL";
    default:
      return "WAIT";
  }
}

cv::Mat ProgressBoard::render(int height) const
{
  const int w = 520;
  const int h = std::max(height, 680);
  cv::Mat p(h, w, CV_8UC3, cv::Scalar(28, 28, 32));
  int y = 34;
  put(p, "ROS-style coverage", {18, y}, {240, 240, 240}, 0.8, 2);
  y += 16;
  cv::line(p, {18, y}, {w - 18, y}, {70, 70, 80}, 1);
  y += 30;

  struct Row
  {
    const char * name;
    StepState st;
    std::string extra;
  };
  const Row rows[] = {
    {"1  CAPTURE", capture, fmt::format("{} samples", saved)},
    {"2  INTRINSICS", intra,
     intra_n > 0 ? fmt::format("{}/{} ok {}", intra_i, intra_n, intra_ok) : ""},
    {"3  HAND-EYE", hand, imu ? "" : "no IMU"},
    {"4  SAVE YAML", save, ""},
  };
  for (const auto & row : rows) {
    cv::circle(p, {30, y - 6}, 8, state_color(row.st), -1);
    put(p, row.name, {48, y}, {230, 230, 230}, 0.52, 1);
    put(p, state_text(row.st), {240, y}, state_color(row.st), 0.48, 1);
    if (!row.extra.empty()) put(p, row.extra, {350, y}, {180, 200, 220}, 0.45, 1);
    y += 26;
  }

  y += 10;
  put(p, "X / Y / Size / Skew   (fill until GREEN)", {18, y}, {180, 180, 180}, 0.48, 1);
  y += 18;

  const int track_x = 70;
  const int track_w = 320;
  for (int i = 0; i < 4; ++i) {
    const bool done = prog[i] >= 1.0 - 1e-6;
    const cv::Scalar col =
      done ? cv::Scalar(40, 210, 80) : cv::Scalar(0, static_cast<int>(prog[i] * 255), 255);
    put(p, kParamName[i], {18, y + 14}, col, 0.55, 2);
    cv::Rect track{track_x, y + 6, track_w, 16};
    cv::rectangle(p, track, {45, 45, 50}, -1);
    cv::rectangle(p, track, {120, 120, 125}, 1);
    const int x0 = track_x + static_cast<int>(lo[i] * track_w);
    const int x1 = track_x + static_cast<int>(hi[i] * track_w);
    if (saved > 0)
      cv::line(p, {x0, y + 14}, {std::max(x0 + 2, x1), y + 14}, col, 6);
    if (live_ok) {
      const int xt = track_x + static_cast<int>(live[i] * track_w);
      cv::line(p, {xt, y + 2}, {xt, y + 26}, {240, 240, 240}, 2);
    }
    put(p, fmt::format("{:3.0f}%", prog[i] * 100), {track_x + track_w + 10, y + 16}, col, 0.5, 1);
    y += 22;
    put(p, kParamHow[i], {track_x, y}, {140, 140, 145}, 0.42, 1);
    y += 22;
  }

  y += 4;
  const cv::Rect next_box{18, y, w - 36, 88};
  cv::rectangle(p, next_box, goodenough ? cv::Scalar(20, 90, 30) : cv::Scalar(20, 70, 110), -1);
  cv::rectangle(
    p, next_box, goodenough ? cv::Scalar(40, 220, 80) : cv::Scalar(0, 200, 255), 2);
  put(p, "WHAT TO DO NOW", {28, y + 24}, {200, 200, 200}, 0.45, 1);
  put(p, hint, {28, y + 58}, {0, 255, 255}, 0.58, 2);
  y += 104;

  put(p, "sample map  (X-Y)", {18, y}, {180, 180, 180}, 0.45, 1);
  y += 8;
  const cv::Rect map{18, y, 220, 150};
  cv::rectangle(p, map, {40, 40, 45}, -1);
  cv::rectangle(p, map, {100, 100, 110}, 1);
  for (const auto & s : samples) {
    const cv::Point pt(
      map.x + static_cast<int>(s[0] * map.width), map.y + static_cast<int>(s[1] * map.height));
    cv::circle(p, pt, 4, {40, 180, 80}, -1);
  }
  if (live_ok) {
    const cv::Point pt(
      map.x + static_cast<int>(live[0] * map.width),
      map.y + static_cast<int>(live[1] * map.height));
    cv::circle(p, pt, 7, {0, 220, 255}, 2);
  }
  put(p, "L", {map.x + 4, map.y + map.height / 2}, {120, 120, 120}, 0.4, 1);
  put(p, "R", {map.x + map.width - 16, map.y + map.height / 2}, {120, 120, 120}, 0.4, 1);
  put(p, "UP", {map.x + map.width / 2 - 10, map.y + 16}, {120, 120, 120}, 0.4, 1);
  put(p, "DN", {map.x + map.width / 2 - 10, map.y + map.height - 6}, {120, 120, 120}, 0.4, 1);

  const int bx = map.x + map.width + 24;
  const cv::Rect calib_btn{bx, y, w - 36 - map.width - 24, 56};
  cv::rectangle(
    p, calib_btn, goodenough ? cv::Scalar(30, 160, 50) : cv::Scalar(55, 55, 60), -1);
  cv::rectangle(
    p, calib_btn, goodenough ? cv::Scalar(40, 230, 80) : cv::Scalar(90, 90, 90), 2);
  put(p, goodenough ? "CALIBRATE" : "NOT READY",
      cv::Point(calib_btn.x + 18, calib_btn.y + 36),
      goodenough ? cv::Scalar(255, 255, 255) : cv::Scalar(150, 150, 150), 0.7, 2);
  put(p, goodenough ? "press C once" : "keep moving",
      cv::Point(calib_btn.x + 18, calib_btn.y + 78), cv::Scalar(180, 180, 180), 0.45, 1);

  y += 160;
  put(p, fmt::format("board: {}", found ? "FOUND" : "NOT FOUND"), {18, y},
      found ? cv::Scalar(40, 200, 80) : cv::Scalar(40, 40, 220), 0.5, 1);
  y += 24;
  put(p, fmt::format("IMU: {}", imu ? "ON" : "OFF"), {18, y},
      imu ? cv::Scalar(40, 200, 80) : cv::Scalar(0, 180, 255), 0.48, 1);
  y += 24;

  const char * bright_msg = brightness < 50 ? "TOO DARK  raise exposure_ms"
                           : brightness < 85  ? "a bit dark"
                           : brightness > 210 ? "TOO BRIGHT"
                                              : "OK";
  const cv::Scalar bc = brightness < 50    ? cv::Scalar(40, 40, 220)
                        : brightness < 85  ? cv::Scalar(0, 160, 255)
                        : brightness > 210 ? cv::Scalar(40, 40, 220)
                                           : cv::Scalar(40, 200, 80);
  put(p, fmt::format("brightness {:.0f}  {}", brightness, bright_msg), {18, y}, bc, 0.48, 1);
  y += 10;
  draw_bar(
    p, {18, y, w - 36, 12}, brightness / 255.0,
    brightness < 50 ? cv::Scalar(40, 40, 220) : cv::Scalar(0, 180, 255));
  y += 32;

  if (ypr_deg) {
    put(
      p,
      fmt::format("yaw {:.1f}  pitch {:.1f}  roll {:.1f}", (*ypr_deg)[0], (*ypr_deg)[1],
                  (*ypr_deg)[2]),
      {18, y}, {0, 0, 255}, 0.48, 1);
    y += 24;
  }

  put(p, "C once = calibrate   S force save   Q quit", {18, y}, {150, 150, 150}, 0.42, 1);
  return p;
}

void ProgressBoard::show(const cv::Mat * camera) const
{
  cv::Mat panel;
  if (camera && !camera->empty()) {
    cv::Mat cam = to_bgr(*camera);
    if (flash > 0) cv::rectangle(cam, {2, 2, cam.cols - 4, cam.rows - 4}, {40, 220, 80}, 10);
    const int h = std::max(cam.rows, 680);
    panel = render(h);
    if (cam.rows != panel.rows) {
      const int w = std::max(1, cam.cols * panel.rows / cam.rows);
      cv::resize(cam, cam, {w, panel.rows});
    }
    cv::Mat out;
    cv::hconcat(cam, panel, out);
    cv::imshow(kCalibWin, out);
  } else {
    cv::imshow(kCalibWin, render(680));
  }
}

void draw_camera_hud(
  cv::Mat & vis, bool found, double brightness, const std::string & next, bool ready)
{
  vis = to_bgr(vis);
  const auto color = found ? cv::Scalar(0, 220, 0) : cv::Scalar(0, 0, 255);
  tools::draw_text(vis, found ? "BOARD OK" : "NO BOARD", {20, 36}, color, 0.8, 2);
  const auto bc = brightness < 50 ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 200, 255);
  tools::draw_text(vis, fmt::format("bright {:.0f}", brightness), {20, 72}, bc, 0.7, 2);

  const int bar_h = 54;
  cv::rectangle(
    vis, {0, vis.rows - bar_h}, {vis.cols, vis.rows},
    ready ? cv::Scalar(0, 120, 0) : cv::Scalar(0, 90, 160), -1);
  tools::draw_text(vis, next, {16, vis.rows - 16}, {0, 255, 255}, 0.75, 2);
}

int next_index(const fs::path & dir)
{
  int i = 1;
  while (fs::exists(dir / fmt::format("{}.jpg", i))) ++i;
  return i;
}

int count_images(const fs::path & dir)
{
  int n = 0;
  for (int i = 1; fs::exists(dir / fmt::format("{}.jpg", i)); ++i) ++n;
  return n;
}

void write_q(const fs::path & path, const Eigen::Quaterniond & q)
{
  Eigen::Vector4d xyzw = q.coeffs();  // x y z w
  std::ofstream f(path);
  f << fmt::format("{} {} {} {}", xyzw[3], xyzw[0], xyzw[1], xyzw[2]);
}

Eigen::Quaterniond read_q(const fs::path & path)
{
  std::ifstream f(path);
  double w = 1, x = 0, y = 0, z = 0;
  f >> w >> x >> y >> z;
  return {w, x, y, z};
}

Eigen::Matrix3d gimbal2world(const Eigen::Quaterniond & q, const Config & c)
{
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R_g2imu(c.R_gimbal2imubody.data());
  Eigen::Matrix3d R_imu = q.normalized().toRotationMatrix();
  return R_g2imu.transpose() * R_imu * R_g2imu;
}

struct FrameGrabber
{
  std::unique_ptr<io::Camera> sp_cam;
  cv::VideoCapture cap;
  bool use_cap = false;

  bool read(cv::Mat & img, Clock::time_point & t)
  {
    if (use_cap) {
      cap >> img;
      t = Clock::now();
      return !img.empty();
    }
    if (!sp_cam) return false;
    sp_cam->read(img, t);
    return !img.empty();
  }
};

bool open_grabber(FrameGrabber & g, const Config & c)
{
  if (c.camera_name == "opencv") {
    g.use_cap = true;
    g.cap.open(c.opencv_device, cv::CAP_V4L2);
    if (!g.cap.isOpened()) g.cap.open(c.opencv_device);
    if (!g.cap.isOpened()) {
      tools::logger()->error("打不开 USB 相机 device={}，检查 /dev/video* 或改 opencv_device",
                             c.opencv_device);
      return false;
    }
    g.cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    g.cap.set(cv::CAP_PROP_FRAME_WIDTH, c.opencv_width);
    g.cap.set(cv::CAP_PROP_FRAME_HEIGHT, c.opencv_height);
    tools::logger()->info("OpenCV 相机已打开 device={}", c.opencv_device);
    return true;
  }

  try {
    g.use_cap = false;
    g.sp_cam = std::make_unique<io::Camera>(c.path);
    tools::logger()->info("工业/视频相机已打开 camera_name={}", c.camera_name);
    return true;
  } catch (const std::exception & e) {
    tools::logger()->error("打开相机失败: {}", e.what());
    return false;
  }
}

void write_capture_info(const Config & c)
{
  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "inner_corners_cols" << YAML::Value << c.cols;
  out << YAML::Key << "inner_corners_rows" << YAML::Value << c.rows;
  out << YAML::Key << "square_size_mm" << YAML::Value << c.square_mm;
  out << YAML::Key << "use_imu" << YAML::Value << c.use_imu;
  out << YAML::EndMap;
  std::ofstream f(fs::path(c.data_dir) / "capture_info.yaml");
  f << out.c_str() << "\n";
}

int capture_loop(const Config & c, io::CBoard * cboard, ProgressBoard & pb)
{
  FrameGrabber grabber;
  if (!open_grabber(grabber, c)) return -1;

  fs::create_directories(c.data_dir);
  write_capture_info(c);

  RosSampler sampler;
  int next = next_index(c.data_dir);
  for (int i = 1; i < next; ++i) {
    auto path = fs::path(c.data_dir) / fmt::format("{}.jpg", i);
    cv::Mat old = cv::imread(path.string());
    if (old.empty()) continue;
    std::vector<cv::Point2f> corners;
    RosParam p{};
    if (detect_chessboard(old, c, corners) &&
        ros_params(corners, old.size(), c.cols, c.rows, p))
      sampler.add_if_new(p, true);
  }

  pb.capture = StepState::Run;
  pb.need = c.min_frames;
  pb.imu = cboard != nullptr;
  pb.pull(sampler);
  pb.hint = next_action(false, sampler, pb.lo, pb.hi, pb.prog, pb.goodenough);

  tools::logger()->info(
    "像 ROS 标定那样移动棋盘：条变绿就够了。自动收图，C=开始计算，S=强制保存，Q=结束。已有 {} 张",
    sampler.db.size());

  cv::namedWindow(kCalibWin, cv::WINDOW_NORMAL);

  while (true) {
    cv::Mat img;
    Clock::time_point t;
    if (!grabber.read(img, t)) {
      tools::logger()->warn("读图失败");
      break;
    }

    std::vector<cv::Point2f> corners;
    const bool found = detect_chessboard(img, c, corners);
    const double bright = image_brightness(img);

    std::optional<Eigen::Vector3d> ypr;
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    if (cboard) {
      q = cboard->imu_at(t);
      Eigen::Vector3d ypr_rad = tools::eulers(gimbal2world(q, c), 2, 1, 0);
      ypr = ypr_rad * 57.3;
    }

    RosParam p{};
    pb.found = found;
    pb.brightness = bright;
    pb.ypr_deg = ypr;
    pb.live_ok = found && ros_params(corners, img.size(), c.cols, c.rows, p);
    if (pb.live_ok) pb.live = p;
    pb.pull(sampler);
    pb.hint = next_action(found, sampler, pb.lo, pb.hi, pb.prog, pb.goodenough);
    if (pb.flash > 0) --pb.flash;

    auto commit_sample = [&](bool force) {
      if (c.require_board && !found) {
        tools::logger()->warn("未检测到棋盘格，未保存");
        return false;
      }
      if (found && pb.live_ok) {
        if (!sampler.add_if_new(p, force)) return false;
      } else if (!force) {
        return false;
      }
      auto img_path = fs::path(c.data_dir) / fmt::format("{}.jpg", next);
      cv::imwrite(img_path.string(), img);
      if (cboard) write_q(fs::path(c.data_dir) / fmt::format("{}.txt", next), q);
      ++next;
      pb.pull(sampler);
      pb.flash = 8;
      std::fputc('\a', stdout);
      std::fflush(stdout);
      tools::logger()->info(
        "*** Added sample {}, X={:.3f} Y={:.3f} Size={:.3f} Skew={:.3f}  |  {}", sampler.db.size(),
        pb.live[0], pb.live[1], pb.live[2], pb.live[3], pb.hint);
      return true;
    };

    if (c.auto_save && found && pb.live_ok) commit_sample(false);

    cv::Mat vis = img.clone();
    vis = to_bgr(vis);
    if (found) cv::drawChessboardCorners(vis, pattern_size(c), corners, true);
    draw_camera_hud(vis, found, bright, pb.hint, pb.goodenough);
    if (c.preview_scale > 0 && c.preview_scale != 1.0)
      cv::resize(vis, vis, {}, c.preview_scale, c.preview_scale);

    pb.show(&vis);
    const int key = cv::waitKey(20) & 0xFF;
    if (key == 's' || key == 'S') commit_sample(true);
    if (key == 'c' || key == 'C') {
      if (pb.goodenough || static_cast<int>(sampler.db.size()) >= std::max(4, c.min_frames)) {
        tools::logger()->info("收到一次 C，开始自动计算，后面不用再按键");
        break;
      }
      tools::logger()->warn("还没绿齐，按提示继续移动棋盘（或再采几张）");
    }
    if (key == 'q' || key == 27) {
      if (sampler.db.size() < 4) {
        tools::logger()->warn("有效样本不足 4 张，继续采集");
        continue;
      }
      break;
    }
  }

  pb.found = false;
  pb.live_ok = false;
  pb.pull(sampler);
  if (sampler.db.size() < 4) {
    pb.capture = StepState::Fail;
    pb.hint = "not enough samples";
    tools::logger()->warn("只采到 {} 张，至少 4 张才能标内参", sampler.db.size());
  } else {
    pb.capture = StepState::Done;
    pb.hint = "capture done, computing...";
  }
  pb.show();
  return static_cast<int>(sampler.db.size());
}

std::vector<double> mat_to_vec(const cv::Mat & m)
{
  cv::Mat d;
  m.convertTo(d, CV_64F);
  return {d.begin<double>(), d.end<double>()};
}

double mean_reproj_error(
  const std::vector<std::vector<cv::Point3f>> & obj,
  const std::vector<std::vector<cv::Point2f>> & img, const cv::Mat & K, const cv::Mat & D,
  const std::vector<cv::Mat> & rvecs, const std::vector<cv::Mat> & tvecs)
{
  double err_sum = 0;
  size_t n = 0;
  for (size_t i = 0; i < obj.size(); ++i) {
    std::vector<cv::Point2f> proj;
    cv::projectPoints(obj[i], rvecs[i], tvecs[i], K, D, proj);
    for (size_t j = 0; j < proj.size(); ++j) {
      err_sum += cv::norm(img[i][j] - proj[j]);
      ++n;
    }
  }
  return n ? err_sum / static_cast<double>(n) : -1;
}

struct Intrinsics
{
  cv::Mat K, D;
  double error = -1;
  int n_used = 0;
  cv::Size img_size;
};

bool load_intrinsics_from_yaml(const std::string & path, Intrinsics & out)
{
  auto yaml = YAML::LoadFile(path);
  if (!yaml["camera_matrix"] || !yaml["distort_coeffs"]) return false;
  auto k = yaml["camera_matrix"].as<std::vector<double>>();
  auto d = yaml["distort_coeffs"].as<std::vector<double>>();
  if (k.size() != 9 || d.empty()) return false;
  out.K = cv::Mat(3, 3, CV_64FC1);
  for (int i = 0; i < 9; ++i) out.K.at<double>(i / 3, i % 3) = k[i];
  out.D = cv::Mat(1, static_cast<int>(d.size()), CV_64FC1);
  for (size_t i = 0; i < d.size(); ++i) out.D.at<double>(0, static_cast<int>(i)) = d[i];
  return true;
}

bool calib_intrinsics(const Config & c, Intrinsics & out, ProgressBoard & pb)
{
  pb.intra = StepState::Run;
  pb.hint = "detecting chessboard in saved images...";
  pb.show();

  std::vector<std::string> paths;
  for (int i = 1;; ++i) {
    auto path = fs::path(c.data_dir) / fmt::format("{}.jpg", i);
    if (!fs::exists(path)) break;
    paths.push_back(path.string());
  }
  pb.intra_n = static_cast<int>(paths.size());
  pb.intra_ok = 0;

  std::vector<std::vector<cv::Point3f>> obj_pts;
  std::vector<std::vector<cv::Point2f>> img_pts;
  const auto obj = object_points(c);

  for (int i = 0; i < static_cast<int>(paths.size()); ++i) {
    cv::Mat img = cv::imread(paths[i]);
    if (img.empty()) continue;
    out.img_size = img.size();
    pb.intra_i = i + 1;
    pb.brightness = image_brightness(img);

    std::vector<cv::Point2f> corners;
    const bool ok = detect_chessboard(img, c, corners);
    pb.found = ok;
    pb.live_ok = ok && ros_params(corners, img.size(), c.cols, c.rows, pb.live);
    pb.hint = fmt::format("intrinsics {}/{}  {}", i + 1, paths.size(), ok ? "OK" : "FAIL");

    cv::Mat vis = to_bgr(img);
    cv::drawChessboardCorners(vis, pattern_size(c), corners, ok);
    draw_camera_hud(vis, ok, pb.brightness, pb.hint, false);
    tools::draw_text(
      vis, fmt::format("{}/{}", i + 1, paths.size()), {20, 108}, {0, 255, 255}, 0.7, 2);
    if (c.preview_scale > 0 && c.preview_scale != 1.0)
      cv::resize(vis, vis, {}, c.preview_scale, c.preview_scale);
    pb.show(&vis);

    fmt::print("[{}] {}\n", ok ? "success" : "failure", paths[i]);
    const int k = cv::waitKey(c.show_each ? 0 : 1) & 0xFF;
    if (k == 'c' || k == 'C' || k == 'q' || k == 27) {
      tools::logger()->info("跳过剩余预览，继续计算内参");
      if (ok) {
        img_pts.push_back(corners);
        obj_pts.push_back(obj);
        ++pb.intra_ok;
      }
      for (int j = i + 1; j < static_cast<int>(paths.size()); ++j) {
        cv::Mat more = cv::imread(paths[j]);
        if (more.empty()) continue;
        std::vector<cv::Point2f> more_corners;
        if (!detect_chessboard(more, c, more_corners)) continue;
        img_pts.push_back(more_corners);
        obj_pts.push_back(obj);
        ++pb.intra_ok;
      }
      break;
    }

    if (!ok) continue;
    img_pts.push_back(corners);
    obj_pts.push_back(obj);
    ++pb.intra_ok;
  }

  if (obj_pts.size() < 4) {
    pb.intra = StepState::Fail;
    pb.hint = "not enough valid boards, capture more";
    pb.show();
    tools::logger()->error("有效棋盘格图片不足（{} 张），至少 4 张，建议 {}", obj_pts.size(),
                           c.min_frames);
    return false;
  }

  pb.hint = "solving camera matrix...";
  pb.show();
  out.K = cv::Mat::eye(3, 3, CV_64F);
  out.D = cv::Mat::zeros(1, 5, CV_64F);
  std::vector<cv::Mat> rvecs, tvecs;
  int flags = c.fix_k3 ? cv::CALIB_FIX_K3 : 0;
  auto criteria =
    cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, DBL_EPSILON);
  cv::calibrateCamera(obj_pts, img_pts, out.img_size, out.K, out.D, rvecs, tvecs, flags, criteria);
  out.error = mean_reproj_error(obj_pts, img_pts, out.K, out.D, rvecs, tvecs);
  out.n_used = static_cast<int>(obj_pts.size());
  pb.intra = StepState::Done;
  pb.found = false;
  pb.hint = fmt::format("intrinsics done, reproj err {:.4f} px", out.error);
  pb.show();
  tools::logger()->info("内参完成：用了 {} 张，平均重投影误差 {:.4f} px", out.n_used, out.error);
  return true;
}

struct Handeye
{
  cv::Mat R, t;  // t 单位：米
  Eigen::Vector3d ypr_deg = Eigen::Vector3d::Zero();
  int n_used = 0;
};

bool calib_handeye(const Config & c, const Intrinsics & intra, Handeye & out, ProgressBoard & pb)
{
  pb.hand = StepState::Run;
  pb.hint = "running hand-eye...";
  pb.show();
  if (intra.K.empty()) {
    pb.hand = StepState::Fail;
    tools::logger()->error("手眼标定需要内参，请先开 steps.intrinsics 或在 yaml 里写 camera_matrix");
    return false;
  }

  std::vector<cv::Mat> R_g2w, t_g2w, rvecs, tvecs;
  const auto obj = object_points(c);

  for (int i = 1;; ++i) {
    auto img_path = fs::path(c.data_dir) / fmt::format("{}.jpg", i);
    auto q_path = fs::path(c.data_dir) / fmt::format("{}.txt", i);
    cv::Mat img = cv::imread(img_path.string());
    if (img.empty()) break;
    if (!fs::exists(q_path)) {
      tools::logger()->warn("缺少四元数 {}，跳过", q_path.string());
      continue;
    }

    std::vector<cv::Point2f> corners;
    if (!detect_chessboard(img, c, corners)) continue;

    auto q = read_q(q_path);
    Eigen::Matrix3d R = gimbal2world(q, c);
    cv::Mat R_cv, t_cv = (cv::Mat_<double>(3, 1) << 0, 0, 0);
    cv::eigen2cv(R, R_cv);

    cv::Mat rvec, tvec;
    cv::solvePnP(obj, corners, intra.K, intra.D, rvec, tvec, false, cv::SOLVEPNP_IPPE);
    rvec.convertTo(rvec, CV_64F);
    tvec.convertTo(tvec, CV_64F);
    R_cv.convertTo(R_cv, CV_64F);

    R_g2w.push_back(R_cv);
    t_g2w.push_back(t_cv);
    rvecs.push_back(rvec);
    tvecs.push_back(tvec);
  }

  if (R_g2w.size() < 5) {
    pb.hand = StepState::Fail;
    pb.hint = "not enough gimbal poses for hand-eye";
    pb.show();
    tools::logger()->error("手眼有效姿态太少（{}），至少 5 组，且云台 yaw/pitch 要拉开",
                           R_g2w.size());
    return false;
  }

  cv::calibrateHandEye(
    R_g2w, t_g2w, rvecs, tvecs, out.R, out.t, parse_handeye_method(c.handeye_method));
  out.t /= 1e3;  // mm -> m
  out.n_used = static_cast<int>(R_g2w.size());

  Eigen::Matrix3d R_eigen;
  cv::cv2eigen(out.R, R_eigen);
  Eigen::Matrix3d R_gimbal2ideal{{0, -1, 0}, {0, 0, -1}, {1, 0, 0}};
  Eigen::Matrix3d R_camera2ideal = R_gimbal2ideal * R_eigen;
  out.ypr_deg = tools::eulers(R_camera2ideal, 1, 0, 2) * 57.3;

  tools::logger()->info(
    "手眼完成：用了 {} 组，相机相对理想安装 yaw{:.2f} pitch{:.2f} roll{:.2f} deg", out.n_used,
    out.ypr_deg[0], out.ypr_deg[1], out.ypr_deg[2]);
  pb.hand = StepState::Done;
  pb.hint = fmt::format(
    "hand-eye done yaw{:.1f} pitch{:.1f} roll{:.1f}", out.ypr_deg[0], out.ypr_deg[1],
    out.ypr_deg[2]);
  pb.show();
  return true;
}

std::string flow(const std::vector<double> & v)
{
  YAML::Emitter e;
  e << YAML::Flow << v;
  return e.c_str();
}

void upsert_line(std::string & text, const std::string & key, const std::string & line)
{
  std::regex re("^" + key + R"(:[^\n]*)", std::regex_constants::multiline);
  if (std::regex_search(text, re))
    text = std::regex_replace(text, re, line, std::regex_constants::format_first_only);
  else {
    if (!text.empty() && text.back() != '\n') text += '\n';
    text += line;
    text += '\n';
  }
}

void export_to_robot_yaml(
  const std::string & path, const Intrinsics * intra, const Handeye * hand)
{
  std::ifstream in(path);
  if (!in) {
    tools::logger()->error("export_to 文件不存在: {}", path);
    return;
  }
  std::ostringstream ss;
  ss << in.rdbuf();
  std::string text = ss.str();

  if (intra && !intra->K.empty()) {
    upsert_line(text, "camera_matrix", "camera_matrix: " + flow(mat_to_vec(intra->K)));
    upsert_line(text, "distort_coeffs", "distort_coeffs: " + flow(mat_to_vec(intra->D)));
  }
  if (hand && !hand->R.empty()) {
    upsert_line(text, "R_camera2gimbal", "R_camera2gimbal: " + flow(mat_to_vec(hand->R)));
    upsert_line(text, "t_camera2gimbal", "t_camera2gimbal: " + flow(mat_to_vec(hand->t)));
  }

  std::ofstream out(path);
  out << text;
  tools::logger()->info("已写回 {}", path);
}

void save_result(
  const Config & c, const Intrinsics * intra, const Handeye * hand, bool imu_used)
{
  fs::create_directories(fs::path(c.output_yaml).parent_path());

  YAML::Emitter e;
  e << YAML::BeginMap;
  e << YAML::Comment(fmt::format(
    "棋盘格 {}x{} 内角点, 格距 {} mm, IMU={}", c.cols, c.rows, c.square_mm,
    imu_used ? "on" : "off"));
  e << YAML::Key << "R_gimbal2imubody" << YAML::Value << YAML::Flow << c.R_gimbal2imubody;
  e << YAML::Newline;

  if (intra && !intra->K.empty()) {
    e << YAML::Newline;
    e << YAML::Comment(fmt::format("重投影误差: {:.4f}px  ({} 张)", intra->error, intra->n_used));
    e << YAML::Key << "camera_matrix" << YAML::Value << YAML::Flow << mat_to_vec(intra->K);
    e << YAML::Key << "distort_coeffs" << YAML::Value << YAML::Flow << mat_to_vec(intra->D);
  }

  if (hand && !hand->R.empty()) {
    e << YAML::Newline;
    e << YAML::Comment(fmt::format(
      "相机同理想情况的偏角: yaw{:.2f} pitch{:.2f} roll{:.2f} degree", hand->ypr_deg[0],
      hand->ypr_deg[1], hand->ypr_deg[2]));
    e << YAML::Key << "R_camera2gimbal" << YAML::Value << YAML::Flow << mat_to_vec(hand->R);
    e << YAML::Key << "t_camera2gimbal" << YAML::Value << YAML::Flow << mat_to_vec(hand->t);
  } else {
    e << YAML::Newline;
    e << YAML::Comment("未做手眼：无 IMU 或 steps.handeye=false。把 use_imu 设为 true 后重采即可。");
  }
  e << YAML::Newline;
  e << YAML::EndMap;

  std::ofstream f(c.output_yaml);
  f << e.c_str() << "\n";
  fmt::print("\n{}\n", e.c_str());
  tools::logger()->info("结果已写入 {}", c.output_yaml);
}

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  const auto config_path = cli.get<std::string>(0);
  if (!fs::exists(config_path)) {
    tools::logger()->error("找不到配置文件 {}", config_path);
    return 1;
  }

  Config cfg;
  try {
    cfg = load_config(config_path);
  } catch (const std::exception & e) {
    tools::logger()->error("读取配置失败: {}", e.what());
    return 1;
  }
  bool want_handeye = cfg.do_handeye && cfg.use_imu;

  ProgressBoard pb;
  pb.need = cfg.min_frames;
  pb.imu = cfg.use_imu;
  pb.capture = cfg.do_capture ? StepState::Wait : StepState::Skip;
  pb.intra = cfg.do_intrinsics ? StepState::Wait : StepState::Skip;
  pb.hand = want_handeye ? StepState::Wait : StepState::Skip;
  pb.save = StepState::Wait;
  pb.hint = "准备开始";
  cv::namedWindow(kCalibWin, cv::WINDOW_NORMAL);
  pb.show();

  fmt::print(
    "\n======== 棋盘格标定 ========\n"
    "配置: {}\n"
    "棋盘格内角点: {} x {}  格距: {} mm  检测: {}\n"
    "IMU: {}\n"
    "步骤: {}{}{}\n"
    "数据目录: {}\n"
    "===========================\n\n",
    config_path, cfg.cols, cfg.rows, cfg.square_mm, cfg.detector,
    cfg.use_imu ? "开启（采集四元数 + 可选手眼）" : "关闭（只标内参）",
    cfg.do_capture ? "采集 " : "", cfg.do_intrinsics ? "内参 " : "",
    want_handeye ? "手眼" : (cfg.do_handeye ? "手眼(跳过,无IMU)" : ""), cfg.data_dir);

  if (cfg.do_handeye && !cfg.use_imu)
    tools::logger()->warn("use_imu=false，跳过手眼。接上 IMU 后把 use_imu 改为 true 再采一组即可。");

  std::unique_ptr<io::CBoard> cboard;
  if (cfg.do_capture && cfg.use_imu) {
    try {
      cboard = std::make_unique<io::CBoard>(cfg.path);
      tools::logger()->info("IMU / C 板已连接");
    } catch (const std::exception & e) {
      tools::logger()->error("打开 IMU 失败: {}  —— 改为只标内参", e.what());
      cfg.use_imu = false;
      want_handeye = false;
      pb.imu = false;
      pb.hand = StepState::Skip;
      cboard.reset();
    }
  }

  if (cfg.do_capture) {
    const int n = capture_loop(cfg, cboard.get(), pb);
    if (n < 0) {
      pb.capture = StepState::Fail;
      pb.hint = "failed to open camera";
      pb.show();
      cv::waitKey(1500);
      return 1;
    }
  }

  Intrinsics intra;
  bool have_intra = false;
  if (cfg.do_intrinsics) {
    have_intra = calib_intrinsics(cfg, intra, pb);
    if (!have_intra) {
      cv::waitKey(800);
      return 1;
    }
  } else if (want_handeye) {
    have_intra = load_intrinsics_from_yaml(cfg.path, intra) ||
                 (fs::exists(cfg.output_yaml) && load_intrinsics_from_yaml(cfg.output_yaml, intra));
    if (!have_intra) {
      tools::logger()->error("未开内参步骤，配置里也没有 camera_matrix");
      return 1;
    }
    pb.intra = StepState::Skip;
  }

  Handeye hand;
  bool have_hand = false;
  if (want_handeye) {
    have_hand = calib_handeye(cfg, intra, hand, pb);
    if (!have_hand) {
      cv::waitKey(800);
      return 1;
    }
  } else {
    pb.hand = StepState::Skip;
  }

  pb.save = StepState::Run;
  pb.hint = "writing yaml...";
  pb.show();
  save_result(cfg, have_intra ? &intra : nullptr, have_hand ? &hand : nullptr, cfg.use_imu);

  if (!cfg.export_to.empty())
    export_to_robot_yaml(
      cfg.export_to, have_intra ? &intra : nullptr, have_hand ? &hand : nullptr);

  pb.save = StepState::Done;
  pb.hint = fmt::format("ALL DONE  {}", cfg.output_yaml);
  pb.show();
  tools::logger()->info("标定结束。结果在 {} 。窗口 3 秒后自动关，或按任意键。", cfg.output_yaml);
  cv::waitKey(3000);
  cv::destroyAllWindows();
  return 0;
}
