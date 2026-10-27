#include "video.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <thread>

#include "tools/logger.hpp"

namespace io
{
VideoCamera::VideoCamera(
  const std::string & video_path, double fps, bool loop, const std::string & pose_path)
: video_path_(video_path), loop_(loop), first_read_(true)
{
  if (!cap_.open(video_path_)) {
    throw std::runtime_error("Failed to open video: " + video_path_ + "!");
  }

  double video_fps = cap_.get(cv::CAP_PROP_FPS);
  if (!(video_fps > 0.0) || video_fps > 1000.0) video_fps = 30.0;
  recording_interval_ = 1.0 / video_fps;
  double play_fps = (fps > 0.0) ? fps : video_fps;
  if (!(play_fps > 0.0) || play_fps > 1000.0) play_fps = 30.0;  // 兜底，防止异常帧率

  frame_interval_ = std::chrono::nanoseconds(static_cast<long long>(1e9 / play_fps));

  const auto frame_count = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_COUNT));
  tools::logger()->info(
    "[VideoCamera] Opened {} ({:.1f} playback fps, {})", video_path_, play_fps,
    frame_count > 0 ? std::to_string(frame_count) + " frames" : "frame count unavailable");
  load_poses(pose_path);
}

VideoCamera::~VideoCamera() { cap_.release(); }

void VideoCamera::load_poses(const std::string & pose_path)
{
  auto path = std::filesystem::path(pose_path);
  if (pose_path.empty()) {
    path = std::filesystem::path(video_path_).replace_extension(".txt");
    if (!std::filesystem::exists(path)) {
      tools::logger()->info("[VideoCamera] 无配套姿态文件，使用视频帧率计时和固定水平云台姿态");
      return;
    }
  }
  std::ifstream input(path);
  if (!input) throw std::runtime_error("Cannot open video pose file: " + path.string());
  std::string line;
  std::size_t line_number = 0;
  while (std::getline(input, line)) {
    ++line_number;
    const auto first = line.find_first_not_of(" \t\r");
    if (first == std::string::npos || line[first] == '#') continue;
    std::istringstream row(line);
    double time, w, x, y, z;
    if (!(row >> time >> w >> x >> y >> z) || !std::isfinite(time) ||
        (!poses_.empty() && time <= poses_.back().time)) {
      throw std::runtime_error(
        "Invalid video timestamp/pose at " + path.string() + ":" + std::to_string(line_number));
    }
    Eigen::Quaterniond q(w, x, y, z);
    if (!q.coeffs().allFinite() || q.norm() < 1e-6) {
      throw std::runtime_error("Invalid video quaternion at " + path.string() + ":" +
        std::to_string(line_number));
    }
    poses_.push_back({time, q.normalized()});
  }
  if (poses_.empty()) throw std::runtime_error("Empty video pose file: " + path.string());
  tools::logger()->info("[VideoCamera] Loaded {} recorded poses from {}", poses_.size(), path.string());
}

std::optional<Eigen::Quaterniond> VideoCamera::recorded_attitude() const
{
  return current_attitude_;
}

void VideoCamera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  if (ended_) {
    img.release();
    current_attitude_.reset();
    return;
  }
  // 按视频帧率节流，模拟相机的实时取流节奏
  auto now = std::chrono::steady_clock::now();
  if (first_read_) {
    next_frame_time_ = now;
    recording_base_ = now;
    first_read_ = false;
  }
  if (next_frame_time_ > now) std::this_thread::sleep_until(next_frame_time_);
  next_frame_time_ += frame_interval_;

  cap_ >> img;
  if (!img.empty() && !poses_.empty() && frame_index_ >= poses_.size()) {
    tools::logger()->warn(
      "[VideoCamera] 姿态文件只有 {} 帧，忽略未配对的视频尾部并{}", poses_.size(),
      loop_ ? "同步循环" : "结束回放");
    img.release();
  }
  if (img.empty() && loop_) {
    // 无索引 AVI 的 seek 可能不可靠；重新打开以保证图像与姿态都从第一帧开始。
    cap_.release();
    if (!cap_.open(video_path_)) throw std::runtime_error("Cannot reopen video: " + video_path_);
    cap_ >> img;
    loop_offset_ = last_relative_time_ + recording_interval_;
    frame_index_ = 0;
  }
  if (img.empty()) {
    if (loop_) throw std::runtime_error("Video contains no readable frames: " + video_path_);
    ended_ = true;
    current_attitude_.reset();
    return;
  }

  double relative_time = frame_index_ * recording_interval_;
  current_attitude_.reset();
  if (!poses_.empty()) {
    const auto & pose = poses_[frame_index_];
    relative_time = pose.time - poses_.front().time;
    current_attitude_ = pose.q;
  }
  last_relative_time_ = loop_offset_ + relative_time;
  timestamp = recording_base_ + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(last_relative_time_));
  ++frame_index_;
}

}  // namespace io
