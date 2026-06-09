#include "video.hpp"

#include <stdexcept>
#include <thread>

#include "tools/logger.hpp"

namespace io
{
VideoCamera::VideoCamera(const std::string & video_path, double fps, bool loop)
: video_path_(video_path), loop_(loop), first_read_(true)
{
  if (!cap_.open(video_path_)) {
    throw std::runtime_error("Failed to open video: " + video_path_ + "!");
  }

  double video_fps = cap_.get(cv::CAP_PROP_FPS);
  double play_fps = (fps > 0.0) ? fps : video_fps;
  if (!(play_fps > 0.0) || play_fps > 1000.0) play_fps = 30.0;  // 兜底，防止异常帧率

  frame_interval_ = std::chrono::nanoseconds(static_cast<long long>(1e9 / play_fps));

  tools::logger()->info(
    "[VideoCamera] Opened {} ({:.1f} fps, {} frames)", video_path_, play_fps,
    static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_COUNT)));
}

VideoCamera::~VideoCamera() { cap_.release(); }

void VideoCamera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  // 按视频帧率节流，模拟相机的实时取流节奏
  auto now = std::chrono::steady_clock::now();
  if (first_read_) {
    next_frame_time_ = now;
    first_read_ = false;
  }
  if (next_frame_time_ > now) std::this_thread::sleep_until(next_frame_time_);
  next_frame_time_ += frame_interval_;

  cap_ >> img;
  if (img.empty() && loop_) {
    // 先尝试快速 seek 回开头；部分编码（如 mjpeg/avi）不支持可靠 seek，则重新打开视频兜底
    cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
    cap_ >> img;
    if (img.empty()) {
      cap_.release();
      cap_.open(video_path_);
      cap_ >> img;
    }
  }
  if (img.empty()) {
    throw std::runtime_error("Video ended and could not loop: " + video_path_ + "!");
  }

  timestamp = std::chrono::steady_clock::now();
}

}  // namespace io
