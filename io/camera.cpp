#include "camera.hpp"

#include <stdexcept>

#if defined(__linux__)
#include "hikrobot/hikrobot.hpp"
#include "mindvision/mindvision.hpp"
#endif
#include "tools/yaml.hpp"
#include "video/video.hpp"

namespace io
{
Camera::Camera(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto camera_name = tools::read<std::string>(yaml, "camera_name");

  // 视频源：用本地视频代替工业相机进行调试，不依赖任何相机 SDK
  if (camera_name == "video") {
    auto video_path = tools::read<std::string>(yaml, "video_path");
    double video_fps = yaml["video_fps"] ? yaml["video_fps"].as<double>() : 0.0;
    bool video_loop = yaml["video_loop"] ? yaml["video_loop"].as<bool>() : true;
    camera_ = std::make_unique<VideoCamera>(video_path, video_fps, video_loop);
    return;
  }

#if defined(__linux__)
  auto exposure_ms = tools::read<double>(yaml, "exposure_ms");

  if (camera_name == "mindvision") {
    auto gamma = tools::read<double>(yaml, "gamma");
    auto vid_pid = tools::read<std::string>(yaml, "vid_pid");
    camera_ = std::make_unique<MindVision>(exposure_ms, gamma, vid_pid);
  }

  else if (camera_name == "hikrobot") {
    auto gain = tools::read<double>(yaml, "gain");
    auto vid_pid = tools::read<std::string>(yaml, "vid_pid");
    camera_ = std::make_unique<HikRobot>(exposure_ms, gain, vid_pid);
  }

  else {
    throw std::runtime_error("Unknow camera_name: " + camera_name + "!");
  }
#else
  // 非 Linux 平台只支持视频源（工业相机 SDK 仅 Linux 可用）。
  throw std::runtime_error(
    "Camera '" + camera_name +
    "' is only available on Linux. Use camera_name: \"video\" with a video_path for debugging on "
    "this platform.");
#endif
}

void Camera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  camera_->read(img, timestamp);
}

}  // namespace io