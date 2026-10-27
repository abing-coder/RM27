#ifndef IO__VIDEO_HPP
#define IO__VIDEO_HPP

#include <chrono>
#include <opencv2/opencv.hpp>
#include <string>

#include "io/camera.hpp"

namespace io
{
// 视频文件相机：用本地视频代替工业相机，便于在没有相机的机器上调试。
// 通过 YAML 中 camera_name: "video" 选择，video_path 指定视频路径。
class VideoCamera : public CameraBase
{
public:
  // fps <= 0 时按视频自带帧率播放；loop 为 true 时播放到结尾自动从头循环。
  VideoCamera(const std::string & video_path, double fps = 0.0, bool loop = true);
  ~VideoCamera() override;
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;

private:
  cv::VideoCapture cap_;
  std::string video_path_;
  bool loop_;
  std::chrono::nanoseconds frame_interval_;
  std::chrono::steady_clock::time_point next_frame_time_;
  bool first_read_;
};

}  // namespace io

#endif  // IO__VIDEO_HPP
