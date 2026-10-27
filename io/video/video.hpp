#ifndef IO__VIDEO_HPP
#define IO__VIDEO_HPP

#include <chrono>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <optional>
#include <string>
#include <vector>

#include "io/camera.hpp"

namespace io
{
// 视频文件相机：用本地视频代替工业相机，便于在没有相机的机器上调试。
// 通过 YAML 中 camera_name: "video" 选择，video_path 指定视频路径。
class VideoCamera : public CameraBase
{
public:
  // fps <= 0 时按视频自带帧率播放；loop 为 true 时播放到结尾自动从头循环。
  VideoCamera(
    const std::string & video_path, double fps = 0.0, bool loop = true,
    const std::string & pose_path = "");
  ~VideoCamera() override;
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;
  std::optional<Eigen::Quaterniond> recorded_attitude() const;

private:
  struct RecordedPose
  {
    double time;
    Eigen::Quaterniond q;
  };
  void load_poses(const std::string & pose_path);
  cv::VideoCapture cap_;
  std::string video_path_;
  bool loop_;
  std::chrono::nanoseconds frame_interval_;
  std::chrono::steady_clock::time_point next_frame_time_;
  bool first_read_;
  bool ended_ = false;
  double recording_interval_;
  double loop_offset_ = 0;
  double last_relative_time_ = 0;
  std::size_t frame_index_ = 0;
  std::chrono::steady_clock::time_point recording_base_;
  std::vector<RecordedPose> poses_;
  std::optional<Eigen::Quaterniond> current_attitude_;
};

}  // namespace io

#endif  // IO__VIDEO_HPP
