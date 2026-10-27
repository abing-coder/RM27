#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

#include "io/camera.hpp"
#include "io/video/video.hpp"
#include "tools/math_tools.hpp"
#include "tools/vision_config.hpp"

namespace
{
void require(bool condition, const char * message)
{
  if (!condition) throw std::runtime_error(message);
}

struct TempDirectory
{
  std::filesystem::path path = std::filesystem::temp_directory_path() /
    ("rm27_video_test_" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()));
  TempDirectory() { std::filesystem::create_directory(path); }
  ~TempDirectory()
  {
    std::error_code error;
    std::filesystem::remove_all(path, error);
  }
};

void test_calibration_profile(const std::filesystem::path & folder)
{
  const std::vector<double> live_install{0, 0, -1, -1, 0, 0, 0, 1, 0};
  const std::vector<double> recorded_install{-1, 0, 0, 0, -1, 0, 0, 0, 1};
  YAML::Node profile;
  profile["R_gimbal2imubody"] = recorded_install;
  profile["camera_matrix"] = std::vector<double>{600, 0, 320, 0, 600, 240, 0, 0, 1};
  profile["distort_coeffs"] = std::vector<double>{0, 0, 0, 0, 0};
  profile["R_camera2gimbal"] = std::vector<double>{0, 0, 1, -1, 0, 0, 0, -1, 0};
  profile["t_camera2gimbal"] = std::vector<double>{0.1, 0, 0.05};
  profile["auto_fire"] = false;
  profile["yaw_offset"] = 999;
  const auto profile_path = folder / "calibration.yaml";
  { std::ofstream out(profile_path); out << profile; }

  YAML::Node config;
  config["camera_name"] = "video";
  config["video_calibration_path"] = profile_path.string();
  config["R_gimbal2imubody"] = live_install;
  config["camera_matrix"] = std::vector<double>{900, 0, 320, 0, 900, 240, 0, 0, 1};
  config["auto_fire"] = true;
  config["yaw_offset"] = 1.23;
  const auto config_path = folder / "vision.yaml";
  auto save = [&]() { std::ofstream out(config_path); out << config; };
  save();
  const auto replay = tools::load_vision_config(config_path.string());
  require(replay["R_gimbal2imubody"].as<std::vector<double>>() == recorded_install,
    "replay used the live IMU installation");
  require(replay["camera_matrix"].as<std::vector<double>>()[0] == 600,
    "replay used the live camera intrinsics");
  require(replay["auto_fire"].as<bool>() && replay["yaw_offset"].as<double>() == 1.23,
    "calibration profile replaced unrelated control settings");
  require(YAML::LoadFile(config_path.string())["R_gimbal2imubody"].as<std::vector<double>>() == live_install,
    "loading a recording profile overwrote the live calibration file");

  // 原版 demo 第 210 帧：误用 DM-IMU 安装矩阵会产生截图中的约 78.58 度 roll。
  const Eigen::Quaterniond q(
    0.2746909690270775, -0.09632564400615011, 0.02518388503298312, 0.9563639546618281);
  const Eigen::Matrix3d installation =
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(recorded_install.data());
  const auto ypr = tools::eulers(q.normalized().toRotationMatrix() * installation, 2, 1, 0);
  require(std::abs(ypr[2]) < std::acos(-1.0) / 180.0,
    "recorded frame was rolled sideways by the live installation");

  config["camera_name"] = "hikrobot";
  config["video_calibration_path"] = (folder / "does_not_exist.yaml").string();
  save();
  const auto live = tools::load_vision_config(config_path.string());
  require(live["R_gimbal2imubody"].as<std::vector<double>>() == live_install &&
    live["camera_matrix"].as<std::vector<double>>()[0] == 900,
    "live camera inherited video calibration");

  config["camera_name"] = "video";
  config["video_calibration_path"] = profile_path.string();
  save();
  profile.remove("R_camera2gimbal");
  { std::ofstream out(profile_path); out << profile; }
  bool rejected = false;
  try { tools::load_vision_config(config_path.string()); }
  catch (const std::exception &) { rejected = true; }
  require(rejected, "incomplete recording calibration was mixed with live parameters");
}

void test_replay(const std::filesystem::path & folder)
{
  const auto clip = folder / "recorded.avi";
  {
    cv::VideoWriter writer(clip.string(), cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 25, {32, 24});
    require(writer.isOpened(), "cannot create test video");
    for (int i = 0; i < 3; ++i) writer.write(cv::Mat(24, 32, CV_8UC3, cv::Scalar(i * 80, 0, 0)));
  }
  {
    std::ofstream poses(folder / "recorded.txt");
    poses << std::setprecision(17);
    const double times[] = {10.0, 10.02, 10.06};
    for (int i = 0; i < 3; ++i) {
      poses << times[i] << ' ' << std::cos(i * 0.05) << " 0 0 " << std::sin(i * 0.05) << '\n';
    }
  }
  // 播放速度与录制速度刻意不同，算法时间仍须来自配套文件。
  io::VideoCamera camera(clip.string(), 1000, true);
  cv::Mat image;
  std::chrono::steady_clock::time_point stamps[4];
  for (int i = 0; i < 4; ++i) {
    camera.read(image, stamps[i]);
    require(!image.empty(), "replay unexpectedly ended");
    const auto q = camera.recorded_attitude();
    require(q.has_value(), "paired attitude was not loaded");
    const int frame = i % 3;
    const Eigen::Quaterniond expected(Eigen::AngleAxisd(frame * 0.1, Eigen::Vector3d::UnitZ()));
    require(q->angularDistance(expected) < 1e-10, "attitude and video frame became misaligned");
    require(std::abs(cv::mean(image)[0] - frame * 80) < 5, "video loop did not rewind to first frame");
  }
  require(std::abs(std::chrono::duration<double>(stamps[1] - stamps[0]).count() - 0.02) < 1e-6,
    "playback speed changed the recorded time interval");
  require(std::abs(std::chrono::duration<double>(stamps[2] - stamps[1]).count() - 0.04) < 1e-6,
    "variable recorded time intervals were discarded");
  require(stamps[3] > stamps[2], "loop made timestamps go backward");

  io::VideoCamera finite(clip.string(), 1000, false);
  for (int i = 0; i < 3; ++i) finite.read(image, stamps[0]);
  finite.read(image, stamps[0]);
  require(image.empty() && !finite.recorded_attitude(), "EOF retained an image or stale attitude");

  const auto plain_clip = folder / "plain.avi";
  std::filesystem::copy_file(clip, plain_clip);
  io::VideoCamera plain(plain_clip.string(), 1000, false);
  plain.read(image, stamps[0]);
  plain.read(image, stamps[1]);
  require(!plain.recorded_attitude(), "unpaired video invented recorded attitude");
  require(std::abs(std::chrono::duration<double>(stamps[1] - stamps[0]).count() - 0.04) < 1e-6,
    "unpaired video did not use the source frame rate");

  YAML::Node yaml;
  yaml["camera_name"] = "video";
  yaml["video_path"] = clip.string();
  yaml["video_fps"] = 1000;
  yaml["video_loop"] = false;
  const auto config = folder / "camera.yaml";
  { std::ofstream out(config); out << yaml; }
  io::Camera wrapped(config.string());
  wrapped.read(image, stamps[0]);
  require(wrapped.recorded_attitude().has_value(), "Camera wrapper hid the recorded attitude");

  const auto bad_pose = folder / "bad.txt";
  { std::ofstream out(bad_pose); out << "1 1 0 0 0\n0 1 0 0 0\n"; }
  bool rejected = false;
  try { io::VideoCamera invalid(clip.string(), 1000, false, bad_pose.string()); }
  catch (const std::exception &) { rejected = true; }
  require(rejected, "nonmonotonic recorded timestamps were accepted");

  const auto short_pose = folder / "short.txt";
  { std::ofstream out(short_pose); out << "1 1 0 0 0\n"; }
  io::VideoCamera short_data(clip.string(), 1000, false, short_pose.string());
  short_data.read(image, stamps[0]);
  short_data.read(image, stamps[0]);
  require(image.empty() && !short_data.recorded_attitude(),
    "unpaired video tail silently fell back to a fake pose");
  io::VideoCamera short_loop(clip.string(), 1000, true, short_pose.string());
  short_loop.read(image, stamps[0]);
  short_loop.read(image, stamps[1]);
  require(!image.empty() && cv::mean(image)[0] < 5 && stamps[1] > stamps[0],
    "short pose stream did not rewind the image and clock together");
}
}  // namespace

int main()
{
  try {
    TempDirectory folder;
    test_replay(folder.path);
    test_calibration_profile(folder.path);
    std::cout << "video replay timing, attitude, calibration, loop and EOF checks passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
