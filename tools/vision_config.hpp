#ifndef TOOLS__VISION_CONFIG_HPP
#define TOOLS__VISION_CONFIG_HPP

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace tools
{
// 录像使用录制设备的内外参；实机始终保留主配置中的标定。
// 只覆盖几何字段，不修改文件，也不覆盖角度补偿、敌方颜色或火控设置。
inline YAML::Node load_vision_config(const std::string & path)
{
  auto config = YAML::LoadFile(path);
  if (!config["camera_name"] || config["camera_name"].as<std::string>() != "video" ||
      !config["video_calibration_path"]) return config;
  const auto profile_path = config["video_calibration_path"].as<std::string>();
  if (profile_path.empty()) return config;
  const auto profile = YAML::LoadFile(profile_path);
  const std::vector<std::pair<std::string, std::size_t>> fields{
    {"camera_matrix", 9}, {"distort_coeffs", 5}, {"R_camera2gimbal", 9},
    {"t_camera2gimbal", 3}, {"R_gimbal2imubody", 9}};
  for (const auto & field : fields) {
    if (!profile[field.first]) {
      throw std::runtime_error("Video calibration missing " + field.first + ": " + profile_path);
    }
    const auto values = profile[field.first].as<std::vector<double>>();
    if (values.size() != field.second) {
      throw std::runtime_error("Video calibration has invalid " + field.first + ": " + profile_path);
    }
    for (double value : values) {
      if (!std::isfinite(value)) {
        throw std::runtime_error("Video calibration has nonfinite " + field.first + ": " + profile_path);
      }
    }
    config[field.first] = profile[field.first];
  }
  return config;
}
}  // namespace tools

#endif  // TOOLS__VISION_CONFIG_HPP
