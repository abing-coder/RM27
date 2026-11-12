#ifndef CALIB_CHESSBOARD_SESSION_HPP
#define CALIB_CHESSBOARD_SESSION_HPP

#include <algorithm>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <optional>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace chessboard
{
namespace fs = std::filesystem;

// Scan IDs instead of stopping at the first missing image. Orphan pose files also
// reserve their ID so that continuing a session never overwrites a previous pose.
inline std::optional<int> sample_id(const fs::path & path, bool include_poses = false)
{
  static const std::regex image(R"(^([1-9][0-9]*)\.jpg$)");
  static const std::regex sample(R"(^([1-9][0-9]*)\.(jpg|txt)$)");
  std::smatch match;
  const auto name = path.filename().string();
  if (!std::regex_match(name, match, include_poses ? sample : image)) return std::nullopt;
  const auto id = std::stoull(match[1].str());
  if (id >= static_cast<unsigned long long>(std::numeric_limits<int>::max()))
    throw std::runtime_error("标定样本编号过大: " + name);
  return static_cast<int>(id);
}

inline std::vector<int> sample_ids(const fs::path & dir)
{
  std::vector<int> ids;
  if (!fs::exists(dir)) return ids;
  for (const auto & entry : fs::directory_iterator(dir)) {
    if (!entry.is_regular_file()) continue;
    if (auto id = sample_id(entry.path())) ids.push_back(*id);
  }
  std::sort(ids.begin(), ids.end());
  return ids;
}

inline int next_index(const fs::path & dir)
{
  int next = 1;
  if (!fs::exists(dir)) return next;
  for (const auto & entry : fs::directory_iterator(dir)) {
    if (!entry.is_regular_file()) continue;
    if (auto id = sample_id(entry.path(), true)) next = std::max(next, *id + 1);
  }
  return next;
}

inline std::string timestamp()
{
  const auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::tm local{};
  localtime_r(&now, &local);
  std::ostringstream text;
  text << std::put_time(&local, "%Y%m%d_%H%M%S");
  return text.str();
}

// Move only known capture files. Unrelated files in a custom data_dir stay put.
// Copy the previous result first; leave the active result usable until a new
// calibration succeeds. If a move fails, roll back the moves already completed.
inline fs::path archive_session(
  const fs::path & dir, const fs::path & output, const fs::path & config)
{
  std::vector<fs::path> files;
  if (fs::exists(dir)) {
    for (const auto & entry : fs::directory_iterator(dir)) {
      if (!entry.is_regular_file()) continue;
      if (sample_id(entry.path(), true) || entry.path().filename() == "capture_info.yaml")
        files.push_back(entry.path());
    }
  }
  if (files.empty() && !fs::exists(output)) return {};

  const auto root = dir.parent_path() / "archive";
  fs::create_directories(root);
  const auto stem = dir.filename().string() + "_" + timestamp();
  fs::path archive;
  for (int suffix = 0;; ++suffix) {
    archive = root / (stem + (suffix ? "_" + std::to_string(suffix) : ""));
    if (fs::create_directory(archive)) break;
  }
  fs::create_directory(archive / "data");
  if (fs::exists(output)) fs::copy_file(output, archive / "result.yaml");
  if (fs::exists(config)) fs::copy_file(config, archive / "config.yaml");
  std::vector<fs::path> moved;
  try {
    for (const auto & path : files) {
      fs::rename(path, archive / "data" / path.filename());
      moved.push_back(path);
    }
  } catch (...) {
    for (auto it = moved.rbegin(); it != moved.rend(); ++it)
      fs::rename(archive / "data" / it->filename(), *it);
    throw;
  }
  return archive;
}
}  // namespace chessboard

#endif
