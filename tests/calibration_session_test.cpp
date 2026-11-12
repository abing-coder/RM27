#include "calib_chessboard/session.hpp"

#include <fstream>
#include <iostream>
#include <iterator>

namespace fs = std::filesystem;

void require(bool value, const char * message)
{
  if (!value) throw std::runtime_error(message);
}

void write(const fs::path & path, const std::string & value)
{
  std::ofstream out(path);
  out << value;
}

std::string read(const fs::path & path)
{
  std::ifstream in(path);
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

int main()
{
  const auto root = fs::temp_directory_path() /
    ("calibration_session_test_" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()));
  try {
    const auto data = root / "data_intrinsics";
    const auto output = root / "intrinsics.yaml";
    const auto config = root / "config.yaml";
    fs::create_directories(data);
    write(config, "config snapshot");
    write(data / "notes.txt", "unrelated file");
    require(chessboard::archive_session(data, output, config).empty(),
            "Empty sessions must not move unrelated files");

    write(data / "1.jpg", "first image");
    write(data / "3.jpg", "third image");
    write(data / "3.txt", "third pose");
    write(data / "9.txt", "orphan pose");
    write(data / "capture_info.yaml", "old metadata");
    write(output, "old result");
    require(chessboard::sample_ids(data) == std::vector<int>({1, 3}),
            "An index gap must not hide later images");
    require(chessboard::next_index(data) == 10,
            "An orphan pose must reserve its ID instead of being overwritten");

    const auto first = chessboard::archive_session(data, output, config);
    require(read(first / "data/1.jpg") == "first image", "Old image lost");
    require(read(first / "data/3.txt") == "third pose", "Paired pose lost");
    require(read(first / "data/9.txt") == "orphan pose", "Orphan pose lost");
    require(read(first / "data/capture_info.yaml") == "old metadata", "Metadata lost");
    require(read(first / "result.yaml") == "old result", "Result backup missing");
    require(read(first / "config.yaml") == "config snapshot", "Config snapshot missing");
    require(read(output) == "old result", "Active result must survive until a new solve succeeds");
    require(read(data / "notes.txt") == "unrelated file", "Unrelated file was moved");
    require(chessboard::sample_ids(data).empty() && chessboard::next_index(data) == 1,
            "New capture must start empty at ID 1");

    // A second run may start in the same second. It must preserve both batches.
    write(data / "1.jpg", "second batch image");
    const auto second = chessboard::archive_session(data, output, config);
    require(second != first, "Repeated runs must use distinct archive directories");
    require(read(first / "data/1.jpg") == "first image", "First batch overwritten");
    require(read(second / "data/1.jpg") == "second batch image", "Second batch lost");

    // If the result cannot be copied, capture files must not have been moved.
    write(data / "2.jpg", "keep on failure");
    fs::create_directory(root / "invalid_result");
    bool failed = false;
    try {
      chessboard::archive_session(data, root / "invalid_result", config);
    } catch (const fs::filesystem_error &) {
      failed = true;
    }
    require(failed && read(data / "2.jpg") == "keep on failure",
            "An archive failure must not consume the previous capture");
    fs::remove_all(root);
    std::cout << "Calibration session regression checks passed\n";
    return 0;
  } catch (const std::exception & e) {
    std::cerr << e.what() << "\nTest files retained at " << root << '\n';
    return 1;
  }
}
