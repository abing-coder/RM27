#include "curve_plot.hpp"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace tools
{
namespace
{
const cv::Scalar RED{0, 0, 255};
const cv::Scalar GREEN{0, 255, 0};
const cv::Scalar GRID{60, 60, 60};
const cv::Scalar TEXT{220, 220, 220};

void put_text(cv::Mat & img, const std::string & text, cv::Point org, const cv::Scalar & color)
{
  cv::putText(img, text, org, cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 1, cv::LINE_AA);
}
}  // namespace

CurvePlot::CurvePlot(std::string title, std::string unit, double window_s, double min_span)
: title_(std::move(title)), unit_(std::move(unit)), window_s_(window_s), min_span_(min_span)
{
}

void CurvePlot::push(double t, std::optional<double> red, std::optional<double> green)
{
  samples_.push_back({t, red, green});
  while (samples_.front().t < t - window_s_) samples_.pop_front();
}

void CurvePlot::draw(cv::Mat & img, const cv::Rect & area) const
{
  cv::rectangle(img, area, {25, 25, 25}, cv::FILLED);
  const cv::Rect plot(area.x + 80, area.y + 36, area.width - 100, area.height - 64);
  cv::rectangle(img, plot, {90, 90, 90}, 1);

  // 标题行：最新的预测值、检测值及两者之差
  std::optional<double> red, green;
  for (auto it = samples_.rbegin(); it != samples_.rend() && !(red && green); ++it) {
    if (!red) red = it->red;
    if (!green) green = it->green;
  }
  std::string header = title_ + " (" + unit_ + ")";
  header += "   pred " + (red ? cv::format("%+.2f", *red) : std::string("--"));
  header += "   det " + (green ? cv::format("%+.2f", *green) : std::string("--"));
  if (red && green) header += cv::format("   err %+.2f", *red - *green);
  put_text(img, header, {area.x + 10, area.y + 24}, TEXT);
  put_text(img, "red: prev pred", {area.x + area.width - 300, area.y + 24}, RED);
  put_text(img, "green: det", {area.x + area.width - 120, area.y + 24}, GREEN);

  if (samples_.empty()) return;
  double lo = 1e9, hi = -1e9;
  for (const auto & s : samples_) {
    for (const auto & v : {s.red, s.green}) {
      if (!v) continue;
      lo = std::min(lo, *v);
      hi = std::max(hi, *v);
    }
  }
  if (lo > hi) return;
  const double mid = (lo + hi) / 2;
  const double span = std::max(hi - lo, min_span_) * 1.2;
  lo = mid - span / 2;
  hi = mid + span / 2;

  const double t1 = samples_.back().t;
  const double t0 = t1 - window_s_;
  auto to_px = [&](double t, double v) {
    return cv::Point(
      plot.x + cvRound((t - t0) / window_s_ * plot.width),
      plot.y + cvRound((hi - v) / (hi - lo) * plot.height));
  };

  for (int i = 0; i <= 4; ++i) {
    const double v = lo + (hi - lo) * i / 4;
    const int y = to_px(t0, v).y;
    cv::line(img, {plot.x, y}, {plot.x + plot.width, y}, GRID, 1);
    put_text(img, cv::format("%.2f", v), {area.x + 6, y + 5}, TEXT);
  }
  for (double s = std::ceil(t0); s <= t1; s += 1.0) {
    const int x = to_px(s, lo).x;
    cv::line(img, {x, plot.y}, {x, plot.y + plot.height}, GRID, 1);
    put_text(img, cv::format("%.0fs", s - t1), {x - 14, plot.y + plot.height + 22}, TEXT);
  }

  auto draw_series = [&](std::optional<double> Sample::*value, const cv::Scalar & color, int thickness) {
    std::vector<cv::Point> segment;
    auto flush = [&] {
      if (segment.size() == 1) cv::circle(img, segment[0], thickness, color, cv::FILLED);
      if (segment.size() >= 2) cv::polylines(img, segment, false, color, thickness, cv::LINE_AA);
      segment.clear();
    };
    for (const auto & s : samples_) {
      if (!(s.*value)) {
        flush();
        continue;
      }
      segment.push_back(to_px(s.t, *(s.*value)));
    }
    flush();
  };
  // 检测在下层画粗，预测在上层画细，两条重合时也都看得见
  draw_series(&Sample::green, GREEN, 3);
  draw_series(&Sample::red, RED, 1);
}

}  // namespace tools
