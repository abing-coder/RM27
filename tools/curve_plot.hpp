#ifndef TOOLS__CURVE_PLOT_HPP
#define TOOLS__CURVE_PLOT_HPP

#include <deque>
#include <opencv2/opencv.hpp>
#include <optional>
#include <string>

namespace tools
{
// 按时间滚动的双曲线：红色=预测，绿色=检测。某一时刻缺值时曲线在该处断开。
class CurvePlot
{
public:
  // window_s：显示最近多少秒；min_span：纵轴最小跨度，避免小噪声被放大
  CurvePlot(std::string title, std::string unit, double window_s = 4.0, double min_span = 1.0);

  void push(double t, std::optional<double> red, std::optional<double> green);
  void draw(cv::Mat & img, const cv::Rect & area) const;

private:
  struct Sample
  {
    double t;
    std::optional<double> red, green;
  };

  std::string title_, unit_;
  double window_s_, min_span_;
  std::deque<Sample> samples_;
};

}  // namespace tools

#endif  // TOOLS__CURVE_PLOT_HPP
