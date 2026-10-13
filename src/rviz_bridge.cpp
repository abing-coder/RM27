// UDP JSON -> rviz2 MarkerArray 桥接节点（C++ 版，Ubuntu + ROS2）。
//
// 接收 standard.cpp 通过 UDP(9871) 发来的整车估计与火控状态：
//   - 四块装甲板（绿色立方体）、旋转中心（青色球）、旋转轨迹（黄色圆环）
//   - 火控选中的瞄准点：红色立方体（control 有效），开火时变橙色
//   - 自身位姿：以 TF (world -> gimbal) 广播，rviz2 中以坐标轴显示
//   - 相机 (gimbal -> camera) 与虚拟枪口 (gimbal -> barrel)，枪口另画一条品红射线
//
// 用法（已 source /opt/ros/<distro>/setup.bash）:
//   ./build/rviz_bridge
// 或直接使用一键脚本 scripts/run_rviz.sh。

#include <arpa/inet.h>   // htons, inet_addr
#include <fcntl.h>       // fcntl, O_NONBLOCK
#include <netinet/in.h>  // sockaddr_in
#include <sys/socket.h>  // socket, bind, recvfrom
#include <unistd.h>      // close

#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <map>
#include <memory>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace
{
using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;
using ColorRGBA = std_msgs::msg::ColorRGBA;
using Point = geometry_msgs::msg::Point;

constexpr uint16_t UDP_PORT = 9871;
const char * const UDP_HOST = "127.0.0.1";
const char * const FRAME_ID = "world";

// 装甲板尺寸（米）：x 为厚度，y 为宽度，z 为高度
struct Size3
{
  double x, y, z;
};
const std::map<std::string, Size3> ARMOR_SIZE = {
  {"small", {0.02, 0.135, 0.125}},
  {"big", {0.02, 0.23, 0.127}},
};

ColorRGBA rgba(float r, float g, float b, float a)
{
  ColorRGBA c;
  c.r = r;
  c.g = g;
  c.b = b;
  c.a = a;
  return c;
}

const ColorRGBA GREEN = rgba(0.0f, 1.0f, 0.0f, 0.9f);
const ColorRGBA CYAN = rgba(0.0f, 1.0f, 1.0f, 0.9f);
const ColorRGBA YELLOW = rgba(1.0f, 1.0f, 0.0f, 0.6f);
const ColorRGBA WHITE = rgba(1.0f, 1.0f, 1.0f, 0.9f);
const ColorRGBA RED = rgba(1.0f, 0.0f, 0.0f, 0.95f);
const ColorRGBA ORANGE = rgba(1.0f, 0.6f, 0.0f, 0.95f);
const ColorRGBA MAGENTA = rgba(1.0f, 0.0f, 1.0f, 0.9f);

constexpr double BARREL_RAY_LENGTH = 6.0;  // m

Point make_point(double x, double y, double z)
{
  Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

const Size3 & armor_size_of(const nlohmann::json & j, const char * key)
{
  std::string type = "small";
  if (j.contains(key) && j[key].is_string()) type = j[key].get<std::string>();
  auto it = ARMOR_SIZE.find(type);
  return it != ARMOR_SIZE.end() ? it->second : ARMOR_SIZE.at("small");
}

std::string string_or(const nlohmann::json & j, const char * key, const std::string & fallback)
{
  if (j.contains(key) && j[key].is_string()) return j[key].get<std::string>();
  return fallback;
}

bool bool_or(const nlohmann::json & j, const char * key, bool fallback)
{
  if (j.contains(key) && j[key].is_boolean()) return j[key].get<bool>();
  return fallback;
}

}  // namespace

class RvizBridge : public rclcpp::Node
{
public:
  RvizBridge() : Node("auto_aim_rviz_bridge")
  {
    pub_ = create_publisher<MarkerArray>("/auto_aim/markers", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_ < 0) throw std::runtime_error(std::string("socket: ") + std::strerror(errno));

    int reuse = 1;
    ::setsockopt(socket_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(UDP_PORT);
    addr.sin_addr.s_addr = ::inet_addr(UDP_HOST);
    if (::bind(socket_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
      std::string err = std::strerror(errno);
      ::close(socket_);
      socket_ = -1;
      throw std::runtime_error("bind udp://" + std::string(UDP_HOST) + ":" + std::to_string(UDP_PORT) + " failed: " + err);
    }

    int flags = ::fcntl(socket_, F_GETFL, 0);
    ::fcntl(socket_, F_SETFL, flags | O_NONBLOCK);

    timer_ = create_wall_timer(std::chrono::milliseconds(10), std::bind(&RvizBridge::poll, this));
    RCLCPP_INFO(get_logger(), "listening on udp://%s:%u", UDP_HOST, UDP_PORT);
  }

  ~RvizBridge() override
  {
    if (socket_ >= 0) ::close(socket_);
  }

private:
  void poll()
  {
    // 只保留最新一帧
    bool got = false;
    ssize_t len = 0;
    while (true) {
      ssize_t n = ::recvfrom(socket_, buffer_.data(), buffer_.size(), 0, nullptr, nullptr);
      if (n < 0) break;  // EAGAIN/EWOULDBLOCK: 没有更多数据
      got = true;
      len = n;
    }
    if (!got) return;

    nlohmann::json msg;
    try {
      msg = nlohmann::json::parse(buffer_.data(), buffer_.data() + len);
    } catch (const nlohmann::json::parse_error &) {
      return;
    }
    if (!msg.is_object()) return;

    auto now = get_clock()->now();
    // world 原点即云台旋转中心；camera、barrel 挂在 gimbal 下
    if (msg.contains("self")) broadcast_tf(FRAME_ID, "gimbal", msg["self"], now);
    if (msg.contains("camera")) broadcast_tf("gimbal", "camera", msg["camera"], now);
    const bool has_barrel = msg.contains("barrel") && broadcast_tf("gimbal", "barrel", msg["barrel"], now);
    static const nlohmann::json empty_targets = nlohmann::json::array();
    const nlohmann::json & targets = msg.contains("targets") ? msg["targets"] : empty_targets;
    const nlohmann::json * aim = msg.contains("aim") && !msg["aim"].is_null() ? &msg["aim"] : nullptr;
    publish_markers(targets, aim, has_barrel, now);
  }

  // pose: {"q": [x, y, z, w], "t": [x, y, z]}，t 可省略
  bool broadcast_tf(
    const std::string & parent, const std::string & child, const nlohmann::json & pose,
    const rclcpp::Time & now)
  {
    if (!pose.is_object() || !pose.contains("q") || !pose["q"].is_array() || pose["q"].size() < 4)
      return false;
    const auto & q = pose["q"];

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now;
    tf.header.frame_id = parent;
    tf.child_frame_id = child;
    if (pose.contains("t") && pose["t"].is_array() && pose["t"].size() >= 3) {
      tf.transform.translation.x = pose["t"][0].get<double>();
      tf.transform.translation.y = pose["t"][1].get<double>();
      tf.transform.translation.z = pose["t"][2].get<double>();
    }
    tf.transform.rotation.x = q[0].get<double>();
    tf.transform.rotation.y = q[1].get<double>();
    tf.transform.rotation.z = q[2].get<double>();
    tf.transform.rotation.w = q[3].get<double>();
    tf_broadcaster_->sendTransform(tf);
    return true;
  }

  void publish_markers(
    const nlohmann::json & targets, const nlohmann::json * aim, bool has_barrel,
    const rclcpp::Time & now)
  {
    MarkerArray arr;

    Marker clear;
    clear.header.frame_id = FRAME_ID;
    clear.action = Marker::DELETEALL;
    arr.markers.push_back(clear);

    int mid = 0;

    auto new_marker = [&](int32_t mtype) {
      Marker m;
      m.header.frame_id = FRAME_ID;
      m.header.stamp = now;
      m.ns = "auto_aim";
      m.id = mid++;
      m.type = mtype;
      m.action = Marker::ADD;
      m.pose.orientation.w = 1.0;
      return m;
    };

    auto add_armor_cube = [&](
                            double x, double y, double z, double a, const Size3 & size,
                            const ColorRGBA & color, double scale = 1.0) {
      Marker m = new_marker(Marker::CUBE);
      m.pose.position.x = x;
      m.pose.position.y = y;
      m.pose.position.z = z;
      m.pose.orientation.z = std::sin(a / 2.0);
      m.pose.orientation.w = std::cos(a / 2.0);
      m.scale.x = size.x * scale;
      m.scale.y = size.y * scale;
      m.scale.z = size.z * scale;
      m.color = color;
      arr.markers.push_back(m);
    };

    if (targets.is_array()) {
      for (const auto & target : targets) {
        if (!target.contains("center") || !target.contains("radius") || !target.contains("armors"))
          continue;
        const auto & center = target["center"];
        if (!center.is_array() || center.size() < 3) continue;
        const double cx = center[0].get<double>();
        const double cy = center[1].get<double>();
        const double cz = center[2].get<double>();
        const double radius = target["radius"].get<double>();
        const Size3 & size = armor_size_of(target, "armor_type");

        // 四块装甲板（绿色立方体，朝向为 EKF 估计的 yaw）
        for (const auto & armor : target["armors"]) {
          if (!armor.is_array() || armor.size() < 4) continue;
          add_armor_cube(
            armor[0].get<double>(), armor[1].get<double>(), armor[2].get<double>(),
            armor[3].get<double>(), size, GREEN);
        }

        // 旋转中心（青色球）
        {
          Marker m = new_marker(Marker::SPHERE);
          m.pose.position.x = cx;
          m.pose.position.y = cy;
          m.pose.position.z = cz;
          m.scale.x = m.scale.y = m.scale.z = 0.06;
          m.color = CYAN;
          arr.markers.push_back(m);
        }

        // 旋转轨迹（黄色圆环）
        {
          Marker m = new_marker(Marker::LINE_STRIP);
          m.scale.x = 0.005;
          m.color = YELLOW;
          for (int i = 0; i < 73; ++i) {
            const double theta = 2.0 * M_PI * i / 72.0;
            m.points.push_back(
              make_point(cx + radius * std::cos(theta), cy + radius * std::sin(theta), cz));
          }
          arr.markers.push_back(m);
        }

        // 目标编号文字
        {
          Marker m = new_marker(Marker::TEXT_VIEW_FACING);
          m.pose.position.x = cx;
          m.pose.position.y = cy;
          m.pose.position.z = cz + 0.25;
          m.scale.z = 0.1;
          m.color = WHITE;
          m.text = string_or(target, "name", "?") + " (" + string_or(target, "armor_type", "?") + ")";
          arr.markers.push_back(m);
        }
      }
    }

    // 火控选中的瞄准点：control 有效标红，shoot 开火标橙；
    // 略微放大避免与绿色装甲板完全重叠导致闪烁
    if (aim != nullptr && bool_or(*aim, "control", false) && aim->contains("xyza") &&
        (*aim)["xyza"].is_array() && (*aim)["xyza"].size() >= 4) {
      const auto & xyza = (*aim)["xyza"];
      const double x = xyza[0].get<double>();
      const double y = xyza[1].get<double>();
      const double z = xyza[2].get<double>();
      const double a = xyza[3].get<double>();
      const Size3 & size = armor_size_of(*aim, "armor_type");
      const ColorRGBA & color = bool_or(*aim, "shoot", false) ? ORANGE : RED;
      add_armor_cube(x, y, z, a, size, color, 1.15);

      // 自身到瞄准点的射击线
      Marker m = new_marker(Marker::LINE_STRIP);
      m.scale.x = 0.004;
      m.color = color;
      m.points.push_back(make_point(0.0, 0.0, 0.0));
      m.points.push_back(make_point(x, y, z));
      arr.markers.push_back(m);
    }

    // 虚拟枪口指向：沿 barrel 的 X 轴画长射线，补偿角只有一两度，坐标轴本身看不出差别
    if (has_barrel) {
      Marker m = new_marker(Marker::LINE_STRIP);
      m.header.frame_id = "barrel";
      m.scale.x = 0.006;
      m.color = MAGENTA;
      m.points.push_back(make_point(0.0, 0.0, 0.0));
      m.points.push_back(make_point(BARREL_RAY_LENGTH, 0.0, 0.0));
      arr.markers.push_back(m);
    }

    pub_->publish(arr);
  }

  rclcpp::Publisher<MarkerArray>::SharedPtr pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  int socket_ = -1;
  std::array<char, 65535> buffer_{};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  int ret = 0;
  try {
    auto node = std::make_shared<RvizBridge>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("auto_aim_rviz_bridge"), "%s", e.what());
    ret = 1;
  }
  rclcpp::shutdown();
  return ret;
}
