#!/usr/bin/env python3
"""UDP JSON -> rviz2 MarkerArray 桥接节点。

接收 standard.cpp 通过 UDP(9871) 发来的整车估计与火控状态：
  - 四块装甲板（绿色立方体）、旋转中心（青色球）、旋转轨迹（黄色圆环）
  - 火控选中的瞄准点：红色立方体（control 有效），开火时变橙色
  - 自身位姿：以 TF (world -> gimbal) 广播，rviz2 中以坐标轴显示

用法（conda ros2 环境下）:
    python scripts/rviz_bridge.py
"""

import json
import math
import socket

import rclpy
from geometry_msgs.msg import Point, TransformStamped
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

UDP_PORT = 9871
FRAME_ID = "world"

# 装甲板尺寸（米）：scale.x 为厚度，y 为宽度，z 为高度
ARMOR_SIZE = {
    "small": (0.02, 0.135, 0.125),
    "big": (0.02, 0.23, 0.127),
}

GREEN = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.9)
CYAN = ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.9)
YELLOW = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.6)
WHITE = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
RED = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.95)
ORANGE = ColorRGBA(r=1.0, g=0.6, b=0.0, a=0.95)


class RvizBridge(Node):
    def __init__(self):
        super().__init__("auto_aim_rviz_bridge")
        self.pub = self.create_publisher(MarkerArray, "/auto_aim/markers", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", UDP_PORT))
        self.sock.setblocking(False)

        self.timer = self.create_timer(0.01, self.poll)
        self.get_logger().info(f"listening on udp://127.0.0.1:{UDP_PORT}")

    def poll(self):
        data = None
        while True:  # 只保留最新一帧
            try:
                data, _ = self.sock.recvfrom(65535)
            except BlockingIOError:
                break
        if data is None:
            return
        try:
            msg = json.loads(data)
        except json.JSONDecodeError:
            return

        now = self.get_clock().now().to_msg()
        self.broadcast_self_tf(msg.get("self"), now)
        self.publish_markers(msg.get("targets", []), msg.get("aim"), now)

    def broadcast_self_tf(self, self_pose, now):
        """自身位姿：world 原点即机器人位置，姿态为云台四元数。"""
        if self_pose is None:
            return
        qx, qy, qz, qw = self_pose["q"]
        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = FRAME_ID
        tf.child_frame_id = "gimbal"
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf)

    def publish_markers(self, targets, aim, now):
        arr = MarkerArray()

        clear = Marker()
        clear.header.frame_id = FRAME_ID
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        mid = 0

        def new_marker(mtype):
            nonlocal mid
            m = Marker()
            m.header.frame_id = FRAME_ID
            m.header.stamp = now
            m.ns = "auto_aim"
            m.id = mid
            mid += 1
            m.type = mtype
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            return m

        def add_armor_cube(x, y, z, a, size, color, scale=1.0):
            m = new_marker(Marker.CUBE)
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
            m.pose.orientation.z = math.sin(a / 2.0)
            m.pose.orientation.w = math.cos(a / 2.0)
            m.scale.x, m.scale.y, m.scale.z = (s * scale for s in size)
            m.color = color
            arr.markers.append(m)

        for target in targets:
            cx, cy, cz = target["center"]
            radius = target["radius"]
            size = ARMOR_SIZE.get(target.get("armor_type", "small"), ARMOR_SIZE["small"])

            # 四块装甲板（绿色立方体，朝向为 EKF 估计的 yaw）
            for x, y, z, a in target["armors"]:
                add_armor_cube(x, y, z, a, size, GREEN)

            # 旋转中心（青色球）
            m = new_marker(Marker.SPHERE)
            m.pose.position.x, m.pose.position.y, m.pose.position.z = cx, cy, cz
            m.scale.x = m.scale.y = m.scale.z = 0.06
            m.color = CYAN
            arr.markers.append(m)

            # 旋转轨迹（黄色圆环）
            m = new_marker(Marker.LINE_STRIP)
            m.scale.x = 0.005
            m.color = YELLOW
            for i in range(73):
                theta = 2.0 * math.pi * i / 72.0
                m.points.append(Point(
                    x=cx + radius * math.cos(theta),
                    y=cy + radius * math.sin(theta),
                    z=cz))
            arr.markers.append(m)

            # 目标编号文字
            m = new_marker(Marker.TEXT_VIEW_FACING)
            m.pose.position.x, m.pose.position.y = cx, cy
            m.pose.position.z = cz + 0.25
            m.scale.z = 0.1
            m.color = WHITE
            m.text = f"{target.get('name', '?')} ({target.get('armor_type', '?')})"
            arr.markers.append(m)

        # 火控选中的瞄准点：control 有效标红，shoot 开火标橙；
        # 略微放大避免与绿色装甲板完全重叠导致闪烁
        if aim is not None and aim.get("control", False):
            x, y, z, a = aim["xyza"]
            size = ARMOR_SIZE.get(aim.get("armor_type", "small"), ARMOR_SIZE["small"])
            color = ORANGE if aim.get("shoot", False) else RED
            add_armor_cube(x, y, z, a, size, color, scale=1.15)

            # 自身到瞄准点的射击线
            m = new_marker(Marker.LINE_STRIP)
            m.scale.x = 0.004
            m.color = color
            m.points.append(Point(x=0.0, y=0.0, z=0.0))
            m.points.append(Point(x=x, y=y, z=z))
            arr.markers.append(m)

        self.pub.publish(arr)


def main():
    rclpy.init()
    node = RvizBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
