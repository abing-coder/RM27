#!/usr/bin/env bash
# 一键启动 rviz2 可视化（Ubuntu + ROS2）：C++ 桥接节点 + rviz2 + 自瞄主程序。
# 用法:
#   ./scripts/run_rviz.sh                        # 默认 configs/demo.yaml，带显示窗口
#   ./scripts/run_rviz.sh configs/standard3.yaml # 指定配置
#   NO_STANDARD=1 ./scripts/run_rviz.sh          # 只开桥接和 rviz2，主程序自己另开
#   ROS_DISTRO=humble ./scripts/run_rviz.sh      # 指定 ROS2 发行版（默认 humble）
set -eo pipefail

cd "$(dirname "$0")/.."

CONFIG="${1:-configs/demo.yaml}"

# 未 source ROS2 环境时自动加载 /opt/ros/<distro>/setup.bash
if ! command -v ros2 >/dev/null 2>&1; then
  ROS_SETUP="/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
  if [ -f "$ROS_SETUP" ]; then
    echo "[run_rviz] source $ROS_SETUP"
    # shellcheck disable=SC1090
    source "$ROS_SETUP"
  else
    echo "[run_rviz] 未找到 ROS2 环境: $ROS_SETUP，请先安装 ROS2 或设置 ROS_DISTRO" >&2
    exit 1
  fi
fi

pids=()
cleanup() {
  for pid in "${pids[@]}"; do
    kill "$pid" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if [ ! -x build/rviz_bridge ]; then
  echo "[run_rviz] 未找到 build/rviz_bridge，先编译: cmake --build build --target rviz_bridge -j8"
  [ -f build/CMakeCache.txt ] || cmake -B build
  cmake --build build --target rviz_bridge -j8
fi

echo "[run_rviz] 启动桥接节点 ./build/rviz_bridge ..."
./build/rviz_bridge &
pids+=($!)

echo "[run_rviz] 启动 rviz2 ..."
rviz2 -d scripts/auto_aim.rviz &
pids+=($!)

if [ -n "$NO_STANDARD" ]; then
  echo "[run_rviz] NO_STANDARD=1，主程序请自行启动，例如: ./build/standard $CONFIG -d"
  echo "[run_rviz] Ctrl+C 退出"
  wait
else
  if [ ! -x build/standard ]; then
    echo "[run_rviz] 未找到 build/standard，先编译: cmake --build build --target standard -j8"
    cmake --build build --target standard -j8
  fi
  echo "[run_rviz] 启动主程序: ./build/standard $CONFIG -d"
  ./build/standard "$CONFIG" -d
fi
