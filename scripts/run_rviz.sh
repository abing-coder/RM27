#!/usr/bin/env bash
# 一键启动 rviz2 可视化：桥接节点 + rviz2 + 自瞄主程序。
# 用法:
#   ./scripts/run_rviz.sh                        # 默认 configs/demo.yaml，带显示窗口
#   ./scripts/run_rviz.sh configs/standard3.yaml # 指定配置
#   NO_STANDARD=1 ./scripts/run_rviz.sh          # 只开桥接和 rviz2，主程序自己另开
set -eo pipefail

cd "$(dirname "$0")/.."

CONFIG="${1:-configs/demo.yaml}"
ROS_ENV="${ROS_ENV:-ros2}"

pids=()
cleanup() {
  for pid in "${pids[@]}"; do
    kill "$pid" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "[run_rviz] 启动桥接节点 (conda env: $ROS_ENV) ..."
conda run --no-capture-output -n "$ROS_ENV" python scripts/rviz_bridge.py &
pids+=($!)

echo "[run_rviz] 启动 rviz2 ..."
conda run --no-capture-output -n "$ROS_ENV" rviz2 -d scripts/auto_aim.rviz &
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
