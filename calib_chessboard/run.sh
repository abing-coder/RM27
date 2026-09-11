#!/usr/bin/env bash
# 棋盘格一键标定：编译并运行 calib_chessboard
# 用法:
#   ./calib_chessboard/run.sh
#   ./calib_chessboard/run.sh /path/to/config.yaml
#   ./calib_chessboard/run.sh clean          # 清空采集图，保留结果 yaml
#   ./calib_chessboard/run.sh clean-all      # 采集图 + 标定结果都删
#   SKIP_BUILD=1 ./calib_chessboard/run.sh
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$DIR/.." && pwd)"

clean_data() {
  local n
  n="$(find "$DIR/data" -type f \( -name '*.jpg' -o -name '*.txt' -o -name 'capture_info.yaml' \) 2>/dev/null | wc -l)"
  rm -f "$DIR/data"/*.jpg "$DIR/data"/*.txt "$DIR/data/capture_info.yaml"
  echo "已清空采集数据 $DIR/data/（$n 个文件）"
}

if [[ "${1:-}" == "clean" ]]; then
  clean_data
  echo "结果仍在 $DIR/output/result.yaml ；重采请再跑 ./calib_chessboard/run.sh"
  exit 0
fi

if [[ "${1:-}" == "clean-all" ]]; then
  clean_data
  rm -f "$DIR/output/result.yaml"
  echo "已删除 $DIR/output/result.yaml"
  exit 0
fi

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  echo "用法: $0 [config.yaml | clean | clean-all]"
  exit 0
fi

CONFIG="${1:-$DIR/config.yaml}"

if [[ "$CONFIG" != /* ]]; then
  CONFIG="$(cd "$(dirname "$CONFIG")" && pwd)/$(basename "$CONFIG")"
fi

if [[ ! -f "$CONFIG" ]]; then
  echo "找不到配置: $CONFIG"
  echo "请先编辑 $DIR/config.yaml（棋盘格内角点、格距、use_imu）"
  exit 1
fi

cd "$REPO"

if [[ "${SKIP_BUILD:-0}" != "1" ]]; then
  if [[ ! -f "$REPO/build/CMakeCache.txt" ]]; then
    echo "==> cmake -B build"
    cmake -B build
  fi
  echo "==> 编译 calib_chessboard"
  cmake --build build --target calib_chessboard -j"$(nproc)"
fi

BIN="$REPO/build/calib_chessboard"
if [[ ! -x "$BIN" ]]; then
  echo "找不到可执行文件 $BIN ，请先成功编译"
  exit 1
fi

echo "==> 运行: $BIN $CONFIG"
exec "$BIN" "$CONFIG"
