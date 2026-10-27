#!/usr/bin/env bash
# 棋盘格标定。内参和手眼分开跑，采集目录互不影响。
# 用法:
#   ./calib_chessboard/run.sh intra     # 只标内参：移动棋盘，不读 IMU
#   ./calib_chessboard/run.sh hand      # 只标手眼：固定棋盘，转动云台，读 USB IMU
#   ./calib_chessboard/run.sh clean intra
#   ./calib_chessboard/run.sh clean hand
#   SKIP_BUILD=1 ./calib_chessboard/run.sh intra
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$DIR/.." && pwd)"

data_dir_of() {
  case "$1" in
    intra) echo "$DIR/data_intrinsics" ;;
    hand) echo "$DIR/data_handeye" ;;
    *) echo "未知模式: $1（只能是 intra 或 hand）" >&2; return 1 ;;
  esac
}

clean_data() {
  local dir n
  dir="$(data_dir_of "$1")"
  n="$(find "$dir" -type f \( -name '*.jpg' -o -name '*.txt' -o -name 'capture_info.yaml' \) 2>/dev/null | wc -l)"
  rm -f "$dir"/*.jpg "$dir"/*.txt "$dir/capture_info.yaml"
  echo "已清空 $dir（$n 个文件）"
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" || "${1:-}" == "" ]]; then
  echo "内参: ./calib_chessboard/run.sh intra"
  echo "手眼: ./calib_chessboard/run.sh hand"
  echo "清空: ./calib_chessboard/run.sh clean intra|hand"
  exit 0
fi

if [[ "${1:-}" == "clean" ]]; then
  clean_data "${2:-}"
  exit 0
fi

MODE="$1"
data_dir_of "$MODE" >/dev/null
CONFIG="${2:-$DIR/config.yaml}"

if [[ "$CONFIG" != /* ]]; then
  CONFIG="$(cd "$(dirname "$CONFIG")" && pwd)/$(basename "$CONFIG")"
fi

if [[ ! -f "$CONFIG" ]]; then
  echo "找不到配置: $CONFIG"
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

echo "==> 运行: $BIN $CONFIG --mode=$MODE"
exec "$BIN" "$CONFIG" --mode="$MODE"
