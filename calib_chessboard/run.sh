#!/usr/bin/env bash
# 默认归档旧数据后重新采集；--resume 继续采集；--reuse 只重新计算。
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$DIR/.." && pwd)"

usage() {
  cat <<'HELP'
重新标内参（自动归档旧数据）: ./calib_chessboard/run.sh intra
重新标手眼（自动归档旧数据）: ./calib_chessboard/run.sh hand
继续上次采集:               ./calib_chessboard/run.sh intra --resume
仅使用已有图片重新计算:     ./calib_chessboard/run.sh intra --reuse
无窗口离线计算:             ./calib_chessboard/run.sh hand --reuse --headless
指定配置:                   ./calib_chessboard/run.sh intra /path/config.yaml

可选: --data-dir=目录 --output=结果.yaml --intrinsics=内参.yaml
旧数据归档到数据目录旁的 archive/，被替换的结果保留在 output/history/。
清空（直接删除）: ./calib_chessboard/run.sh clean intra|hand
SKIP_BUILD=1 跳过编译；CALIB_BUILD_DIR 可指定独立编译目录。
HELP
}

data_dir_of() {
  case "$1" in
    intra) echo "$DIR/data_intrinsics" ;;
    hand) echo "$DIR/data_handeye" ;;
    *) echo "未知模式: $1（只能是 intra 或 hand）" >&2; return 1 ;;
  esac
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" || "$#" == 0 ]]; then
  usage
  exit 0
fi

if [[ "$1" == "clean" ]]; then
  CLEAN_DIR="$(data_dir_of "${2:-}")"
  if [[ -d "$CLEAN_DIR" ]]; then
    find "$CLEAN_DIR" -maxdepth 1 -type f \
      \( -regex '.*/[1-9][0-9]*\.\(jpg\|txt\)' -o -name capture_info.yaml \) -delete
  fi
  echo "已清空 $CLEAN_DIR；归档和标定结果保留"
  exit 0
fi

MODE="$1"
shift
data_dir_of "$MODE" >/dev/null
CONFIG="$DIR/config.yaml"
SESSION=new
SESSION_SET=0
CONFIG_SET=0
ARGS=()
for arg in "$@"; do
  case "$arg" in
    --resume|--reuse|--new)
      if [[ "$SESSION_SET" == 1 ]]; then
        echo "--new、--resume、--reuse 只能选择一个" >&2
        exit 1
      fi
      SESSION="${arg#--}"
      SESSION_SET=1
      ;;
    --headless|--data-dir=*|--output=*|--intrinsics=*) ARGS+=("$arg") ;;
    -h|--help) usage; exit 0 ;;
    -*) echo "未知选项: $arg" >&2; exit 1 ;;
    *)
      if [[ "$CONFIG_SET" == 1 ]]; then
        echo "只能指定一份配置文件: $arg" >&2
        exit 1
      fi
      CONFIG="$arg"
      CONFIG_SET=1
      ;;
  esac
done

if [[ ! -f "$CONFIG" ]]; then
  echo "找不到配置: $CONFIG" >&2
  exit 1
fi
CONFIG="$(cd "$(dirname "$CONFIG")" && pwd)/$(basename "$CONFIG")"
cd "$REPO"

BUILD_DIR="${CALIB_BUILD_DIR:-$REPO/build/calib-tool}"
if [[ "$BUILD_DIR" != /* ]]; then BUILD_DIR="$REPO/$BUILD_DIR"; fi
if [[ "${SKIP_BUILD:-0}" != 1 ]]; then
  echo "==> 独立编译标定工具（无需 OpenVINO / ROS / CAN SDK）"
  cmake -S "$DIR" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release
  cmake --build "$BUILD_DIR" --target calib_chessboard -j"${BUILD_JOBS:-$(nproc)}"
fi

BIN="$BUILD_DIR/calib_chessboard"
if [[ ! -x "$BIN" ]]; then
  echo "找不到 $BIN，请去掉 SKIP_BUILD=1 先编译一次" >&2
  exit 1
fi

echo "==> 模式 $MODE，采集方式 $SESSION"
exec "$BIN" "$CONFIG" --mode="$MODE" --session="$SESSION" "${ARGS[@]}"
