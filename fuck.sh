#!/usr/bin/env bash
set -eo pipefail

cd "$(dirname "$0")"

if [[ -n "${AUTOSTART_LOG:-}" ]]; then
	mkdir -p "${HOME}/.cache"
	LOG_FILE="${HOME}/.cache/sp_vision_autostart.log"
	# 追加写入，便于查看每次开机启动的完整日志
	exec >>"$LOG_FILE" 2>&1
	echo "===== $(date '+%F %T') sp_vision autostart ====="
	# 给图形会话/USB/CAN 一点初始化时间（.desktop 的 delay 在不同桌面环境不一定生效）
	sleep 5
fi

SCRIPT_PATH="$(readlink -f "$0")"
BIN_PATH="$(readlink -f ./build/standard)"
RUN_ARGS=(configs/demo.yaml -d)

cleanup() {
	local exit_code=$?
	trap - INT TERM EXIT
	if [[ -n "${CHILD_PID:-}" ]] && kill -0 "$CHILD_PID" 2>/dev/null; then
		echo "[cleanup] stopping process group -$CHILD_PID"
		kill -TERM -- "-$CHILD_PID" 2>/dev/null || kill -TERM "$CHILD_PID" 2>/dev/null || true
		wait "$CHILD_PID" 2>/dev/null || true
	fi
	exit "$exit_code"
}

trap cleanup INT TERM EXIT

echo "[check] cleanup stale script instances..."
mapfile -t STALE_SCRIPT_PIDS < <(pgrep -f "$SCRIPT_PATH" || true)
for pid in "${STALE_SCRIPT_PIDS[@]}"; do
	if [[ "$pid" != "$$" ]]; then
		echo "[check] kill stale script pid=$pid"
		kill -TERM "$pid" 2>/dev/null || true
	fi
done

if [[ -n "${BIN_PATH}" ]]; then
	echo "[check] cleanup stale standard process: $BIN_PATH ${RUN_ARGS[*]}"
	pkill -TERM -f "$BIN_PATH ${RUN_ARGS[*]}" 2>/dev/null || true
fi

source /opt/intel/openvino_2026/setupvars.sh


export DM_CAN_DEVICE_INDEX="${DM_CAN_DEVICE_INDEX:-2}"

cmake --build build --target standard -- -j"$(nproc)"

if [[ ! -s ./build/standard ]]; then
	echo "[warn] ./build/standard is empty, force relink..."
	rm -f ./build/standard
	cmake --build build --target standard -- -j"$(nproc)"
fi

echo "[run] DM_CAN_DEVICE_INDEX=${DM_CAN_DEVICE_INDEX}"
echo "[run] ./build/standard configs/demo.yaml -d"
setsid ./build/standard "${RUN_ARGS[@]}" &
CHILD_PID=$!
wait "$CHILD_PID"
