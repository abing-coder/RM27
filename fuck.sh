#!/usr/bin/env bash
set -eo pipefail

cd "$(dirname "$0")"

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
exec ./build/standard configs/demo.yaml -d
