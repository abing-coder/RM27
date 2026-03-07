#!/usr/bin/env bash
set -e

cd "$(dirname "$0")"
make -C build -j$(nproc)
./build/standard configs/demo.yaml -d
