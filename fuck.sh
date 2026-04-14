#!/usr/bin/env bash
set -eo pipefail

cd "$(dirname "$0")"

./build/standard configs/demo.yaml -d
