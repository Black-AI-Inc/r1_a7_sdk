#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cmake -S "$script_dir/keyboard_native" -B "$script_dir/../build/keyboard" -DCMAKE_BUILD_TYPE=Release
cmake --build "$script_dir/../build/keyboard" -j4
