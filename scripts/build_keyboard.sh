#!/usr/bin/env bash
# Extract the upstream URDF package if needed, then build the native worker
# and its unit tests into build/keyboard.
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
urdf_dir="$script_dir/../urdf"
if [[ ! -f "$urdf_dir/R1_Robotic_Arm_R_V1.2-1023/R1_Robotic_Arm_V1.2/R1_Robotic_Arm/urdf/R1_Robotic_Arm.urdf" ]]; then
    tar -xzf "$urdf_dir/URDF_R1-7A.tar.gz" -C "$urdf_dir"
fi
cmake -S "$script_dir/keyboard_native" -B "$script_dir/../build/keyboard" -DCMAKE_BUILD_TYPE=Release
cmake --build "$script_dir/../build/keyboard" -j4
