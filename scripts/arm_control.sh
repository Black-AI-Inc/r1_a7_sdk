#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
sdk_dir="$(cd -- "$script_dir/.." && pwd)"
if [[ -n "${R1_IK_PYTHON:-}" ]]; then
    python_bin="$R1_IK_PYTHON"
elif [[ -x "$sdk_dir/.venv-ik/bin/python" ]]; then
    python_bin="$sdk_dir/.venv-ik/bin/python"
else
    python_bin=python3
fi
exec "$python_bin" "$script_dir/arm_control.py" "$@"
