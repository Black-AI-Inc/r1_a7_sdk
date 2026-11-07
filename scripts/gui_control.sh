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
# Optional user-local Ubuntu Qt runtime, installed by setup_gui.sh.
for runtime_lib in "$sdk_dir"/build/gui-runtime/root/usr/lib/*-linux-gnu; do
    if [[ -d "$runtime_lib" ]]; then
        export LD_LIBRARY_PATH="$runtime_lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    fi
done
exec "$python_bin" "$script_dir/gui_control.py" "$@"
