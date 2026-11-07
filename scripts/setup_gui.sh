#!/usr/bin/env bash
# One-time GUI setup: install requirements-gui.txt into $R1_IK_PYTHON or
# .venv-ik (created if missing), unpack libxcb-cursor0 locally under
# build/gui-runtime when the system lacks it, and build the native worker.
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
sdk_dir="$(cd -- "$script_dir/.." && pwd)"
if [[ -n "${R1_IK_PYTHON:-}" ]]; then
    python_bin="$R1_IK_PYTHON"
elif [[ -x "$sdk_dir/.venv-ik/bin/python" ]]; then
    python_bin="$sdk_dir/.venv-ik/bin/python"
else
    python3 -m venv "$sdk_dir/.venv-ik"
    python_bin="$sdk_dir/.venv-ik/bin/python"
fi
"$python_bin" -m pip install -r "$script_dir/requirements-gui.txt"
for runtime_lib in "$sdk_dir"/build/gui-runtime/root/usr/lib/*-linux-gnu; do
    if [[ -d "$runtime_lib" ]]; then
        export LD_LIBRARY_PATH="$runtime_lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    fi
done
if [[ "$(uname -s)" == Linux ]] && ! "$python_bin" -c 'import ctypes; ctypes.CDLL("libxcb-cursor.so.0")' 2>/dev/null; then
    if command -v apt-get >/dev/null && command -v dpkg-deb >/dev/null; then
        mkdir -p "$sdk_dir/build/gui-runtime"
        (
            cd "$sdk_dir/build/gui-runtime"
            apt-get download libxcb-cursor0
            for package in libxcb-cursor0_*.deb; do
                dpkg-deb -x "$package" root
            done
        )
    else
        echo 'Install the Qt xcb cursor runtime using your system package manager.' >&2
        exit 1
    fi
fi
"$script_dir/build_keyboard.sh"
echo "GUI ready: $script_dir/gui_control.sh"
