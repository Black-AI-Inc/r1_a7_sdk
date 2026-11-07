# r1-7a_sdk
R1 robotic arm low-level control SDK (IM6014 serial bus).

### Prebuild environment
* OS  (Ubuntu 20.04 LTS or higher)
* CPU  (aarch64 and x86_64)
* Compiler  (gcc version 9.4.0)

### Environment Setup

Before building or running the SDK, ensure the following dependencies are installed:

- CMake (version 3.10 or higher)
- GCC (version 9.4.0)
- Make
- pthread

You can install the required packages on Ubuntu 20.04 with:

```bash
apt-get update
apt-get install -y cmake g++ build-essential
```

Serial access usually requires membership in the `dialout` group (or equivalent) for `/dev/ttyUSB*`.

### Build examples

To build the examples inside this repository:

```bash
mkdir build
cd build
cmake ..
make
```

Binaries are written to `bin/`:

- `test_arm_lowlevel` — low-level FOC / stop / home / goto commands
- `calibrate_joints` — joint calibration tool (URDF ↔ motor frame)
- `demo_arm_motion` — pre-programmed joint motion demo

### Usage

Default serial port is `/dev/ttyUSB3`, baudrate `6000000`. Joint calibration is loaded from `config/joint_calib.txt` (path relative to the working directory; when run from `bin/`, the default `../config/joint_calib.txt` is used).

**Hold current pose (FOC):**

```bash
cd bin
./test_arm_lowlevel --foc --port /dev/ttyUSB3
```

**Move to URDF zero (home):**

```bash
./test_arm_lowlevel --home --kp 60
```

**Move to a target pose (goto):**

```bash
./test_arm_lowlevel --goto --q 0 0 0 0 0 0 0 --kp 60
```

**Cartesian target / inverse kinematics (Python wrapper):**

See [单臂 IK 控制脚本使用说明](scripts/README_ik.md) for SDK-URDF FK and
Unitree-model IK with cached analytic Jacobians and adaptive fixed-orientation search (bounded IPOPT mode retained). From the repository root:

```bash
./scripts/arm_control.sh fk
./scripts/arm_control.sh ik --delta 0 0 0.005  # offline, relative to FK(zero seed)
# Add --hold-ms 3000 --execute to send the target through the existing SDK goto.
```

**Other commands:** `--stop`, `--zero-torque`, `--clear`, `--reset`. Use exactly one command per run. Ctrl-C sends stop (mode 0) and exits for long-running modes.

**GUI joint/XYZ/RPY sliders, runtime motion settings and live URDF:** Run `./scripts/gui_control.sh` for simulation,
or `./scripts/gui_control.sh --execute` and click Connect for hardware.
See [GUI 控制说明](scripts/README_gui.md) for live target/command/feedback readings,
the interactive URDF view, installation, and hold/stop behavior.

**Keyboard XYZ jogging with hold on key release:**

See [键盘控制说明](scripts/README_keyboard.md). From the repository root,
run `./scripts/keyboard_control.sh` for simulation or add `--execute` for hardware.
W/S, A/D, R/F jog XYZ; release/Space holds; P goes to a configurable ready pose;
H returns to joint zero. Jog, P and H use velocity/acceleration/jerk-limited smooth
trajectories; held keys precompute the next point to avoid per-step dwell.
Esc stops and releases torque.

**Run the pre-programmed motion demo:**

```bash
cd bin
./demo_arm_motion --port /dev/ttyUSB3
```

The demo waits for feedback from all 7 joints, enables FOC at the current pose, and then runs a fixed sequence of cosine-profile moves. Use `--speed` and `--wrist-speed` to adjust joint velocities, `--kp` / `--kd` to tune gains, and `--rate` to change the control loop frequency (default 100 Hz). Ctrl-C stops all motors and exits.

**Calibrate joints** (pose the arm at the desired URDF zero first):

```bash
./calibrate_joints --port /dev/ttyUSB3 --out ../config/joint_calib.txt
```

Calibration formula: `urdf = dir * scale * motor + offset`.

### 本仓库新增内容（基于 Unitree 上游）

| 目录 / 文件 | 内容 |
|---|---|
| `scripts/keyboard_control.sh`、`scripts/gui_control.sh` | 键盘末端点动、Qt GUI（关节、末端位姿、参数、三维 URDF） |
| `scripts/arm_control.sh` | 命令行 FK/IK，可经 SDK goto 执行 |
| `scripts/keyboard_native/` | 两个工具共用的原生 100 Hz 控制 worker：Ruckig 平滑轨迹、重力前馈、关节积分外环、逐关节增益、软故障阻尼保持与恢复、启动越限恢复 |
| `scripts/tuned_profile.py` | 默认控制配置 `--profile tuned`（见下） |
| `scripts/gravity_model.py`、`config/dex1_gravity.json` | URDF 加 DEX1-1 夹爪的静态重力模型 |
| `include/`、`src/arm_motor_bus.*`、`thirdparty/motor_controller/` | 反馈快照（带时间戳和有效标记）、串口独占打开 |
| `tests/`、`scripts/keyboard_native/test_*.cpp` | Python 单元和集成测试（mock worker），原生单元测试 |

**安装与编译**（Ubuntu，Python 3.12）：

```bash
python3 -m venv .venv-ik
.venv-ik/bin/python -m pip install -r scripts/requirements-gui.txt   # 包含键盘控制和 IK 的依赖
./scripts/build_keyboard.sh          # 编译原生 worker，输出到 build/keyboard/
./scripts/setup_gui.sh               # 可选：检查 GUI 运行库
```

启动脚本按 `R1_IK_PYTHON` → `.venv-ik` → `python3` 的顺序选择解释器。

**默认控制配置 `--profile tuned`**：2026-11-03 在 R1-7A + DEX1-1 实机上验证。

- J2、J4 为 Kp 200 / Kd 1.41，其余关节 100 / 1；重力补偿 100%。
- 积分外环权限 3 N·m，带防粘滑振荡；跟踪误差停止阈值 0.1 rad。
- 软故障进入阻尼保持（不释放力矩）：键盘按 Enter、GUI 点 Recover 即可恢复。
- 实测：10 mm 单步误差 0.26–0.66 mm，静置 0.62 mm。
- `--profile legacy` 恢复原默认值：Kp 60 / Kd 0.5，不补偿重力，故障时释放力矩。
- 详见 [scripts/README_keyboard.md](scripts/README_keyboard.md)、[scripts/README_gui.md](scripts/README_gui.md)。

**关节标定**：`config/joint_calib.txt` 是**验证上述参数所用那台机械臂**的标定。换一台机械臂时，先按上文用 `calibrate_joints` 重新标定并覆盖该文件，再使用 tuned 配置；增益和重力参数也应在新机械臂上重新确认。

**测试**：

```bash
.venv-ik/bin/python -m pip install -r scripts/requirements-dev.txt
./scripts/build_keyboard.sh && (cd build/keyboard && ctest)
QT_QPA_PLATFORM=offscreen .venv-ik/bin/python -m pytest -q tests
```

全部测试只用 mock worker 和离线计算，不打开串口。

### Notice

This package drives a 7-DOF R1 arm over one serial bus via the vendored `thirdparty/motor_controller` (IM6014). Public `q` / `dq` / `tau` use the URDF frame; conversion happens at the `ArmMotorBus` edge.
