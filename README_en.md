<div align="center">

[简体中文](README.md) | **English**

# r1_a7_sdk

SDK for the R1-7A 7-DoF arm (IM6014 serial bus), with keyboard jogging, a Qt GUI and command-line IK on top

</div>

This repository is a fork of Unitree's official [unitreerobotics/r1-7a_sdk](https://github.com/unitreerobotics/r1-7a_sdk),
based on upstream commit `2c89fe4f` (2026-10-12). The low-level SDK is unchanged. Everything added lives in
`scripts/`, `tests/` and two config files, plus small extensions to `ArmMotorBus` and the serial driver.
Later upstream changes (left/right arm selection and others) have not been merged yet.

This is not an official Unitree repository. Please report problems with the added tools here.

- [Official SDK](#official-sdk): build, low-level commands, demo, joint calibration
- [High-level control tools](#high-level-control-tools): keyboard jogging, GUI, command-line FK/IK, gravity compensation
- [License](#license)

## Official SDK

### Prebuild environment

- OS: Ubuntu 20.04 LTS or later
- CPU: aarch64, x86_64
- Compiler: gcc 9.4.0

### Dependencies

Install the following before building or running the SDK:

- CMake (3.10 or later)
- GCC (9.4.0)
- Make
- pthread

On Ubuntu 20.04:

```bash
apt-get update
apt-get install -y cmake g++ build-essential
```

Serial access usually requires membership in the `dialout` group for `/dev/ttyUSB*`.

### Build the examples

```bash
mkdir build
cd build
cmake ..
make
```

Binaries are written to `bin/`:

- `test_arm_lowlevel`: low-level FOC / stop / home / goto commands
- `calibrate_joints`: joint calibration tool (URDF ↔ motor frame)
- `demo_arm_motion`: pre-programmed joint motion demo

### Usage

The default serial port is `/dev/ttyUSB3` at `6000000` baud. Joint calibration is read from `config/joint_calib.txt`,
relative to the working directory; when run from `bin/`, the default `../config/joint_calib.txt` is used.

Hold the current pose (FOC):

```bash
cd bin
./test_arm_lowlevel --foc --port /dev/ttyUSB3
```

Move to URDF zero (home):

```bash
./test_arm_lowlevel --home --kp 60
```

Move to a joint target (goto):

```bash
./test_arm_lowlevel --goto --q 0 0 0 0 0 0 0 --kp 60
```

Other commands: `--stop`, `--zero-torque`, `--clear`, `--reset`. Use exactly one command per run.
For long-running modes, Ctrl-C sends stop (mode 0) and exits.

Run the pre-programmed motion demo:

```bash
cd bin
./demo_arm_motion --port /dev/ttyUSB3
```

The demo waits for feedback from all 7 joints, enables FOC at the current pose, and then runs a fixed sequence of
cosine-profile moves. Use `--speed` and `--wrist-speed` for joint velocities, `--kp` / `--kd` for gains and `--rate`
for the control loop frequency (default 100 Hz). Ctrl-C stops all motors and exits.

Calibrate the joints (pose the arm at the desired URDF zero first):

```bash
./calibrate_joints --port /dev/ttyUSB3 --out ../config/joint_calib.txt
```

Calibration formula: `urdf = dir * scale * motor + offset`.

### Notice

This package drives a 7-DoF R1 arm over one serial bus via the vendored `thirdparty/motor_controller` (IM6014).
Public `q` / `dq` / `tau` use the URDF frame; conversion happens at the `ArmMotorBus` edge.

## High-level control tools

| Entry point | What it does | Details (Chinese) |
|---|---|---|
| `scripts/keyboard_control.sh` | Jog the end effector along XYZ from the keyboard; holds on key release | [README_keyboard.md](scripts/README_keyboard.md) |
| `scripts/gui_control.sh` | Qt GUI: joint sliders, end-effector pose, runtime parameters, 3D URDF view | [README_gui.md](scripts/README_gui.md) |
| `scripts/arm_control.sh` | Command-line FK/IK; can execute through the official `test_arm_lowlevel --goto` | [README_ik.md](scripts/README_ik.md) |
| `scripts/gravity_model.py`, `config/dex1_gravity.json` | Static gravity model of the arm with a DEX1-1 gripper | [README_gravity.md](scripts/README_gravity.md) |

The keyboard tool and the GUI share one C++ control process, `scripts/keyboard_native/worker.cpp`. It runs at
100 Hz and handles Ruckig trajectory smoothing, gravity feedforward, the joint integral term, fault handling and
recovery of slightly out-of-limit joints at startup. Both tools start against a simulated driver and do not open
the serial port unless `--execute` is given.

Changes to the SDK itself: `ArmMotorBus` gains a time-stamped feedback snapshot (`feedbackSnapshot()`,
`feedbackAges()`), and `open()` and the serial driver gain an option to open the port exclusively.

### Installation

Requires Ubuntu and Python 3.12. From the repository root:

```bash
python3.12 -m venv .venv-ik
.venv-ik/bin/python -m pip install -r scripts/requirements-gui.txt   # includes the keyboard and IK dependencies
./scripts/build_keyboard.sh    # builds the control process into build/keyboard/
./scripts/setup_gui.sh         # optional: checks for and fetches the Qt runtime the GUI needs
```

The launch scripts pick Python from `R1_IK_PYTHON`, then the repository's `.venv-ik`, then the system `python3`.

### Quick start

```bash
# Keyboard jogging: learn the keys in simulation first
./scripts/keyboard_control.sh
./scripts/keyboard_control.sh --execute --port /dev/ttyUSB3

# GUI: in hardware mode the serial port is opened only after clicking Connect
./scripts/gui_control.sh
./scripts/gui_control.sh --execute --port /dev/ttyUSB3

# Command line: FK at zero; IK for a 5 mm move along +Z (solve only, no motion)
./scripts/arm_control.sh fk
./scripts/arm_control.sh ik --delta 0 0 0.005
```

Keyboard keys: W/S, A/D and R/F jog along ±X, ±Y and ±Z of `base_link`. Releasing the key or pressing Space holds.
P moves to the ready pose, H returns to zero, Enter recovers from a damping hold, Esc stops and releases torque,
Q quits.

### Default control profile

The keyboard tool and the GUI default to `--profile tuned`, defined in `scripts/tuned_profile.py` and tuned on an
R1-7A with a DEX1-1 gripper on 2026-11-03. `--profile legacy` restores the tools' original defaults.

| Item | `tuned` (default) | `legacy` |
|---|---|---|
| Gains | J2, J4: Kp 200 / Kd 1.41; other joints Kp 100 / Kd 1 | all joints Kp 60 / Kd 0.5 |
| Gravity compensation | 100 %, using `config/dex1_gravity.json` | off |
| Joint integral | ki 1.5/s, at most 3 N·m per joint | none |
| Tracking error limit | 0.1 rad | 0.2 rad |
| Heartbeat loss, tracking limit, invalid target | damping hold at the measured pose, torque kept, recoverable | release torque |
| Joint slightly outside URDF limits at connect | moved back inside if less than 0.2 rad out | refuse to enable |

Measured with this profile: 0.26–0.66 mm error on 10 mm steps, 0.62 mm while holding.

Options given explicitly on the command line override the profile. A `--kp` or `--kd` different from the profile
switches all joints to that single pair of gains. Both profiles release torque on feedback loss, a motor error,
leaving FOC, Esc or Stop, and program exit.

`config/joint_calib.txt` is the calibration of the arm used for these measurements. On another arm, recalibrate with
`calibrate_joints` and overwrite this file first, then re-check the gains and the gravity parameters.

### Before running on hardware

- There is no self-collision or environment collision checking. Home and ready moves are joint-space motions.
- The software stop is not an emergency stop. The arm drops when torque is released, so be ready to support it.
- Close any other program using the same serial port. The control process opens the port exclusively, but it cannot
  take the port from a process that already has it open.
- The gripper mass and centre of mass in the gravity profile are CAD values scaled to the measured mass. Update the
  profile when the gripper or the grasped object changes.

### Tests

```bash
.venv-ik/bin/python -m pip install -r scripts/requirements-dev.txt
./scripts/build_keyboard.sh && ctest --test-dir build/keyboard --output-on-failure
QT_QPA_PLATFORM=offscreen .venv-ik/bin/python -m pytest -q tests
```

The tests use only the simulated driver and offline computation and never open a serial port.

## License

This repository keeps the upstream [BSD 3-Clause License](LICENSE) (Copyright (c) 2026 Unitree Robotics), and the
added code is released under the same license, with these exceptions:

- The IK in `scripts/arm_control.py` is adapted from Unitree's [xr_teleoperate](https://github.com/unitreerobotics/xr_teleoperate)
  and stays under Apache-2.0; the license text and upstream attribution are in `scripts/ik_licenses/`.
- `scripts/keyboard_native/third_party/ruckig/` is an unmodified copy of [Ruckig](https://github.com/pantor/ruckig)
  v0.15.3 under the MIT license; the version and file hashes are recorded in `SOURCE.json`.
