<div align="center">

**简体中文** | [English](README_en.md)

# r1_a7_sdk

R1-7A 七轴机械臂 SDK（IM6014 串口总线），附键盘点动、Qt GUI 和命令行 IK 等高层控制工具

</div>

本仓库 fork 自宇树官方 [unitreerobotics/r1-7a_sdk](https://github.com/unitreerobotics/r1-7a_sdk)，
并已合并上游截至 `0803d033`（2026-11-03）的更新。上游的底层 SDK 保持原样，
新增内容都在 `scripts/`、`tests/` 和两份配置文件中，另对 `ArmMotorBus` 和串口驱动做了少量扩展。

本仓库不是宇树官方仓库，新增功能出现问题请在本仓库提 issue。

- [官方 SDK](#官方-sdk)：编译、底层命令、示例、关节标定
- [高层控制工具](#高层控制工具)：键盘点动、GUI、命令行 FK/IK、重力补偿
- [许可证](#许可证)

## 官方 SDK

### 运行环境

- 操作系统：Ubuntu 20.04 LTS 及以上
- CPU：aarch64、x86_64
- 编译器：gcc 9.4.0

### 安装依赖

编译和运行 SDK 前需要安装：

- CMake（3.10 及以上）
- GCC（9.4.0）
- Make
- pthread

在 Ubuntu 20.04 上可以这样安装：

```bash
apt-get update
apt-get install -y cmake g++ build-essential
```

访问 `/dev/ttyUSB*` 串口通常需要把用户加入 `dialout` 组。

### 编译示例

```bash
mkdir build
cd build
cmake ..
make
```

可执行文件输出到 `bin/`：

- `test_arm_lowlevel`：底层命令，包括 FOC、停止、回零和 goto
- `demo_arm_motion`：预设的关节运动演示

关节标定工具 `calibrate_joints`（`src/calibrate_joints.cpp`）在上游 `CMakeLists.txt` 中已被注释掉，默认不编译。

左右臂的 URDF 打包在 `urdf/URDF_R1-7A.tar.gz`，解压到 `urdf/` 下使用：

```bash
tar -xzf urdf/URDF_R1-7A.tar.gz -C urdf
```

### 使用方法

波特率 `6000000`，下面的例子使用串口 `/dev/ttyUSB3`，请用 `--port` 指定实际设备。
`test_arm_lowlevel` 和 `demo_arm_motion` 必须用 `-r` / `--right` 或 `-l` / `--left` 选择机械臂，
分别读取 `config/joint_calib_right.txt` 和 `config/joint_calib_left.txt`。
标定文件路径相对于当前工作目录，在 `bin/` 下运行时使用默认的 `../config/...`。

保持当前姿态（FOC）：

```bash
cd bin
./test_arm_lowlevel --foc -r --port /dev/ttyUSB3
```

回到 URDF 零位：

```bash
./test_arm_lowlevel --home -r --kp 60
```

移动到指定关节角：

```bash
./test_arm_lowlevel --goto -r --q 0 0 0 0 0 0 0 --kp 60
```

`--goto-hold` 与 `--goto` 相同，但到位后一直保持，直到 Ctrl-C。
其他命令：`--stop`、`--zero-torque`、`--clear`、`--reset`。每次运行只能使用一个命令。
对持续运行的模式，Ctrl-C 会发送停止（mode 0）并退出。

运行预设运动演示：

```bash
cd bin
./demo_arm_motion -r --port /dev/ttyUSB3
```

演示程序等待 7 个关节都有反馈后，在当前姿态使能 FOC，然后按固定顺序执行余弦速度曲线的运动。
`--speed`、`--wrist-speed` 调整关节速度，`--kp`、`--kd` 调整增益，`--rate` 设置控制频率（默认 100 Hz）。
Ctrl-C 停止所有电机并退出。

标定公式：`urdf = dir * scale * motor + offset`。

### 说明

本 SDK 通过 `thirdparty/motor_controller`（IM6014）在一条串口总线上驱动 7 自由度 R1 机械臂。
对外的 `q`、`dq`、`tau` 均为 URDF 坐标，换算在 `ArmMotorBus` 内完成。

## 高层控制工具

| 入口 | 功能 | 详细说明 |
|---|---|---|
| `scripts/keyboard_control.sh` | 键盘控制末端沿 XYZ 点动，松键即保持 | [README_keyboard.md](scripts/README_keyboard.md) |
| `scripts/gui_control.sh` | Qt 界面：关节滑块、末端位姿、运行参数、三维 URDF 显示 | [README_gui.md](scripts/README_gui.md) |
| `scripts/arm_control.sh` | 命令行 FK/IK，可通过官方 `test_arm_lowlevel --goto` 执行 | [README_ik.md](scripts/README_ik.md) |
| `scripts/gravity_model.py`、`config/dex1_gravity.json` | 机械臂加 DEX1-1 夹爪的静态重力模型 | [README_gravity.md](scripts/README_gravity.md) |

键盘工具和 GUI 共用一个 C++ 控制进程 `scripts/keyboard_native/worker.cpp`。它以 100 Hz 运行，
负责 Ruckig 平滑轨迹、重力前馈、关节积分、故障处理和启动时的越限恢复。
两个工具默认都只连接模拟驱动，不打开串口；加 `--execute` 才连接真机。
目前这些工具只支持右臂：使用上游的右臂 URDF（`build_keyboard.sh` 会自动从压缩包解压）和 `config/joint_calib_right.txt`。

对 SDK 本身的改动：`ArmMotorBus` 增加带时间戳的反馈快照（`feedbackSnapshot()`、`feedbackAges()`），
`open()` 和串口驱动增加独占打开选项。

### 安装

需要 Ubuntu 和 Python 3.12。在仓库根目录：

```bash
python3.12 -m venv .venv-ik
.venv-ik/bin/python -m pip install -r scripts/requirements-gui.txt   # 已包含键盘工具和 IK 的依赖
./scripts/build_keyboard.sh    # 编译控制进程，输出到 build/keyboard/
./scripts/setup_gui.sh         # 可选：检查并补齐 GUI 需要的 Qt 运行库
```

启动脚本按 `R1_IK_PYTHON` 环境变量、仓库内 `.venv-ik`、系统 `python3` 的顺序选择 Python。

### 快速开始

```bash
# 键盘点动：先在模拟模式下熟悉按键
./scripts/keyboard_control.sh
./scripts/keyboard_control.sh --execute --port /dev/ttyUSB3

# GUI：真机模式下点击“连接真机”后才打开串口
./scripts/gui_control.sh
./scripts/gui_control.sh --execute --port /dev/ttyUSB3

# 命令行：零位 FK；沿 +Z 平移 5 mm 的 IK（只求解，不运动）
./scripts/arm_control.sh fk
./scripts/arm_control.sh ik --delta 0 0 0.005
```

键盘按键：W/S、A/D、R/F 分别沿 `base_link` 的 X、Y、Z 正负方向点动；松键或空格保持；
P 移到准备姿态；H 回零；Enter 在阻尼保持后恢复；Esc 停止并释放力矩；Q 退出。

### 默认控制参数

键盘工具和 GUI 默认使用 `--profile tuned`，参数定义在 `scripts/tuned_profile.py`，
2026-11-03 在一台 R1-7A + DEX1-1 上实测确定。`--profile legacy` 恢复工具最初的默认值。

| 项目 | `tuned`（默认） | `legacy` |
|---|---|---|
| 增益 | J2、J4：Kp 200 / Kd 1.41；其余关节 Kp 100 / Kd 1 | 全部关节 Kp 60 / Kd 0.5 |
| 重力补偿 | 100%，使用 `config/dex1_gravity.json` | 关闭 |
| 关节积分 | ki 1.5/s，每个关节最多 3 N·m | 无 |
| 跟踪误差阈值 | 0.1 rad | 0.2 rad |
| 心跳丢失、跟踪超限、非法目标 | 在实测位置阻尼保持，不释放力矩，可恢复 | 释放力矩 |
| 连接时关节略超出 URDF 限位 | 超出不到 0.2 rad 时自动移回限位内 | 拒绝使能 |

实测结果：10 mm 单步定位误差 0.26–0.66 mm，静止保持误差 0.62 mm。

命令行显式给出的参数优先于配置。如果给出与配置不同的 `--kp` 或 `--kd`，所有关节改用这一组统一增益。
反馈中断、电机报错、退出 FOC、按 Esc 或停止、程序退出时，两种配置都会释放力矩。

`config/joint_calib_right.txt` 是做上述实测的那台机械臂的标定，不是上游原有的数值。换一台机械臂前，
先重新标定并覆盖该文件，增益和重力参数也要在新机械臂上重新确认。

### 使用须知

- 程序不做自碰撞和环境碰撞检查，回零和准备姿态都是关节空间运动。
- 软件停止不能代替硬件急停。释放力矩后机械臂会下落，运行前要能扶住或支撑机械臂。
- 真机运行前关闭其他占用同一串口的程序。控制进程以独占方式打开串口，但无法赶走已经打开串口的进程。
- 重力补偿的夹爪质量和重心来自 CAD 按实测总质量缩放，换夹爪或抓取物体时需要修改配置。

### 测试

```bash
.venv-ik/bin/python -m pip install -r scripts/requirements-dev.txt
./scripts/build_keyboard.sh && ctest --test-dir build/keyboard --output-on-failure
QT_QPA_PLATFORM=offscreen .venv-ik/bin/python -m pytest -q tests
```

测试只使用模拟驱动和离线计算，不会打开串口。

## 许可证

本仓库沿用上游的 [BSD 3-Clause 许可证](LICENSE)（Copyright (c) 2026 Unitree Robotics），新增代码也按此许可发布。以下文件例外：

- `scripts/arm_control.py` 的 IK 改编自宇树 [xr_teleoperate](https://github.com/unitreerobotics/xr_teleoperate)，
  保留 Apache-2.0 许可，许可证全文和上游署名见 `scripts/ik_licenses/`。
- `scripts/keyboard_native/third_party/ruckig/` 是未修改的 [Ruckig](https://github.com/pantor/ruckig) v0.15.3，
  MIT 许可，版本和文件哈希记录在 `SOURCE.json`。
