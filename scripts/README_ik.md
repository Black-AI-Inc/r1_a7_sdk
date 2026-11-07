# 命令行 FK / IK

入口：`scripts/arm_control.sh`，实现在 `scripts/arm_control.py`。

流程：读取 SDK 的单臂 URDF → CasADi 构建 FK 和解析雅可比 → 有界最小二乘 IK → 用独立的 FK 校验结果 →
调用官方 `test_arm_lowlevel --goto --right --q ...` 执行。使用现有的 `config/joint_calib_right.txt`，
不另做电机角度换算，不依赖 ROS、DDS 或其他外部服务。

## 使用

```bash
# 零位末端位姿（离线）；也可以用 --q 给出七个关节角
./scripts/arm_control.sh fk

# 关节目标，与官方 test_arm_lowlevel --goto 相同；不加 --execute 只打印命令
./scripts/arm_control.sh goto --q 0 0 0 0 0 0 0 --kp 60 --execute

# 从零位末端沿 base_link +Z 平移 5 mm，保持零位朝向（只求解）
./scripts/arm_control.sh ik --delta 0 0 0.005

# 执行同一目标，到位后保持 3 s 再停止。执行前请确认机械臂确实在种子姿态附近
./scripts/arm_control.sh ik --delta 0 0 0.005 --hold-ms 3000 --execute

# 绝对位置（m）；没有给 --rpy 或 --quat 时，保持 FK(seed) 的朝向
./scripts/arm_control.sh ik --xyz 0.002674538 -0.034121936 0.197920591

# 完整位姿：位置 + 四元数（x y z w），这里是零位附近的一个目标
./scripts/arm_control.sh ik --xyz 0.002674538 -0.034121936 0.197920591 \
  --quat -0.00006780364456 0.999928440527 -0.008722393743 0.008187128588
```

加 `--execute` 后直接运行，不再确认；不加时只求解和打印，不打开串口。
脚本可以从任意目录运行，执行前会切换到 `bin/`，使官方程序读到 `../config/joint_calib_right.txt`。
执行时脚本用 `exec` 替换为官方程序，Ctrl-C 和 SIGTERM 由官方程序处理。

## 坐标、种子和 TCP

- 关节顺序为 SDK 的 J1..J7（电机 ID 0..6），单位 rad。
- 目标坐标系是 SDK URDF 的 `base_link`，不是整机、相机或世界坐标系。
- 默认末端是 `Link7` 原点，不是夹爪指尖。全局参数 `--tcp x y z roll pitch yaw` 设置 `Link7` 到实际 TCP 的固定变换，
  写在 `fk`、`goto`、`ik` 子命令之前。
- `--rpy` 是固定轴 XYZ 欧拉角，旋转矩阵 `Rz(yaw) Ry(pitch) Rx(roll)`，单位 rad。`--quat` 顺序为 `x y z w`，必须是单位四元数。
- `--seed` 默认为七个零，**不会读取实时反馈**。机械臂是七自由度冗余结构，建议把当前关节角作为种子传入。
  `--delta` 相对的是 FK(seed)，不是串口读到的当前末端。
- 零位的末端朝向不是单位旋转，`--rpy 0 0 0` 并不表示保持零位朝向。
- 零位默认 FK 位置约为 `(0.002674538, -0.034121936, 0.192920591) m`。J2、J4 的 URDF 下限为 0，零位正好在边界上。

已知当前关节角为 `0.1 0.2 0.1 0.3 0 0 0` 时：

```bash
./scripts/arm_control.sh ik --delta 0 0 0.005 --seed 0.1 0.2 0.1 0.3 0 0 0
# TCP 在 Link7 局部 Z 方向 0.08 m 处（示例，请换成实测值）
./scripts/arm_control.sh --tcp 0 0 0.08 0 0 0 fk
```

## 求解方式

- 默认 `--search adaptive`：固定目标朝向，从种子附近 ±0.6 rad、±1.4 rad 到完整 URDF 范围逐级扩大搜索，
  所有初值共用 `--ik-timeout` 秒（默认 3 s）的总时间。远目标不需要手动调大 `--max-step`。
- 给出 `--max-step`，或使用 `--search bounded`（此时 `--max-step` 默认 0.5 rad）时，改用 IPOPT 有界求解，
  解与种子的关节差不超过 `--max-step`。这个限制针对解，不是速度限制。`--search adaptive` 不能与 `--max-step` 同时使用。
- 自适应模式的输出中有 `ik_search`，包括找到解的搜索级别、尝试次数、耗时和误差。超时或未找到解时不发送任何命令。
  找不到解不代表目标一定不可达，也不保证找到离种子最近的解。
- 结果用独立的 NumPy/SciPy FK 校验：自适应模式要求位置误差 ≤ 0.1 mm、方向误差 ≤ 0.001 rad，
  有界模式为 0.2 mm、0.002 rad。这是模型误差，不代表真机精度。

## 执行参数

`goto` 和 `ik` 都支持 `--port`（默认 `/dev/ttyUSB3`）、`--kp 60`、`--kd 0.5`、`--home-lead 0.2`、`--rate 50`、
`--move-ms 3000`、`--hold-ms 0`，默认值与官方程序一致。

- 官方 `goto` 是关节目标控制，不是笛卡尔直线轨迹。`--home-lead` 限制命令领先反馈的角度，不是速度。
- 官方程序的到位阈值约为每轴 0.07 rad，5 mm 这类小目标的关节变化可能小于阈值，所以示例加了 `--hold-ms 3000`，
  让程序在到位后继续发送目标。`--move-ms` 是超时的下限，程序会按距离自动延长，不是运动时长。
- 没有重力前馈。到位、保持结束或 Ctrl-C 后发送 mode 0 释放力矩，机械臂需要支撑。
- 官方程序移动超时也可能返回 0，脚本直接沿用它的退出码。是否到位请看程序输出的反馈和 `Creep timeout` 信息。
- 不做自碰撞和环境碰撞检查，也不是实时闭环 IK。运行时不要让其他程序同时使用该串口。

## IK 来源与改动

1. [unitreerobotics/r1-7a_sdk](https://github.com/unitreerobotics/r1-7a_sdk)：右臂 URDF（`R1_Robotic_Arm_R_V1.2-1023`）、关节顺序、标定和 `goto` 执行。
   与之前的 V1.1-0915 相比只把 J6 限位从 ±0.524 rad 放宽到 ±0.785 rad，运动学和质量参数不变。
2. [xr_teleoperate 中的 R1_A7_ArmIK](https://github.com/unitreerobotics/xr_teleoperate/blob/817fb00c63cde15e5f24a0f8fa08e1e33ed89d3b/teleop/robot_control/robot_arm_ik.py#L1753)：
   宇树提供的 R1 七自由度 IK，原版针对整机双臂，使用 Pinocchio + CasADi/IPOPT。
   本实现沿用了它的四项优化目标和权重：位置 50、方向 1、关节正则 0.02、种子平滑 0.1。

相对原版的改动：

- 用 SDK 自带的单臂串联 URDF 构建符号 FK，代替 Pinocchio 整机模型，只处理单臂；
- 不使用整机模型中腕部 +X 0.05 m 的偏移，改为显式的 `Link7 -> TCP` 参数；
- 方向误差用 `0.5 * ||R - R_target||_F²`，在局部与 log3 范数平方等价，并且在转角为 0 和 π 时都没有数值奇异；
- 增加位置和方向误差约束、关节范围约束、确定性的多初值重试，以及独立的 FK 校验；
- 去掉可视化、滤波、模型缓存、重力力矩输出，求解失败时不返回中间结果。

以上对应 `ArmIK.solve()`，即 `--search bounded`，键盘工具的小步点动也用它。

`ArmIK.solve_pose()` 是在此基础上新增的自适应模式，供 GUI 和命令行默认使用。它使用相同的 URDF、TCP、
旋转矩阵误差和 FK 校验，但把 CasADi 的残差和解析雅可比缓存下来，用 SciPy 的有界最小二乘代替每次重建的 IPOPT 问题。
残差为 `[5 * (p - p_target), vec(R - R_target)]`，不加关节零位惩罚，以免与精确位姿冲突；
通过从当前姿态出发、先局部后全范围的搜索顺序保持解的连续性。
局部和扩大范围两级各最多 2 个初值，完整范围最多 12 个，每次迭代和总时间都有上限。

原版的 Apache-2.0 许可证和署名在 `ik_licenses/`，`arm_control.py` 也按 Apache-2.0 发布。

## 安装与测试

依赖固定为 Python 3.12 下的版本：

```bash
python3.12 -m venv .venv-ik
.venv-ik/bin/python -m pip install -r scripts/requirements-ik.txt
./scripts/arm_control.sh fk

# 测试只做离线计算，不运行真机程序
.venv-ik/bin/python -m pip install -r scripts/requirements-dev.txt
.venv-ik/bin/python -m pytest -q tests/test_arm_control.py
```

启动脚本按 `R1_IK_PYTHON` 环境变量、仓库内 `.venv-ik`、系统 `python3` 的顺序选择 Python。
把脚本复制到别处使用时，还需要 SDK 的 URDF、编译好的程序、动态库和对应机械臂的标定文件，
或者用全局参数 `--sdk-dir /path/to/r1-7a_sdk` 指向完整的 SDK。
