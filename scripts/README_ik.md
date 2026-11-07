# R1-7A 简单末端位姿控制

入口：`scripts/arm_control.sh`，Python 实现：`scripts/arm_control.py`。
流程：SDK 单臂 URDF → 缓存的 CasADi FK/雅可比 + 有界最小二乘 IK → 独立 FK 校验 → 原 SDK `test_arm_lowlevel --goto --q ... --kp 60`。
保留当前已校准的 `config/joint_calib.txt`，不重复换算电机角度，不依赖外部服务、ROS 或 DDS。

## 本机直接使用

```bash
cd r1-7a_sdk   # 仓库根目录

# 离线查询零位末端坐标，也可以提供 --q 七个关节角
./scripts/arm_control.sh fk

# 与原 SDK 相同的七轴零位命令；不加 --execute 仅打印
./scripts/arm_control.sh goto --q 0 0 0 0 0 0 0 --kp 60 --execute

# 根据零位模型沿 base_link 的 +Z 平移 5 mm，保持零位末端方向：仅求解
./scripts/arm_control.sh ik --delta 0 0 0.005

# 执行同一目标，保持 3 秒后停止。实际起点须由操作者确认。
./scripts/arm_control.sh ik --delta 0 0 0.005 --hold-ms 3000 --execute

# 绝对坐标示例，单位 m；未传 --rpy/--quat 时保持 FK(seed) 的方向
./scripts/arm_control.sh ik --xyz 0.002674538 -0.034121936 0.197920591

# 全位姿：xyz + 四元数 xyzw（此例是上面的近零位目标）
./scripts/arm_control.sh ik --xyz 0.002674538 -0.034121936 0.197920591 \
  --quat -0.00006780364456 0.999928440527 -0.008722393743 0.008187128588
```

`--execute` 表示直接运行，不另弹确认。离线命令不打开串口。依赖（NumPy/SciPy/CasADi）见
[requirements-ik.txt](requirements-ik.txt)，安装在 `.venv-ik` 或 `R1_IK_PYTHON` 指定的解释器中。
可从任意工作目录执行脚本；它在执行 SDK 前切到 SDK 的 `bin/`，确保使用
`../config/joint_calib.txt`。Ctrl-C / SIGTERM 直接交给 SDK 处理。

## 位姿、种子和工具端点

- 关节顺序为 SDK 的 `J1..J7` / 电机 ID `0..6`，角度单位 rad。
- 目标坐标系是 SDK URDF 的 `base_link`，不是整机胸部、相机或世界坐标系。
- 默认末端是 `Link7` 的坐标原点；不是夹爪指尖。`--tcp x y z roll pitch yaw`
  设置从 `Link7` 到实际 TCP 的固定变换，放在 `fk/goto/ik` 子命令之前。
- `--rpy` 是固定轴 XYZ 角，旋转矩阵为 `Rz(yaw) Ry(pitch) Rx(roll)`，单位 rad；
  `--quat` 顺序为 `x y z w`，需要单位四元数。
- `--seed` 默认七个零，**不是实时关节反馈**。七自由度存在冗余，建议提供当前已知关节角
  作为种子。`--delta` 相对于 **FK(seed)**，不是从串口读取的当前末端。
- 默认 `--search adaptive`：固定所选朝向，从种子附近 ±0.6 rad → ±1.4 rad → 完整 URDF 范围
  逐级扩大搜索。多初值共享 `--ik-timeout 3` 秒墙钟预算；无须为远目标手动放大 `--max-step`。
- 显式指定 `--max-step` 时保持兼容，自动使用原来的严格有界 IPOPT 求解；
  `--search bounded` 未给 `--max-step` 时取 0.5 rad。它限制解相对种子的关节变化，
  不是速度限制。`--search adaptive` 与 `--max-step` 不能混用，避免无意忽略硬范围。
- 自适应结果包含 `ik_search` 搜索级别、尝试次数、耗时和误差；超时/未找到解均不发送命令。
  数值搜索失败不是不可达证明，算法也不保证找到全局最近解。

例如已知关节为 `0.1 0.2 0.1 0.3 0 0 0`：

```bash
./scripts/arm_control.sh ik --delta 0 0 0.005 --seed 0.1 0.2 0.1 0.3 0 0 0
# TCP 沿 Link7 局部 Z 偏移 0.08 m（示例，需换成实际测量值）
./scripts/arm_control.sh --tcp 0 0 0.08 0 0 0 fk
```

零位默认 FK：位置约 `(0.002674538, -0.034121936, 0.192920591) m`；
零位末端方向不是单位旋转，因此不要把 `--rpy 0 0 0` 当作“保持零位方向”。
J2、J4 的 URDF 下限为 0，零位正好在边界；脚本支持这一情况，不人为移动零位。

## SDK 执行参数与实际行为

`goto/ik` 均支持 `--port`（默认 `/dev/ttyUSB3`）、`--kp 60`、`--kd 0.5`、
`--home-lead 0.2`、`--rate 50`、`--move-ms 3000`、`--hold-ms 0`。
默认值与当前 SDK 实际参数一致。

当前 SDK 的 `goto` 是关节目标控制，并非笛卡尔直线轨迹；`home-lead` 限制命令领先反馈的角度，
不是速度。SDK 的到位阈值约为每轴 **0.07 rad**，5 mm 示例的关节变化可能小于阈值，
因此示例设置 `--hold-ms 3000`，让 SDK 持续发送目标。`move-ms` 是自动延长的超时下限，
不是指定运动耗时。

自适应 IK 的独立 FK 验收阈值为 **0.1 mm / 0.001 rad**；旧有界模式为 **0.2 mm / 0.002 rad**，
均不代表真机精度。固定朝向约束用于目标位姿，SDK 的关节插值不保证中间路径朝向恒定。
底层没有重力前馈；到位、保持结束或 Ctrl-C 后发送 **mode 0，释放力矩**，需要机械支撑。
原 SDK 移动超时仍可能返回 0；包装脚本继承其退出码，不将返回 0 宣称为到位证明，
应检查 SDK 的反馈及 `Creep timeout` 输出。没有自碰撞、环境碰撞规划或实时反馈闭环 IK。
运行时确保没有其他进程同时控制该串口。

## IK 来源与适配

1. [unitreerobotics/r1-7a_sdk](https://github.com/unitreerobotics/r1-7a_sdk)：提供实际使用的
   单臂 URDF、七轴顺序、标定和 `goto` 执行。本地 SDK 基线 commit
   `2c89fe4f60f0c98f1bc06795cb2477cc73d0c92c`，保留已有现场修改。
2. [xr_teleoperate / R1_A7_ArmIK](https://github.com/unitreerobotics/xr_teleoperate/blob/817fb00c63cde15e5f24a0f8fa08e1e33ed89d3b/teleop/robot_control/robot_arm_ik.py#L1753)：
   宇树已提供对应 R1 七自由度 IK，原版是整机双臂 Pinocchio + CasADi/IPOPT。
   本实现参考其四项优化目标和权重：位置 50、方向 1、关节正则 0.02、种子平滑 0.1。

适配内容：使用 SDK 自身串联 URDF 的符号 FK 替代 Pinocchio 整机模型；从双臂变为单臂；
不沿用整机模型的腕部 +X 0.05 m 偏移，改为明确的 `Link7 -> TCP`；方向误差使用
`0.5 * ||R-R_target||_F²`（局部等价于 log3 范数平方，避免 0/π 处的数值奇异）；
增加位置/方向误差约束、种子范围、确定性重试和独立 NumPy/SciPy FK 验证；
移除可视化、滤波、模型缓存、重力力矩输出和失败后返回调试解的行为。
这是旧 `solve(max_step=...)` / `--search bounded` 模式，仍供键盘小步点动使用。

2026-10-28 新增 `solve_pose()` 自适应模式，供 GUI 与 CLI 默认使用：保持同一 URDF、TCP、
旋转矩阵误差和独立 FK 验证；缓存 CasADi 的残差及解析雅可比，以 SciPy 有界最小二乘
替代每目标重建 IPOPT 优化问题。残差为 `[5*(p-p_target), vec(R-R_target)]`，
不添加会与精确位姿竞争的关节零位惩罚；通过从当前姿态开始、先局部再全范围的顺序维持连续性。
局部/扩大各至多 2 个初值，全范围至多 12 个初值，各次迭代和总墙钟均受预算约束。
这是本地进一步改进，并非直接调用上游原版类。

上游 Apache-2.0 许可证及署名在 `ik_licenses/`。

## 在其他环境安装与离线验证

下面固定版本对应本机 Python 3.12 环境：

```bash
python3.12 -m venv .venv-ik
.venv-ik/bin/python -m pip install -r scripts/requirements-ik.txt
./scripts/arm_control.sh fk

# 可选验证依赖，测试不会运行真实电机程序
.venv-ik/bin/python -m pip install pytest pybullet
.venv-ik/bin/python -m pytest -q tests/test_arm_control.py
```

启动器优先使用 `R1_IK_PYTHON` 环境变量，其次 SDK 内 `.venv-ik`，最后系统 `python3`。仅复制脚本时还须保留 SDK 的 URDF、
已编译程序、动态库和对应机械臂的标定；也可用全局 `--sdk-dir /path/to/r1-7a_sdk` 指定。
