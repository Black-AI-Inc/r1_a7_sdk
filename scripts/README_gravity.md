# R1-7A + DEX1-1 静态重力补偿

键盘工具和 GUI 共用的控制进程会根据实测关节角计算重力力矩，作为前馈叠加到位置控制上。
`--profile tuned`（默认）以 100% 比例使用 `config/dex1_gravity.json`；`--profile legacy` 默认不补偿。
官方 `test_arm_lowlevel --goto` 和 `arm_control.sh` 走官方 SDK，没有这项补偿。

## 负载与安装参数

`config/dex1_gravity.json` 记录的是 2026-10-28 确认的安装情况：

| 参数 | 值 |
| --- | --- |
| 空夹爪总成 | 550 g，已含 3 g 转接件 |
| 抓取物体 | 约 50 g，只在加 `--carrying` 时计入，总负载 600 g |
| 安装方向 | 零位附近指尖朝基坐标 −Y，两指沿 X 方向开合 |
| 安装间隙 | 腕部端面到夹爪安装面约 2 mm |
| 重力方向 | 底座正放，`base_link` 中为 `[0, 0, -9.81] m/s²` |
| 线缆 | 不牵拉末端，没有计入线缆外力 |

夹爪的质量分布取自 DEX1-1 参考 URDF（`g1_with_dex.urdf`）中的 `left_dex1_base_link` 和两个等重的手指，
局部 +X 指向指尖，±Y 是开合方向。参考模型总质量 364.946 g，与实测不同，因此保持 CAD 合成重心不变，
把夹爪质量缩放到 547 g，再单独加上 3 g 转接件。这假设质量分布与 CAD 一致，重心并没有实际测量过。
两指对称开合时合成重心不变，约在安装面沿指尖方向 57.93 mm 处。

腕部端面取 SDK `Link7.STL` 的 `y = -47.73612 mm`，中心取 `x = z = 0`（CAD 估计）。
夹爪安装面在 `Link7` 的 `[0, -49.73612, 0] mm`，RPY 为 `[π, 0, -π/2]`，即夹爪局部 +X 对应 `Link7` 的 −Y，
局部 +Y 对应 `Link7` 的 −X。绕指尖方向翻转 180° 的两种安装无法从开合方向区分，这里取其中一种；
CAD 的横向重心偏移小于 0.1 mm，影响可以忽略。空总成的合成重心约在 `Link7` 的 `[0.0681, -107.3453, -0.0073] mm`。

抓取物体的重心暂定在安装面沿指尖方向 120 mm、两指中间。实际抓取位置不同时修改 `object_com_in_gripper_m`。
这个点只用于重力计算，不是 TCP 标定值，不会改变 IK 的 `--tcp`、默认准备姿态或 URDF 显示。

## 计算与控制

重力力矩是各连杆和工具重力势能对关节角的梯度 `g(q) = ∂U/∂q`，即零速度、零加速度时的逆动力学，
只需要质量和重心，不需要惯量。控制进程每个 100 Hz 周期用最新的实测关节角计算，叠加到位置 PD 上：

`tau = Kp * (q_command - q_measured) - Kd * dq_measured + scale * g(q_measured)`

PD 由电机执行，控制进程把平滑和限幅后的前馈写入 `ArmJointCmd.tau`。单位为 URDF 坐标下关节输出端的 N·m；
SDK 已处理标定方向和减速比，这里不再乘减速比。当前七个关节的标定 scale 都是 1；
如果标定 scale 不为 1，真机模式会拒绝开启补偿，需要先确认力矩的坐标换算。

- 补偿比例至少用 2 s 渐入，前馈每轴变化不超过 2 N·m/s。
- 前馈上限 J1–J7 为 `[8, 8, 6, 6, 4, 1.5, 1.5] N·m`，定义在 `keyboard_native/gravity_model.h`。
  它不是电机的额定力矩，也不限制电机内部 PD 加前馈后的总力矩。达到上限时状态中的 `gravity_saturated` 会置位。
- 启动探测和使能阶段不施加补偿，保持和阻尼保持期间持续补偿，停止、释放力矩和断开时撤销。
- 关节限位、平滑、命令领先和跟踪保护、心跳和反馈超时照常生效。
- 补偿比例和是否抓取物体在启动时指定，程序不会自动判断是否抓住了物体，负载变化后需要重启。
- 模拟驱动按理想位置跟踪计算，只验证力矩预测和状态切换，不模拟负载动态。

不包含惯性力、科氏力、摩擦辨识、外力估计和碰撞规划。

## 使用

```bash
./scripts/build_keyboard.sh
# 离线计算零位和准备姿态的空载、抓物力矩，不访问串口
.venv-ik/bin/python scripts/gravity_model.py
# 默认 tuned 配置：100% 补偿
./scripts/gui_control.sh --execute --port /dev/ttyUSB3
# 抓着约 50 g 物体时
./scripts/gui_control.sh --execute --port /dev/ttyUSB3 --carrying
# 指定其他比例或配置文件
./scripts/gui_control.sh --gravity-profile config/dex1_gravity.json --gravity-scale 0.25
```

`legacy` 配置下只给 `--gravity-profile` 不给 `--gravity-scale` 时，只计算预测值，前馈为零。
键盘工具接受同样的参数，但它在 `--execute` 下启动后直接连接真机。改变比例需要重启控制程序，
重启或断开会释放力矩，要先扶住机械臂。

## 在新的机械臂或夹爪上调试

换了机械臂、夹爪或安装方式时，不要直接用 100%：

1. 用空夹爪，保持默认速度和保护阈值，在能稳定支撑的姿态以 `--gravity-scale 0.25` 连接。
2. 静止观察几秒，确认没有持续漂移、跟踪误差变小，再做小幅关节运动。
3. 确认力矩符号和模型正确后，按 0.25 → 0.5 → 0.75 → 1 逐步提高。
4. 只有确实夹住物体、并核对过物体重心后才加 `--carrying`。

静态预测值（N·m，含机械臂本体和负载，未乘比例、未限幅）：

| 姿态 / 负载 | J2 | J4 | J6 |
| --- | ---: | ---: | ---: |
| 零位 / 550 g | 2.059 | 4.376 | 0.700 |
| 准备姿态 / 550 g | 0.471 | 4.648 | 0.700 |
| 准备姿态 / 600 g | 0.347 | 4.846 | 0.788 |

不补偿时，承重全靠位置误差：以 `legacy` 的 Kp 60 估算，550 g 下 J4 约有 `4.648 / 60 ≈ 0.078 rad` 的静差，
600 g 时约 0.081 rad，接近或超过默认 0.08 rad 的命令领先阈值，轨迹可能因此等待。
这种情况下动作迟缓不一定是 IK 无解。

## 需要的实测数据

调试时保留几秒静止和小幅运动的日志 `build/keyboard/logs/gui-*.jsonl`，其中包含
`q`、`command`、`dq`、`tau_est`、`tau_ff`、`gravity_arm_nm`、`gravity_tool_nm`、`gravity_gain`、
`gravity_saturated`、`lead_limited`。`tau_est` 是电机的估计值，不是力传感器测量。
如果能测量夹爪重心到安装面的距离，以及抓取时物体中心到安装面的距离，用实测值替换上面的 CAD 估计和抓取假设。

## 测试

离线测试覆盖：势能的有限差分、与 PyBullet 逆动力学的对比、Python 与 C++ 计算一致、
补偿渐入、限幅和变化率、保持时不重置承重参考、停止锁定，以及故障释放力矩时撤销补偿。

```bash
QT_QPA_PLATFORM=offscreen .venv-ik/bin/python -m pytest -q tests
ctest --test-dir build/keyboard --output-on-failure
```
