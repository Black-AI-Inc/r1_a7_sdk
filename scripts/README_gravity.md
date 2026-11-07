# R1-7A + DEX1-1 静态重力补偿

适用入口：`gui_control.sh`、`keyboard_control.sh` 共用的原生 worker。
默认补偿比例为 **0（关闭）**。已经离线验证，没有进行真机运动或负载验收。
旧 `test_arm_lowlevel --goto` 和 `arm_control.sh` 的执行仍走原 SDK，不自动获得本补偿。

## 负载与几何参数

`config/dex1_gravity.json` 记录 2026-10-28 确认的安装数据：

| 参数 | 当前值 |
| --- | --- |
| 空夹爪总成 | 550 g，已含 3 g 转接件，不重复相加 |
| 抓取物体 | 约 50 g，仅 `--carrying` 时计入，总负载 600 g |
| 安装方向 | 全零姿态附近指尖朝基坐标 −Y，两指沿 X 张开 |
| 安装间隙 | 腕部端面到夹爪安装面约 2 mm |
| 重力 | 底座正放，base_link 中 `[0,0,-9.81] m/s²` |
| 线缆 | 用户确认不牵拉末端，没有加入线缆外力 |

模型来源为工作区 `Dex1-1-r1-adapter-main/references/g1_with_dex.urdf` 中的
`left_dex1_base_link` 和两个等重手指；局部 +X 指向指尖，±Y 是开合方向。
参考总质量 364.946 g 与实测不同：目前将夹爪缩放到 547 g，保持 CAD 合成重心不变，
再单独加入 3 g 转接件。这是质量分布假设，尚不能替代实际重心测量。
对称开合时两指重心位移相抵，夹爪重心约在安装面沿指尖方向 **57.93 mm**。

腕部端面采用 SDK `Link7.STL` 的 `y=-47.73612 mm`，中心取 `x=z=0`，属于 CAD 估计。
夹爪安装面设在 Link7 的 `[0,-49.73612,0] mm`，RPY 为 `[π,0,-π/2]`。
夹爪局部 +X 对应 Link7 −Y，局部 +Y 对应 Link7 −X。
选择了两种可能的 180° 翻转方向之一；CAD 横向重心偏心不足 0.1 mm。
空总成合成重心约为 Link7 中 `[0.0681,-107.3453,-0.0073] mm`。
没有沿用旧转接件 14.5 mm 的间隔。

物体重心暂取夹爪安装面沿指尖方向 120 mm、开合中心处。实际抓取位置改变时，修改
`object_com_in_gripper_m`。这个假设**不是 TCP 标定值**。
本补偿不会修改 IK 的 `--tcp`、默认 P 关节角或 URDF 显示模型。
夹爪中心的位置控制仍需单独配置/标定 Link7→TCP。

## 控制方式

计算各臂节和工具重力势能的梯度 `g(q)=∂U/∂q`，相当于零速度、零加速度时的逆动力学。
100 Hz 循环使用新鲜实测关节角，叠加到原位置 PD：

`tau = Kp*(q_command-q_measured) - Kd*dq_measured + scale*g(q_measured)`。

PD 由电机执行；worker 将平滑、限幅后的前馈写入 `ArmJointCmd.tau`。
单位是 URDF 坐标、关节输出端 N·m。SDK 已处理校准方向和减速比，不再乘减速比。
当前七轴校准 scale 均为 1；若改成非 1，Python 入口拒绝开启补偿，须先审查力矩坐标映射。

- 比例至少用 2 秒渐入，前馈变化不超过每轴 2 N·m/s。
- 前馈初调上限 J1–J7 为 `[8,8,6,6,4,1.5,1.5] N·m`，定义在 `keyboard_native/gravity_model.h`。
  这不是电机持续额定力矩，也不限制电机内部 PD 加前馈后的总力矩。
  限幅通过 `gravity_saturated` 回报；不要用扩大跟踪阈值代替负载能力核验。
- probing/enabling 不施加补偿；保持时持续补偿；停止/故障/断开撤销力矩。
  **STOP 是释放力矩，不等于负载保持。**
- 原关节限位、平滑、命令领先/跟踪保护、心跳和反馈超时保持生效。
- 比例及空载/抓物状态在启动时指定；负载改变后重启对应配置，不自动判断是否抓住物体。
- 模拟驱动仍是理想位置模型，验证力矩预测和状态机，不模拟真实负载动态。

静态重力补偿只需要质量和重心，不需要惯量张量；当前不含惯性、科氏力、摩擦辨识、
外力估计和工具碰撞规划，不能保证零位到任意 P 位的路径无碰撞。

## 使用与初调

在 SDK 目录运行：

```bash
./scripts/build_keyboard.sh
# 离线预测 Home/P 的空载和抓物力矩，不访问串口
.venv-ik/bin/python scripts/gravity_model.py
# 模拟界面，观察 25% 补偿渐入和逐轴力矩读数
./scripts/gui_control.sh --gravity-profile config/dex1_gravity.json --gravity-scale 0.25
```

不传 `--gravity-scale` 时，即使给了 profile，也只计算预测、前馈保持为零。
真机初调使用空夹爪，保留原速度和保护阈值，先在能稳定支撑的姿态记录静止反馈。
下面命令打开 GUI，点击连接后才保持当前位置并渐入 25% 补偿，不自动回零/P：

```bash
./scripts/gui_control.sh --execute --port /dev/ttyUSB3 \
  --gravity-profile config/dex1_gravity.json --gravity-scale 0.25
```

先观察几秒静止过程中是否持续漂移、跟踪偏差是否减小，再做小幅关节运动。
确认符号与模型后，再按 0.25→0.5→0.75→1 逐步验证；首轮不直接用 1。
改变比例需要重启控制器，重启/断开会释放力矩，应先支撑住机械臂。
键盘入口接受同样参数，但 `--execute` 启动后直接连接真机。
确实夹住约 50 g 物体并核对物体重心后，再加 `--carrying`；空夹爪不加。

静态预测（N·m，含臂本体与负载，未乘比例、未限幅）：

| 姿态 / 负载 | J2 | J4 | J6 |
| --- | ---: | ---: | ---: |
| Home / 550 g | 2.059 | 4.376 | 0.700 |
| P / 550 g | 0.471 | 4.648 | 0.700 |
| P / 600 g | 0.347 | 4.846 | 0.788 |

P 按钮执行预设关节目标，不运行 IK。按 Kp=60 估算，550 g 时 J4 的纯 PD 静差约
`4.648/60=0.0775 rad`；600 g 时约 `0.0808 rad`，接近/超过默认 0.08 rad 领先阈值。
所以可能出现轨迹等待，不能仅根据动作迟缓判定 IK 无解。这是模型解释，需要实测验证。

## 需要配合的实测

保留数秒静止和小幅运动的 `build/keyboard/logs/gui-*.jsonl`，包含
`q/command/dq/tau_est/tau_ff/gravity_arm_nm/gravity_tool_nm/gravity_gain/gravity_saturated/lead_limited`。
其中 `tau_est` 是电机反馈估计，非外置力传感器。
再测量夹爪重心到安装面的距离；抓物时测量物体中心到安装面的距离，用于替换上述 CAD/抓取假设。

离线验证包含：势能有限差分、独立 PyBullet 逆动力学、Python/C++ 一致性，
补偿渐入/限幅/变化率、保持不重置承重参考、STOP 锁定和心跳故障撤销补偿。

```bash
QT_QPA_PLATFORM=offscreen .venv-ik/bin/python -m pytest tests -q
ctest --test-dir build/keyboard --output-on-failure
```
