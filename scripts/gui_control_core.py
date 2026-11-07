"""GUI control model. Units: URDF radians and base_link metres; no Qt imports."""
import multiprocessing as mp
import queue
import time

import numpy as np
from scipy.spatial.transform import Rotation

from arm_control import ArmIK, errors, vector, POSE_IK_TIMEOUT
from gui_settings import validate, config_command

# Parent allows IPC/scheduling margin after the shared absolute solve deadline.
GUI_IK_TIMEOUT = POSE_IK_TIMEOUT + .5


def feedback_pose(arm, q):
    """Display real feedback, including permitted small excursions past limits."""
    transform = np.eye(4)
    for origin, axis, angle in zip(arm.origins, arm.axes, vector(q, 7)):
        rotation = np.eye(4)
        rotation[:3, :3] = Rotation.from_rotvec(axis*angle).as_matrix()
        transform = transform @ origin @ rotation
    return transform @ arm.tcp


def pose_solver_loop(requests, results, tcp):
    """Body of the solver process: solve_pose() for each (generation, target, seed, deadline)."""
    arm = ArmIK(tcp=tcp)
    arm.prepare_pose_solver()
    while True:
        job = requests.get()
        if job is None:
            return
        generation, target, seed, deadline = job
        try:
            q = arm.solve_pose(np.asarray(target), seed, deadline=deadline)
            results.put((generation, q.tolist(), "", arm.last_pose_report))
        except Exception as exc:
            results.put((generation, None, str(exc), getattr(arm, "last_pose_report", {})))


class PoseSolver:
    """One in-flight solve; the controller keeps only the latest slider target."""
    def __init__(self, tcp):
        self.tcp = tcp
        self.last_report = None
        self._start()

    def _start(self):
        context = mp.get_context("spawn")
        self.requests = context.Queue(maxsize=1)
        self.results = context.Queue(maxsize=1)
        self.process = context.Process(target=pose_solver_loop,
                                       args=(self.requests, self.results, self.tcp), daemon=True)
        self.process.start()

    def submit(self, generation, target, seed):
        self.requests.put_nowait((generation, target.tolist(), seed.tolist(), time.monotonic()+POSE_IK_TIMEOUT))

    def poll(self):
        if not self.process.is_alive():
            raise RuntimeError("IK 进程已退出")
        try:
            generation, solution, error, self.last_report = self.results.get_nowait()
            return generation, solution, error
        except queue.Empty:
            return None

    def close(self):
        self.process.terminate()
        self.process.join(timeout=.05)
        if self.process.is_alive():
            self.process.kill()
            self.process.join(timeout=.05)
        self.requests.cancel_join_thread()
        self.results.cancel_join_thread()
        self.requests.close()
        self.results.close()

    def restart(self):
        self.close()
        self._start()


class GuiController:
    """Slider/button state machine of the GUI, independent of Qt.

    Joint edits are sent as MOVE at most every 50 ms. Cartesian edits are
    solved asynchronously with a fixed orientation and only the latest
    target's result is executed; every interruption bumps `generation` so
    older results are dropped. Settings changes are applied with CONFIG and
    wait for the worker's acknowledgement.
    """

    def __init__(self, send, arm, solver, ready_q):
        self.send, self.arm, self.solver = send, arm, solver
        self.ready_q = arm.check_q(ready_q)
        self.state = None
        self.mode = "joint"
        self.initialized = self.stopped = self.recovering = False
        self.target_q = np.zeros(7)
        self.target_xyz = arm.fk(self.target_q)[:3, 3].copy()
        self.orientation = arm.fk(self.target_q)[:3, :3].copy()
        self.target_rpy = Rotation.from_matrix(self.orientation).as_euler("xyz")
        self.settings = None
        self.pending_config = None
        self.config_requested = 0
        self.config_started = 0.
        self.generation = 0
        self.dirty = False
        self.busy = False
        self.solve_generation = -1
        self.solve_started = self.last_edit = self.last_send = 0.
        self.await_hold = None
        self.preview_valid = True
        self.note = "等待驱动反馈"

    @property
    def enabled(self):
        return (self.initialized and not self.stopped and self.await_hold is None and self.pending_config is None
                and self.state is not None and self.state["mode"] in
                ("hold", "moving", "position", "home", "ready"))

    def _invalidate(self):
        self.generation += 1
        self.dirty = False

    def _sync(self, q):
        # Set the slider targets to q. Callers pass the command reference or a
        # new target, never measured q: re-seeding a load-supporting PD
        # reference from feedback would drop its equilibrium error.
        self.target_q = self.arm.check_q(q).copy()
        pose = self.arm.fk(q)
        self.target_xyz = pose[:3, 3].copy()
        self.orientation = pose[:3, :3].copy()
        self.target_rpy = Rotation.from_matrix(self.orientation).as_euler("xyz")
        self.preview_valid = True

    def update_state(self, state):
        """Follow one worker status record."""
        if state is None:
            return
        self.state = state
        if "config" in state:
            config = dict(state["config"])
            if "kp_base" in config:   # per-joint gains report kp/kd as arrays; edit the base pair
                config["kp"], config["kd"] = config["kp_base"], config["kd_base"]
            self.settings = validate(config)
        if self.pending_config is not None and state.get("config_ack") == self.pending_config:
            self.pending_config = None
            if state["mode"] == "hold":
                self._sync(state["command"])
                self.await_hold = None
            self.note = ("参数被拒绝，保留原设置：" + state["config_error"]
                         if state.get("config_error") else "参数已生效；当前位置保持")
        if self.recovering and state["mode"] == "damping":
            return   # frames queued before RECOVER took effect
        self.recovering = False
        if state["mode"] in ("fault", "stopped", "damping"):
            self.pending_config = None
            if not self.stopped:
                self._invalidate()
                self.stopped = True
            self.note = (("故障阻尼保持（未释放力矩）：" if state["mode"] == "damping" else "已停止 / 释放力矩：")
                         + state.get("reason", ""))
            return
        if not self.initialized and not self.stopped and state["mode"] == "hold":
            if "move" not in state.get("capabilities", []):
                raise RuntimeError("驱动版本过旧，请运行 scripts/build_keyboard.sh")
            self._sync(state["command"])
            self.initialized = True
            self.note = "已连接，滑块目标会平滑执行"
        if (self.await_hold is not None and state["seq"] > self.await_hold
                and state["mode"] == "hold"):
            self._sync(state["command"])
            self.await_hold = None

    def hold(self, note="保持当前命令位置"):
        self._invalidate()
        if self.initialized and not self.stopped:
            self.send("HOLD")
            self.await_hold = self.state["seq"]
        self.note = note

    def recover(self):
        """Damping (soft fault, torque kept) -> hold at the measured pose."""
        if self.state is None or self.state["mode"] != "damping":
            raise RuntimeError("恢复仅在故障阻尼保持时可用")
        self._invalidate()
        self.send("RECOVER")
        self.stopped = self.initialized = False   # re-sync at the next hold
        self.recovering = True
        self.note = "已发送恢复：在当前实测位置恢复位置保持"

    def stop(self):
        self._invalidate()
        self.stopped = True
        self.pending_config = None
        self.send("STOP")
        self.note = "已停止并释放力矩；断开后重新连接才能继续"

    def set_mode(self, mode):
        if mode not in ("joint", "cartesian"):
            raise ValueError("Unknown control mode")
        if mode != self.mode:
            self.mode = mode
            self.hold("切换控制模式，已取消旧目标")

    def edit_joint(self, index, radians, now):
        if not self.enabled or self.mode != "joint":
            return
        q = self.target_q.copy()
        q[index] = radians
        self._sync(self.arm.check_q(q))
        self.generation += 1
        self.dirty, self.last_edit = True, now
        self.note = "关节目标已更新"

    def edit_xyz(self, index, metres, now):
        if not self.enabled or self.mode != "cartesian":
            return
        xyz = self.target_xyz.copy()
        xyz[index] = metres
        self.target_xyz = vector(xyz, 3)
        self.generation += 1
        self.dirty, self.last_edit = True, now
        self.preview_valid = False
        self.note = "XYZ 目标待求解；上一有效轨迹继续执行"

    def edit_rpy(self, index, radians, now):
        if not self.enabled or self.mode != "cartesian":
            return
        rpy = self.target_rpy.copy()
        rpy[index] = radians
        self.target_rpy = vector(rpy, 3)
        self.orientation = Rotation.from_euler("xyz", rpy).as_matrix()
        self.generation += 1
        self.dirty, self.last_edit = True, now
        self.preview_valid = False
        self.note = "旋转目标待求解；保持所选 XYZ，上一有效轨迹继续执行"

    def apply_settings(self, values, now):
        values = validate(values)
        if not self.enabled:
            return
        if "config" not in self.state.get("capabilities", []):
            raise RuntimeError("驱动不支持参数设置，请运行 scripts/build_keyboard.sh")
        self.hold("正在保持并应用参数…")
        self.config_requested = max(self.config_requested, self.state.get("config_ack", 0)) + 1
        self.pending_config = self.config_requested
        self.config_started = now
        self.send(config_command(self.pending_config, values))

    def preset(self, name):
        if not self.enabled or name not in ("HOME", "READY"):
            return
        self._invalidate()
        self.mode = "joint"
        self._sync(np.zeros(7) if name == "HOME" else self.ready_q)
        self.send("HOLD")
        self.send(name)
        self.note = "平滑回零中" if name == "HOME" else "平滑移动到 P 点"

    def _move(self, q, now):
        q = self.arm.check_q(q)
        self.send("MOVE " + " ".join(format(v, ".12g") for v in q))
        self.last_send = now

    def tick(self, now):
        """Periodic step: settings timeout, IK results, solver restarts, new MOVE/solve."""
        if self.pending_config is not None and now-self.config_started > 1.:
            self.stop()
            raise RuntimeError("驱动参数确认超时")
        result = self.solver.poll()
        if result is not None:
            generation, solution, error = result
            self.busy = False
            if generation == self.generation and self.enabled and self.mode == "cartesian":
                if now-self.solve_started > GUI_IK_TIMEOUT:
                    self.hold("IK 结果已超时，已保持；请缩小位置变化")
                    self.preview_valid = False
                elif error:
                    self.hold("固定朝向 IK 未找到解，已保持：" + error)
                    self.preview_valid = False
                else:
                    q = self.arm.check_q(solution)
                    target = np.eye(4)
                    target[:3, 3], target[:3, :3] = self.target_xyz, self.orientation
                    pos_error, rot_error = errors(self.arm.fk(q), target)
                    if pos_error > .0002 or rot_error > .002:
                        self.hold("IK 校验失败，已保持")
                        self.preview_valid = False
                    else:
                        self.target_q = q.copy()
                        self.preview_valid = True
                        self._move(q, now)
                        report = getattr(self.solver, "last_report", None)
                        detail = ""
                        if report:
                            stages = {"seed": "原位", "local": "局部", "expanded": "扩大", "full": "全范围"}
                            detail = (f"（{stages.get(report['stage'], report['stage'])}搜索，"
                                      f"{report['elapsed_s']*1000:.0f} ms）")
                        self.note = "固定朝向位姿目标已提交平滑轨迹" + detail
        if (self.busy and self.solve_generation != self.generation and self.dirty and self.enabled
                and self.mode == "cartesian" and now-self.last_edit >= .08
                and now-self.solve_started >= .15):
            # Superseded hard targets must not occupy the entire search budget.
            self.solver.restart()
            self.busy = False
        if self.busy and now-self.solve_started > GUI_IK_TIMEOUT:
            # A long/stale solve cannot block STOP, rendering, or joint control.
            current = self.solve_generation == self.generation and self.mode == "cartesian"
            self.solver.restart()
            self.busy = False
            if current and self.enabled:
                self.hold("IK 超时，已保持；请缩小位置变化")
                self.preview_valid = False
        if not self.enabled or not self.dirty:
            return
        if self.mode == "joint" and now-self.last_send >= .05:
            self._move(self.target_q, now)
            self.dirty = False
        elif self.mode == "cartesian" and not self.busy and now-self.last_edit >= .08:
            target = np.eye(4)
            target[:3, 3], target[:3, :3] = self.target_xyz, self.orientation
            self.solver.submit(self.generation, target, self.arm.check_q(self.state["command"]))
            self.busy, self.dirty = True, False
            self.solve_started = now
            self.solve_generation = self.generation
            self.note = "正在求解末端位姿；保持按钮可随时中止"
