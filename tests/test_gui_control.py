"""GUI controller, Qt window, URDF view and native worker tests.

All robot control tests use --mock. Qt and URDF rendering run offscreen.
"""
import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import json
import signal
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from PySide6.QtCore import Qt, QPoint
from PySide6.QtTest import QTest
from PySide6.QtGui import QFont
from PySide6.QtWidgets import QApplication

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"scripts"))
from arm_control import ArmIK
from keyboard_control import DEFAULT_READY_Q, Worker
from gui_control_core import GuiController, feedback_pose, GUI_IK_TIMEOUT
from gui_control import ControlWindow, parse_args as _parse_args, STYLE
from gui_urdf import URDFScene


# These tests run the legacy profile (the original tool defaults);
# tests/test_tuned_profile.py covers the default tuned profile.
def parse_args(argv=()):
    return _parse_args(["--profile", "legacy", *argv])


class FakeSolver:
    def __init__(self):
        self.jobs, self.result, self.restarts = [], None, 0

    def submit(self, *job):
        self.jobs.append(job)

    def poll(self):
        result, self.result = self.result, None
        return result

    def restart(self):
        self.restarts += 1


def state(q=DEFAULT_READY_Q, seq=0, mode="hold", load=None):
    command = np.asarray(q)
    return {"q": (command-(np.zeros(7) if load is None else load)).tolist(),
            "command": command.tolist(), "target": command.tolist(), "seq": seq,
            "mode": mode, "capabilities": ["move"], "reason": "test"}


def make_controller():
    arm, sent, solver = ArmIK(), [], FakeSolver()
    controller = GuiController(sent.append, arm, solver, DEFAULT_READY_Q)
    controller.update_state(state())
    return controller, sent, solver


def test_joint_edits_coalesce_and_feedback_never_rebases_target():
    c, sent, solver = make_controller()
    initial = c.target_q.copy()
    for angle in [.01, .02, .03]:
        c.edit_joint(0, angle, 1)
    c.update_state(state(load=[0, .015, 0, .01, 0, 0, 0]))
    c.tick(1.1)
    assert len(sent) == 1 and sent[0].startswith("MOVE ")
    commanded = np.array([float(v) for v in sent[0].split()[1:]])
    assert commanded[0] == .03
    np.testing.assert_allclose(commanded[1:], initial[1:])
    assert not solver.jobs


def test_hold_waits_for_fresh_ack_and_preserves_load_error():
    c, sent, solver = make_controller()
    c.hold()
    assert not c.enabled
    c.update_state(state(seq=0, load=[0, .015, 0, .01, 0, 0, 0]))
    assert not c.enabled
    c.update_state(state(seq=1, load=[0, .015, 0, .01, 0, 0, 0]))
    assert c.enabled
    np.testing.assert_allclose(c.target_q, DEFAULT_READY_Q)
    assert sent == ["HOLD"]


def test_cartesian_latest_wins_and_stale_solution_cannot_move():
    c, sent, solver = make_controller()
    c.set_mode("cartesian")
    c.update_state(state(seq=1))
    locked = c.orientation.copy()
    z = c.target_xyz[2]
    c.edit_xyz(2, z+.002, 1)
    c.tick(1.1)
    old_generation = solver.jobs[-1][0]
    c.edit_xyz(2, z+.004, 1.2)
    solver.result = (old_generation, DEFAULT_READY_Q, "")
    c.tick(1.3)
    assert not any(s.startswith("MOVE") for s in sent)
    generation, target, seed = solver.jobs[-1]
    np.testing.assert_allclose(seed, DEFAULT_READY_Q)
    np.testing.assert_allclose(target[:3, :3], locked)
    solution = c.arm.solve(target, seed, max_step=.6)
    solver.result = (generation, solution, "")
    c.tick(1.4)
    assert sent[-1].startswith("MOVE ")
    assert c.preview_valid
    np.testing.assert_allclose(c.arm.fk(c.target_q)[:3, 3], target[:3, 3], atol=.0002)


@pytest.mark.parametrize("action", ["hold", "stop", "joint", "HOME", "READY"])
def test_interrupt_invalidates_pending_pose_solve(action):
    c, sent, solver = make_controller()
    c.set_mode("cartesian")
    c.update_state(state(seq=1))
    c.edit_xyz(2, c.target_xyz[2]+.002, 1)
    c.tick(1.1)
    generation = solver.jobs[-1][0]
    if action in ("hold", "stop"):
        getattr(c, action)()
    elif action == "joint":
        c.set_mode("joint")
    else:
        c.preset(action)
    solver.result = (generation, DEFAULT_READY_Q, "")
    c.tick(1.2)
    assert not any(s.startswith("MOVE") for s in sent)


def test_unreachable_pose_holds_and_does_not_release():
    c, sent, solver = make_controller()
    c.set_mode("cartesian")
    c.update_state(state(seq=1))
    c.edit_xyz(2, 2., 1)
    c.tick(1.1)
    solver.result = (c.generation, None, "unreachable")
    c.tick(1.2)
    assert sent[-1] == "HOLD" and "MOVE" not in " ".join(sent)
    assert not c.stopped


def test_slow_stale_solver_cannot_interrupt_new_joint_motion():
    c, sent, solver = make_controller()
    c.set_mode("cartesian")
    c.update_state(state(seq=1))
    c.edit_xyz(2, c.target_xyz[2]+.002, 1)
    c.tick(1.1)
    c.set_mode("joint")
    c.update_state(state(seq=2))
    c.edit_joint(0, .02, 2)
    c.tick(2)
    assert sent[-1].startswith("MOVE ")
    count = len(sent)
    c.tick(1.1 + GUI_IK_TIMEOUT + .1)
    assert solver.restarts == 1 and len(sent) == count


def test_current_solver_timeout_holds_and_restarts_solver():
    c, sent, solver = make_controller()
    c.set_mode("cartesian")
    c.update_state(state(seq=1))
    c.edit_xyz(2, c.target_xyz[2]+.002, 1)
    c.tick(1.1)
    c.tick(1.1 + GUI_IK_TIMEOUT + .1)
    assert solver.restarts == 1 and sent[-1] == "HOLD"


def test_late_valid_ik_result_is_not_executed():
    c, sent, solver = make_controller()
    c.set_mode("cartesian")
    c.update_state(state(seq=1))
    c.edit_xyz(2, c.target_xyz[2]+.002, 1)
    c.tick(1.1)
    generation, target, seed = solver.jobs[-1]
    solver.result = (generation, c.arm.solve(target, seed, max_step=.6), "")
    c.tick(1.1 + GUI_IK_TIMEOUT + .1)
    assert sent[-1] == "HOLD"
    assert not any(s.startswith("MOVE ") for s in sent)


def test_stop_latch_and_capability_validation():
    c, sent, solver = make_controller()
    c.stop()
    c.edit_joint(0, .1, 1)
    c.tick(1.1)
    c.preset("HOME")
    assert sent == ["STOP"]
    other = GuiController(sent.append, c.arm, solver, DEFAULT_READY_Q)
    old = state()
    old.pop("capabilities")
    with pytest.raises(RuntimeError, match="版本过旧"):
        other.update_state(old)


def test_render_urdf_matches_fk_and_changes_with_joints():
    scene = URDFScene()
    camera = {"distance": .75, "yaw": 45., "pitch": -22.}
    try:
        first = scene.render(np.zeros(7), camera, 320, 240)
        second = scene.render(DEFAULT_READY_Q, camera, 320, 240)
        assert first != second and len(second) == 320*240*4
        pixels = np.frombuffer(second, dtype=np.uint8).reshape(240, 320, 4)
        np.testing.assert_array_equal(pixels[[0, 0, -1, -1], [0, -1, 0, -1], :3], 255)
        actual = scene.p.getLinkState(scene.robot, scene.joints["J7"], computeForwardKinematics=True,
                                      physicsClientId=scene.client)
        fk = scene.arm.fk(DEFAULT_READY_Q)
        np.testing.assert_allclose(actual[4], fk[:3, 3], atol=1e-7)
        np.testing.assert_allclose(Rotation.from_quat(actual[5]).as_matrix(), fk[:3, :3], atol=2e-7)
        biased = np.zeros(7)
        biased[1] = -.015
        scene.set_q(biased)
        actual = scene.p.getLinkState(scene.robot, scene.joints["J7"], computeForwardKinematics=True,
                                      physicsClientId=scene.client)
        np.testing.assert_allclose(actual[4], feedback_pose(scene.arm, biased)[:3, 3], atol=1e-7)
    finally:
        scene.close()


def wait_worker(worker, predicate, timeout=4):
    end = time.monotonic()+timeout
    while time.monotonic() < end:
        worker.send("PING")
        status = worker.poll()
        if status and predicate(status):
            return status
        time.sleep(.01)
    raise AssertionError(worker.state)


def test_native_absolute_move_keeps_smoothing_and_can_hold():
    worker = Worker(parse_args([]), ArmIK())
    try:
        wait_worker(worker, lambda s: s["mode"] == "hold")
        worker.send("MOVE 0 .35 0 .3 0 .15 0")
        moved = wait_worker(worker, lambda s: s["mode"] == "position" and s["q"][1] > .01)
        assert moved["q"][1] < .35
        assert max(np.abs(moved["command_velocity"])) <= .15+1e-8
        worker.send("HOLD")
        held = wait_worker(worker, lambda s: s["mode"] == "hold")
        time.sleep(.1)
        np.testing.assert_allclose(worker.poll()["command"], held["command"])
        worker.send("MOVE 0 -0.1 0 0 0 0 0")
        assert wait_worker(worker, lambda s: s["mode"] == "fault")["reason"] == "invalid_joint_target"
    finally:
        worker.close()


@pytest.fixture(scope="module")
def app():
    application = QApplication.instance() or QApplication([])
    application.setStyle("Fusion")
    application.setFont(QFont("Noto Sans CJK SC", 10))
    application.setStyleSheet(STYLE)
    application.setQuitOnLastWindowClosed(False)
    return application


def qt_wait(app, predicate, timeout=8):
    end = time.monotonic()+timeout
    while time.monotonic() < end:
        app.processEvents()
        if predicate():
            return
        time.sleep(.01)
    raise AssertionError("Qt condition timed out")


def test_gui_hardware_never_autoconnects(app, tmp_path):
    window = ControlWindow(parse_args(["--execute", "--log", str(tmp_path/"hardware-no-connect.jsonl")]),
                            start_renderer=False)
    try:
        window.show()
        app.processEvents()
        assert window.worker is None and window.solver is None
        assert not window.joint_rows[0].slider.isEnabled()
    finally:
        window.close()


def test_gui_gravity_readout_and_stop(app, tmp_path):
    from gravity_model import DEFAULT_PROFILE
    window = ControlWindow(parse_args(["--log", str(tmp_path/"gravity-gui.jsonl"),
        "--gravity-profile", str(DEFAULT_PROFILE), "--gravity-scale", ".25"]))
    try:
        window.show()
        qt_wait(app, lambda: window.controller is not None and window.controller.enabled
                and window.controller.state.get("gravity_gain") == .25)
        qt_wait(app, lambda: window.view.frame is not None)
        assert "25%" in window.gravity_status.text() and "N·m" in window.gravity_status.text()
        screenshot = os.getenv("R1_GRAVITY_SCREENSHOT")
        if screenshot:
            window.grab().save(screenshot)
        QTest.mouseClick(window.stop_button, Qt.LeftButton)
        qt_wait(app, lambda: window.controller.state["mode"] == "stopped")
        assert window.controller.state["tau_ff"] == [0]*7
        assert "0%" in window.gravity_status.text()
    finally:
        window.close()


def test_qt_sliders_live_values_urdf_hold_and_stop(app, tmp_path):
    log = tmp_path/"gui.jsonl"
    window = ControlWindow(parse_args(["--log", str(log)]))
    try:
        window.show()
        qt_wait(app, lambda: window.controller is not None and window.controller.enabled)
        qt_wait(app, lambda: window.view.frame is not None)
        initial = window.view.frame.copy()
        QTest.mouseClick(window.p_button, Qt.LeftButton)
        qt_wait(app, lambda: window.controller.state["mode"] == "hold"
                and np.max(np.abs(np.asarray(window.controller.state["command"])-DEFAULT_READY_Q)) < 1e-6)
        qt_wait(app, lambda: window.view.frame != initial)
        assert window.pose_table.item(2, 2).text() != "—"
        window.joint_rows[0].spin.setFocus()
        previous = window.controller.target_q[0]
        QTest.keyClick(window.joint_rows[0].spin, Qt.Key_Up)
        qt_wait(app, lambda: window.controller.target_q[0] > previous)
        qt_wait(app, lambda: abs(window.controller.state["command"][0]-window.controller.target_q[0]) < 1e-6)
        QTest.mouseClick(window.xyz_mode, Qt.LeftButton, pos=QPoint(10, window.xyz_mode.height()//2))
        qt_wait(app, lambda: window.controller.enabled and window.controller.mode == "cartesian")
        z = window.controller.target_xyz[2]
        window.xyz_rows[2].spin.setValue(z*1000+2)
        qt_wait(app, lambda: window.controller.preview_valid and window.controller.target_xyz[2] > z+.0015)
        qt_wait(app, lambda: window.controller.state["mode"] == "hold" and
                abs(feedback_pose(window.arm, window.controller.state["q"])[2, 3]-window.controller.target_xyz[2]) < .0002)
        screenshot = os.getenv("R1_GUI_TEST_SCREENSHOT")
        if screenshot:
            Path(screenshot).parent.mkdir(parents=True, exist_ok=True)
            window.grab().save(screenshot)
        QTest.mouseClick(window.joint_mode, Qt.LeftButton, pos=QPoint(10, window.joint_mode.height()//2))
        qt_wait(app, lambda: window.controller.enabled and window.controller.mode == "joint")
        window.joint_rows[0].spin.setValue(30.)
        qt_wait(app, lambda: window.controller.state["mode"] == "position")
        QTest.mouseClick(window.hold_button, Qt.LeftButton)
        qt_wait(app, lambda: window.controller.enabled and window.controller.state["mode"] == "hold")
        frozen = window.controller.state["command"].copy()
        app.processEvents()
        np.testing.assert_allclose(window.controller.target_q, frozen)
        QTest.mouseClick(window.stop_button, Qt.LeftButton)
        qt_wait(app, lambda: window.controller.state["mode"] == "stopped")
        assert not window.joint_rows[0].slider.isEnabled()
        assert window.controller.stopped
        processes = [window.worker.process, window.solver.process, window.renderer.process]
    finally:
        window.close()
    assert processes[0].poll() is not None
    assert all(not p.is_alive() for p in processes[1:])
    records = [json.loads(line) for line in log.read_text().splitlines()]
    assert records[-1]["event"] == "gui_exit"


def test_gui_cli_mock_sigterm(tmp_path):
    log = tmp_path/"cli.jsonl"
    proc = subprocess.Popen([sys.executable, str(Path(__file__).resolve().parents[1]/"scripts/gui_control.py"),
                             "--log", str(log)], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
        time.sleep(2)
        assert proc.poll() is None
        proc.send_signal(signal.SIGTERM)
        _, stderr = proc.communicate(timeout=6)
        assert proc.returncode == 0, stderr.decode()
        records = [json.loads(line) for line in log.read_text().splitlines()]
        assert any(r["event"] == "state" for r in records)
        assert records[-1]["event"] == "gui_exit"
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.wait()


@pytest.mark.parametrize("axis", range(3))
def test_rpy_rotation_preserves_xyz_and_reaches_requested_orientation(axis):
    c, sent, solver = make_controller()
    c.set_mode("cartesian")
    c.update_state(state(seq=1))
    xyz = c.target_xyz.copy()
    requested = c.target_rpy.copy()
    requested[axis] += np.radians(.5)
    c.edit_rpy(axis, requested[axis], 1)
    c.tick(1.1)
    generation, target, seed = solver.jobs[-1]
    np.testing.assert_array_equal(target[:3, 3], xyz)
    np.testing.assert_allclose(target[:3, :3], Rotation.from_euler("xyz", requested).as_matrix())
    solution = c.arm.solve(target, seed, max_step=.6)
    solver.result = (generation, solution, "")
    c.tick(1.2)
    assert sent[-1].startswith("MOVE ") and c.preview_valid
    actual = c.arm.fk(c.target_q)
    assert np.linalg.norm(actual[:3, 3]-xyz) < .0002
    assert Rotation.from_matrix(actual[:3, :3]@target[:3, :3].T).magnitude() < .002
    np.testing.assert_array_equal(c.target_rpy, requested)


def test_rotation_invalidates_older_translation_and_preserves_angle_representation():
    c, sent, solver = make_controller()
    c.set_mode("cartesian")
    c.update_state(state(seq=1))
    c.edit_xyz(2, c.target_xyz[2]+.001, 1)
    c.tick(1.1)
    old = c.generation
    c.edit_rpy(2, np.pi, 1.2)
    solver.result = (old, DEFAULT_READY_Q, "")
    c.tick(1.3)
    assert not any(s.startswith("MOVE") for s in sent)
    assert c.target_rpy[2] == np.pi
    assert solver.jobs[-1][0] == c.generation


def test_settings_ack_cancels_ik_and_never_rebases_to_loaded_feedback():
    from gui_settings import DEFAULTS
    c, sent, solver = make_controller()
    c.set_mode("cartesian")
    updated = state(seq=1)
    updated.update(capabilities=["move", "config"], config=DEFAULTS, config_ack=0)
    c.update_state(updated)
    c.edit_rpy(0, c.target_rpy[0]+.01, 1)
    c.tick(1.1)
    old_generation = c.generation
    values = dict(DEFAULTS, joint_speed=.05)
    c.apply_settings(values, 1.2)
    assert sent[-2] == "HOLD" and sent[-1].startswith("CONFIG 1 ")
    assert not c.enabled
    solver.result = (old_generation, DEFAULT_READY_Q, "")
    c.tick(1.3)
    assert not any(s.startswith("MOVE") for s in sent)
    c.update_state(dict(updated, seq=2))
    assert not c.enabled and c.settings["joint_speed"] == .15
    confirmed = state(seq=3, load=[0, .015, 0, .01, 0, 0, 0])
    confirmed.update(config=values, config_ack=1, config_error="")
    c.update_state(confirmed)
    assert c.enabled and c.settings == values
    np.testing.assert_allclose(c.target_q, DEFAULT_READY_Q)


def test_settings_missing_ack_stops_and_invalid_input_does_not_send():
    from gui_settings import DEFAULTS
    c, sent, solver = make_controller()
    updated = state()
    updated["capabilities"].append("config")
    c.update_state(updated)
    with pytest.raises(ValueError):
        c.apply_settings(dict(DEFAULTS, lead_limit=.2), 1)
    assert not sent
    c.apply_settings(DEFAULTS, 1)
    with pytest.raises(RuntimeError, match="确认超时"):
        c.tick(2.1)
    assert c.stopped and sent[-1] == "STOP"


@pytest.mark.parametrize("key,value", [("joint_speed", 0), ("joint_speed", 1.01),
    ("joint_accel", float("nan")), ("joint_jerk", 51), ("kp", 101), ("kd", 0),
    ("lead_limit", .2), ("tracking_limit", .08), ("move_timeout", 301)])
def test_settings_validation(key, value):
    from gui_settings import DEFAULTS, validate
    with pytest.raises(ValueError):
        validate(dict(DEFAULTS, **{key: value}))


def test_native_runtime_settings_atomic_ack_limits_and_stop_latch():
    from gui_settings import DEFAULTS, config_command
    worker = Worker(parse_args([]), ArmIK(), mock_q=DEFAULT_READY_Q,
                    mock_load=[0, .015, 0, .01, 0, 0, 0])
    try:
        initial = wait_worker(worker, lambda s: s["mode"] == "hold")
        assert {k: initial["config"][k] for k in DEFAULTS} == DEFAULTS
        # The legacy profile leaves the optional worker features off: no
        # integral, torque release on faults, no startup recovery.
        assert {k: initial["config"][k] for k in ("ki", "fault_kd", "recover_margin")} == dict(
            ki=0, fault_kd=0, recover_margin=0)
        values = dict(DEFAULTS, joint_speed=.05, joint_accel=.2, joint_jerk=1., kp=55., kd=.6,
                      lead_limit=.06, tracking_limit=.15, move_timeout=30.)
        worker.send(config_command(1, values))
        ack = wait_worker(worker, lambda s: s["config_ack"] == 1)
        assert ({k: ack["config"][k] for k in DEFAULTS} == values and ack["config_revision"] == 1
                and not ack["config_error"])
        np.testing.assert_allclose(ack["command"], initial["command"])
        # Slow motion actually uses the new native trajectory bounds.
        goal = np.array(DEFAULT_READY_Q)
        goal[0] += .2
        worker.send("MOVE " + " ".join(map(str, goal)))
        moving = wait_worker(worker, lambda s: abs(s["command_velocity"][0]) > .045)
        assert max(np.abs(moving["command_velocity"])) <= .05+1e-8
        assert max(np.abs(moving["command_acceleration"])) <= .2+1e-8
        # Invalid interdependent thresholds reject the entire update and hold.
        worker.send("CONFIG 2 .2 .5 2 60 .5 .2 .08 60")
        rejected = wait_worker(worker, lambda s: s["config_ack"] == 2)
        assert rejected["mode"] == "hold" and rejected["config_error"]
        assert {k: rejected["config"][k] for k in DEFAULTS} == values and rejected["config_revision"] == 1
        assert rejected["command"][1]-rejected["q"][1] == pytest.approx(.015)
        # A further valid update after rejection succeeds.
        worker.send(config_command(3, DEFAULTS))
        restored = wait_worker(worker, lambda s: s["config_revision"] == 3)
        assert {k: restored["config"][k] for k in DEFAULTS} == DEFAULTS
        np.testing.assert_allclose(restored["command"], rejected["command"])
        worker.send("STOP")
        wait_worker(worker, lambda s: s["mode"] == "stopped")
        worker.send(config_command(4, values))
        last = wait_worker(worker, lambda s: s["seq"] > restored["seq"]+3)
        assert last["mode"] == "stopped" and last["config_revision"] == 3
    finally:
        worker.close()


@pytest.mark.parametrize("guard", ["lead", "tracking"])
def test_native_configured_tracking_guards(guard):
    from gui_settings import DEFAULTS, config_command
    worker = Worker(parse_args([]), ArmIK(), mock_q=DEFAULT_READY_Q, mock_load=[.03, 0, 0, 0, 0, 0, 0])
    try:
        wait_worker(worker, lambda s: s["mode"] == "hold")
        values = dict(DEFAULTS, lead_limit=.01, tracking_limit=.02 if guard == "tracking" else .2)
        worker.send(config_command(1, values))
        ack = wait_worker(worker, lambda s: s["config_ack"] == 1)
        if guard == "tracking":
            assert ack["mode"] == "fault" and ack["reason"] == "tracking_error"
        else:
            goal = np.array(DEFAULT_READY_Q)
            goal[0] += .1
            worker.send("MOVE " + " ".join(map(str, goal)))
            limited = wait_worker(worker, lambda s: s["lead_limited"])
            np.testing.assert_allclose(limited["command"], ack["command"])
            assert limited["mode"] == "position"
    finally:
        worker.close()


def test_qt_settings_apply_defaults_and_rotation(app, tmp_path):
    from gui_settings import DEFAULTS
    window = ControlWindow(parse_args(["--log", str(tmp_path/"settings.jsonl")]))
    try:
        window.show()
        qt_wait(app, lambda: window.controller is not None and window.controller.enabled)
        QTest.mouseClick(window.p_button, Qt.LeftButton)
        qt_wait(app, lambda: window.controller.state["mode"] == "hold" and
                np.max(np.abs(np.asarray(window.controller.state["command"])-DEFAULT_READY_Q)) < 1e-6)
        QTest.mouseClick(window.xyz_mode, Qt.LeftButton)
        qt_wait(app, lambda: window.controller.enabled and window.controller.mode == "cartesian")
        old_xyz = window.controller.target_xyz.copy()
        old_rpy = window.controller.target_rpy.copy()
        window.rpy_rows[0].spin.setValue(np.degrees(old_rpy[0])+.5)
        qt_wait(app, lambda: window.controller.preview_valid and not window.controller.dirty and
                window.controller.state["mode"] == "hold" and not window.controller.busy and
                np.max(np.abs(np.asarray(window.controller.state["command"])-window.controller.target_q)) < 1e-6)
        pose = feedback_pose(window.arm, window.controller.state["q"])
        assert np.linalg.norm(pose[:3, 3]-old_xyz) < .0002
        assert Rotation.from_matrix(pose[:3, :3]@window.controller.orientation.T).magnitude() < .002
        assert window.controller.target_rpy[0]-old_rpy[0] > .008
        QTest.mouseClick(window.settings_mode, Qt.LeftButton)
        qt_wait(app, lambda: window.controller.enabled and window.pages.currentIndex() == 2)
        window.setting_inputs["joint_speed"].setValue(.08)
        assert window.controller.settings["joint_speed"] == .15
        QTest.mouseClick(window.apply_button, Qt.LeftButton)
        qt_wait(app, lambda: window.controller.enabled and window.controller.settings["joint_speed"] == .08)
        assert window.args.joint_speed == .08 and window.setting_actual["joint_speed"].text() == "0.08"
        screenshot = os.getenv("R1_GUI_SETTINGS_SCREENSHOT")
        if screenshot:
            window.grab().save(screenshot)
        QTest.mouseClick(window.defaults_button, Qt.LeftButton)
        assert window.setting_inputs["joint_speed"].value() == .15
        assert window.controller.settings["joint_speed"] == .08
        QTest.mouseClick(window.apply_button, Qt.LeftButton)
        qt_wait(app, lambda: window.controller.enabled and window.controller.settings == DEFAULTS)
        window.setting_inputs["lead_limit"].setValue(.25)
        QTest.mouseClick(window.apply_button, Qt.LeftButton)
        assert "必须小于" in window.settings_status.text()
        assert window.controller.enabled and window.controller.settings == DEFAULTS
        QTest.mouseClick(window.stop_button, Qt.LeftButton)
        qt_wait(app, lambda: window.controller.stopped)
        assert not window.apply_button.isEnabled()
    finally:
        window.close()


def test_stale_hard_pose_search_is_preempted_for_latest_target():
    c, sent, solver = make_controller()
    c.set_mode("cartesian")
    c.update_state(state(seq=1))
    c.edit_xyz(2, c.target_xyz[2]+.1, 1)
    c.tick(1.1)
    old = c.generation
    c.edit_xyz(0, c.target_xyz[0]+.002, 1.2)
    c.tick(1.4)
    assert solver.restarts == 1 and len(solver.jobs) == 2
    assert solver.jobs[-1][0] != old
    assert not any(s.startswith("MOVE") for s in sent)


def test_pose_result_after_two_seconds_within_gui_timeout_executes():
    c, sent, solver = make_controller()
    c.set_mode("cartesian")
    c.update_state(state(seq=1))
    c.edit_xyz(2, c.target_xyz[2]+.2, 1)
    c.tick(1.1)
    generation, target, seed = solver.jobs[-1]
    solution = c.arm.solve_pose(target, seed)
    solver.result = (generation, solution, "")
    c.tick(3.2)   # 2.1 s after submission, inside GUI_IK_TIMEOUT
    assert c.preview_valid and sent[-1].startswith("MOVE ")


def test_pose_process_uses_expanded_fixed_orientation_search():
    from gui_control_core import PoseSolver
    arm = ArmIK()
    seed = np.asarray(DEFAULT_READY_Q)
    target = arm.fk(seed)
    target[2, 3] += .2
    solver = PoseSolver([0]*6)
    try:
        solver.submit(42, target, seed)
        until = time.monotonic()+GUI_IK_TIMEOUT
        result = None
        while result is None and time.monotonic() < until:
            result = solver.poll()
            time.sleep(.01)
        assert result is not None
        generation, q, error = result
        assert generation == 42 and not error
        from arm_control import errors
        pe, re = errors(arm.fk(q), target)
        assert pe <= .0001 and re <= .001
        assert solver.last_report["stage"] in ("expanded", "full")
    finally:
        solver.close()


def test_gui_large_fixed_orientation_target_reaches_in_mock(app, tmp_path):
    # Faster trajectory settings only for the ideal mock plant; no hardware.
    window = ControlWindow(parse_args(["--joint-speed", ".5", "--joint-accel", "1",
                                       "--joint-jerk", "5", "--log", str(tmp_path/"large-pose.jsonl")]),
                           start_renderer=False)
    try:
        window.show()
        qt_wait(app, lambda: window.controller is not None and window.controller.enabled)
        QTest.mouseClick(window.p_button, Qt.LeftButton)
        qt_wait(app, lambda: window.controller.state["mode"] == "hold" and
                np.max(np.abs(np.asarray(window.controller.state["command"])-DEFAULT_READY_Q)) < 1e-6)
        QTest.mouseClick(window.xyz_mode, Qt.LeftButton)
        qt_wait(app, lambda: window.controller.enabled and window.controller.mode == "cartesian")
        before = window.arm.fk(window.controller.state["command"])
        window.xyz_rows[2].spin.setValue(before[2, 3]*1000+200)
        qt_wait(app, lambda: window.controller.preview_valid and window.controller.target_xyz[2] > before[2, 3]+.199)
        qt_wait(app, lambda: window.controller.state["mode"] == "hold" and
                np.max(np.abs(np.asarray(window.controller.state["command"])-window.controller.target_q)) < 1e-6)
        actual = window.arm.fk(window.controller.state["command"])
        assert abs(actual[2, 3]-before[2, 3]-.2) < .0002
        assert Rotation.from_matrix(actual[:3, :3]@before[:3, :3].T).magnitude() < .001
        assert window.solver.last_report["stage"] in ("expanded", "full")
    finally:
        window.close()
