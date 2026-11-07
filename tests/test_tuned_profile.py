"""Tuned profile (default) for the keyboard and GUI tools: native mock worker only."""
import sys
import time
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"scripts"))
from arm_control import ArmIK
import keyboard_control as kb
import gui_control
from gui_control_core import GuiController
from gui_settings import config_command
from tuned_profile import JOINT_KD, JOINT_KP, TUNED

ARM = ArmIK()


class IdleSolver:
    def poll(self):
        return None


def wait_for(worker, predicate, timeout=3, ping=True):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if ping:
            worker.send("PING")
        state = worker.poll()
        if state and predicate(state):
            return state
        time.sleep(.01)
    raise AssertionError(f"worker condition timed out: {worker.state}")


@pytest.mark.parametrize("parse", [kb.parse_args, gui_control.parse_args])
def test_tuned_is_the_default_and_legacy_restores_the_original(parse):
    tuned = parse([])
    assert (tuned.kp, tuned.kd, tuned.gravity_scale, tuned.tracking_limit) == (100, 1, 1, .1)
    assert tuned.joint_kp == JOINT_KP and tuned.joint_kd == JOINT_KD and tuned.ki == 1.5
    assert tuned.joint_integral_limit[1] == pytest.approx(.015) and tuned.fault_kd == 1
    assert Path(tuned.gravity_profile).name == "dex1_gravity.json" and tuned.recover_margin == .2
    legacy = parse(["--profile", "legacy"])
    assert (legacy.kp, legacy.kd, legacy.gravity_scale) == (60, .5, 0)
    assert getattr(legacy, "joint_kp", None) is None and getattr(legacy, "ki", 0) == 0
    uniform = parse(["--kp", "80"])   # an explicit Kp means uniform gains
    assert uniform.joint_kp is None and uniform.integral_limit == pytest.approx(3/80) and uniform.ki == 1.5


@pytest.fixture
def tuned_worker():
    worker = kb.Worker(kb.parse_args([]), ARM, mock_q=kb.DEFAULT_READY_Q)
    try:
        yield worker
    finally:
        worker.close()


def test_worker_runs_the_tuned_settings(tuned_worker):
    state = wait_for(tuned_worker, lambda s: s["mode"] == "hold")
    config = state["config"]
    assert config["kp"] == JOINT_KP and config["kd"] == JOINT_KD and (config["kp_base"], config["kd_base"]) == (100, 1)
    assert config["ki"] == 1.5 and config["fault_kd"] == 1 and config["tracking_limit"] == .1
    assert state["gravity_model"] and state["gravity_scale"] == 1


def test_heartbeat_loss_damps_and_enter_recovers(tuned_worker):
    sent = []
    controller = kb.JogController(lambda m: (sent.append(m), tuned_worker.send(m)), ARM)
    wait_for(tuned_worker, lambda s: s["mode"] == "hold")
    damped = wait_for(tuned_worker, lambda s: s["mode"] == "damping", ping=False)
    assert damped["reason"] == "UI_heartbeat_timeout"   # legacy releases torque here
    controller.tick(damped, IdleSolver(), time.monotonic())
    assert controller.damping and not controller.stopped and "Enter" in controller.note
    controller.key("w", True)          # no jogging while damped
    assert not controller.keys
    controller.key("return", True)
    assert sent[-1] == "RECOVER"
    held = wait_for(tuned_worker, lambda s: s["mode"] == "hold")
    controller.tick(held, IdleSolver(), time.monotonic())
    assert not controller.damping and held["reason"] == "recovered"


def test_gui_apply_keeps_per_joint_gains_and_a_new_kp_makes_them_uniform(tuned_worker):
    state = wait_for(tuned_worker, lambda s: s["mode"] == "hold")
    controller = GuiController(tuned_worker.send, ARM, None, kb.DEFAULT_READY_Q)
    controller.update_state(state)
    assert controller.settings["kp"] == 100   # the editable base pair, not the array
    values = dict(controller.settings, joint_speed=.2)
    tuned_worker.send(config_command(1, values))
    state = wait_for(tuned_worker, lambda s: s.get("config_ack") == 1)
    assert state["config"]["kp"] == JOINT_KP and state["config"]["joint_speed"] == .2
    tuned_worker.send(config_command(2, dict(values, kp=80)))
    state = wait_for(tuned_worker, lambda s: s.get("config_ack") == 2)
    assert state["config"]["kp"] == 80 and not state["config_error"]


def test_gui_recover_from_damping():
    sent = []
    controller = GuiController(sent.append, ARM, None, kb.DEFAULT_READY_Q)
    q = np.asarray(kb.DEFAULT_READY_Q).tolist()
    base = dict(q=q, command=q, target=q, capabilities=["move"], reason="t")
    controller.update_state(dict(base, seq=1, mode="hold"))
    controller.update_state(dict(base, seq=2, mode="damping", reason="UI_heartbeat_timeout"))
    assert controller.stopped and not controller.enabled
    controller.recover()
    assert sent[-1] == "RECOVER" and not controller.stopped
    controller.update_state(dict(base, seq=3, mode="hold", reason="recovered"))
    assert controller.initialized and controller.enabled
    with pytest.raises(RuntimeError):
        controller.recover()   # only from damping


def test_gui_window_runs_tuned_and_recovers_from_damping(tmp_path):
    from PySide6.QtCore import Qt
    from PySide6.QtTest import QTest
    from test_gui_control import app as _app, qt_wait   # noqa: F401
    from PySide6.QtWidgets import QApplication
    application = QApplication.instance() or QApplication([])
    window = gui_control.ControlWindow(gui_control.parse_args(["--log", str(tmp_path/"tuned-gui.jsonl")]))
    try:
        window.show()
        qt_wait(application, lambda: window.controller is not None and window.controller.enabled)
        assert "J2/J4 Kp 200" in window.profile_status.text()
        assert window.controller.settings["kp"] == 100 and window.controller.settings["tracking_limit"] == .1
        assert not window.recover_button.isEnabled()
        window._send("DAMP")   # a soft fault: compliant hold, torque kept
        qt_wait(application, lambda: window.controller.state["mode"] == "damping" and window.recover_button.isEnabled())
        QTest.mouseClick(window.recover_button, Qt.LeftButton)
        # Damping frames still queued when RECOVER lands must not re-latch the stop.
        qt_wait(application, lambda: window.controller.enabled and window.controller.state["reason"] == "recovered")
        window.restore_defaults()
        assert window.setting_inputs["kp"].value() == TUNED["kp"] and window.setting_inputs["tracking_limit"].value() == .1
    finally:
        window.close()
