"""Keyboard/native integration tests; all workers use --mock, never serial IO."""
import os
import json
from pathlib import Path
import signal
import subprocess
import sys
import time

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import keyboard_control as kb
import numpy as np
import pygame
import pytest


# These tests run the legacy profile (the original tool defaults);
# tests/test_tuned_profile.py covers the default tuned profile.
def legacy_args(argv=()):
    return kb.parse_args(["--profile", "legacy", *argv])


def state_for(q=None, mode="hold", command=None):
    q = list(q) if q is not None else [0]*7
    return {"mode": mode, "q": q, "command": list(command) if command is not None else q.copy()}


@pytest.fixture
def arm():
    return kb.ArmIK()


@pytest.fixture
def worker(arm):
    instance = kb.Worker(legacy_args([]), arm)
    wait_for(instance, lambda s: s["mode"] == "hold")
    yield instance
    instance.close()


def wait_for(worker, predicate, timeout=2, ping=True):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if ping:
            worker.send("PING")
        state = worker.poll()
        if state and predicate(state):
            return state
        time.sleep(.01)
    raise AssertionError(f"Worker condition timed out: {worker.state}")


class Solver:
    """Synchronous stand-in for IKProcess: records jobs, returns a preset result."""

    def __init__(self):
        self.jobs = []
        self.result = None

    def submit(self, *job):
        self.jobs.append(job)

    def poll(self):
        result, self.result = self.result, None
        return result


LOAD_BIAS = np.array([0, .015, 0, .01, 0, 0, 0])   # mock plant sag under load, rad


@pytest.fixture
def loaded_worker(arm):
    instance = kb.Worker(legacy_args([]), arm, mock_q=kb.DEFAULT_READY_Q, mock_load=LOAD_BIAS)
    wait_for(instance, lambda s: s["mode"] == "hold")
    yield instance
    instance.close()


def test_hold_does_not_ratchet_under_static_load(loaded_worker):
    initial = loaded_worker.state
    assert np.max(np.abs(np.array(initial["command"])-initial["q"])) > .01
    for _ in range(4):
        loaded_worker.send("HOLD")
        loaded_worker.send("PING")
        time.sleep(.06)
        state = loaded_worker.poll()
        np.testing.assert_allclose(state["command"], initial["command"], atol=1e-8)
        np.testing.assert_allclose(state["q"], initial["q"], atol=1e-8)


def test_ik_seed_preserves_load_supporting_reference(arm):
    solver = Solver()
    c = kb.JogController(lambda _: None, arm)
    reference = np.array(kb.DEFAULT_READY_Q)
    state = {"mode": "hold", "command": reference.tolist(), "q": (reference-LOAD_BIAS).tolist()}
    c.key("r", True)
    c.tick(state, solver, 1)
    np.testing.assert_allclose(solver.jobs[-1][1], reference, atol=1e-12)


@pytest.mark.parametrize("key", ["r", "f", "a", "d"])
def test_loaded_key_taps_follow_requested_axis_without_sag(loaded_worker, arm, key):
    solver = Solver()
    c = kb.JogController(loaded_worker.send, arm)
    direction = np.array(kb.AXES[key])
    for _ in range(4):
        before = loaded_worker.poll()
        xyz_before = arm.fk(before["q"])[:3, 3]
        c.key(key, True)
        job_count = len(solver.jobs)
        deadline = time.monotonic()+2
        while len(solver.jobs) == job_count:
            assert time.monotonic() < deadline
            loaded_worker.send("PING")
            c.tick(loaded_worker.poll(), solver, time.monotonic())
            time.sleep(.01)
        generation, seed, delta, step, orientation = solver.jobs[-1]
        target = arm.fk(seed)
        target[:3, 3] += np.asarray(delta)*step
        target[:3, :3] = orientation
        q = arm.solve(target, seed, max_step=.1)
        solver.result = (generation, q.tolist(), "")
        c.tick(loaded_worker.poll(), solver, time.monotonic())
        reached = wait_for(loaded_worker, lambda s: s["mode"] == "hold" and
                           np.max(np.abs(np.array(s["command"])-q)) < 1e-7)
        c.key(key, False)
        loaded_worker.send("PING")
        time.sleep(.07)
        after = loaded_worker.poll()
        np.testing.assert_allclose(after["command"], reached["command"], atol=1e-8)
        displacement = arm.fk(after["q"])[:3, 3]-xyz_before
        along = float(displacement@direction)
        assert .0017 < along < .0023, (key, displacement)
        cross = displacement-along*direction
        assert np.linalg.norm(cross) < .0003, (key, displacement)


def test_tracking_lag_blocks_new_ik_without_rebasing(arm):
    solver = Solver()
    sent = []
    c = kb.JogController(sent.append, arm)
    reference = np.array(kb.DEFAULT_READY_Q)
    measured = reference.copy()
    measured[1] -= .07
    c.key("r", True)
    c.tick(state_for(measured, command=reference), solver, 1)
    assert not solver.jobs
    assert sent == ["HOLD"]
    assert "Tracking lag" in c.note


def test_jog_result_exceeding_feedback_margin_holds_instead_of_faulting(arm):
    solver, sent = Solver(), []
    c = kb.JogController(sent.append, arm)
    reference = np.array(kb.DEFAULT_READY_Q)
    measured = reference.copy()
    measured[1] -= .05
    state = state_for(measured, command=reference)
    c.key("r", True)
    c.tick(state, solver, 1)
    solution = reference.copy()
    solution[1] += .09
    solver.result = (c.generation, solution, "")
    c.tick(state, solver, 1.1)
    assert not any(command.startswith("TARGET") for command in sent)
    assert sent[-1] == "HOLD" and not c.stopped
    assert "tracking margin" in c.note


def test_mock_load_is_rejected_for_hardware_before_launch(arm):
    args = legacy_args(["--execute"])
    with pytest.raises(ValueError, match="Mock plant"):
        kb.Worker(args, arm, mock_load=LOAD_BIAS)


def test_keyup_cancels_inflight_ik(arm):
    sent = []
    controller = kb.JogController(sent.append, arm)
    solver = Solver()
    state = state_for()
    controller.key("r", True)
    controller.tick(state, solver, 1)
    generation = solver.jobs[-1][0]
    controller.key("r", False)
    solver.result = (generation, [0, .02, 0, .03, 0, -.01, 0], "")
    controller.tick(state, solver, 1.1)
    assert sent[-1] == "HOLD"
    assert not any(s.startswith("TARGET") for s in sent)


def test_valid_ik_only_while_pressed_and_fresh(arm):
    sent, solver = [], Solver()
    c = kb.JogController(sent.append, arm)
    state = state_for()
    c.key("r", True)
    c.tick(state, solver, 1)
    target = arm.fk(state["command"])
    target[2, 3] += c.step
    solver.result = (c.generation, arm.solve(target, state["command"], max_step=.1), "")
    c.tick(state, solver, 1.1)
    assert sent[-1].startswith("TARGET ")


def test_continuous_jog_preserves_initial_orientation(arm):
    sent, solver = [], Solver()
    c = kb.JogController(sent.append, arm)
    state = state_for()
    c.key("r", True)
    c.tick(state, solver, 1)
    first_orientation = solver.jobs[-1][4]
    target = arm.fk(state["command"])
    target[2, 3] += c.step
    q = arm.solve(target, state["command"], max_step=.1).tolist()
    q[5] += .001  # Small orientation error must not accumulate on the next step.
    solver.result = (c.generation, q, "")
    c.tick(state, solver, 1.1)
    assert sent[-1].startswith("TARGET ")
    state["q"] = q
    state["command"] = q.copy()
    c.tick(state, solver, 1.3)
    np.testing.assert_array_equal(solver.jobs[-1][4], first_orientation)
    c.hold()
    assert c.orientation is None


@pytest.mark.parametrize("delay,moved", [(.6, 0), (.1, .05)])
def test_stale_ik_discarded(arm, delay, moved):
    sent, solver = [], Solver()
    c = kb.JogController(sent.append, arm)
    state = state_for()
    c.key("r", True)
    c.tick(state, solver, 1)
    solver.result = (c.generation, [0]*7, "")
    state["q"][0] = moved
    c.tick(state, solver, 1+delay)
    assert not any(s.startswith("TARGET") for s in sent)


def test_opposing_and_diagonal_keys(arm):
    c = kb.JogController(lambda _: None, arm)
    c.key("w", True)
    c.key("s", True)
    np.testing.assert_array_equal(c.direction(), [0, 0, 0])
    c.key("s", False)
    c.key("a", True)
    assert np.linalg.norm(c.direction()) == pytest.approx(1)
    assert c.direction()[2] == 0


def test_home_and_stop_invalidate_ik(arm):
    sent, solver = [], Solver()
    c = kb.JogController(sent.append, arm)
    c.key("r", True)
    c.tick(state_for(), solver, 1)
    old = c.generation
    c.key("h", True)
    assert sent[-2:] == ["HOLD", "HOME"]
    assert not c.keys
    solver.result = (old, [0]*7, "")
    c.tick(state_for(mode="home"), solver, 1.1)
    c.key("escape", True)
    c.key("h", True)
    c.key("r", True)
    assert sent[-1] == "STOP"
    assert c.stopped


def test_ready_key_invalidates_pending_jog(arm):
    sent, solver = [], Solver()
    c = kb.JogController(sent.append, arm)
    c.key("r", True)
    c.tick(state_for(), solver, 1)
    old = c.generation
    c.key("p", True)
    assert sent[-2:] == ["HOLD", "READY"]
    solver.result = (old, [0]*7, "")
    c.tick(state_for(mode="ready"), solver, 1.1)
    assert sent[-1] == "READY"
    c.key("space", True)
    assert sent[-1] == "HOLD"


def test_default_ready_pose_six_directions(arm):
    # Ten repeated 2 mm steps per direction, with the initial rotation locked.
    for axis in range(3):
        for sign in (-1, 1):
            q = np.array(kb.DEFAULT_READY_Q)
            initial = arm.fk(q)
            for _ in range(10):
                target = arm.fk(q)
                target[:3, :3] = initial[:3, :3]
                target[axis, 3] += sign * .002
                q = arm.solve(target, q, max_step=.1)
            displacement = (arm.fk(q)[axis, 3] - initial[axis, 3]) * sign
            assert .018 <= displacement <= .022


@pytest.mark.parametrize("tcp", [[0]*6, [.02, -.01, .08, .1, .2, -.1]])
def test_ready_pose_matches_home_offset_and_orientation(tcp):
    arm = kb.ArmIK(tcp=tcp)
    home = arm.fk(np.zeros(7))
    ready = arm.fk(kb.DEFAULT_READY_Q)
    np.testing.assert_allclose(ready[:3, 3]-home[:3, 3], [0, -.05, .05], atol=1e-9)
    np.testing.assert_allclose(ready[:3, :3], home[:3, :3], atol=1e-9)


def test_worker_ready_holds_and_is_cancellable(worker):
    worker.send("READY")
    state = wait_for(worker, lambda s: s["mode"] == "ready" and s["q"][3] > .02)
    assert state["q"][3] < kb.DEFAULT_READY_Q[3]
    worker.send("HOLD")
    wait_for(worker, lambda s: s["mode"] == "hold")
    worker.send("READY")
    state = wait_for(worker, lambda s: s["mode"] == "hold" and
                     np.max(np.abs(np.array(s["q"])-kb.DEFAULT_READY_Q)) < 1e-7, timeout=6)
    np.testing.assert_allclose(state["q"], kb.DEFAULT_READY_Q, atol=1e-7)
    worker.send("STOP")
    wait_for(worker, lambda s: s["mode"] == "stopped")
    worker.send("READY")
    time.sleep(.07)
    assert worker.poll()["mode"] == "stopped"


@pytest.mark.parametrize("ready", [[0, -.1, 0, 0, 0, 0, 0], [float("nan")]*7])
def test_invalid_ready_pose_rejected_before_worker_start(arm, ready):
    args = legacy_args([])
    args.ready_q = ready
    with pytest.raises(ValueError):
        kb.Worker(args, arm)


def test_focus_loss_and_keyup_events(arm):
    sent = []
    c = kb.JogController(sent.append, arm)
    kb.handle_event(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_r), c, pygame)
    assert c.keys == {"r"}
    kb.handle_event(pygame.event.Event(pygame.KEYUP, key=pygame.K_r), c, pygame)
    assert not c.keys and sent[-1] == "HOLD"
    c.key("w", True)
    kb.handle_event(pygame.event.Event(pygame.WINDOWFOCUSLOST), c, pygame)
    assert not c.keys and sent[-1] == "HOLD"


def test_worker_release_freezes_setpoint_and_holds(worker):
    worker.send("TARGET 0 0.1 0 0.1 0 0 0")
    moved = wait_for(worker, lambda s: s["q"][1] > .015)
    assert moved["q"][1] < .1
    worker.send("HOLD")
    held = wait_for(worker, lambda s: s["mode"] == "hold")
    assert held["q"][1] < .1
    until = time.monotonic() + .25
    while time.monotonic() < until:
        worker.send("PING")
        state = worker.poll()
        np.testing.assert_allclose(state["q"], held["q"], atol=1e-8)
        time.sleep(.01)


def test_worker_home_returns_to_zero_and_keeps_holding(worker):
    worker.send("TARGET 0.04 0.05 0 0.06 0 0 0")
    wait_for(worker, lambda s: s["mode"] == "hold" and s["q"][3] > .059)
    worker.send("HOME")
    state = wait_for(worker, lambda s: s["mode"] == "hold" and max(np.abs(s["q"])) < 1e-7)
    assert state["mode"] != "stopped"


def test_worker_home_is_cancellable(worker):
    worker.send("TARGET 0.1 0.1 0 0.1 0 0 0")
    wait_for(worker, lambda s: s["q"][0] > .09)
    worker.send("HOME")
    wait_for(worker, lambda s: s["mode"] == "home")
    worker.send("HOLD")
    state = wait_for(worker, lambda s: s["mode"] == "hold")
    assert state["q"][0] > .01


def test_worker_stop_is_latched(worker):
    worker.send("STOP")
    wait_for(worker, lambda s: s["mode"] == "stopped")
    worker.send("HOME")
    worker.send("TARGET 0 0.05 0 0 0 0 0")
    time.sleep(.07)
    state = worker.poll()
    assert state["mode"] == "stopped"
    np.testing.assert_array_equal(state["q"], [0]*7)


@pytest.mark.parametrize("command", ["TARGET 0 -0.1 0 0 0 0 0", "TARGET 0.2 0 0 0 0 0 0",
                                     "TARGET nan 0 0 0 0 0 0", "TARGET 0 0", "BAD"])
def test_worker_rejects_invalid_targets(worker, command):
    worker.send(command)
    state = wait_for(worker, lambda s: s["mode"] == "fault")
    assert state["reason"] in ("invalid_joint_target", "invalid_command")


def test_legacy_heartbeat_loss_releases_torque(worker):
    state = wait_for(worker, lambda s: s["mode"] == "fault", ping=False)
    assert state["reason"] == "UI_heartbeat_timeout"


def test_native_sigterm_clean_shutdown(worker):
    worker.process.send_signal(signal.SIGTERM)
    assert worker.process.wait(timeout=2) == 0


def test_end_to_end_async_ik_and_native(worker, arm):
    solver = kb.IKProcess([0]*6)
    c = kb.JogController(worker.send, arm)
    try:
        c.key("r", True)
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            worker.send("PING")
            state = worker.poll()
            c.tick(state, solver, time.monotonic())
            if state and state["q"][3] > .005:
                break
            time.sleep(.01)
        else:
            raise AssertionError(c.note)
        c.key("r", False)
        held = wait_for(worker, lambda s: s["mode"] == "hold")
        time.sleep(.1)
        c.tick(worker.poll(), solver, time.monotonic())
        np.testing.assert_allclose(worker.state["q"], held["q"], atol=1e-8)
    finally:
        solver.close()


def test_window_renders_offline(arm, tmp_path):
    pygame.display.init()
    pygame.font.init()
    try:
        screen = pygame.display.set_mode((1020, 550))
        c = kb.JogController(lambda _: None, arm)
        kb.draw(screen, pygame.font.Font(None, 32), pygame.font.Font(None, 25), c,
                state_for(), legacy_args([]), arm, pygame)
        pygame.image.save(screen, str(tmp_path / "keyboard.png"))
        assert (tmp_path / "keyboard.png").stat().st_size > 1000
    finally:
        pygame.quit()


def test_full_app_mock_start_and_sigterm(tmp_path):
    log = tmp_path / "trace.jsonl"
    app = subprocess.Popen([sys.executable, str(kb.SDK / "scripts/keyboard_control.py"), "--log", str(log)],
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
        time.sleep(1)
        assert app.poll() is None
        app.send_signal(signal.SIGTERM)
        stdout, stderr = app.communicate(timeout=5)
        assert app.returncode == 0, stderr.decode()
        assert b"W/S" in stdout
        records = [json.loads(line) for line in log.read_text().splitlines()]
        assert records[0]["event"] == "startup"
        assert any(r["event"] == "state" and r["protocol"] == 3
                   and "command_velocity" in r and "command_acceleration" in r for r in records)
        assert records[-1]["event"] == "exit"
    finally:
        if app.poll() is None:
            app.kill()
            app.wait()


def test_next_ik_is_computed_during_motion_and_cancelled_on_release(arm):
    sent, solver = [], Solver()
    c = kb.JogController(sent.append, arm)
    reference = np.array(kb.DEFAULT_READY_Q)
    state = state_for(reference)
    c.key("r", True)
    c.tick(state, solver, 1)
    target = arm.fk(reference)
    target[2, 3] += c.step
    first_q = arm.solve(target, reference, max_step=.1)
    solver.result = (c.generation, first_q, "")
    c.tick(state, solver, 1.1)
    assert sent[-1].startswith("TARGET")
    # Only 20 ms later, while the reference is still moving toward step one.
    intermediate = reference + .1*(first_q-reference)
    c.tick(state_for(intermediate, mode="moving"), solver, 1.12)
    assert len(solver.jobs) == 2
    np.testing.assert_allclose(solver.jobs[-1][1], first_q)
    target[2, 3] += c.step
    second_q = arm.solve(target, first_q, max_step=.1)
    solver.result = (c.generation, second_q, "")
    c.tick(state_for(intermediate, mode="moving"), solver, 1.15)
    assert c.ready_result is not None
    assert len([s for s in sent if s.startswith("TARGET")]) == 1
    intermediate = reference + .85*(first_q-reference)
    c.tick(state_for(intermediate, mode="moving"), solver, 1.16)
    assert len([s for s in sent if s.startswith("TARGET")]) == 2
    c.tick(state_for(first_q + .1*(second_q-first_q), mode="moving"), solver, 1.18)
    generation = c.generation
    c.key("r", False)
    solver.result = (generation, second_q, "")
    c.tick(state_for(first_q), solver, 1.2)
    assert c.plan_target is None and sent[-1] == "HOLD"
    assert len([s for s in sent if s.startswith("TARGET")]) == 2


def test_lookahead_is_bounded_even_when_feedback_does_not_move(arm):
    solver, sent = Solver(), []
    c = kb.JogController(sent.append, arm)
    reference = np.array(kb.DEFAULT_READY_Q)
    state = state_for(reference)
    c.key("r", True)
    for k in range(15):
        now = 1+k*.04
        c.tick(state, solver, now)
        if c.busy and c.ready_result is None:
            generation, seed, direction, step, orientation = solver.jobs[-1]
            target = arm.fk(seed)
            target[:3, 3] += np.asarray(direction)*step
            target[:3, :3] = orientation
            solver.result = (generation, arm.solve(target, seed, max_step=.1), "")
            c.tick(state, solver, now+.01)
    assert len([s for s in sent if s.startswith("TARGET")]) == 1
    assert np.linalg.norm(arm.fk(c.plan_target)[:3, 3]-arm.fk(reference)[:3, 3]) <= 1.1*c.step+.0002


@pytest.mark.parametrize("option,value", [("--joint-accel", "0"), ("--joint-accel", "nan"),
                                          ("--joint-jerk", "0"), ("--joint-jerk", "21")])
def test_invalid_smoothing_limits_rejected(option, value):
    with pytest.raises(SystemExit):
        legacy_args([option, value])


@pytest.mark.parametrize("key", ["w", "s", "a", "d", "r", "f"])
def test_continuous_async_jog_retargets_before_arrival(loaded_worker, arm, key):
    solver = kb.IKProcess([0]*6)
    retarget_modes, samples = [], []

    def send(message):
        if message.startswith("TARGET"):
            retarget_modes.append(loaded_worker.state["mode"])
        loaded_worker.send(message)

    c = kb.JogController(send, arm)
    initial_xyz = arm.fk(loaded_worker.state["command"])[:3, 3]
    began = None
    previous_seq = -1
    try:
        c.key(key, True)
        deadline = time.monotonic()+8
        while time.monotonic() < deadline:
            loaded_worker.send("PING")
            state = loaded_worker.poll()
            now = time.monotonic()
            c.tick(state, solver, now)
            assert not c.stopped, c.note
            if state["mode"] == "moving" and began is None:
                began = now
            if began is not None and state["seq"] != previous_seq:
                samples.append(state)
                previous_seq = state["seq"]
            if began is not None and now-began > 1.5:
                break
            time.sleep(.01)
        assert began is not None, c.note
        assert retarget_modes.count("moving") >= 2, retarget_modes
        assert len(samples) >= 15
        assert all(s["mode"] == "moving" for s in samples), "Stopped between points"
        assert all(np.linalg.norm(s["command_velocity"]) > 1e-5 for s in samples[3:])
        displacement = arm.fk(samples[-1]["command"])[:3, 3]-initial_xyz
        direction = np.asarray(kb.AXES[key])
        along = displacement@direction
        assert along > .004 and np.linalg.norm(displacement-along*direction) < .0005
        for sample in samples:
            delta = arm.fk(sample["command"])[:3, 3]-initial_xyz
            assert np.linalg.norm(delta-(delta@direction)*direction) < .0005
        c.key(key, False)
        held = wait_for(loaded_worker, lambda s: s["mode"] == "hold")
        time.sleep(.1)
        c.tick(loaded_worker.poll(), solver, time.monotonic())
        np.testing.assert_allclose(loaded_worker.state["command"], held["command"], atol=1e-8)
        np.testing.assert_array_equal(held["command_velocity"], [0]*7)
    finally:
        solver.close()
