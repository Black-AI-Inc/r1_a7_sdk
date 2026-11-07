"""Gravity physics and native lifecycle checks. No serial IO or hardware tests."""
import json
from pathlib import Path
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from arm_control import SDK, URDF_REL
from gravity_model import GravityModel, DEFAULT_PROFILE, worker_gravity_args
from keyboard_control import Worker, DEFAULT_READY_Q, parse_args as _parse_args


# These tests run the legacy profile (gravity off unless requested);
# tests/test_tuned_profile.py covers the default tuned profile.
def parse_args(argv=()):
    return _parse_args(["--profile", "legacy", *argv])


@pytest.mark.parametrize("carrying", [False, True])
def test_gravity_is_potential_gradient_and_native_matches(carrying):
    model = GravityModel(profile=DEFAULT_PROFILE, carrying=carrying)
    rng = np.random.default_rng(518)
    poses = np.vstack(([0]*7, DEFAULT_READY_Q,
                       rng.uniform(model.arm.lower, model.arm.upper, size=(24, 7))))
    expected = []
    for q in poses:
        actual = model.evaluate(q)["total_nm"]
        gradient = np.array([(model.evaluate(q+np.eye(7)[j]*1e-6)["potential_j"]
                              -model.evaluate(q-np.eye(7)[j]*1e-6)["potential_j"])/2e-6
                             for j in range(7)])
        np.testing.assert_allclose(actual, gradient, atol=2e-8, rtol=1e-7)
        expected.append(actual)
    result = subprocess.run([str(SDK/"build/keyboard/test_gravity"), model.native_record()],
                            input="\n".join(" ".join(map(str, q)) for q in poses)+"\n",
                            capture_output=True, text=True, check=True)
    observed = np.array([[float(v) for v in line.split()] for line in result.stdout.splitlines()])
    np.testing.assert_allclose(observed, expected, atol=2e-13, rtol=1e-12)


@pytest.mark.parametrize("tool", [False, True])
def test_against_bullet_inverse_dynamics(tmp_path, tool):
    import pybullet as pb
    model = GravityModel(profile=DEFAULT_PROFILE if tool else None, carrying=tool)
    root = ET.parse(SDK / URDF_REL).getroot()
    # Strip geometry so this independent dynamics test needs no renderer/meshes.
    for link in root.findall("link"):
        for el in list(link):
            if el.tag in ("visual", "collision"):
                link.remove(el)
    if tool:
        inertial = root.find("link[@name='Link7']/inertial")
        mass_el = inertial.find("mass")
        old_mass = float(mass_el.get("value"))
        old_com = np.fromstring(inertial.find("origin").get("xyz"), sep=" ")
        total = old_mass+model.tool_mass
        com = (old_mass*old_com+model.tool_mass*model.tool_com)/total
        mass_el.set("value", str(total))
        inertial.find("origin").set("xyz", " ".join(map(str, com)))
        # Inertia tensor does not enter inverse dynamics when dq=ddq=0.
    path = tmp_path / "arm.urdf"
    ET.ElementTree(root).write(path)
    client = pb.connect(pb.DIRECT)
    try:
        pb.setGravity(0, 0, -9.81, physicsClientId=client)
        robot = pb.loadURDF(str(path), useFixedBase=True, flags=pb.URDF_USE_INERTIA_FROM_FILE,
                            physicsClientId=client)
        rng = np.random.default_rng(97)
        for q in np.vstack(([0]*7, DEFAULT_READY_Q,
                            rng.uniform(model.arm.lower, model.arm.upper, size=(16, 7)))):
            actual = pb.calculateInverseDynamics(robot, q.tolist(), [0]*7, [0]*7, physicsClientId=client)
            np.testing.assert_allclose(actual, model.evaluate(q)["total_nm"], atol=2e-8, rtol=1e-7)
    finally:
        pb.disconnect(client)


def test_profile_uses_total_mass_without_counting_adapter_twice():
    empty = GravityModel(profile=DEFAULT_PROFILE)
    full = GravityModel(profile=DEFAULT_PROFILE, carrying=True)
    assert empty.tool_mass == .550
    assert full.tool_mass == pytest.approx(.600)
    # COM is behind the gripper face by ~58 mm along its extension direction.
    assert empty.tool_com[1] == pytest.approx(-.10734525254)
    R = empty.arm.fk([0]*7)[:3, :3]
    from arm_control import pose_matrix
    mount = empty.profile["link7_to_gripper_xyz_rpy"]
    axes = R @ pose_matrix(mount[:3], mount[3:])[:3, :3]
    assert np.dot(axes[:, 0], [0, -1, 0]) > .999
    assert np.dot(axes[:, 1], [1, 0, 0]) > .999


@pytest.mark.parametrize("bad", ["mass", "mount", "com", "nan", "schema"])
def test_reject_invalid_profile_before_start(tmp_path, bad):
    data = json.loads(DEFAULT_PROFILE.read_text())
    if bad == "mass":
        data["empty_mass_kg"] = -.5
    if bad == "mount":
        data["mount_confirmed"] = False
    if bad == "com":
        data["gripper_com_in_gripper_m"] = [100, 0, 0]
    if bad == "nan":
        data["object_mass_kg"] = float("nan")
    if bad == "schema":
        data["schema"] = 2
    path = tmp_path/"bad.json"
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError):
        GravityModel(profile=path, carrying=True)


@pytest.mark.parametrize("record", ["1 0 0 -9.81", "nan", "2", "1 0 0 9.81", ""])
def test_native_rejects_incomplete_model(record):
    result = subprocess.run([str(SDK/"build/keyboard/test_gravity"), record], capture_output=True)
    assert result.returncode != 0


@pytest.mark.parametrize("args", [["--gravity-scale", "nan"], ["--gravity-scale", "1.1"],
                                  ["--gravity-scale", ".25"], ["--carrying"]])
def test_reject_unsafe_or_missing_options(args):
    with pytest.raises(ValueError):
        worker_gravity_args(parse_args(args))


def test_gravity_uses_selected_calibration_before_hardware_start(tmp_path, monkeypatch):
    monkeypatch.setattr(subprocess, "Popen", lambda *a, **kw: pytest.fail("unexpected child"))
    args = parse_args(["--gravity-profile", str(DEFAULT_PROFILE), "--gravity-scale", ".25"])
    args.execute = True  # only validates files; never constructs a worker
    calib = np.loadtxt(SDK / "config/joint_calib.txt", comments="#")
    calib[0, 2] = 2.
    args.calibration_file = tmp_path / "custom_calib.txt"
    np.savetxt(args.calibration_file, calib)
    with pytest.raises(ValueError, match="unit calibration scale"):
        worker_gravity_args(args)
    calib[:, 2] = 1.
    np.savetxt(args.calibration_file, calib)
    assert worker_gravity_args(args)[-1] == "0.25"


def test_gravity_uses_selected_urdf(tmp_path):
    args = parse_args(["--gravity-profile", str(DEFAULT_PROFILE), "--gravity-scale", ".25"])
    root = ET.parse(SDK / URDF_REL)
    root.getroot().find("link[@name='Link7']/inertial/mass").set("value", "0.8")
    args.urdf = tmp_path / "custom.urdf"
    root.write(args.urdf)
    actual = worker_gravity_args(args)
    assert actual[1] == GravityModel(urdf=args.urdf, profile=DEFAULT_PROFILE).native_record()
    assert actual[1] != GravityModel(profile=DEFAULT_PROFILE).native_record()


def wait(worker, condition, timeout=3, ping=True):
    end = time.monotonic()+timeout
    while time.monotonic() < end:
        if ping:
            worker.send("PING")
        state = worker.poll()
        if state and condition(state):
            return state
        time.sleep(.01)
    raise AssertionError(worker.state)


def test_native_compensation_ramps_uses_measured_q_and_stops():
    args = parse_args(["--gravity-profile", str(DEFAULT_PROFILE), "--gravity-scale", ".25"])
    model = GravityModel(profile=DEFAULT_PROFILE)
    worker = Worker(args, model.arm, mock_q=DEFAULT_READY_Q, mock_load=[0,.02,0,.01,0,0,0])
    try:
        initial = wait(worker, lambda s: s.get("gravity_gain", 0)>0)
        assert 0 < initial["gravity_gain"] < .25
        state = wait(worker, lambda s: s.get("gravity_gain") == .25)
        expected = model.evaluate(state["q"])
        np.testing.assert_allclose(state["gravity_arm_nm"], expected["arm_nm"], atol=1e-8)
        np.testing.assert_allclose(state["gravity_tool_nm"], expected["tool_nm"], atol=1e-8)
        np.testing.assert_allclose(state["tau_ff"], .25*expected["total_nm"], atol=1e-8)
        assert not state["gravity_saturated"]
        assert state["gravity_tool_mass_kg"] == .550
        np.testing.assert_allclose(state["gravity_tool_com_m"], model.tool_com, atol=1e-10)
        worker.send("HOLD")
        held = wait(worker, lambda s: s["seq"]>state["seq"])
        np.testing.assert_allclose(held["command"], state["command"], atol=1e-9)
        assert held["gravity_gain"] == .25
        worker.send("STOP")
        stopped = wait(worker, lambda s: s["mode"] == "stopped")
        assert stopped["tau_ff"] == [0]*7 and stopped["gravity_gain"] == 0
        worker.send("READY")
        latched = wait(worker, lambda s: s["seq"]>stopped["seq"])
        assert latched["mode"] == "stopped" and latched["tau_ff"] == [0]*7
    finally:
        worker.close()


def test_legacy_gravity_is_off_by_default_even_with_profile():
    args = parse_args(["--gravity-profile", str(DEFAULT_PROFILE)])
    model = GravityModel(profile=DEFAULT_PROFILE)
    worker = Worker(args, model.arm)
    try:
        state = wait(worker, lambda s: s.get("gravity_model"))
        assert state["gravity_gain"] == 0 and state["tau_ff"] == [0]*7
        assert max(np.abs(state["gravity_arm_nm"])) > 1
    finally:
        worker.close()


def test_legacy_heartbeat_fault_zeroes_gravity():
    args = parse_args(["--gravity-profile", str(DEFAULT_PROFILE), "--gravity-scale", ".25"])
    worker = Worker(args, GravityModel().arm)
    try:
        wait(worker, lambda s: s.get("gravity_gain", 0) > .01)
        state = wait(worker, lambda s: s["mode"] == "fault", ping=False)
        assert state["reason"] == "UI_heartbeat_timeout"
        assert state["tau_ff"] == [0]*7 and state["gravity_gain"] == 0
    finally:
        worker.close()
