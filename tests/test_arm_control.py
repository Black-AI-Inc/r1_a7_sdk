"""Offline tests: never use the real SDK executable or a serial device."""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

SCRIPT = Path(__file__).resolve().parents[1] / "scripts/arm_control.py"
spec = importlib.util.spec_from_file_location("arm_control", SCRIPT)
control = importlib.util.module_from_spec(spec)
spec.loader.exec_module(control)


@pytest.fixture(scope="module")
def arm():
    return control.ArmIK()


def test_fk_against_pybullet(arm):
    """Independent engine verifies joint axes, frames, rotations and TCP."""
    pb = pytest.importorskip("pybullet")
    client = pb.connect(pb.DIRECT)
    try:
        robot = pb.loadURDF(str(control.SDK / control.URDF_REL), useFixedBase=True,
                            physicsClientId=client)
        tcp_arm = control.ArmIK(tcp=[.01, -.02, .06, .2, -.3, .4])
        rng = np.random.default_rng(12)
        for q in [np.zeros(7), *rng.uniform(arm.lower, arm.upper, (20, 7))]:
            for i, angle in enumerate(q):
                pb.resetJointState(robot, i, angle, physicsClientId=client)
            state = pb.getLinkState(robot, 6, computeForwardKinematics=True, physicsClientId=client)
            transform = np.eye(4)
            transform[:3, 3] = state[4]  # URDF link frame, not inertial frame
            transform[:3, :3] = Rotation.from_quat(state[5]).as_matrix()
            np.testing.assert_allclose(arm.fk(q), transform, atol=5e-7)
            np.testing.assert_allclose(tcp_arm.fk(q), transform @ tcp_arm.tcp, atol=5e-7)
    finally:
        pb.disconnect(client)


def test_zero_pose_is_exact_hold(arm):
    q = arm.solve(arm.fk(np.zeros(7)), np.zeros(7))
    np.testing.assert_array_equal(q, np.zeros(7))


def test_zero_seed_translation(arm):
    target = arm.fk(np.zeros(7))
    target[2, 3] += .005
    q = arm.solve(target, np.zeros(7))
    pe, re = control.errors(arm.fk(q), target)
    assert pe <= .0002 and re <= .002


def test_ik_fk_roundtrips(arm):
    rng = np.random.default_rng(100)
    for _ in range(12):
        seed = rng.uniform(arm.lower + .1, arm.upper - .1)
        known_q = np.clip(seed + rng.normal(0, .04, 7), arm.lower, arm.upper)
        target = arm.fk(known_q)
        q = arm.solve(target, seed)
        pe, re = control.errors(arm.fk(q), target)
        assert pe <= .0002 and re <= .002
        assert np.max(np.abs(q - seed)) <= .5


def test_unreachable_and_step_limit(arm):
    target = arm.fk(np.zeros(7))
    target[0, 3] = 10
    with pytest.raises(ValueError, match="IK failed"):
        arm.solve(target, np.zeros(7))
    target = arm.fk([.2, .2, .1, .3, .1, .1, .1])
    with pytest.raises(ValueError, match="IK failed"):
        arm.solve(target, np.zeros(7), max_step=.001)


@pytest.mark.parametrize("q", [[float("nan")] * 7, [float("inf")] * 7,
                               [0, -.1, 0, 0, 0, 0, 0], [0] * 6])
def test_invalid_joints(arm, q):
    with pytest.raises(ValueError):
        arm.check_q(q)


def test_pi_rotation_not_false_success(arm):
    target = arm.fk(np.zeros(7))
    target[:3, :3] = Rotation.from_rotvec([np.pi, 0, 0]).as_matrix() @ target[:3, :3]
    with pytest.raises(ValueError, match="IK failed"):
        arm.solve(target, np.zeros(7), max_step=.01)


@pytest.fixture
def fake_sdk(tmp_path):
    sdk = tmp_path / "sdk with spaces"
    (sdk / "bin").mkdir(parents=True)
    (sdk / "config").mkdir()
    (sdk / "config/joint_calib.txt").write_text("TEST ONLY\n")
    urdf = sdk / control.URDF_REL
    urdf.parent.mkdir(parents=True)
    urdf.write_bytes((control.SDK / control.URDF_REL).read_bytes())
    executable = sdk / "bin/test_arm_lowlevel"
    executable.write_text("#!/usr/bin/env python3\nimport os, sys, json\n"
                          "from pathlib import Path\n"
                          "Path('../called.json').write_text(json.dumps({'cwd': os.getcwd(), 'argv': sys.argv}))\n"
                          "sys.exit(7)\n")
    executable.chmod(0o755)
    return sdk


def cli(sdk, *args):
    return subprocess.run([sys.executable, str(SCRIPT), "--sdk-dir", str(sdk), *args],
                          cwd="/tmp", capture_output=True, text=True)


def test_dry_run_never_executes_sdk(fake_sdk):
    result = cli(fake_sdk, "goto", "--q", *["0"] * 7)
    assert result.returncode == 0, result.stderr
    assert "Dry run" in result.stdout
    assert not (fake_sdk / "called.json").exists()


def test_execute_exact_command_cwd_and_exit_status(fake_sdk):
    result = cli(fake_sdk, "goto", "--q", *["0"] * 7, "--execute", "--hold-ms", "2000")
    assert result.returncode == 7, result.stderr
    call = json.loads((fake_sdk / "called.json").read_text())
    assert call["cwd"] == str(fake_sdk / "bin")
    assert call["argv"][1:10] == ["--goto", "--q", *["0"] * 7]
    assert call["argv"][-2:] == ["--hold-ms", "2000"]


@pytest.mark.parametrize("args", [
    ["goto", "--q", *["0"] * 7, "--kp", "nan"],
    ["goto", "--q", *["0"] * 7, "--hold-ms", "-1"],
    ["ik", "--xyz", "nan", "0", "0"],
    ["ik", "--delta", "0", "0", "0", "--quat", "0", "0", "0", "0"],
    ["ik", "--xyz", "10", "0", "0"],
])
def test_invalid_execute_never_calls_sdk(fake_sdk, args):
    result = cli(fake_sdk, *args, "--execute")
    assert result.returncode == 2
    assert not (fake_sdk / "called.json").exists()


def test_adaptive_analytic_jacobian_and_tcp():
    arm = control.ArmIK(tcp=[.01, -.02, .06, .2, -.3, .4])
    arm.prepare_pose_solver()
    cached = arm._pose_residual
    q = (arm.lower+arm.upper)/2
    target = arm.fk(np.zeros(7))
    residual, jac = arm._pose_residual(q, target)
    actual = arm.fk(q)
    expected = np.r_[5*(actual[:3, 3]-target[:3, 3]),
                     (actual[:3, :3]-target[:3, :3]).ravel(order="F")]
    np.testing.assert_allclose(np.asarray(residual).ravel(), expected, atol=1e-12)
    h = 1e-6
    numerical = np.column_stack([(np.asarray(arm._pose_residual(q+np.eye(7)[i]*h, target)[0]).ravel() -
                                  np.asarray(arm._pose_residual(q-np.eye(7)[i]*h, target)[0]).ravel())/(2*h)
                                 for i in range(7)])
    np.testing.assert_allclose(jac, numerical, atol=1e-8)
    arm.prepare_pose_solver()
    assert arm._pose_residual is cached


@pytest.mark.parametrize("delta", [[0, 0, .002], [0, -.1, .1], [.1, 0, .1], [-.1, 0, .1],
                                   [0, 0, .2], [0, -.2, .1]])
def test_adaptive_translation_keeps_fixed_orientation_beyond_local_radius(arm, delta):
    seed = np.array([.006792101712, .605442587728, .003982975938, .349301756762,
                     -.004018032920, .256020348855, .009633129608])
    target = arm.fk(seed)
    orientation = target[:3, :3].copy()
    target[:3, 3] += delta
    q = arm.solve_pose(target, seed)
    pe, re = control.errors(arm.fk(q), target)
    assert pe <= .0001 and re <= .001
    np.testing.assert_array_equal(target[:3, :3], orientation)
    if delta == [0, 0, .002]:
        assert arm.last_pose_report["stage"] == "local"
    else:
        # Beyond the 0.6 rad local stage (and the bounded solver's 0.5 rad box).
        assert np.max(np.abs(q-seed)) > .6
        assert arm.last_pose_report["stage"] in ("expanded", "full")


def test_adaptive_full_range_known_fk_targets(arm):
    seed = np.zeros(7)
    rng = np.random.default_rng(20261028)
    stages = set()
    for known in rng.uniform(arm.lower+.05, arm.upper-.05, (20, 7)):
        target = arm.fk(known)
        q = arm.solve_pose(target, seed)
        pe, re = control.errors(arm.fk(q), target)
        assert pe <= .0001 and re <= .001
        assert np.all(q >= arm.lower) and np.all(q <= arm.upper)
        stages.add(arm.last_pose_report["stage"])
    assert "full" in stages


def test_adaptive_deadline_and_no_false_pi_solution(arm):
    import time
    seed = np.zeros(7)
    target = arm.fk(seed)
    with pytest.raises(ValueError, match="timeout"):
        arm.solve_pose(target, seed, deadline=time.monotonic()-1)
    target[:3, :3] = Rotation.from_rotvec([np.pi, 0, 0]).as_matrix()@target[:3, :3]
    try:
        q = arm.solve_pose(target, seed)
    except ValueError as exc:
        assert "IK" in str(exc)
    else:
        pe, re = control.errors(arm.fk(q), target)
        assert pe <= .0001 and re <= .001
        assert not np.allclose(q, seed)


@pytest.mark.parametrize("timeout", [0, -1, float("nan"), float("inf")])
def test_adaptive_rejects_bad_timeout(arm, timeout):
    with pytest.raises(ValueError, match="timeout"):
        arm.solve_pose(arm.fk(np.zeros(7)), np.zeros(7), timeout=timeout)


def test_cli_explicit_max_step_remains_a_hard_bound(fake_sdk):
    result = cli(fake_sdk, "ik", "--delta", "0", "0", ".05", "--max-step", ".001", "--execute")
    assert result.returncode == 2 and not (fake_sdk/"called.json").exists()
    result = cli(fake_sdk, "ik", "--delta", "0", "0", ".05", "--max-step", ".001",
                 "--search", "adaptive", "--execute")
    assert result.returncode == 2 and not (fake_sdk/"called.json").exists()
