#!/usr/bin/env python3
"""Standalone R1-7A FK / CasADi IK -> official test_arm_lowlevel --goto.

fk prints the TCP pose; goto and ik print the planned SDK command and run it
only with --execute. The SDK goto uses its own --kp/--kd and releases torque
after --hold-ms; it does not use the keyboard/GUI worker or its profiles.

IK formulation adapted from Unitree xr_teleoperate R1_A7_ArmIK.
Copyright [2025] [HangZhou YuShu TECHNOLOGY CO.,LTD. ("Unitree Robotics")]
SPDX-License-Identifier: Apache-2.0
See ik_licenses/ and README_ik.md for sources and adaptation details.
"""
import argparse
import json
import os
from pathlib import Path
import shlex
import sys
import xml.etree.ElementTree as ET

# Avoid an oversized BLAS thread pool for a seven-variable problem.
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("OMP_NUM_THREADS", "1")

import numpy as np
from scipy.spatial.transform import Rotation

POSE_IK_TIMEOUT = 3.0   # s, default wall-time budget of one solve_pose() request

SDK = Path(__file__).resolve().parents[1]
# The tools drive the right arm: upstream's right-arm URDF, extracted from
# urdf/URDF_R1-7A.tar.gz by build_keyboard.sh, and the matching calibration.
URDF_REL = Path("urdf/R1_Robotic_Arm_R_V1.2-1023/R1_Robotic_Arm_V1.2/"
                "R1_Robotic_Arm/urdf/R1_Robotic_Arm.urdf")
CALIB_REL = Path("config/joint_calib_right.txt")


def vector(values, count):
    """values as a finite float array of shape (count,), else ValueError."""
    result = np.asarray(values, dtype=float)
    if result.shape != (count,) or not np.isfinite(result).all():
        raise ValueError(f"Expected {count} finite numbers")
    return result


def pose_matrix(xyz, rpy):
    """4x4 transform from a translation and fixed-axis (extrinsic xyz) RPY."""
    result = np.eye(4)
    result[:3, 3] = vector(xyz, 3)
    result[:3, :3] = Rotation.from_euler("xyz", vector(rpy, 3)).as_matrix()
    return result


class ArmIK:
    """SDK serial-chain model; all q are URDF radians, not motor angles."""

    def __init__(self, urdf=SDK / URDF_REL, tcp=(0, 0, 0, 0, 0, 0)):
        joints = ET.parse(urdf).getroot().findall("joint")
        by_name = {j.get("name"): j for j in joints}
        if set(by_name) != {f"J{i}" for i in range(1, 8)} or len(joints) != 7:
            raise ValueError("Expected SDK single-arm J1..J7 URDF")
        self.origins, self.axes, lower, upper = [], [], [], []
        parent = "base_link"
        for i in range(1, 8):
            joint = by_name[f"J{i}"]
            if (joint.get("type") != "revolute" or
                    joint.find("parent").get("link") != parent or
                    joint.find("child").get("link") != f"Link{i}"):
                raise ValueError("Unexpected SDK joint chain")
            parent = f"Link{i}"
            origin = joint.find("origin")
            xyz = [float(v) for v in origin.get("xyz").split()]
            rpy = [float(v) for v in origin.get("rpy").split()]
            self.origins.append(pose_matrix(xyz, rpy))
            axis = vector([float(v) for v in joint.find("axis").get("xyz").split()], 3)
            if abs(np.linalg.norm(axis) - 1) > 1e-9:
                raise ValueError("URDF joint axis must be normalized")
            self.axes.append(axis)
            lower.append(float(joint.find("limit").get("lower")))
            upper.append(float(joint.find("limit").get("upper")))
        self.lower, self.upper = vector(lower, 7), vector(upper, 7)
        if np.any(self.lower >= self.upper):
            raise ValueError("Invalid URDF joint limits")
        tcp = vector(tcp, 6)
        self.tcp = pose_matrix(tcp[:3], tcp[3:])

    def check_q(self, q):
        q = vector(q, 7)
        if np.any(q < self.lower) or np.any(q > self.upper):
            raise ValueError("Joint angles outside SDK URDF limits (order J1..J7)")
        return q

    def fk(self, q):
        q = self.check_q(q)
        transform = np.eye(4)
        for origin, axis, angle in zip(self.origins, self.axes, q):
            rotation = np.eye(4)
            rotation[:3, :3] = Rotation.from_rotvec(axis * angle).as_matrix()
            transform = transform @ origin @ rotation
        return transform @ self.tcp

    def solve(self, target, seed, max_step=0.5):
        """Bounded IK. max_step constrains solution vs seed, not actual motion."""
        import casadi as ca

        seed = self.check_q(seed)
        target = np.asarray(target, dtype=float)
        if (target.shape != (4, 4) or not np.isfinite(target).all() or
                not np.allclose(target[3], [0, 0, 0, 1]) or
                not np.allclose(target[:3, :3].T @ target[:3, :3], np.eye(3), atol=1e-8) or
                not np.isclose(np.linalg.det(target[:3, :3]), 1)):
            raise ValueError("Target must be a rigid 4x4 transform")
        if not np.isfinite(max_step) or max_step <= 0:
            raise ValueError("max-step must be finite and positive")
        if np.allclose(self.fk(seed), target, atol=1e-10, rtol=0):
            return seed.copy()
        lo = np.maximum(self.lower, seed - max_step)
        hi = np.minimum(self.upper, seed + max_step)
        opt = ca.Opti()
        q = opt.variable(7)
        transform = ca.SX.eye(4)
        symbolic_q = ca.SX.sym("q", 7)
        for i, (origin, axis) in enumerate(zip(self.origins, self.axes)):
            x, y, z = axis
            skew = ca.DM([[0, -z, y], [z, 0, -x], [-y, x, 0]])
            rotation = ca.SX.eye(4)
            rotation[:3, :3] = (ca.DM.eye(3) + ca.sin(symbolic_q[i]) * skew
                               + (1 - ca.cos(symbolic_q[i])) * (skew @ skew))
            transform = transform @ ca.DM(origin) @ rotation
        fk = ca.Function("single_arm_fk", [symbolic_q], [transform @ ca.DM(self.tcp)])
        actual = fk(q)
        pos_cost = ca.sumsqr(actual[:3, 3] - target[:3, 3])
        # Chordal SO(3) cost is finite at both zero and pi; locally equals log3^2.
        rot_cost = 0.5 * ca.sumsqr(actual[:3, :3] - target[:3, :3])
        opt.minimize(50 * pos_cost + rot_cost + 0.02 * ca.sumsqr(q)
                     + 0.1 * ca.sumsqr(q - seed))
        opt.subject_to(opt.bounded(lo, q, hi))
        # Endpoint accuracy constraints added to the upstream soft objective.
        opt.subject_to(pos_cost <= 0.0001 ** 2)
        opt.subject_to(rot_cost <= 0.001 ** 2)
        opt.solver("ipopt", {"expand": True, "print_time": False,
                   "ipopt.sb": "yes", "ipopt.print_level": 0,
                   "ipopt.max_iter": 300, "ipopt.tol": 1e-9,
                   "ipopt.constr_viol_tol": 1e-10, "ipopt.bound_relax_factor": 0,
                   "ipopt.max_cpu_time": 3.0})
        # Deterministic nearby restarts; never accept a failed solver's debug q.
        rng = np.random.default_rng(7)
        starts = [seed] + [np.clip(seed + rng.normal(0, min(max_step / 3, 0.2), 7), lo, hi)
                           for _ in range(3)]
        for start in starts:
            opt.set_initial(q, start)
            try:
                solution = np.asarray(opt.solve().value(q)).reshape(7)
            except RuntimeError:
                continue
            # Independent NumPy/SciPy FK checks the symbolic optimization result.
            if not np.isfinite(solution).all() or np.any(solution < lo) or np.any(solution > hi):
                continue
            pos_error, angle_error = errors(self.fk(solution), target)
            if pos_error <= 0.0002 and angle_error <= 0.002:
                return solution
        raise ValueError("IK failed: unreachable target, unsuitable seed, or max-step too small; no command sent")

    def prepare_pose_solver(self):
        """Cache exact FK residual/Jacobian; built once per arm/TCP, never per target."""
        if hasattr(self, "_pose_residual"):
            return
        import casadi as ca
        q, target = ca.SX.sym("q", 7), ca.SX.sym("target", 4, 4)
        actual = ca.SX.eye(4)
        for i, (origin, axis) in enumerate(zip(self.origins, self.axes)):
            x, y, z = axis
            skew = ca.DM([[0, -z, y], [z, 0, -x], [-y, x, 0]])
            rotation = ca.SX.eye(4)
            rotation[:3, :3] = (ca.DM.eye(3) + ca.sin(q[i])*skew
                               + (1-ca.cos(q[i]))*(skew@skew))
            actual = actual @ ca.DM(origin) @ rotation
        actual = actual @ ca.DM(self.tcp)
        # Full rotation matrix avoids the false zero at pi of a cross-product error.
        residual = ca.vertcat(5*(actual[:3, 3]-target[:3, 3]),
                             ca.reshape(actual[:3, :3]-target[:3, :3], 9, 1))
        self._pose_residual = ca.Function("pose_residual", [q, target],
                                           [residual, ca.jacobian(residual, q)])

    def solve_pose(self, target, seed, *, timeout=POSE_IK_TIMEOUT, deadline=None):
        """Fixed-orientation IK: nearby first, then URDF-wide bounded multi-start.

        Wall time bounds the entire request, not each restart. A numerical search
        failure is not proof of geometric unreachability. No motion is sent here.
        The bounded solve(max_step=...) keeps its strict joint bound; the
        keyboard jog uses it. last_pose_report records the stage, attempts,
        elapsed time and final residuals of the latest request.
        """
        import time
        from scipy.optimize import least_squares

        started = time.monotonic()
        seed = self.check_q(seed).copy()
        target = np.asarray(target, dtype=float)
        if (target.shape != (4, 4) or not np.isfinite(target).all() or
                not np.allclose(target[3], [0, 0, 0, 1]) or
                not np.allclose(target[:3, :3].T@target[:3, :3], np.eye(3), atol=1e-8) or
                not np.isclose(np.linalg.det(target[:3, :3]), 1)):
            raise ValueError("Target must be a rigid 4x4 transform")
        if not np.isfinite(timeout) or timeout <= 0:
            raise ValueError("IK timeout must be finite and positive")
        if deadline is not None and not np.isfinite(deadline):
            raise ValueError("IK deadline must be finite")
        end = min(started+timeout, deadline) if deadline is not None else started+timeout
        self.last_pose_report = {"stage": "initial", "attempts": 0, "elapsed_s": 0.,
                                 "position_error_m": None, "orientation_error_rad": None}
        report = self.last_pose_report

        class BudgetExpired(Exception):
            pass

        def budget():
            if time.monotonic() >= end:
                raise BudgetExpired()

        def valid(q):
            if not np.isfinite(q).all() or np.any(q < self.lower) or np.any(q > self.upper):
                return False
            pe, re = errors(self.fk(q), target)
            report.update(position_error_m=pe, orientation_error_rad=re)
            return pe <= .0001 and re <= .001

        try:
            budget()
            if np.allclose(self.fk(seed), target, atol=1e-10, rtol=0):
                report.update(stage="seed", position_error_m=0., orientation_error_rad=0.)
                return seed
            # Conservative geometric bound, valid for every joint orientation and TCP.
            reach = sum(np.linalg.norm(o[:3, 3]) for o in self.origins) + np.linalg.norm(self.tcp[:3, 3])
            if np.linalg.norm(target[:3, 3]) > reach+.0001:
                report["stage"] = "reach_bound"
                raise ValueError("IK failed: target outside conservative arm reach")
            self.prepare_pose_solver()
            budget()
            rng = np.random.default_rng(7)
            best, best_cost = seed.copy(), float("inf")
            # Small nudges get one cheap local attempt; hard targets cannot consume
            # all of the budget in the first basin. Wide starts cover other branches.
            stages = [("local", .6, 2, 55), ("expanded", 1.4, 2, 80),
                      ("full", None, 12, 100)]
            for stage, radius, count, evaluations in stages:
                report["stage"] = stage
                lo = self.lower if radius is None else np.maximum(self.lower, seed-radius)
                hi = self.upper if radius is None else np.minimum(self.upper, seed+radius)
                for attempt in range(count):
                    budget()
                    if attempt == 0:
                        start = np.clip(best, lo, hi)
                    elif radius is not None:
                        start = np.clip(seed+rng.normal(0, radius/3, 7), lo, hi)
                    elif attempt == 1:
                        start = (lo+hi)/2
                    elif attempt % 2 == 0:
                        start = np.clip(seed+rng.normal(0, 1.2, 7), lo, hi)
                    else:
                        start = rng.uniform(lo, hi)
                    cached_q, cached_value = None, None
                    # Cap each basin as well as the whole search. Leave time for restarts.
                    attempt_end = min(end, time.monotonic()+(.12 if radius is not None else .18))

                    class AttemptExpired(Exception):
                        pass

                    def evaluate(q):
                        nonlocal cached_q, cached_value, best, best_cost
                        budget()
                        if time.monotonic() >= attempt_end:
                            raise AttemptExpired()
                        if cached_q is None or not np.array_equal(q, cached_q):
                            residual, jacobian = self._pose_residual(q, target)
                            cached_value = (np.asarray(residual).reshape(12), np.asarray(jacobian))
                            cached_q = q.copy()
                            cost = float(cached_value[0]@cached_value[0])
                            if cost < best_cost:
                                best, best_cost = q.copy(), cost
                        return cached_value

                    report["attempts"] += 1
                    try:
                        fit = least_squares(lambda q: evaluate(q)[0], start,
                                            jac=lambda q: evaluate(q)[1], bounds=(lo, hi),
                                            method="trf", max_nfev=evaluations,
                                            ftol=1e-10, xtol=1e-10, gtol=1e-10)
                        candidate = fit.x
                    except AttemptExpired:
                        candidate = best
                    budget()
                    # Accept by independent physical residuals, not optimizer status.
                    if valid(candidate):
                        return candidate.copy()
                    if valid(best):
                        return best.copy()
            raise ValueError("IK failed: no fixed-orientation solution found within search budget; "
                             "target may be unreachable or require another initial posture")
        except BudgetExpired:
            raise ValueError("IK timeout: fixed-orientation search budget exhausted; no command sent") from None
        finally:
            report["elapsed_s"] = time.monotonic()-started


def errors(actual, target):
    """(position error in m, rotation angle error in rad) between two poses."""
    return (float(np.linalg.norm(actual[:3, 3] - target[:3, 3])),
            float(Rotation.from_matrix(actual[:3, :3] @ target[:3, :3].T).magnitude()))


def describe(transform):
    """JSON-friendly position, RPY and quaternion of a 4x4 pose."""
    rotation = Rotation.from_matrix(transform[:3, :3])
    return {"xyz_m": transform[:3, 3].tolist(),
            "rpy_rad": rotation.as_euler("xyz").tolist(),
            "quat_xyzw": rotation.as_quat().tolist()}


def sdk_command(args, q):
    """argv for the SDK's test_arm_lowlevel --goto to joint target q."""
    for name in ("kp", "kd", "home_lead"):
        value = getattr(args, name)
        if not np.isfinite(value) or value < 0:
            raise ValueError(f"{name} must be finite and nonnegative")
    if args.home_lead < 0.01 or args.rate <= 0 or args.rate > 1000 or args.move_ms <= 0 or args.hold_ms < 0:
        raise ValueError("Require home-lead >= 0.01, rate 1..1000, move-ms > 0, hold-ms >= 0")
    return [str(args.sdk_dir / "bin/test_arm_lowlevel"), "--goto", "--q",
            *[format(float(v), ".12g") for v in q], "--port", args.port,
            "--kp", str(args.kp), "--kd", str(args.kd),
            "--home-lead", str(args.home_lead), "--rate", str(args.rate),
            "--move-ms", str(args.move_ms), "--hold-ms", str(args.hold_ms), "--right"]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sdk-dir", type=Path, default=SDK)
    parser.add_argument("--tcp", type=float, nargs=6, default=[0] * 6,
                        metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"),
                        help="Link7 -> TCP, metres/radians (default identity)")
    subs = parser.add_subparsers(dest="mode", required=True)
    fk_parser = subs.add_parser("fk", help="Offline forward kinematics")
    fk_parser.add_argument("--q", nargs=7, type=float, default=[0] * 7)
    joint_parser = subs.add_parser("goto", help="Joint target -> SDK goto")
    joint_parser.add_argument("--q", nargs=7, type=float, required=True)
    ik_parser = subs.add_parser("ik", help="Cartesian target -> IK -> SDK goto")
    target_group = ik_parser.add_mutually_exclusive_group(required=True)
    target_group.add_argument("--xyz", nargs=3, type=float, help="Absolute TCP position in base_link, metres")
    target_group.add_argument("--delta", nargs=3, type=float, help="Translation from FK(seed), in base_link, metres")
    orientation = ik_parser.add_mutually_exclusive_group()
    orientation.add_argument("--rpy", nargs=3, type=float, help="Absolute RPY radians: Rz(yaw) Ry(pitch) Rx(roll)")
    orientation.add_argument("--quat", nargs=4, type=float, help="Absolute normalized quaternion x y z w")
    ik_parser.add_argument("--seed", nargs=7, type=float, default=[0] * 7,
                           help="IK reference joints; defaults to zero, NOT hardware feedback")
    ik_parser.add_argument("--max-step", type=float, default=None,
                           help="Strict joint bound; explicitly setting this selects bounded search (radians)")
    ik_parser.add_argument("--search", choices=("adaptive", "bounded"), default=None,
                           help="Adaptive fixed-orientation search (default), or strict --max-step bound")
    ik_parser.add_argument("--ik-timeout", type=float, default=POSE_IK_TIMEOUT,
                           help="Adaptive IK total wall-time budget in seconds (default 3)")
    for sub in (joint_parser, ik_parser):
        sub.add_argument("--execute", action="store_true", help="Execute SDK; otherwise only print plan")
        sub.add_argument("--port", default="/dev/ttyUSB3")
        sub.add_argument("--kp", type=float, default=60)
        sub.add_argument("--kd", type=float, default=0.5)
        sub.add_argument("--home-lead", type=float, default=0.2)
        sub.add_argument("--rate", type=int, default=50)
        sub.add_argument("--move-ms", type=int, default=3000)
        sub.add_argument("--hold-ms", type=int, default=0)
    args = parser.parse_args(argv)
    try:
        args.sdk_dir = args.sdk_dir.expanduser().resolve()
        arm = ArmIK(args.sdk_dir / URDF_REL, args.tcp)
        report = {"base_frame": "base_link", "tip_frame": "Link7 + tcp",
                  "tcp_xyz_rpy": args.tcp}
        if args.mode == "ik":
            if args.search == "adaptive" and args.max_step is not None:
                raise ValueError("--max-step is a hard bound; use --search bounded or omit --max-step")
            args.search = args.search or ("bounded" if args.max_step is not None else "adaptive")
            seed = arm.check_q(args.seed)
            target = arm.fk(seed)
            if args.xyz is not None:
                target[:3, 3] = vector(args.xyz, 3)
            else:
                target[:3, 3] += vector(args.delta, 3)
            if args.rpy is not None:
                target[:3, :3] = Rotation.from_euler("xyz", vector(args.rpy, 3)).as_matrix()
            if args.quat is not None:
                quat = vector(args.quat, 4)
                if abs(np.linalg.norm(quat) - 1) > 1e-5:
                    raise ValueError("Quaternion must be normalized, order x y z w")
                target[:3, :3] = Rotation.from_quat(quat).as_matrix()
            q = (arm.solve_pose(target, seed, timeout=args.ik_timeout) if args.search == "adaptive"
                 else arm.solve(target, seed, .5 if args.max_step is None else args.max_step))
            if args.search == "adaptive":
                report["ik_search"] = arm.last_pose_report
            pe, re = errors(arm.fk(q), target)
            report.update(target=describe(target), seed_q_rad=seed.tolist(),
                          position_error_m=pe, orientation_error_rad=re)
        else:
            q = arm.check_q(args.q)
        report.update(q_rad=q.tolist(), fk=describe(arm.fk(q)))
        command = None if args.mode == "fk" else sdk_command(args, q)
        print(json.dumps(report, indent=2, allow_nan=False), flush=True)
        if command is None:
            return 0
        print("SDK command: " + shlex.join(command), flush=True)
        print("SDK cwd: " + str(args.sdk_dir / "bin"), flush=True)
        if not args.execute:
            print("Dry run: serial port not opened. Add --execute to move.")
            return 0
        if not Path(command[0]).is_file() or not os.access(command[0], os.X_OK):
            raise ValueError("SDK executable missing; build test_arm_lowlevel first")
        if not (args.sdk_dir / CALIB_REL).is_file():
            raise ValueError("SDK joint calibration file missing")
        print("Executing SDK goto; it stops/releases torque after hold-ms. Ctrl-C is handled by SDK.", flush=True)
        # Replace process so SIGINT/SIGTERM reach the SDK directly. No shell.
        os.chdir(args.sdk_dir / "bin")
        os.execv(command[0], command)
    except (ValueError, OSError, ImportError, ET.ParseError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
