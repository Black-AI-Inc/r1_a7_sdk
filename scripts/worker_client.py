"""Headless client shared by the keyboard tool, the GUI and external services.

Importing this module never starts a worker or accesses hardware.
"""
import json
import os
from pathlib import Path
import subprocess
import time

if __package__:
    from .arm_control import SDK, vector
    from .gravity_model import worker_gravity_args
else:
    from arm_control import SDK, vector
    from gravity_model import worker_gravity_args

class Worker:
    def __init__(self, args, arm, *, mock_q=None, mock_load=None, binary=None, on_state=None):
        self.on_state = on_state
        ready = arm.check_q(args.ready_q)
        binary = Path(binary) if binary is not None else SDK / "build/keyboard/r1_keyboard_worker"
        if not binary.is_file():
            raise RuntimeError("Build worker first: ./scripts/build_keyboard.sh")
        command = [str(binary), "--hardware" if args.execute else "--mock",
                   "--port", args.port, "--calib", str(getattr(args, "calibration_file", SDK / "config/joint_calib.txt")),
                   "--kp", str(args.kp), "--kd", str(args.kd),
                   "--joint-speed", str(args.joint_speed),
                   "--joint-accel", str(args.joint_accel), "--joint-jerk", str(args.joint_jerk),
                   "--ready-q", *map(str, ready), "--limits",
                   *map(str, arm.lower), *map(str, arm.upper)]
        # Optional; the worker defaults keep the original behaviour (no integral,
        # torque release on every fault, no startup recovery).
        for option in ("lead_limit", "tracking_limit", "move_timeout", "ki", "integral_limit",
                       "fault_kp", "fault_kd", "recover_margin"):
            if hasattr(args, option):
                command += ["--" + option.replace("_", "-"), str(getattr(args, option))]
        # Per-joint gains/integral limits (7 values each) override the scalars above.
        for option in ("joint_kp", "joint_kd", "joint_integral_limit"):
            if getattr(args, option, None) is not None:
                command += ["--" + option.replace("_", "-"), *map(str, vector(getattr(args, option), 7))]
        if getattr(args, "recover_q", None) is not None:
            command += ["--recover-q", *map(str, arm.check_q(args.recover_q))]
        command += worker_gravity_args(args)
        if mock_q is not None or mock_load is not None:
            if args.execute:
                raise ValueError("Mock plant options are not allowed for hardware")
            if mock_q is not None:
                # A mock plant may start slightly outside the limits only to
                # exercise the worker's own startup recovery.
                margin = float(getattr(args, "recover_margin", 0) or 0)
                mock_q = vector(mock_q, 7) if margin else arm.check_q(mock_q)
                if (mock_q < arm.lower-margin).any() or (mock_q > arm.upper+margin).any():
                    raise ValueError("mock_q outside joint limits and recovery margin")
                command += ["--mock-q", *map(str, mock_q)]
            if mock_load is not None:
                command += ["--mock-load", *map(str, vector(mock_load, 7))]
        self.process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                        bufsize=0, start_new_session=True)
        os.set_blocking(self.process.stdout.fileno(), False)
        os.set_blocking(self.process.stdin.fileno(), False)
        self.pending = b""
        self.state = None
        self.state_time = time.monotonic()

    def send(self, message):
        data = (message + "\n").encode("ascii")
        count = os.write(self.process.stdin.fileno(), data)
        if count != len(data):
            raise RuntimeError("Worker command pipe blocked")

    def poll(self):
        while True:
            try:
                data = os.read(self.process.stdout.fileno(), 4096)
            except BlockingIOError:
                break
            if not data:
                break
            self.pending += data
            while b"\n" in self.pending:
                line, self.pending = self.pending.split(b"\n", 1)
                self.state = json.loads(line)
                if self.state.get("protocol") != 3:
                    raise RuntimeError("Old keyboard worker: run ./scripts/build_keyboard.sh and restart")
                if self.on_state is not None:
                    self.on_state(self.state)
                self.state_time = time.monotonic()
        if self.process.poll() is not None:
            raise RuntimeError(f"SDK worker exited ({self.process.returncode})")
        if time.monotonic() - self.state_time > .4:
            raise RuntimeError("SDK worker status timed out")
        return self.state

    def close(self):
        if self.process.poll() is None:
            try:
                self.send("QUIT")
            except (OSError, RuntimeError):
                self.process.terminate()
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.terminate()
                try:
                    self.process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait(timeout=1)
        self.process.stdin.close()
        self.process.stdout.close()

