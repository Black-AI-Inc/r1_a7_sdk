#!/usr/bin/env python3
"""Keyboard Cartesian jog with persistent SDK hold. Default: offline simulation.

Held keys step the TCP along base_link axes at a fixed orientation; the native
worker holds the arm in between. --execute connects the real arm.
"""
import argparse
import json
import multiprocessing as mp
import os
from pathlib import Path
import queue
import signal
import sys
import time

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
from arm_control import ArmIK, SDK, vector
from gravity_model import add_gravity_args
from tuned_profile import add_profile_argument, describe, parse_args_with_profile
import numpy as np

AXES = {"w": (1, 0, 0), "s": (-1, 0, 0),
        "a": (0, 1, 0), "d": (0, -1, 0),
        "r": (0, 0, 1), "f": (0, 0, -1)}
# FK(P) = FK(Home) translated [0, -0.05, +0.05] m in base_link,
# with the same orientation. Offline IK plus endpoint refinement; URDF radians.
DEFAULT_READY_Q = [0.006792101712, 0.605442587728, 0.003982975938,
                   0.349301756762, -0.004018032920, 0.256020348855, 0.009633129608]


class DiagnosticLog:
    """Local key/command/feedback trace; never changes control state."""
    def __init__(self, path=None):
        self.path = Path(path) if path else SDK / "build/keyboard/logs" / f"keyboard-{time.time_ns()}.jsonl"
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.file = self.path.open("a", buffering=1)

    def write(self, event, **data):
        if self.file is not None:
            try:
                self.file.write(json.dumps({"t": time.monotonic(), "event": event, **data},
                                           allow_nan=False, default=str) + "\n")
            except OSError as exc:
                print(f"Diagnostic log disabled: {exc}", file=sys.stderr)
                try:
                    self.file.close()
                except OSError:
                    pass
                self.file = None

    def close(self):
        if self.file is not None:
            self.file.close()


def solve_loop(requests, results, tcp):
    """IK runs in another process: slow/infeasible solves cannot block key-up."""
    arm = ArmIK(tcp=tcp)
    while True:
        job = requests.get()
        if job is None:
            return
        generation, seed, direction, step, orientation = job
        try:
            target = arm.fk(seed)
            target[:3, 3] += np.asarray(direction) * step
            target[:3, :3] = np.asarray(orientation)
            q = arm.solve(target, seed, max_step=.1)
            results.put((generation, q.tolist(), ""))
        except Exception as exc:
            results.put((generation, None, str(exc)))


class IKProcess:
    """solve_loop in a spawned process, with one request and one result in flight."""

    def __init__(self, tcp):
        context = mp.get_context("spawn")
        self.requests = context.Queue(maxsize=1)
        self.results = context.Queue(maxsize=1)
        self.process = context.Process(target=solve_loop, args=(self.requests, self.results, tcp), daemon=True)
        self.process.start()

    def submit(self, generation, seed, direction, step, orientation):
        self.requests.put_nowait((generation, seed, direction, step, orientation))

    def poll(self):
        try:
            return self.results.get_nowait()
        except queue.Empty:
            return None

    def close(self):
        self.process.terminate()
        self.process.join(timeout=1)
        if self.process.is_alive():
            self.process.kill()
            self.process.join(timeout=1)
        self.requests.close()
        self.results.close()


# Re-exported: the GUI and the tests import Worker from this module.
from worker_client import Worker


class JogController:
    """Testable key-state machine. Each change invalidates old IK results.

    Holding direction keys moves the TCP in steps of `step` metres at the
    orientation captured when the jog started; the next step is solved while
    the current one is still being executed. send() delivers worker commands.
    """

    def __init__(self, send, arm, step=.002):
        self.send, self.arm, self.step = send, arm, step
        self.keys = set()
        self.generation = 0
        self.stopped = False
        self.busy = False
        self.submitted = 0.0
        self.seed = None
        self.feedback_seed = None
        self.orientation = None
        self.plan_target = None
        self.ready_result = None
        self.next_jog = 0.0
        self.damping = False   # soft fault: compliant hold, torque kept; Enter recovers
        self.note = "Ready: hold a direction key."

    def direction(self):
        direction = np.sum([AXES[k] for k in self.keys], axis=0) if self.keys else np.zeros(3)
        norm = np.linalg.norm(direction)
        return direction / norm if norm else np.zeros(3)

    def hold(self, note="Holding current setpoint (load support preserved)"):
        self.keys.clear()
        self.generation += 1
        self.orientation = None
        self.plan_target = None
        if not self.stopped:
            self.send("HOLD")
        self.note = note

    def key(self, name, down):
        """Handle one key press or release (pygame key name)."""
        if self.stopped:
            return
        if self.damping:
            if down and name in ("return", "enter"):
                self.send("RECOVER")
                self.note = "RECOVER sent: holding at the measured pose"
            elif down and name == "escape":
                self.keys.clear()
                self.stopped = True
                self.send("STOP")
                self.note = "STOPPED: torque released. Restart to enable."
            return   # no jogging while damped
        if name in AXES:
            before = self.keys.copy()
            self.keys.add(name) if down else self.keys.discard(name)
            if before != self.keys:
                self.generation += 1
                self.orientation = None
                self.plan_target = None
                self.send("HOLD")  # cancels previous target before a new direction
                self.note = "Jogging" if np.any(self.direction()) else "Holding current setpoint"
        elif down and name == "space":
            self.hold()
        elif down and name == "h":
            self.hold("Returning to calibrated joint zero")
            self.send("HOME")
        elif down and name == "p":
            self.hold("Moving to configured ready pose")
            self.send("READY")
        elif down and name == "escape":
            self.keys.clear()
            self.generation += 1
            self.stopped = True
            self.send("STOP")
            self.note = "STOPPED: torque released. Restart to enable."

    def tick(self, state, solver, now):
        """One UI frame: follow the worker state, send a ready IK result as
        TARGET, or submit the next jog step to the solver.
        """
        if state is None:
            return
        if state["mode"] in ("fault", "stopped"):
            self.stopped = True
            self.keys.clear()
            self.generation += 1
            self.note = state["reason"]
        if state["mode"] == "damping":
            if not self.damping:
                self.keys.clear()
                self.generation += 1
                self.plan_target = self.orientation = None
                self.busy = False
            self.damping = True
            self.note = "DAMPING (" + state["reason"] + "): torque kept. Enter = recover, Esc = release"
            return
        if self.damping and state["mode"] == "hold":
            self.damping = False
            self.note = "Recovered: holding at the measured pose"
        result = solver.poll()
        if result is not None:
            self.ready_result = result
        result = self.ready_result
        if result is not None:
            self.ready_result = None
            generation, solution, error = result
            self.busy = False
            if generation == self.generation and not self.stopped and np.any(self.direction()):
                measured = vector(state["q"], 7)
                reference = self.arm.check_q(state["command"])
                if error:
                    self.hold("IK rejected target; holding. Release and try another direction.")
                    print(error, file=sys.stderr)
                # Use a result only while it is fresh (0.5 s), was solved from the
                # current plan point and feedback has not moved 0.03 rad since.
                elif (state["mode"] in ("hold", "moving") and now - self.submitted <= .5
                      and self.plan_target is not None
                      and np.max(np.abs(self.plan_target - self.seed)) < 1e-5
                      and np.max(np.abs(measured - self.feedback_seed)) < .03):
                    q = self.arm.check_q(solution)
                    # The worker rejects a TARGET more than 0.12 rad from feedback.
                    if np.max(np.abs(q-measured)) > .12:
                        self.hold("Jog exceeds tracking margin; holding current setpoint")
                        return
                    if (state.get("lead_limited", False)
                            or np.max(np.abs(reference-measured)) > .06):
                        return
                    # Bound how far an asynchronous solution can lead the sent
                    # reference. No queue of distant points survives release.
                    if (np.linalg.norm(self.arm.fk(q)[:3, 3]-self.arm.fk(reference)[:3, 3])
                            > 1.1*self.step + .0002):
                        self.ready_result = result
                        self.busy = True
                        return
                    self.send("TARGET " + " ".join(format(v, ".12g") for v in q))
                    self.plan_target = q.copy()
                    self.next_jog = now + .01
        if (not self.stopped and not self.busy and state["mode"] in ("hold", "moving")
                and np.any(self.direction()) and now >= self.next_jog):
            # Keep the load-supporting reference continuous. Re-seeding from q
            # would erase the PD equilibrium error on every millimetre-sized jog.
            reference = self.arm.check_q(state["command"])
            measured = vector(state["q"], 7)
            if state.get("lead_limited", False) or np.max(np.abs(reference-measured)) > .06:
                self.note = "Tracking lag > 0.06 rad: waiting; reference unchanged"
                return
            if self.plan_target is None:
                if state["mode"] != "hold":
                    return  # Wait until the direction-change HOLD is processed.
                self.plan_target = reference.copy()
            # Precompute the next endpoint before the current segment finishes.
            # Solve early, then retain one result until the preceding point is
            # close. This limits corner cutting without waiting for IK at arrival.
            if (np.linalg.norm(self.arm.fk(self.plan_target)[:3, 3]-self.arm.fk(reference)[:3, 3])
                    > self.step or np.max(np.abs(self.plan_target-reference)) > .06):
                return
            self.seed = self.plan_target.copy()
            self.feedback_seed = measured.copy()
            if self.orientation is None:
                self.orientation = self.arm.fk(self.seed)[:3, :3]
            self.submitted = now
            self.busy = True
            solver.submit(self.generation, self.seed.tolist(), self.direction().tolist(), self.step,
                          self.orientation.tolist())


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--execute", action="store_true", help="Connect real arm; default simulation")
    p.add_argument("--port", default="/dev/ttyUSB3", help="Arm serial port (with --execute)")
    p.add_argument("--kp", type=float, default=60,
                   help="Position gain, .01..100 (tuned profile: 100 with J2/J4 200; "
                        "any other Kp/Kd pair makes all gains uniform)")
    p.add_argument("--kd", type=float, default=.5, help="Damping gain, .01..5 (tuned profile: 1 with J2/J4 1.41)")
    p.add_argument("--step-mm", type=float, default=2, help="Cartesian increment per jog, 0.5..5 mm")
    p.add_argument("--joint-speed", type=float, default=.15, help="Joint command speed cap, .01..0.3 rad/s")
    p.add_argument("--joint-accel", type=float, default=.5, help="Joint acceleration cap, .1..2 rad/s^2")
    p.add_argument("--joint-jerk", type=float, default=2, help="Joint jerk cap, .5..20 rad/s^3")
    p.add_argument("--tcp", nargs=6, type=float, default=[0]*6, help="Link7 -> TCP xyz/rpy, metres/radians")
    p.add_argument("--ready-q", nargs=7, type=float, default=DEFAULT_READY_Q.copy(),
                   help="P pose J1..J7 radians (default: Home -Y 5cm, +Z 5cm, same orientation)")
    p.add_argument("--log", type=Path, help="JSONL diagnostics (default: build/keyboard/logs/keyboard-TIMESTAMP.jsonl)")
    add_gravity_args(p)
    add_profile_argument(p)
    args = parse_args_with_profile(p, argv)
    for name, lower, upper in [("kp", .01, 100), ("kd", .01, 5),
                                ("step_mm", .5, 5), ("joint_speed", .01, .3),
                                ("joint_accel", .1, 2), ("joint_jerk", .5, 20)]:
        value = getattr(args, name)
        if not np.isfinite(value) or not lower <= value <= upper:
            p.error(f"{name} must be {lower}..{upper}")
    vector(args.tcp, 6)
    vector(args.ready_q, 7)
    return args


def handle_event(event, controller, pygame):
    if event.type == pygame.QUIT:
        return False
    if event.type in (pygame.WINDOWFOCUSLOST, pygame.WINDOWMINIMIZED):
        controller.hold("Window inactive: holding current setpoint")
    elif event.type in (pygame.KEYDOWN, pygame.KEYUP):
        if event.type == pygame.KEYDOWN and event.key == pygame.K_q:
            return False
        controller.key(pygame.key.name(event.key), event.type == pygame.KEYDOWN)
    return True


def draw(screen, font, small, controller, state, args, arm, pygame):
    screen.fill((19, 25, 35))
    title = "R1-7A  |  " + ("HARDWARE" if args.execute else "SIMULATION - no serial IO")
    lines = [title, "W / S : +X / -X       A / D : +Y / -Y       R / F : +Z / -Z",
             "Release/Space: HOLD    P: READY    H: ZERO    Enter: RECOVER    Esc: STOP    Q: QUIT",
             f"Step {args.step_mm:g} mm | V {args.joint_speed:g} | A {args.joint_accel:g} | "
             f"J {args.joint_jerk:g} (rad/s^n) | {describe(args)}",
             "Base: base_link | Tool: Link7 + TCP | Keep this window focused."]
    for i, line in enumerate(lines):
        screen.blit((font if i == 0 else small).render(line, True, (225, 233, 242)), (24, 24+i*38))
    mode = state["mode"] if state else "connecting"
    color = (255, 115, 110) if controller.stopped else (255, 190, 110) if controller.damping else (95, 220, 170)
    screen.blit(font.render("State: " + mode.upper(), True, color), (24, 236))
    if state and state.get("gravity_model"):
        gravity_text = (f"Gravity {state.get('gravity_gain', 0):.0%} | ff Nm: "
                        + " ".join(f"{v:+.2f}" for v in state.get("tau_ff", [])))
        if state.get("gravity_saturated"):
            gravity_text += " [CAPPED]"
        screen.blit(small.render(gravity_text, True, color), (24, 205))
    screen.blit(small.render(controller.note[:100], True, color), (24, 280))
    screen.blit(small.render("P ready (rad): " + "  ".join(f"{v:+.4f}" for v in args.ready_q),
                            True, (175, 190, 210)), (24, 460))
    if state and state["mode"] not in ("probing", "enabling"):
        q = np.clip(np.asarray(state["q"]), arm.lower, arm.upper)
        xyz = arm.fk(q)[:3, 3]
        reference_xyz = arm.fk(state["command"])[:3, 3]
        error = np.max(np.abs(np.asarray(state["command"])-state["q"]))
        screen.blit(small.render("Measured TCP (m): " + "  ".join(f"{v:+.5f}" for v in xyz),
                                 True, (225, 233, 242)), (24, 330))
        screen.blit(small.render("Reference TCP (m): " + "  ".join(f"{v:+.5f}" for v in reference_xyz),
                                 True, (150, 205, 255)), (24, 360))
        screen.blit(small.render(f"Max joint tracking error: {error:.4f} rad | HOLD = frozen reference",
                                 True, (225, 233, 242)), (24, 390))
        screen.blit(small.render("J1..J7 (rad): " + "  ".join(f"{v:+.3f}" for v in state["q"]),
                                 True, (225, 233, 242)), (24, 420))
    screen.blit(small.render("Esc / Q / window close releases torque. Space keeps holding.",
                             True, (255, 190, 110)), (24, 510))
    pygame.display.flip()


def main(argv=None):
    args = parse_args(argv)
    import pygame
    arm = ArmIK(tcp=args.tcp)
    arm.check_q(args.ready_q)
    worker = solver = diagnostic = None
    alive = True

    def interrupted(_signum, _frame):
        nonlocal alive
        alive = False

    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    try:
        # Initialize the keyboard window before touching hardware.
        pygame.display.init()
        pygame.font.init()
        screen = pygame.display.set_mode((1020, 550))
        pygame.display.set_caption("R1-7A keyboard control" + (" [HARDWARE]" if args.execute else " [SIMULATION]"))
        font = pygame.font.Font(None, 32)
        small = pygame.font.Font(None, 25)
        diagnostic = DiagnosticLog(args.log)
        diagnostic.write("startup", options=vars(args), protocol=3)
        print(f"Diagnostic log: {diagnostic.path}", flush=True)
        solver = IKProcess(args.tcp)
        worker = Worker(args, arm)

        def send(message):
            worker.send(message)
            if message != "PING":
                diagnostic.write("command", command=message)

        controller = JogController(send, arm, args.step_mm / 1000)
        clock = pygame.time.Clock()
        last_ping = 0.0
        focused = True
        last_state_seq = None
        print("W/S +/-X; A/D +/-Y; R/F +/-Z; release/Space hold; P ready; H zero; "
              "Enter recover; Esc stop; Q quit.", flush=True)
        while alive:
            now = time.monotonic()
            state = worker.poll()
            if state and state["seq"] != last_state_seq:
                diagnostic.write("state", **state)
                last_state_seq = state["seq"]
            for event in pygame.event.get():
                if event.type in (pygame.KEYDOWN, pygame.KEYUP):
                    diagnostic.write("key", key=pygame.key.name(event.key), down=event.type == pygame.KEYDOWN)
                elif event.type in (pygame.WINDOWFOCUSLOST, pygame.WINDOWMINIMIZED, pygame.QUIT):
                    diagnostic.write("window", kind=pygame.event.event_name(event.type))
                if not handle_event(event, controller, pygame):
                    alive = False
            if not alive:
                break
            # Independently reconcile held keys to recover from a dropped KEYUP.
            has_focus = pygame.key.get_focused()
            if not has_focus and focused:
                controller.hold("Window inactive: holding current setpoint")
            focused = has_focus
            if has_focus:
                pressed = pygame.key.get_pressed()
                for key in list(controller.keys):
                    if not pressed[pygame.key.key_code(key)]:
                        controller.key(key, False)
            elif controller.keys:
                controller.hold()
            if now-last_ping >= .05:
                worker.send("PING")
                last_ping = now
            if not solver.process.is_alive():
                raise RuntimeError("IK process exited")
            controller.tick(state, solver, now)
            draw(screen, font, small, controller, state, args, arm, pygame)
            clock.tick(60)
        return 0
    except (OSError, RuntimeError, ValueError, pygame.error) as exc:
        print(f"Keyboard control error: {exc}", file=sys.stderr)
        if diagnostic:
            diagnostic.write("error", message=str(exc))
        return 2
    finally:
        if worker:
            worker.close()
        if solver:
            solver.close()
        if diagnostic:
            diagnostic.write("exit")
            diagnostic.close()
        pygame.quit()


if __name__ == "__main__":
    sys.exit(main())
