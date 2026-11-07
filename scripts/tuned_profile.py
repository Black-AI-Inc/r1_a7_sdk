"""Hardware-validated control profile for the keyboard and GUI tools.

Tuned and validated on an R1-7A + DEX1-1 arm on 2026-11-03: gravity feedforward
at 100 %, J2/J4 stiffer against static friction (Kp 200 / Kd 1.41), a joint
integral outer loop with 3 N*m authority, soft faults held compliantly
(damping, then RECOVER) instead of releasing torque, and slightly
out-of-limit joints (J6 after a release) recovered on enable. Final field
round: 0.26-0.66 mm error on 10 mm steps, 0.62 mm while holding, 0.02 mm
drift in the damping hold.

--profile tuned is the default; --profile legacy restores the original tool
defaults (Kp 60 / Kd 0.5, no gravity, release on every fault). Options given
explicitly on the command line override the profile; a --kp/--kd other than
the profile's turns the per-joint J2/J4 gains off (uniform gains).
"""
import argparse

from gravity_model import DEFAULT_PROFILE

PROFILES = ("tuned", "legacy")
# Defaults for options the tools expose on the command line.
TUNED = dict(kp=100., kd=1., lead_limit=.08, tracking_limit=.1, gravity_scale=1.,
             gravity_profile=DEFAULT_PROFILE)
# Per-joint gains, used while kp/kd keep the TUNED base pair.
JOINT_KP = [100., 200., 100., 200., 100., 100., 100.]
JOINT_KD = [1., 1.41, 1., 1.41, 1., 1., 1.]
INTEGRAL_NM = 3.   # equal torque authority: 0.03 rad at Kp 100, 0.015 rad at Kp 200
# Worker-only options: integral gain (1/s), damping-hold gains, and the
# startup recovery margin (rad) and target pose.
EXTRA = dict(ki=1.5, fault_kp=20., fault_kd=1., recover_margin=.2, recover_q=[0.]*7)


def add_profile_argument(parser):
    parser.add_argument("--profile", choices=PROFILES, default="tuned",
                        help="tuned (default): validated gravity/gains/integral/damping; "
                             "legacy: original tool defaults")


def parse_args_with_profile(parser, argv=None):
    """Parse with the profile's values as defaults, then add its non-CLI options."""
    pre = argparse.ArgumentParser(add_help=False)
    add_profile_argument(pre)
    profile = pre.parse_known_args(argv)[0].profile
    if profile == "tuned":
        # Also sets lead/tracking limits for tools without such options; the
        # worker client passes on every option present on args.
        parser.set_defaults(**TUNED)
    args = parser.parse_args(argv)
    if profile == "tuned":
        apply_tuned_extras(args)
    return args


def apply_tuned_extras(args):
    """Set the worker-only options of the tuned profile on args (in place).

    The per-joint J2/J4 gains apply only while kp/kd equal the profile's base
    pair; any other pair gives uniform gains, with an integral limit of the
    same torque authority (at most 0.05 rad).
    """
    for key, value in EXTRA.items():
        setattr(args, key, value)
    if (args.kp, args.kd) == (TUNED["kp"], TUNED["kd"]):
        args.joint_kp, args.joint_kd = list(JOINT_KP), list(JOINT_KD)
        args.joint_integral_limit = [INTEGRAL_NM/kp for kp in JOINT_KP]
    else:
        args.joint_kp = args.joint_kd = args.joint_integral_limit = None
        args.integral_limit = min(.05, INTEGRAL_NM/args.kp)
    return args


def describe(args):
    """One-line summary of the active profile for the tool windows."""
    if getattr(args, "profile", "legacy") != "tuned":
        return f"legacy: Kp {args.kp:g} / Kd {args.kd:g}, gravity {getattr(args, 'gravity_scale', 0):.0%}"
    gains = ("J2/J4 Kp 200 / Kd 1.41, others 100/1" if getattr(args, "joint_kp", None)
             else f"Kp {args.kp:g} / Kd {args.kd:g}")
    return f"tuned: {gains}, gravity {args.gravity_scale:.0%}, integral, damping on soft faults"
