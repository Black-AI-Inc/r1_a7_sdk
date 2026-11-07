"""GUI/native CONFIG wire order, ranges and legacy defaults (SI units).

The defaults equal the native worker's own defaults (the legacy profile); the
tuned profile overrides kp, kd, lead_limit and tracking_limit (tuned_profile.TUNED).
"""
import math

# key, label, unit, default, minimum, maximum, decimals, step.
# The order is the CONFIG wire order expected by worker.cpp.
FIELDS = (
    ("joint_speed", "关节最大速度", "rad/s", .15, .01, 1., 3, .01),
    ("joint_accel", "关节最大加速度", "rad/s²", .5, .1, 5., 2, .1),
    ("joint_jerk", "关节最大 jerk", "rad/s³", 2., .5, 50., 2, .5),
    ("kp", "位置增益 Kp", "", 60., .01, 100., 2, 1.),
    ("kd", "阻尼增益 Kd", "", .5, .01, 5., 2, .1),
    ("lead_limit", "命令领先暂停阈值", "rad", .08, .01, .3, 3, .01),
    ("tracking_limit", "跟踪误差停止阈值", "rad", .2, .02, .5, 3, .01),
    ("move_timeout", "单目标运动超时", "s", 60., 5., 300., 1, 5.),
)
DEFAULTS = {key: default for key, _, _, default, *_ in FIELDS}


def validate(values):
    """The FIELDS values as floats; ValueError with a GUI message if out of range."""
    result = {}
    for key, label, _, _, lower, upper, *_ in FIELDS:
        value = float(values[key])
        if not math.isfinite(value) or not lower <= value <= upper:
            raise ValueError(f"{label}允许范围：{lower:g}–{upper:g}")
        result[key] = value
    if result["lead_limit"] >= result["tracking_limit"]:
        raise ValueError("命令领先暂停阈值必须小于跟踪误差停止阈值")
    return result


def config_command(revision, values):
    """CONFIG line for the worker; revision must exceed the last acknowledged one."""
    values = validate(values)
    return f"CONFIG {revision} " + " ".join(format(values[key], ".12g") for key in DEFAULTS)
