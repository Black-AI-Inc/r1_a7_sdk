"""URDF static gravity model, in URDF joint coordinates and output-side Nm.

No serial IO. The native worker evaluates the same model at measured q
(keyboard_native/gravity_model.h). Only masses and centres of mass are needed
for static gravity compensation. Run directly for an offline torque report.
"""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

if __package__:
    from .arm_control import ArmIK, SDK, URDF_REL, pose_matrix, vector
else:
    from arm_control import ArmIK, SDK, URDF_REL, pose_matrix, vector

DEFAULT_PROFILE = SDK / "config/dex1_gravity.json"


def bounded(value, low, high, name):
    """float(value), or ValueError unless finite and within low..high."""
    value = float(value)
    if not np.isfinite(value) or not low <= value <= high:
        raise ValueError(f"{name} must be finite and in {low}..{high}")
    return value


class GravityModel:
    """Arm links from the URDF plus an optional tool from a gravity profile JSON.

    The tool (gripper and adapter, plus the profile's object when carrying)
    is lumped into one mass and COM in the Link7 frame.
    """

    def __init__(self, urdf=SDK / URDF_REL, profile=None, carrying=False):
        self.arm = ArmIK(urdf=urdf)
        root = ET.parse(urdf).getroot()
        links = {link.get("name"): link for link in root.findall("link")}
        self.masses, self.coms = [], []
        for i in range(1, 8):
            inertial = links[f"Link{i}"].find("inertial")
            self.masses.append(bounded(inertial.find("mass").get("value"), 0, 10, "link mass"))
            origin = inertial.find("origin")
            # The inertial-frame origin IS the COM; its rotation does not rotate xyz.
            self.coms.append(vector(origin.get("xyz").split(), 3))
        self.tool_mass, self.tool_com = 0., np.zeros(3)
        self.profile = None
        if profile is not None:
            data = json.loads(Path(profile).read_text())
            if data.get("schema") != 1:
                raise ValueError("Unsupported gravity profile schema")
            if data.get("mount_confirmed") is not True:
                raise ValueError("Gravity profile mounting direction is not confirmed")
            mount = vector(data["link7_to_gripper_xyz_rpy"], 6)
            if np.linalg.norm(mount[:3]) > .3:
                raise ValueError("Gripper mount translation exceeds 0.3 m")
            transform = pose_matrix(mount[:3], mount[3:])
            total = bounded(data["empty_mass_kg"], .001, 2, "empty mass")
            adapter = bounded(data["adapter_mass_kg"], 0, total, "adapter mass")
            dex_com = vector(data["gripper_com_in_gripper_m"], 3)
            adapter_com = vector(data["adapter_com_in_link7_m"], 3)
            if np.linalg.norm(dex_com) > .3 or np.linalg.norm(adapter_com) > .3:
                raise ValueError("Invalid tool centre of mass (> 0.3 m)")
            moment = (total-adapter) * (transform[:3, :3] @ dex_com + transform[:3, 3])
            moment += adapter * adapter_com
            if carrying:
                mass = bounded(data["object_mass_kg"], 0, .5, "object mass")
                com = vector(data["object_com_in_gripper_m"], 3)
                if np.linalg.norm(com) > .4:
                    raise ValueError("Invalid object centre of mass (> 0.4 m)")
                moment += mass * (transform[:3, :3] @ com + transform[:3, 3])
                total += mass
            self.tool_mass, self.tool_com = total, moment/total
            self.profile = data
        elif carrying:
            raise ValueError("--carrying requires a gravity profile")
        # Upright base_link. Mounting on a wall requires another gravity vector.
        self.gravity = np.array([0., 0., -9.81])

    def evaluate(self, q):
        """Feedforward torques that hold q (arm, tool, total) and the potential energy."""
        q = vector(q, 7)
        transform = np.eye(4)
        positions, axes, world_coms = [], [], []
        for origin, axis, angle, com in zip(self.arm.origins, self.arm.axes, q, self.coms):
            transform = transform @ origin
            positions.append(transform[:3, 3].copy())
            axes.append(transform[:3, :3] @ axis)
            rotation = np.eye(4)
            rotation[:3, :3] = Rotation.from_rotvec(axis*angle).as_matrix()
            transform = transform @ rotation
            world_coms.append(transform[:3, :3] @ com + transform[:3, 3])
        arm, tool = np.zeros(7), np.zeros(7)
        for i, (com, mass) in enumerate(zip(world_coms, self.masses)):
            for j in range(i+1):
                arm[j] -= np.dot(axes[j], np.cross(com-positions[j], mass*self.gravity))
        tool_com = transform[:3, :3] @ self.tool_com + transform[:3, 3]
        for j in range(7):
            tool[j] = -np.dot(axes[j], np.cross(tool_com-positions[j], self.tool_mass*self.gravity))
        potential = -sum(m*np.dot(self.gravity, c) for m, c in zip(self.masses, world_coms))
        potential -= self.tool_mass*np.dot(self.gravity, tool_com)
        return {"arm_nm": arm, "tool_nm": tool, "total_nm": arm+tool,
                "potential_j": float(potential)}

    def native_record(self):
        """Versioned numeric record, passed as ONE argv value (no temp files)."""
        values = [1, *self.gravity]
        for origin, axis, mass, com in zip(self.arm.origins, self.arm.axes, self.masses, self.coms):
            values += [*origin[:3, :3].ravel(), *origin[:3, 3], *axis, mass, *com]
        values += [self.tool_mass, *self.tool_com]
        return " ".join(format(float(x), ".17g") for x in values)


def add_gravity_args(parser):
    parser.add_argument("--gravity-profile", type=Path,
                        help="Tool mass/COM JSON (tuned profile: config/dex1_gravity.json; legacy: none)")
    parser.add_argument("--gravity-scale", type=float, default=0.,
                        help="Fraction of the gravity torque fed forward, 0..1 (tuned profile: 1; legacy: 0 = off)")
    parser.add_argument("--carrying", action="store_true", help="Include profile object mass; restart after changing load")


def worker_gravity_args(args):
    """Worker options for gravity compensation; [] without a profile."""
    scale = bounded(getattr(args, "gravity_scale", 0.), 0, 1, "gravity scale")
    profile = getattr(args, "gravity_profile", None)
    carrying = getattr(args, "carrying", False)
    if scale and profile is None:
        raise ValueError("Gravity compensation requires --gravity-profile (include the mounted tool)")
    if profile is None and not carrying:
        return []
    model = GravityModel(urdf=getattr(args, "urdf", SDK / URDF_REL),
                         profile=profile, carrying=carrying)
    # The SDK converts torque with the same dir*scale factor as velocity. That
    # is right for sign changes but not for a non-unit transmission scale.
    if getattr(args, "execute", False) and scale:
        calibration = np.loadtxt(getattr(args, "calibration_file", SDK / "config/joint_calib.txt"),
                                 comments="#")
        if calibration.shape != (7, 4) or not np.allclose(calibration[:, 2], 1., rtol=0, atol=1e-9):
            raise ValueError("Gravity compensation requires unit calibration scale; "
                             "torque mapping must be audited otherwise")
    return ["--gravity-model", model.native_record(), "--gravity-scale", str(scale)]


def main(argv=None):
    parser = argparse.ArgumentParser(description="Offline gravity torque prediction; never connects to hardware")
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--q", type=float, nargs=7)
    args = parser.parse_args(argv)
    from keyboard_control import DEFAULT_READY_Q
    poses = {"Home": [0.]*7, "P": DEFAULT_READY_Q} if args.q is None else {"q": args.q}
    result = {"units": "Nm at joint output, URDF sign", "hardware_validated": False,
              "com_quality": "CAD estimate scaled to measured mass; object COM is an assumed grasp centre",
              "poses": {}}
    for name, q in poses.items():
        result["poses"][name] = {}
        for carrying in (False, True):
            model = GravityModel(profile=args.profile, carrying=carrying)
            model.arm.check_q(q)
            values = model.evaluate(q)
            result["poses"][name]["loaded" if carrying else "empty"] = {
                "mass_kg": model.tool_mass, "com_in_link7_m": model.tool_com.tolist(),
                **{k: v.tolist() for k, v in values.items() if k.endswith("_nm")}}
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
