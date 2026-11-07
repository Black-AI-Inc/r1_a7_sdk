"""URDF mesh visualization only. Never steps physics or talks to hardware."""
import multiprocessing as mp
import queue

import numpy as np
from scipy.spatial.transform import Rotation

from arm_control import ArmIK, SDK, URDF_REL, vector
from gui_control_core import feedback_pose


class URDFScene:
    """Headless pybullet scene: the arm URDF, base axes and a TCP marker with axes."""

    def __init__(self, tcp=(0, 0, 0, 0, 0, 0)):
        import pybullet as p
        self.p = p
        self.client = p.connect(p.DIRECT)
        self.arm = ArmIK(tcp=tcp)
        self.robot = p.loadURDF(str(SDK/URDF_REL), useFixedBase=True, physicsClientId=self.client)
        self.joints = {}
        for i in range(p.getNumJoints(self.robot, physicsClientId=self.client)):
            name = p.getJointInfo(self.robot, i, physicsClientId=self.client)[1].decode()
            self.joints[name] = i
        if set(self.joints) != {f"J{i}" for i in range(1, 8)}:
            raise ValueError("URDF render joint mapping mismatch")

        def box(half, position, color):
            shape = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=color,
                                        physicsClientId=self.client)
            return p.createMultiBody(baseMass=0, baseVisualShapeIndex=shape, basePosition=position,
                                      physicsClientId=self.client)

        for axis, color in enumerate(([.9, .25, .25, 1], [.2, .7, .35, 1], [.15, .45, .95, 1])):
            half, position = [.002]*3, [0., 0., .002]
            half[axis] = .06
            position[axis] += .06
            box(half, position, color)
        marker = p.createVisualShape(p.GEOM_SPHERE, radius=.006, rgbaColor=[1, .48, .12, 1],
                                     physicsClientId=self.client)
        self.tcp_marker = p.createMultiBody(baseMass=0, baseVisualShapeIndex=marker,
                                            physicsClientId=self.client)
        self.tcp_axes = []
        for axis, color in enumerate(([.9, .25, .25, 1], [.2, .7, .35, 1], [.15, .45, .95, 1])):
            half, offset = [.0015]*3, np.zeros(3)
            half[axis], offset[axis] = .02, .02
            self.tcp_axes.append((box(half, [0, 0, 0], color), offset))

    def set_q(self, q):
        for i, angle in enumerate(vector(q, 7), 1):
            self.p.resetJointState(self.robot, self.joints[f"J{i}"], float(angle),
                                    physicsClientId=self.client)
        pose = feedback_pose(self.arm, q)
        xyz = pose[:3, 3]
        self.p.resetBasePositionAndOrientation(self.tcp_marker, xyz, [0, 0, 0, 1],
                                               physicsClientId=self.client)
        orientation = Rotation.from_matrix(pose[:3, :3]).as_quat()
        for body, offset in self.tcp_axes:
            self.p.resetBasePositionAndOrientation(body, xyz+pose[:3, :3]@offset, orientation,
                                                   physicsClientId=self.client)

    def render(self, q, camera, width=640, height=480):
        """RGBA bytes of the arm at q; camera has yaw/pitch (deg) and distance (m)."""
        self.set_q(q)
        p = self.p
        view = p.computeViewMatrixFromYawPitchRoll([0, -.04, .19], camera["distance"],
                                                   camera["yaw"], camera["pitch"], 0, 2)
        projection = p.computeProjectionMatrixFOV(45, width/height, .02, 5)
        result = p.getCameraImage(width, height, view, projection, shadow=1,
                                  lightDirection=[-1, -1, 3], renderer=p.ER_TINY_RENDERER,
                                  flags=p.ER_NO_SEGMENTATION_MASK, physicsClientId=self.client)
        return np.asarray(result[2], dtype=np.uint8).reshape(height, width, 4).tobytes()

    def close(self):
        self.p.disconnect(self.client)


def render_loop(requests, results, tcp):
    """Body of the render process; reports a failure as an error result."""
    scene = None
    try:
        scene = URDFScene(tcp)
        while True:
            item = requests.get()
            if item is None:
                break
            tag, q, camera = item
            pixels = scene.render(q, camera)
            results.put((tag, pixels, ""))
    except Exception as exc:
        results.put(("", None, str(exc)))
    finally:
        if scene is not None:
            scene.close()


class RenderProcess:
    """render_loop in a spawned process; a new frame is requested only when idle."""

    def __init__(self, tcp):
        context = mp.get_context("spawn")
        self.requests = context.Queue(maxsize=1)
        self.results = context.Queue(maxsize=1)
        self.process = context.Process(target=render_loop, args=(self.requests, self.results, tcp), daemon=True)
        self.process.start()
        self.busy = False

    def submit(self, tag, q, camera):
        if not self.busy:
            if not self.process.is_alive():
                raise RuntimeError("URDF 渲染进程已退出")
            self.requests.put_nowait((tag, np.asarray(q).tolist(), camera.copy()))
            self.busy = True

    def poll(self):
        try:
            result = self.results.get_nowait()
            self.busy = False
            return result
        except queue.Empty:
            if not self.process.is_alive():
                raise RuntimeError("URDF 渲染进程已退出")
            return None

    def close(self):
        self.process.terminate()
        self.process.join(timeout=.1)
        if self.process.is_alive():
            self.process.kill()
            self.process.join(timeout=.1)
        self.requests.cancel_join_thread()
        self.results.cancel_join_thread()
        self.requests.close()
        self.results.close()
