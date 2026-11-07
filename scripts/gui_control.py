#!/usr/bin/env python3
"""R1-7A joint/pose sliders, live feedback, and an interactive URDF view."""
import argparse
import math
from pathlib import Path
import signal
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation
from PySide6.QtCore import Qt, QTimer, Signal, QSignalBlocker
from PySide6.QtGui import QFont, QImage, QKeySequence, QPixmap, QShortcut
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QLabel, QPushButton,
    QVBoxLayout, QHBoxLayout, QGridLayout, QStackedWidget, QSlider, QDoubleSpinBox,
    QButtonGroup, QScrollArea, QSplitter, QComboBox, QTableWidget,
    QTableWidgetItem, QHeaderView, QAbstractItemView, QSizePolicy)

from arm_control import ArmIK, SDK, vector
from keyboard_control import Worker, DiagnosticLog, DEFAULT_READY_Q
from gui_control_core import GuiController, PoseSolver, feedback_pose
from gui_urdf import RenderProcess
from gui_settings import FIELDS, DEFAULTS, validate
from tuned_profile import TUNED, add_profile_argument, apply_tuned_extras, describe, parse_args_with_profile


STYLE = """
QWidget { background: #ffffff; color: #20252b; font-family: "Noto Sans CJK SC"; font-size: 13px; }
QLabel#title { font-size: 24px; font-weight: 600; }
QLabel#subtitle, QLabel#readout { color: #707780; }
QLabel#section { font-weight: 600; padding-top: 10px; padding-bottom: 8px; }
QLabel#badge { color: #14705f; padding: 8px 12px; font-weight: 600; }
QPushButton { min-height: 20px; border: 1px solid #d7dce1; border-radius: 5px; padding: 7px 14px; }
QPushButton:hover { border-color: #2279c8; color: #1768ac; }
QPushButton:pressed { background: #edf4fb; }
QPushButton:disabled { color: #a6acb2; border-color: #e8eaed; }
QPushButton#nav { border: none; border-radius: 0; padding: 10px 18px; color: #707780; }
QPushButton#nav:checked { color: #1768ac; border-bottom: 2px solid #1768ac; font-weight: 600; }
QPushButton#hold, QPushButton#apply { background: #1768ac; color: white; border-color: #1768ac; }
QPushButton#hold:disabled, QPushButton#apply:disabled { background: #a6b8c8; border-color: #a6b8c8; }
QPushButton#stop { color: #b72e37; border-color: #d9a5a9; }
QPushButton#stop:hover { background: #fff4f4; }
QSlider::groove:horizontal { height: 4px; background: #e5e8ec; border-radius: 2px; }
QSlider::sub-page:horizontal { background: #4386bf; border-radius: 2px; }
QSlider::handle:horizontal { background: #1768ac; width: 14px; margin: -5px 0; border-radius: 7px; }
QSlider::handle:horizontal:disabled { background: #b7bec5; }
QDoubleSpinBox { border: 1px solid #d7dce1; border-radius: 4px; padding: 3px; }
QDoubleSpinBox::up-button { subcontrol-origin: border; subcontrol-position: top right; width: 18px; border-left: 1px solid #e5e8ec; border-bottom: none; background: white; }
QDoubleSpinBox::down-button { subcontrol-origin: border; subcontrol-position: bottom right; width: 18px; border-left: 1px solid #e5e8ec; border-top: none; background: white; }
QDoubleSpinBox::up-arrow { image: url(@UI@/up.svg); width: 10px; height: 8px; }
QDoubleSpinBox::down-arrow { image: url(@UI@/down.svg); width: 10px; height: 8px; }
QDoubleSpinBox:focus { border-color: #1768ac; }
QDoubleSpinBox:disabled { color: #9299a1; border-color: #e8eaed; }
QLabel#note { color: #535e68; padding: 8px 0; }
QComboBox { padding: 6px; border: 1px solid #d7dce1; border-radius: 4px; }
QComboBox::drop-down { border: none; width: 22px; }
QComboBox::down-arrow { image: url(@UI@/down.svg); width: 10px; height: 8px; }
QTableWidget { border: none; }
QHeaderView::section { background: white; color: #707780; border: none; padding: 5px; }
QTableCornerButton::section { background: white; border: none; }
QScrollArea, QStackedWidget { border: none; }
QSplitter::handle { background: white; }
""".replace("@UI@", (Path(__file__).resolve().parent/"ui").as_posix())


class NoWheelSlider(QSlider):
    def wheelEvent(self, event):
        event.ignore()


class NoWheelSpin(QDoubleSpinBox):
    def wheelEvent(self, event):
        event.ignore()


class SliderRow(QWidget):
    changed = Signal(float)

    def __init__(self, name, lower, upper, decimals, unit):
        super().__init__()
        self.scale, self.unit = 10**decimals, unit
        lower = math.ceil(lower*self.scale)/self.scale
        upper = math.floor(upper*self.scale)/self.scale
        layout = QGridLayout(self)
        layout.setContentsMargins(0, 1, 0, 1)
        layout.setVerticalSpacing(0)
        label = QLabel(name)
        label.setFixedWidth(48)
        self.slider = NoWheelSlider(Qt.Horizontal)
        self.slider.setRange(round(lower*self.scale), round(upper*self.scale))
        self.slider.setMinimumWidth(150)
        self.spin = NoWheelSpin()
        self.spin.setDecimals(decimals)
        self.spin.setRange(lower, upper)
        self.spin.setSingleStep(.1 if unit == "°" else 1.)
        self.spin.setSuffix(" " + unit)
        self.spin.setKeyboardTracking(False)
        self.spin.setFixedWidth(108)
        self.spin.setFixedHeight(32)
        self.readout = QLabel("命令 —     反馈 —")
        self.readout.setObjectName("readout")
        self.readout.setFixedHeight(22)
        self.setFixedHeight(64)
        layout.addWidget(label, 0, 0)
        layout.addWidget(self.slider, 0, 1)
        layout.addWidget(self.spin, 0, 2)
        layout.addWidget(self.readout, 1, 1, 1, 2)
        layout.setColumnStretch(1, 1)
        self.slider.valueChanged.connect(lambda v: self.spin.setValue(v/self.scale))
        self.spin.valueChanged.connect(self._changed)

    def _changed(self, value):
        with QSignalBlocker(self.slider):
            self.slider.setValue(round(value*self.scale))
        self.changed.emit(value)

    def set_target(self, value):
        if self.spin.hasFocus() and self.spin.lineEdit().isModified():
            return
        with QSignalBlocker(self.spin), QSignalBlocker(self.slider):
            self.spin.setValue(value)
            self.slider.setValue(round(value*self.scale))

    def set_readout(self, command, measured):
        self.readout.setText(f"命令 {command:+.2f} {self.unit}     反馈 {measured:+.2f} {self.unit}")

    def set_editable(self, enabled):
        self.slider.setEnabled(enabled)
        self.spin.setEnabled(enabled)


class RobotView(QLabel):
    def __init__(self):
        super().__init__("正在加载 URDF 网格…")
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(440, 300)
        self.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Expanding)
        self.setStyleSheet("background: white; color: #707780;")
        self.frame = None
        self.anchor = None
        self.reset_camera()

    def reset_camera(self):
        self.camera = {"yaw": 45., "pitch": -22., "distance": .75}

    def show_pixels(self, pixels):
        self.frame = QImage(pixels, 640, 480, 640*4, QImage.Format_RGBA8888).copy()
        self._display()

    def _display(self):
        if self.frame is not None:
            self.setPixmap(QPixmap.fromImage(self.frame).scaled(self.size(), Qt.KeepAspectRatio,
                                                                Qt.SmoothTransformation))

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._display()

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.anchor = event.position()

    def mouseMoveEvent(self, event):
        if self.anchor is not None:
            delta = event.position()-self.anchor
            self.camera["yaw"] += delta.x()*.45
            self.camera["pitch"] = float(np.clip(self.camera["pitch"]+delta.y()*.35, -85, 85))
            self.anchor = event.position()

    def mouseReleaseEvent(self, event):
        self.anchor = None

    def wheelEvent(self, event):
        self.camera["distance"] = float(np.clip(self.camera["distance"]*
                                                  math.exp(-event.angleDelta().y()/1200), .25, 2.))
        event.accept()


class ControlWindow(QMainWindow):
    def __init__(self, args, *, start_renderer=True, autoconnect=True):
        super().__init__()
        self.args, self.arm = args, ArmIK(tcp=args.tcp)
        self.arm.check_q(args.ready_q)
        self.worker = self.solver = self.controller = self.renderer = None
        self.last_seq = -1
        self.last_ping = self.last_render = 0.
        self.closing = False
        self.diagnostic = DiagnosticLog(args.log or SDK/"build/keyboard/logs"/f"gui-{time.time_ns()}.jsonl")
        self.diagnostic.write("gui_startup", options=vars(args))
        self.setWindowTitle("R1-7A · 关节 / 末端位姿 / URDF" + (" [真机]" if args.execute else " [模拟]"))
        self.resize(1360, 900)
        self.setMinimumSize(1160, 780)
        self._build_ui()
        if start_renderer:
            self.renderer = RenderProcess(args.tcp)
        self.timer = QTimer(self)
        self.timer.setInterval(20)
        self.timer.timeout.connect(self._tick)
        self.timer.start()
        self.shortcuts = []
        for key, action in [("Space", self.hold), ("Escape", self.stop),
                            ("H", lambda: self.preset("HOME")), ("P", lambda: self.preset("READY"))]:
            shortcut = QShortcut(QKeySequence(key), self)
            shortcut.activated.connect(action)
            self.shortcuts.append(shortcut)
        QApplication.instance().applicationStateChanged.connect(self._app_state)
        if autoconnect and not args.execute:
            QTimer.singleShot(0, self.connect_control)

    def _build_ui(self):
        central = QWidget()
        central.setObjectName("main")
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setContentsMargins(20, 16, 20, 16)
        layout.setSpacing(12)
        header = QHBoxLayout()
        title_col = QVBoxLayout()
        title = QLabel("R1-7A  控制台")
        title.setObjectName("title")
        title_col.addWidget(title)
        subtitle = QLabel("Joint · Cartesian · URDF   /   平滑位置控制")
        subtitle.setObjectName("subtitle")
        title_col.addWidget(subtitle)
        header.addLayout(title_col)
        header.addStretch()
        self.badge = QLabel("真机 · 未连接" if self.args.execute else "模拟 · 无串口通信")
        self.badge.setObjectName("badge")
        header.addWidget(self.badge)
        self.connect_button = QPushButton("连接真机" if self.args.execute else "连接模拟")
        self.connect_button.clicked.connect(self.connect_control)
        header.addWidget(self.connect_button)
        layout.addLayout(header)

        toolbar = QHBoxLayout()
        self.p_button, self.h_button = QPushButton("P · 准备姿态"), QPushButton("H · 回零")
        self.hold_button, self.stop_button = QPushButton("保持  /  Space"), QPushButton("停止 · 释放力矩  /  Esc")
        self.hold_button.setObjectName("hold")
        self.stop_button.setObjectName("stop")
        self.p_button.clicked.connect(lambda: self.preset("READY"))
        self.h_button.clicked.connect(lambda: self.preset("HOME"))
        self.recover_button = QPushButton("恢复 · Recover")
        self.recover_button.setToolTip("故障阻尼保持（未释放力矩）时，在当前位置恢复位置保持")
        self.hold_button.clicked.connect(self.hold)
        self.stop_button.clicked.connect(self.stop)
        self.recover_button.clicked.connect(lambda: self._action("recover"))
        for button in (self.p_button, self.h_button, self.hold_button, self.recover_button, self.stop_button):
            toolbar.addWidget(button)
        toolbar.addStretch()
        toolbar.addWidget(QLabel(self.args.port if self.args.execute else "离线模拟"))
        layout.addLayout(toolbar)

        splitter = QSplitter(Qt.Horizontal)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        left = QWidget()
        controls = QVBoxLayout(left)
        controls.setContentsMargins(0, 0, 8, 0)
        modes = QHBoxLayout()
        modes.setSpacing(0)
        self.joint_mode = QPushButton("关节")
        self.xyz_mode = QPushButton("末端位姿")
        self.settings_mode = QPushButton("参数")
        self.mode_group = QButtonGroup(self)
        for button in (self.joint_mode, self.xyz_mode, self.settings_mode):
            button.setObjectName("nav")
            button.setCheckable(True)
            self.mode_group.addButton(button)
            modes.addWidget(button)
        modes.addStretch()
        self.joint_mode.setChecked(True)
        self.joint_mode.clicked.connect(lambda: self.mode_changed("joint"))
        self.xyz_mode.clicked.connect(lambda: self.mode_changed("cartesian"))
        self.settings_mode.clicked.connect(self.show_settings)
        controls.addLayout(modes)
        self.pages = QStackedWidget()
        controls.addWidget(self.pages)
        self.joint_rows, self.xyz_rows, self.rpy_rows = [], [], []
        joints = QWidget()
        joint_layout = QVBoxLayout(joints)
        joint_layout.setContentsMargins(0, 6, 0, 0)
        joint_layout.setSpacing(4)
        joint_layout.addWidget(QLabel("关节角度 · °", objectName="section"))
        joint_layout.addWidget(QLabel("滑块 / 数字框为目标值，下方为命令与反馈", objectName="subtitle"))
        for i in range(7):
            row = SliderRow(f"J{i+1}", np.degrees(self.arm.lower[i]), np.degrees(self.arm.upper[i]), 2, "°")
            row.changed.connect(lambda v, i=i: self.edit_joint(i, v))
            self.joint_rows.append(row)
            joint_layout.addWidget(row)
        joint_layout.addStretch()
        self.pages.addWidget(joints)
        cartesian = QWidget()
        xyz_layout = QVBoxLayout(cartesian)
        xyz_layout.setContentsMargins(0, 6, 0, 0)
        xyz_layout.setSpacing(4)
        xyz_layout.addWidget(QLabel("末端位置 · base_link · mm", objectName="section"))
        reach = math.ceil((sum(np.linalg.norm(o[:3, 3]) for o in self.arm.origins)
                           + np.linalg.norm(self.arm.tcp[:3, 3]))*1000/10)*10
        for i, axis in enumerate("XYZ"):
            row = SliderRow(axis, -reach, reach, 1, "mm")
            row.changed.connect(lambda v, i=i: self.edit_xyz(i, v))
            self.xyz_rows.append(row)
            xyz_layout.addWidget(row)
        xyz_layout.addWidget(QLabel("末端旋转 · 固定轴 RPY · °", objectName="section"))
        for i, axis in enumerate(("Roll", "Pitch", "Yaw")):
            row = SliderRow(axis, -180, 180, 2, "°")
            row.changed.connect(lambda v, i=i: self.edit_rpy(i, v))
            self.rpy_rows.append(row)
            xyz_layout.addWidget(row)
        xyz_layout.addWidget(QLabel("旋转保持所选 XYZ；平移保持所选朝向。", objectName="subtitle"))
        xyz_layout.addStretch()
        self.pages.addWidget(cartesian)
        settings = QWidget()
        settings_layout = QVBoxLayout(settings)
        settings_layout.setContentsMargins(0, 6, 0, 0)
        settings_layout.addWidget(QLabel("运动与保护参数", objectName="section"))
        explanation = QLabel("默认使用推荐值。修改后点击应用，驱动会先保持当前位置。")
        explanation.setWordWrap(True)
        explanation.setObjectName("subtitle")
        settings_layout.addWidget(explanation)
        form = QGridLayout()
        form.setVerticalSpacing(14)
        self.setting_inputs, self.setting_actual = {}, {}
        for row, (key, label, unit, default, low, high, decimals, step) in enumerate(FIELDS):
            form.addWidget(QLabel(label), row, 0)
            spin = NoWheelSpin()
            spin.setDecimals(decimals)
            spin.setRange(low, high)
            spin.setSingleStep(step)
            spin.setSuffix(" " + unit if unit else "")
            spin.setKeyboardTracking(False)
            spin.setMinimumHeight(32)
            spin.setValue(getattr(self.args, key))
            spin.setToolTip(f"推荐 {default:g} {unit}；范围 {low:g}–{high:g} {unit}")
            self.setting_inputs[key] = spin
            form.addWidget(spin, row, 1)
            actual = QLabel("—")
            actual.setObjectName("subtitle")
            self.setting_actual[key] = actual
            form.addWidget(actual, row, 2)
        form.setColumnStretch(1, 1)
        settings_layout.addLayout(form)
        settings_layout.addWidget(QLabel("右列为驱动当前生效值；暂停阈值必须小于停止阈值。", objectName="subtitle"))
        setting_actions = QHBoxLayout()
        self.defaults_button = QPushButton("恢复推荐值")
        self.defaults_button.clicked.connect(self.restore_defaults)
        self.apply_button = QPushButton("应用参数")
        self.apply_button.setObjectName("apply")
        self.apply_button.clicked.connect(self.apply_settings)
        setting_actions.addWidget(self.defaults_button)
        setting_actions.addWidget(self.apply_button)
        settings_layout.addLayout(setting_actions)
        self.settings_status = QLabel("待连接")
        self.settings_status.setWordWrap(True)
        settings_layout.addWidget(self.settings_status)
        fixed = QLabel("关节限位、反馈检查和心跳保护始终启用。\n设置只在本次窗口内保留，重启恢复启动参数。")
        fixed.setWordWrap(True)
        fixed.setObjectName("subtitle")
        settings_layout.addWidget(fixed)
        settings_layout.addStretch()
        self.pages.addWidget(settings)
        hint = QLabel("松开滑块后继续平滑到目标；保持 / 失焦会取消运动。\n末端不可达时保持，可先按 P 进入准备姿态。")
        hint.setWordWrap(True)
        hint.setObjectName("subtitle")
        controls.addWidget(hint)
        scroll.setWidget(left)
        scroll.setMinimumWidth(510)
        splitter.addWidget(scroll)

        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.setContentsMargins(8, 0, 0, 0)
        viewer = QWidget()
        viewer_layout = QVBoxLayout(viewer)
        viewer_layout.setContentsMargins(0, 0, 0, 0)
        view_options = QHBoxLayout()
        view_options.addWidget(QLabel("URDF 实时姿态", objectName="section"))
        self.view_mode = QComboBox()
        for label, value in [("实际反馈", "feedback"), ("平滑命令", "command"), ("目标预览", "target")]:
            self.view_mode.addItem(label, value)
        view_options.addWidget(self.view_mode)
        view_options.addStretch()
        self.camera_button = QPushButton("复位视角")
        view_options.addWidget(self.camera_button)
        viewer_layout.addLayout(view_options)
        self.view = RobotView()
        self.camera_button.clicked.connect(self.view.reset_camera)
        viewer_layout.addWidget(self.view, 1)
        self.view_caption = QLabel("左键拖动旋转 · 滚轮缩放 | XYZ 轴：红 / 绿 / 蓝 · 橙点：TCP")
        self.view_caption.setObjectName("subtitle")
        viewer_layout.addWidget(self.view_caption)
        right_layout.addWidget(viewer, 1)
        readings = QWidget()
        readings_layout = QVBoxLayout(readings)
        readings_layout.setContentsMargins(0, 0, 0, 0)
        readings_layout.addWidget(QLabel("末端实时数值 · base_link", objectName="section"))
        self.pose_table = QTableWidget(6, 3)
        self.pose_table.setHorizontalHeaderLabels(["目标", "命令", "实际反馈"])
        self.pose_table.setVerticalHeaderLabels(["X (mm)", "Y (mm)", "Z (mm)", "Roll (°)", "Pitch (°)", "Yaw (°)"])
        self.pose_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.pose_table.verticalHeader().setDefaultSectionSize(25)
        self.pose_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.pose_table.setSelectionMode(QAbstractItemView.NoSelection)
        self.pose_table.setFixedHeight(185)
        self.pose_table.setShowGrid(False)
        for row in range(6):
            for col in range(3):
                item = QTableWidgetItem("—")
                item.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
                self.pose_table.setItem(row, col, item)
        readings_layout.addWidget(self.pose_table)
        self.tracking = QLabel("尚无反馈")
        readings_layout.addWidget(self.tracking)
        self.gravity_status = QLabel("重力补偿关闭")
        self.gravity_status.setWordWrap(True)
        readings_layout.addWidget(self.gravity_status)
        self.profile_status = QLabel("控制配置 " + describe(self.args))
        self.profile_status.setWordWrap(True)
        readings_layout.addWidget(self.profile_status)
        right_layout.addWidget(readings)
        splitter.addWidget(right)
        splitter.setSizes([550, 750])
        layout.addWidget(splitter, 1)
        self.note = QLabel("点击连接后以当前反馈初始化，不自动移动。" if self.args.execute else "正在启动模拟驱动…")
        self.note.setObjectName("note")
        self.note.setWordWrap(True)
        layout.addWidget(self.note)
        self._refresh_controls()

    def _send(self, message):
        if message != "PING":
            self.diagnostic.write("command", message=message)
        self.worker.send(message)

    def connect_control(self):
        if self.worker is not None:
            self.disconnect_control("已断开并释放力矩")
            return
        try:
            self.solver = PoseSolver(self.args.tcp)
            self.worker = Worker(self.args, self.arm)
            self.controller = GuiController(self._send, self.arm, self.solver, self.args.ready_q)
            self.last_seq = -1
            self.connect_button.setText("断开 · 释放力矩")
            self.note.setText("读取反馈并使能位置保持…")
            self.diagnostic.write("connect", hardware=self.args.execute)
        except Exception as exc:
            self.disconnect_control(str(exc))

    def disconnect_control(self, reason):
        # Release hardware before waiting on any auxiliary process.
        if self.worker is not None:
            self.worker.close()
            self.worker = None
        if self.solver is not None:
            self.solver.close()
            self.solver = None
        self.controller = None
        self.connect_button.setText("连接真机" if self.args.execute else "连接模拟")
        self.note.setText(reason)
        self.diagnostic.write("disconnect", reason=reason)
        self._refresh_controls()

    def _action(self, name, *args):
        if self.controller is not None:
            try:
                getattr(self.controller, name)(*args)
                self.diagnostic.write("gui_action", action=name, arguments=args)
                self._refresh_controls()
            except Exception as exc:
                self.disconnect_control("控制错误，已断开：" + str(exc))

    def edit_joint(self, i, degrees):
        self._action("edit_joint", i, math.radians(degrees), time.monotonic())

    def edit_xyz(self, i, mm):
        self._action("edit_xyz", i, mm/1000, time.monotonic())

    def edit_rpy(self, i, degrees):
        self._action("edit_rpy", i, math.radians(degrees), time.monotonic())

    def mode_changed(self, mode):
        self.pages.setCurrentIndex(0 if mode == "joint" else 1)
        self._action("set_mode", mode)

    def show_settings(self):
        self.pages.setCurrentIndex(2)
        self._action("hold", "调整参数前，已保持当前位置")

    def restore_defaults(self):
        recommended = dict(DEFAULTS)
        if self.args.profile == "tuned":
            recommended.update({key: TUNED[key] for key in DEFAULTS if key in TUNED})
        for key, value in recommended.items():
            self.setting_inputs[key].setValue(value)
        self.settings_status.setText("推荐值已填入，点击应用后生效")

    def apply_settings(self):
        try:
            values = validate({key: spin.value() for key, spin in self.setting_inputs.items()})
        except ValueError as exc:
            self.settings_status.setText(str(exc))
            return
        self._action("apply_settings", values, time.monotonic())

    def preset(self, name):
        self.pages.setCurrentIndex(0)
        self._action("preset", name)

    def hold(self):
        self._action("hold")

    def stop(self):
        self._action("stop")

    def _app_state(self, state):
        if state != Qt.ApplicationActive and self.controller is not None and self.controller.enabled:
            self._action("hold", "窗口失焦，已保持当前命令位置")

    def _refresh_controls(self):
        c = self.controller
        enabled = c is not None and c.enabled
        for button in (self.p_button, self.h_button, self.joint_mode, self.xyz_mode):
            button.setEnabled(enabled)
        self.hold_button.setEnabled(c is not None and c.initialized and not c.stopped)
        self.recover_button.setEnabled(c is not None and c.state is not None and c.state["mode"] == "damping")
        self.stop_button.setEnabled(self.worker is not None)
        for row in self.joint_rows:
            row.set_editable(enabled and c.mode == "joint")
        for row in self.xyz_rows + self.rpy_rows:
            row.set_editable(enabled and c.mode == "cartesian")
        self.apply_button.setEnabled(enabled)
        self.defaults_button.setEnabled(c is None or c.pending_config is None)
        for key, spin in self.setting_inputs.items():
            spin.setEnabled(c is None or c.pending_config is None)
            value = c.settings[key] if c is not None and c.settings is not None else None
            self.setting_actual[key].setText(f"{value:g}" if value is not None else "—")
        if c is not None and c.settings is not None:
            # Reconnect in this window uses only confirmed settings, not un-applied edits.
            for key, value in c.settings.items():
                setattr(self.args, key, value)
            if self.args.profile == "tuned":   # a changed Kp/Kd turns the per-joint gains off
                apply_tuned_extras(self.args)
            if c.pending_config is not None:
                self.settings_status.setText("等待驱动确认…")
            elif self.settings_status.text() == "等待驱动确认…":
                self.settings_status.setText(c.note)
            elif self.settings_status.text() == "待连接":
                self.settings_status.setText("已读取驱动参数；修改后点击应用")
        if c is None or not c.initialized:
            self.badge.setText(("真机" if self.args.execute else "模拟") + " · 未连接 / 初始化")
            self.tracking.setText("暂无实时反馈")
            self.gravity_status.setText("重力补偿：等待驱动反馈")
            for row in self.joint_rows + self.xyz_rows + self.rpy_rows:
                row.readout.setText("命令 —     反馈 —")
            for row in range(6):
                for col in range(3):
                    self.pose_table.item(row, col).setText("—")
            return
        if self.pages.currentIndex() != 2:
            with QSignalBlocker(self.joint_mode), QSignalBlocker(self.xyz_mode):
                self.joint_mode.setChecked(c.mode == "joint")
                self.xyz_mode.setChecked(c.mode == "cartesian")
            self.pages.setCurrentIndex(0 if c.mode == "joint" else 1)
        measured, command = np.asarray(c.state["q"]), np.asarray(c.state["command"])
        measured_pose, command_pose = feedback_pose(self.arm, measured), self.arm.fk(command)
        for i, row in enumerate(self.joint_rows):
            row.set_target(np.degrees(c.target_q[i]))
            row.set_readout(np.degrees(command[i]), np.degrees(measured[i]))
        for i, row in enumerate(self.xyz_rows):
            row.set_target(c.target_xyz[i]*1000)
            row.set_readout(command_pose[i, 3]*1000, measured_pose[i, 3]*1000)
        command_rpy = Rotation.from_matrix(command_pose[:3, :3]).as_euler("xyz", degrees=True)
        measured_rpy = Rotation.from_matrix(measured_pose[:3, :3]).as_euler("xyz", degrees=True)
        for i, row in enumerate(self.rpy_rows):
            row.set_target(np.degrees(c.target_rpy[i]))
            row.set_readout(command_rpy[i], measured_rpy[i])
        targets = np.r_[c.target_xyz*1000, np.degrees(c.target_rpy)]
        columns = [targets] + [np.r_[p[:3, 3]*1000, Rotation.from_matrix(p[:3, :3]).as_euler("xyz", degrees=True)]
                                for p in (command_pose, measured_pose)]
        for col, values in enumerate(columns):
            for row, value in enumerate(values):
                self.pose_table.item(row, col).setText(f"{value:+.2f}")
        self.tracking.setText(f"最大跟踪误差 {np.degrees(np.max(np.abs(command-measured))):.2f}°  |  "
                             f"速度 ≤ {self.args.joint_speed:g} rad/s")
        if c.state.get("gravity_model"):
            torques = " / ".join(f"{v:+.2f}" for v in c.state.get("tau_ff", []))
            self.gravity_status.setText(f"重力补偿 {c.state.get('gravity_gain', 0):.0%} · J1–J7: {torques} N·m"
                                        + (" · 补偿力矩已限幅" if c.state.get("gravity_saturated") else ""))
        else:
            self.gravity_status.setText("重力补偿关闭")
        if c.stopped:
            self.tracking.setText("已停止 / 释放力矩；数值为最后收到的反馈")
        self.badge.setText(("真机" if self.args.execute else "模拟") + " · " + c.state["mode"].upper())
        self.note.setText(c.note)

    def _render(self, now):
        if self.renderer is None:
            return
        try:
            result = self.renderer.poll()
            if result is not None:
                tag, pixels, error = result
                if error:
                    raise RuntimeError(error)
                self.view.show_pixels(pixels)
                self.view.setToolTip(tag)
            if now-self.last_render >= .08 and not self.renderer.busy:
                c = self.controller
                view_mode = self.view_mode.currentData()
                if c is None or not c.initialized:
                    q, tag = np.zeros(7), "未连接：仅显示 URDF 零位，非实际反馈"
                elif view_mode == "target":
                    q = c.target_q
                    tag = "目标预览" if c.preview_valid else "IK 待完成：上一有效目标预览"
                else:
                    q = c.state["q" if view_mode == "feedback" else "command"]
                    tag = "实际反馈姿态" if view_mode == "feedback" else "平滑命令姿态"
                    if c.stopped:
                        tag += "（已停止，最后一帧）"
                self.renderer.submit(tag, q, self.view.camera)
                self.last_render = now
                self.view_caption.setText(tag + " | 拖动旋转 · 滚轮缩放 | 橙点 TCP")
        except Exception as exc:
            self.renderer.close()
            self.renderer = None
            self.view.frame = None
            self.view.clear()
            self.view.setText("URDF 渲染不可用：" + str(exc))
            self.diagnostic.write("render_error", error=str(exc))

    def _tick(self):
        now = time.monotonic()
        if self.worker is not None:
            try:
                if now-self.last_ping >= .05:
                    self.worker.send("PING")
                    self.last_ping = now
                state = self.worker.poll()
                self.controller.update_state(state)
                if state is not None and state["seq"] != self.last_seq:
                    self.diagnostic.write("state", **state)
                    self.last_seq = state["seq"]
                self.controller.tick(now)
                self._refresh_controls()
            except Exception as exc:
                self.disconnect_control("驱动异常，已停止：" + str(exc))
        self._render(now)

    def closeEvent(self, event):
        if not self.closing:
            self.closing = True
            self.timer.stop()
            self.disconnect_control("窗口关闭")
            if self.renderer is not None:
                self.renderer.close()
                self.renderer = None
            self.diagnostic.write("gui_exit")
            self.diagnostic.close()
        event.accept()


def parse_args(argv=None):
    from gravity_model import add_gravity_args
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true", help="Enable hardware mode; click Connect in GUI")
    parser.add_argument("--port", default="/dev/ttyUSB3")
    for key, default in DEFAULTS.items():
        parser.add_argument("--"+key.replace("_", "-"), type=float, default=default)
    parser.add_argument("--tcp", nargs=6, type=float, default=[0]*6)
    parser.add_argument("--ready-q", nargs=7, type=float, default=DEFAULT_READY_Q.copy())
    parser.add_argument("--log", type=Path)
    add_gravity_args(parser)
    add_profile_argument(parser)
    args = parse_args_with_profile(parser, argv)
    try:
        validate(vars(args))
    except ValueError as exc:
        parser.error(str(exc))
    vector(args.tcp, 6)
    vector(args.ready_q, 7)
    return args


def main(argv=None):
    args = parse_args(argv)
    app = QApplication(sys.argv[:1])
    app.setStyle("Fusion")
    app.setFont(QFont("Noto Sans CJK SC", 10))
    app.setStyleSheet(STYLE)
    window = ControlWindow(args)
    signal.signal(signal.SIGINT, lambda *_: window.close())
    signal.signal(signal.SIGTERM, lambda *_: window.close())
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
