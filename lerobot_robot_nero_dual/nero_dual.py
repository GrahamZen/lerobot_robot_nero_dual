"""Nero 双臂 follower（pyAgxArm 后端）。

observation / action 的特征名与单位同旧实现完全一致：
    left_joint_1.pos … left_joint_7.pos, left_gripper.pos   (rad / m)
    left_joint_1.effort … left_gripper.effort               (N·m / N)
旧数据集与新数据集可直接混用。
"""

from __future__ import annotations

import logging
from functools import cached_property
from typing import Any

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.robots.robot import Robot
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from lerobot_robot_nero_dual.agx_dual_arm import AgxArmUnit, InterpolatedJointWriter
from lerobot_robot_nero_dual.config_nero_dual import NeroDualConfig

logger = logging.getLogger(__name__)

N_JOINTS = 7
SIDES = ("left", "right")


class NeroDual(Robot):
    config_class = NeroDualConfig
    name = "nero_dual"

    def __init__(self, config: NeroDualConfig):
        super().__init__(config)
        self.config = config
        self.units = {
            "left": self._make_unit(config.left_port),
            "right": self._make_unit(config.right_port),
        }
        self.cameras = make_cameras_from_configs(config.cameras)
        self._writer: InterpolatedJointWriter | None = None
        self._is_connected = False
        self._last_gripper: dict[str, float | None] = {s: None for s in SIDES}

    def _make_unit(self, channel: str) -> AgxArmUnit:
        return AgxArmUnit(
            channel=channel,
            model="nero",
            firmware=self.config.firmware,
            interface=self.config.can_interface,
            gripper_force_n=self.config.gripper_force_n,
        )

    # ------------------------------ 特征 ------------------------------- #

    @property
    def _motor_names(self) -> list[str]:
        return [f"joint_{i}" for i in range(1, N_JOINTS + 1)] + ["gripper"]

    @cached_property
    def action_features(self) -> dict[str, type]:
        return {f"{s}_{m}.pos": float for s in SIDES for m in self._motor_names}

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        feats: dict[str, type | tuple] = {}
        for s in SIDES:
            for m in self._motor_names:
                feats[f"{s}_{m}.pos"] = float
                feats[f"{s}_{m}.effort"] = float
        for cam_key, cam in self.cameras.items():
            feats[cam_key] = (cam.height, cam.width, 3)
        return feats

    # ------------------------------ 生命周期 --------------------------- #

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def configure(self, **kwargs) -> None:
        pass

    def connect(self, calibrate: bool = True) -> None:
        if self._is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")
        for unit in self.units.values():
            unit.connect(enable=not self.config.read_only)
        for cam in self.cameras.values():
            cam.connect()
        if self.config.control_hz > 0 and not self.config.read_only:
            self._writer = InterpolatedJointWriter(
                self.units, self.config.control_hz, self.config.policy_hz
            )
            self._writer.start()
        self._is_connected = True
        logger.info("%s connected (backend=pyAgxArm, model=nero)", self)

    def disconnect(self) -> None:
        if not self._is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        if self._writer is not None:
            self._writer.stop()
            self._writer = None
        for cam in self.cameras.values():
            cam.disconnect()
        for unit in self.units.values():
            unit.disconnect()
        self._is_connected = False

    def emergency_stop(self) -> None:
        if self._writer is not None:
            self._writer.stop()
            self._writer = None
        for unit in self.units.values():
            unit.emergency_stop()
        logger.warning("%s EMERGENCY STOP", self)

    # ------------------------------ 读写 ------------------------------- #

    def get_observation(self) -> dict[str, Any]:
        if not self._is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        obs: dict[str, Any] = {}
        for s in SIDES:
            unit = self.units[s]
            joints = unit.read_joints() or [0.0] * N_JOINTS
            efforts = unit.read_efforts()
            g_pos, g_force = unit.read_gripper()
            for i in range(N_JOINTS):
                obs[f"{s}_joint_{i + 1}.pos"] = float(joints[i])
                obs[f"{s}_joint_{i + 1}.effort"] = float(efforts[i])
            obs[f"{s}_gripper.pos"] = g_pos
            obs[f"{s}_gripper.effort"] = g_force
        for name, cam in self.cameras.items():
            obs[name] = cam.async_read()
        return obs

    def send_action(self, action: dict[str, float]) -> dict[str, float]:
        if not self._is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        if self.config.read_only:
            return action

        targets, grippers = {}, {}
        if "action" in action and "left_joint_1.pos" not in action:
            raw = list(action["action"])  # 扁平 16 维 [左 7+1, 右 7+1]
            if len(raw) != 2 * (N_JOINTS + 1):
                return action
            targets = {"left": raw[:N_JOINTS], "right": raw[N_JOINTS + 1 : 2 * N_JOINTS + 1]}
            grippers = {"left": raw[N_JOINTS], "right": raw[-1]}
        else:
            for s in SIDES:
                targets[s] = [action[f"{s}_joint_{i}.pos"] for i in range(1, N_JOINTS + 1)]
                grippers[s] = action[f"{s}_gripper.pos"]

        if self._writer is not None:
            self._writer.send_target(targets)
        else:
            for s in SIDES:
                self.units[s].write_joints_js(targets[s])
        for s in SIDES:  # 夹爪按策略频率直发，去抖：变化 <0.5mm 不重发
            g = min(max(float(grippers[s]), 0.0), self.config.gripper_max_m)
            if self._last_gripper[s] is None or abs(g - self._last_gripper[s]) > 5e-4:
                self.units[s].write_gripper(g)
                self._last_gripper[s] = g
        return action
