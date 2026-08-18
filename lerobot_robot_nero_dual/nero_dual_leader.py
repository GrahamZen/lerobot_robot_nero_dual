"""Nero 双臂主手 teleoperator（pyAgxArm leader 模式）。

get_action() 输出与 follower 的 action 特征名一致：
    left_joint_1.pos … left_gripper.pos（rad / m）
"""

from __future__ import annotations

import logging
import time

from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from lerobot_robot_nero_dual.agx_dual_arm import AgxArmUnit
from lerobot_robot_nero_dual.config_nero_dual_leader import NeroDualLeaderConfig

logger = logging.getLogger(__name__)

N_JOINTS = 7
SIDES = ("left", "right")


class NeroDualLeader(Teleoperator):
    config_class = NeroDualLeaderConfig
    name = "nero_dual_leader"

    def __init__(self, config: NeroDualLeaderConfig):
        super().__init__(config)
        self.config = config
        self.units = {
            "left": AgxArmUnit(config.left_port, "nero", config.firmware, config.can_interface),
            "right": AgxArmUnit(config.right_port, "nero", config.firmware, config.can_interface),
        }
        self._is_connected = False

    @property
    def action_features(self) -> dict[str, type]:
        names = [f"joint_{i}" for i in range(1, N_JOINTS + 1)] + ["gripper"]
        return {f"{s}_{m}.pos": float for s in SIDES for m in names}

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

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

    def send_feedback(self, *args, **kwargs) -> None:
        pass

    def connect(self, calibrate: bool = True) -> None:
        if self._is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")
        for unit in self.units.values():
            unit.connect(enable=False)  # 主手不使能，人拖动
            if self.config.set_leader_mode_on_connect:
                unit.set_leader_mode()
        self._is_connected = True

    def disconnect(self) -> None:
        if not self._is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        for unit in self.units.values():
            unit.disconnect()
        self._is_connected = False

    def get_action(self) -> dict[str, float]:
        if not self._is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        start = time.perf_counter()
        action: dict[str, float] = {}
        for s in SIDES:
            unit = self.units[s]
            joints = unit.read_joints() or [0.0] * N_JOINTS
            g_pos, _ = unit.read_gripper()
            for i in range(N_JOINTS):
                action[f"{s}_joint_{i + 1}.pos"] = float(joints[i])
            action[f"{s}_gripper.pos"] = g_pos
        logger.debug("%s read action: %.1fms", self, (time.perf_counter() - start) * 1e3)
        return action
