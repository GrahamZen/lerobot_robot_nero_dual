"""Nero 双臂主手 teleoperator（pyAgxArm leader 模式）。

get_action() 输出与 follower 的 action 特征名一致：
    left_joint_1.pos … left_gripper.pos（rad / m）

主手在 leader 模式下广播的是**控制帧**（旧 piper_sdk 的 GetArmJointCtrl /
GetArmGripperCtrl），不是普通关节反馈，所以读 get_leader_joint_angles() /
get_gripper_ctrl_states()，而不是 get_joint_angles() / get_gripper_status()。
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
        self._last: dict[str, list[float]] = {}

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
            unit.connect(enable=False, can_push=False)  # 主手不使能，人拖动
            if self.config.set_leader_mode_on_connect:
                unit.set_leader_mode()
        # 等主手控制帧：收不到就报错，绝不拿全零当动作发给从臂
        t0 = time.monotonic()
        while True:
            missing = [s for s in SIDES if self.units[s].read_leader_joints() is None]
            if not missing:
                break
            if time.monotonic() - t0 > self.config.connect_timeout_s:
                for unit in self.units.values():
                    unit.disconnect()
                raise TimeoutError(
                    f"{self}: {missing} 主手 {self.config.connect_timeout_s}s 内无控制帧。"
                    "检查 CAN 口是否对应主手、firmware 档位是否正确、主手是否处于 leader 模式"
                    "（python -m lerobot_robot_nero_dual.probe 可查）。"
                )
            time.sleep(0.02)
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
            joints = unit.read_leader_joints()
            if joints is None:  # 偶发丢帧：沿用上一拍
                joints = self._last[s]
            self._last[s] = joints
            for i in range(N_JOINTS):
                action[f"{s}_joint_{i + 1}.pos"] = float(joints[i])
            action[f"{s}_gripper.pos"] = unit.read_leader_gripper()
        logger.debug("%s read action: %.1fms", self, (time.perf_counter() - start) * 1e3)
        return action
