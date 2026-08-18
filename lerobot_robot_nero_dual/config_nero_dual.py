from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.robots.config import RobotConfig


@RobotConfig.register_subclass("nero_dual")
@dataclass
class NeroDualConfig(RobotConfig):
    """AgileX Nero 双臂（7-DoF ×2 + AgxGripper，pyAgxArm 后端）。

    与 piper_dual 同构：同一套 SDK、同一套单位（rad / m / N·m）、
    同一套高频插值下发。首次接线见 pyAgxArm docs/nero/first_time_user_guide_can.md。
    """

    left_port: str = "can_left"
    right_port: str = "can_right"
    firmware: str = "default"     # get_firmware() 可查；NeroFW 常量对照 SDK 文档
    can_interface: str = "socketcan"
    read_only: bool = False
    # move_js 为 MIT 直通、无平滑 —— 高频插值线程见 agx_dual_arm.py
    control_hz: int = 200
    policy_hz: int = 30
    gripper_max_m: float = 0.08   # 标定后按实际行程改（SDK 支持 0.07/0.1）
    gripper_force_n: float = 1.0
    cameras: dict[str, CameraConfig] = field(
        default_factory=lambda: {
            "wrist_left": OpenCVCameraConfig(
                index_or_path="/dev/video0", fps=30, width=480, height=640, rotation=-90,
            ),
            "wrist_right": OpenCVCameraConfig(
                index_or_path="/dev/video2", fps=30, width=480, height=640, rotation=90,
            ),
            "top": OpenCVCameraConfig(
                index_or_path="/dev/video4", fps=30, width=640, height=480,
            ),
        }
    )
