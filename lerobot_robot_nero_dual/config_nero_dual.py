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
    # "auto" = 连接时 get_firmware() 自动选档位（同官方 agx_arm_ros）。
    # 手动指定：default(≤1.10) / v111 / v112 / v120 / v121(≥1.21)，
    # 可用 `python -m lerobot_robot_nero_dual.probe` 查。
    firmware: str = "auto"
    can_interface: str = "socketcan"
    read_only: bool = False
    # 缺省 0 = send_action 直接写一拍（位置模式 move_j，控制器自带规划）。
    # MIT 模式（move_js）无平滑、风险极高且模式残留，不作默认路径。
    control_hz: int = 0
    policy_hz: int = 30
    speed_percent: int = 50      # 位置模式速度百分比
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
