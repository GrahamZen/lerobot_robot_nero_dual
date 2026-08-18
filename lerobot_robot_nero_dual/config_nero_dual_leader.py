from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("nero_dual_leader")
@dataclass
class NeroDualLeaderConfig(TeleoperatorConfig):
    """Nero 双臂主手（leader/示教模式）。"""

    left_port: str = "can_l_leader"
    right_port: str = "can_r_leader"
    firmware: str = "default"
    can_interface: str = "socketcan"
    set_leader_mode_on_connect: bool = True
