"""pyAgxArm 双臂控制核心（Piper / Nero 通用，由各插件包各带一份）。

SDK: github.com/agilexrobotics/pyAgxArm —— Piper 与 Nero 统一 API。
单位约定（与旧 piper_sdk 实现保持一致，数据集无缝衔接）：
    关节: rad · 夹爪: m（0~行程） · 力矩: N·m

连接流程对齐官方 ROS2 驱动 agx_arm_ros（agx_arm_ctrl_single_node._init_agx_arm / _enable_arm）：
    connect → firmware="auto" 时 get_firmware() + resolve_firmware_profile() 并按档位重建
    → 等低速反馈 → enable 循环 → 设速度。

控制路径（与官方 agx_arm_ros 默认 fast_mode=false 一致）：
    joints  → **move_j()：位置模式（MOVE J），控制器侧自带轨迹规划/平滑**，
              配 set_speed_percent(speed_percent)（旧实现 MotionCtrl_2(1, 1, 50, 0)
              = 位置模式 + 50% 速度 + mit_mode 关闭）。
              绝不默认用 move_js —— 那是 move_j + mit_mode，SDK 自己标注
              "无平滑、无轨迹规划、可能剧烈震荡、风险极高"，实测会让除末端外
              的关节持续抖动，且模式会残留在控制器上（位置模式工具随后失效）。
              需要 MIT 时显式调 write_joints_mit()，自行承担风险。
    gripper → move_gripper_m()，按策略频率直发（避免刷爆 CAN）。
"""

from __future__ import annotations

import logging
import threading
import time

logger = logging.getLogger(__name__)


class AgxArmUnit:
    """一条臂 + 一个 AgxGripper，占一个 CAN 通道。"""

    def __init__(
        self,
        channel: str,
        model: str = "piper",
        firmware: str = "auto",
        interface: str = "socketcan",
        gripper_force_n: float = 1.0,
        speed_percent: int = 50,
    ) -> None:
        self.channel = channel
        self.model = model
        self.interface = interface
        # "auto" = 连接时读固件自动选档位（官方 agx_arm_ros 的默认做法）；
        # 主从臂接同一个 CAN 模块时官方要求手动指定（两条臂会用相同 ID 回包）
        self.firmware = str(firmware).lower()
        self.gripper_force_n = gripper_force_n
        self.speed_percent = speed_percent
        self._build("default" if self.firmware == "auto" else self.firmware)
        self._connected = False

    def _build(self, firmware: str) -> None:
        from pyAgxArm import AgxArmFactory, create_agx_arm_config

        cfg = create_agx_arm_config(
            robot=self.model, firmeware_version=firmware, interface=self.interface,
            channel=self.channel,
        )
        self.arm = AgxArmFactory.create_arm(cfg)
        self.gripper = self.arm.init_effector(self.arm.OPTIONS.EFFECTOR.AGX_GRIPPER)
        self.n_joints: int = self.arm.joint_nums

    @property
    def needs_can_push(self) -> bool:
        """Nero 固件 ≤1.11 上电不推送 CAN 反馈，需 set_normal_mode() 打开（≥1.12 上电即推送）。"""
        return self.model == "nero" and self.firmware in ("default", "v111")

    # ---------------------------------------------------------------- #

    def connect(self, enable: bool = True, timeout_s: float = 5.0, can_push: bool = True) -> None:
        """can_push: 旧固件（≤1.11）上打开 CAN 反馈推送。主手传 False ——
        set_normal_mode() 会把主从配置清成普通模式。"""
        self.arm.connect()
        if self.firmware == "auto":
            self._detect_firmware(timeout_s)
        if can_push and self.needs_can_push:
            self.arm.set_normal_mode()
        if enable:
            self._wait_low_speed_feedback(timeout_s * 6)
            t0 = time.monotonic()
            while not self.arm.enable():
                if can_push and self.needs_can_push:  # 官方首次使用指南"方法二"
                    self.arm.set_normal_mode()
                if time.monotonic() - t0 > timeout_s:
                    raise TimeoutError(f"[{self.channel}] enable timeout ({timeout_s}s)")
                time.sleep(0.01)
            # 位置模式 + 限速：也把上一次进程可能残留在控制器上的 MIT 模式清掉
            self.set_position_mode()
        self._connected = True

    def _detect_firmware(self, timeout_s: float) -> None:
        """同 agx_arm_ros _init_agx_arm：get_firmware → resolve_firmware_profile → 非 default 则按档位重建。"""
        from pyAgxArm import resolve_firmware_profile

        fw = None
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout_s:
            fw = self.arm.get_firmware()
            if fw:
                break
            time.sleep(0.005)
        if not fw:
            raise TimeoutError(
                f"[{self.channel}] get_firmware 无应答 ({timeout_s}s)：臂没上电 / CAN 口不对 / bitrate 不是 1M"
            )
        self.firmware = resolve_firmware_profile(self.model, fw["software_version"])
        logger.info("[%s] 固件 %s → firmware=%s", self.channel, fw["software_version"], self.firmware)
        if self.firmware != "default":
            self.arm.disconnect()
            self._build(self.firmware)
            self.arm.connect()

    def _wait_low_speed_feedback(self, timeout_s: float) -> None:
        """同 agx_arm_ros _enable_arm：使能前等关节 1 低速反馈到达，超时只告警。"""
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout_s:
            st = self.arm.get_driver_states(1)
            if st is not None and getattr(st, "hz", 0) > 0:
                return
            time.sleep(0.2)
        logger.warning("[%s] 使能前等低速反馈超时 (%.0fs)，继续尝试使能", self.channel, timeout_s)

    def set_position_mode(self) -> None:
        """强制位置模式（MOVE J）+ 限速。MIT 模式会残留在控制器上，必须显式复位。"""
        self.arm.set_speed_percent(int(self.speed_percent))
        self.arm.set_motion_mode(self.arm.OPTIONS.MOTION_MODE.J)
        logger.info("[%s] 位置模式 (MOVE J), 速度 %d%%", self.channel, self.speed_percent)

    def set_leader_mode(self) -> None:
        # 旧固件按官方 demo（PyAgxArm_demo/nero/nero_mode_set_leader.py）：先回普通模式再切主臂；
        # ≥1.12 set_normal_mode 是 no-op，直接切
        if self.needs_can_push:
            self.arm.set_normal_mode()
            time.sleep(0.2)
        self.arm.set_leader_mode()

    def read_leader_joints(self) -> list[float] | None:
        """主手控制帧（官方 demo: nero_read_leader_joint_angles.py）。"""
        msg = self.arm.get_leader_joint_angles()
        return None if msg is None else list(msg.msg)

    def read_leader_gripper(self) -> float:
        """主手夹爪控制帧宽度 m（官方 demo: nero_gripper_read_ctrl_states.py）；无帧时 0。"""
        gcs = self.gripper.get_gripper_ctrl_states()
        return 0.0 if gcs is None else float(gcs.msg.value)

    def disconnect(self, disable: bool = False) -> None:
        if disable:
            try:
                self.arm.disable()
            except Exception:  # noqa: BLE001  断连收尾不抛
                logger.warning("[%s] disable failed during disconnect", self.channel)
        self.arm.disconnect()
        self._connected = False

    @property
    def is_connected(self) -> bool:
        return self._connected and self.arm.is_ok()

    # ------------------------------ 读 -------------------------------- #

    def read_joints(self) -> list[float] | None:
        msg = self.arm.get_joint_angles()
        return None if msg is None else list(msg.msg)

    def read_efforts(self) -> list[float]:
        out = []
        for i in range(1, self.n_joints + 1):
            st = self.arm.get_motor_states(i)
            out.append(float(st.msg.torque) if st is not None else 0.0)
        return out

    def read_gripper(self) -> tuple[float, float]:
        """-> (宽度 m, 夹持力 N)；未收到反馈时返回 (0, 0)。"""
        gs = self.gripper.get_gripper_status()
        if gs is None:
            return 0.0, 0.0
        return float(gs.msg.value), float(gs.msg.force)

    # ------------------------------ 写 -------------------------------- #

    def write_joints(self, joints: list[float]) -> None:
        """位置模式下发（默认路径）。控制器自带规划，30Hz 直发即可平滑。"""
        self.arm.move_j(list(joints))

    def write_joints_mit(self, joints: list[float]) -> None:
        """MIT 直通下发 —— 无平滑、风险极高，且模式会残留。仅在明确需要时使用。"""
        self.arm.move_js(list(joints))

    def write_gripper(self, width_m: float) -> None:
        self.gripper.move_gripper_m(max(0.0, float(width_m)), self.gripper_force_n)

    def emergency_stop(self) -> None:
        self.arm.electronic_emergency_stop()


class InterpolatedJointWriter:
    """高频插值下发线程（两臂共用一个线程，保持左右同步）。

    send_target() 以策略频率被调；线程以 control_hz 把上一次真正下发的
    关节值向最新目标线性插值（插值窗口 = 1/policy_hz），逐拍位置模式下发。
    """

    def __init__(
        self,
        units: dict[str, AgxArmUnit],
        control_hz: float,
        policy_hz: float,
    ) -> None:
        self.units = units
        self.control_hz = control_hz
        self.policy_hz = policy_hz
        self._lock = threading.Lock()
        self._target: dict[str, list[float]] = {}
        self._base: dict[str, list[float]] = {}
        self._written: dict[str, list[float]] = {}
        self._t_set = 0.0
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True, name="agx-interp")
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

    def send_target(self, targets: dict[str, list[float]]) -> None:
        with self._lock:
            for side, q in targets.items():
                # 插值起点 = 上一拍实际下发值（绝不回退）
                self._base[side] = list(self._written.get(side, self._target.get(side, q)))
                self._target[side] = list(q)
            self._t_set = time.monotonic()

    def _loop(self) -> None:
        period = 1.0 / self.control_hz
        window = 1.0 / self.policy_hz
        while not self._stop.is_set():
            t0 = time.monotonic()
            with self._lock:
                if self._target:
                    alpha = min(1.0, (t0 - self._t_set) / window)
                    cmds = {}
                    for side, tgt in self._target.items():
                        base = self._base.get(side, tgt)
                        q = [b + alpha * (t - b) for b, t in zip(base, tgt)]
                        cmds[side] = q
                        self._written[side] = q
                else:
                    cmds = {}
            for side, q in cmds.items():
                try:
                    self.units[side].write_joints(q)
                except Exception:
                    logger.exception("interp write failed (%s)", side)
            dt = time.monotonic() - t0
            if (sleep := period - dt) > 0:
                time.sleep(sleep)
