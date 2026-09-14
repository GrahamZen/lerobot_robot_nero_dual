"""只读探测 CAN 口上的 Nero：固件版本 → SDK 档位、有无关节反馈、有无主手控制帧。

不使能、不下发运动指令、不改主从模式；只发一帧固件查询。
用来确认 firmware 该填什么，以及哪个 CAN 口接的是哪条臂（边跑边手动掰一下某条臂）。

    python -m lerobot_robot_nero_dual.probe can0 can1 can2 can3
"""

from __future__ import annotations

import argparse
import time


def _fmt(msg) -> str:
    if msg is None:
        return "无"
    return f"{msg.hz:5.0f}Hz [{', '.join(f'{q:+.2f}' for q in msg.msg)}]"


def _socketcan_state(channel: str) -> str | None:
    """None = 正常 UP；否则返回问题描述。"""
    try:
        with open(f"/sys/class/net/{channel}/flags") as f:
            flags = int(f.read().strip(), 16)
    except FileNotFoundError:
        return "不存在（ip -br link show type can 看名字）"
    return None if flags & 0x1 else "DOWN（先按 README 3.2 以 1Mbps 拉起）"


def probe(channel: str, listen_s: float, interface: str) -> None:
    from pyAgxArm import AgxArmFactory, create_agx_arm_config, resolve_firmware_profile

    if interface == "socketcan" and (problem := _socketcan_state(channel)):
        print(f"{channel:14s} {problem}")
        return

    def make(firmware: str):
        cfg = create_agx_arm_config(
            robot="nero", firmeware_version=firmware, interface=interface, channel=channel
        )
        return AgxArmFactory.create_arm(cfg)

    # 1) 用 default 档位查固件版本（查询帧各档位通用）
    arm = make("default")
    try:
        arm.connect()
    except Exception as e:  # noqa: BLE001  口没拉起 / 不存在
        print(f"{channel:14s} 打不开: {e}")
        return
    fw = arm.get_firmware(timeout=1.0)
    arm.disconnect()
    version = fw["software_version"] if fw else None
    try:
        profile = resolve_firmware_profile("nero", version) if version else None
    except ValueError:
        profile = None

    # 2) 用解析出的档位监听（主手控制帧的 CAN ID 随档位变化）
    arm = make(profile or "default")
    arm.connect()
    time.sleep(listen_s)
    joints = arm.get_joint_angles()
    leader = arm.get_leader_joint_angles()
    arm.disconnect()

    print(f"{channel:14s} 固件 {version or '无应答'} → firmware={profile or '?'}")
    print(f"{'':14s}   关节反馈   {_fmt(joints)}")
    print(f"{'':14s}   主手控制帧 {_fmt(leader)}")
    if joints is not None and leader is not None:
        print(f"{'':14s}   ⚠ 同一总线上既有从臂反馈又有主手控制帧：这是硬件联动接法。"
              "软件遥操前必须把主手从这条 CAN 上断开，否则从臂会同时收主手和 PC 指令而失控。")


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("channels", nargs="+", help="CAN 口名，如 can0 can1 或 can_left")
    p.add_argument("--listen", type=float, default=1.0, help="每个口监听秒数")
    p.add_argument("--interface", default="socketcan")
    args = p.parse_args()
    for ch in args.channels:
        probe(ch, args.listen, args.interface)


if __name__ == "__main__":
    main()
