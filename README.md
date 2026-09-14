# lerobot_robot_nero_dual

AgileX **Nero 双臂**（7-DoF ×2 + AgxGripper）的 LeRobot 插件，底层是 AgileX 官方统一 SDK
[pyAgxArm](https://github.com/agilexrobotics/pyAgxArm)。提供两个设备：

| 类型 | `--robot.type` / `--teleop.type` | 作用 |
|---|---|---|
| 从臂（follower） | `nero_dual` | 读关节/力矩/夹爪/相机，执行动作 |
| 主手（leader） | `nero_dual_leader` | 人拖动主手，输出动作给从臂 |

特征名与单位（与 `piper_dual` 同构，只是 7 关节）：

```
observation: left_joint_1.pos … left_joint_7.pos, left_gripper.pos      # rad / m
             left_joint_1.effort … left_gripper.effort                   # N·m / N
             right_…（同上）, 以及每个相机一路图像
action:      left_joint_1.pos … left_gripper.pos, right_…               # 共 16 维
```

> **状态（2026-09-14）**：接口已对照 pyAgxArm `e7aef17` 核对，并用 pyAgxArm 自带的虚拟 Nero 从机
> （`tests/slaves/nero_can_slave.py`）跑通了从臂/主手/probe；**还没在 Nero 真机上跑过**。
> 第一次上机请按下面"首次上机"一步步来，先低速、先不接相机。

---

## 1. 软件遥操（soft teleop）是怎么做的

AgileX 官方有两种主从遥操：

| | 硬件联动（官方默认套件接法） | **软件遥操（本插件）** |
|---|---|---|
| 接线 | 主手和从臂接**同一条** CAN | **每条臂各接一个 USB-CAN**，主手和从臂的 CAN **不相连** |
| 谁让从臂跟随 | 从臂固件直接执行主手发出的控制帧 | PC 读主手 → PC 给从臂下指令 |
| 能否录数据 / 跑策略 | 只能旁听 | 可以，LeRobot 的 teleop / record / 策略部署都走这条 |

软件遥操按官方 ROS2 驱动 [agx_arm_ros](https://github.com/agilexrobotics/agx_arm_ros) 的做法实现
（主手节点发布 `/feedback/leader_joint_states` → 从臂节点订阅 `/control/joint_states`）：

- **主手**：连接时设为 leader 模式（`set_leader_mode()`，旧固件先 `set_normal_mode()` 再切，同官方
  `PyAgxArm_demo/nero/nero_mode_set_leader.py`），不使能；读主手控制帧
  `get_leader_joint_angles()` + 夹爪控制帧 `get_gripper_ctrl_states()`。
- **从臂**：使能后用位置模式 `move_j()`（= agx_arm_ros 默认 `fast_mode=false`）+ `move_gripper_m(width, force)`。
- **固件档位**默认 `auto`：连接时 `get_firmware()` → `resolve_firmware_profile()` 自动选（同 agx_arm_ros）。

> ⚠ 官方文档（piper_sdk `asserts/double_piper.MD`）：**PC 控制从臂时主手必须和从臂的 CAN 断开**，
> 否则从臂同时收主手和 PC 的指令会失控。如果你的 Nero 是按硬件联动接好的，先把主手的 CAN 线从从臂上拔下，
> 改成每条臂一个 USB-CAN。`probe`（见 3.3）能检测出同一总线上同时有主手和从臂的情况。

---

## 2. 安装

需要 Linux + SocketCAN，Python ≥ 3.10。pyAgxArm 不在 PyPI，依赖里直接从 GitHub 装（已固定到核对过的提交）。

```bash
mkdir -p ~/workspace/lerobot-plugins && cd ~/workspace/lerobot-plugins
git clone https://github.com/GrahamZen/lerobot_robot_nero_dual
cd lerobot_robot_nero_dual

uv venv --python 3.12
source .venv/bin/activate
uv pip install -e . "lerobot[dataset]>=0.6.1,<0.7"
```

验证：

```bash
python -c "import lerobot_robot_nero_dual, pyAgxArm; print('ok', pyAgxArm.__version__)"
```

LeRobot 会自动导入所有以 `lerobot_robot_` 开头的已安装包，`lerobot-teleoperate` / `lerobot-record`
里直接写 `--robot.type=nero_dual` 即可。

在别的项目里用：把 `lerobot_robot_nero_dual` 加进依赖，uv 源写
`lerobot_robot_nero_dual = { git = "https://github.com/GrahamZen/lerobot_robot_nero_dual" }`。
注意 pyAgxArm 需要 2026-09 之后的版本才有 `v121`（固件 ≥1.21）驱动，旧锁文件要
`uv lock --upgrade-package pyagxarm`。

---

## 3. 首次上机

### 3.1 系统依赖（一次性，需要 sudo）

```bash
sudo apt update && sudo apt install -y can-utils ethtool
```

### 3.2 拉起 CAN 口

四条臂（2 主手 + 2 从臂）各接一个 USB-CAN，插上后系统里是 `can0`…`can3`，**默认是 DOWN**，
需要以 1 Mbps 拉起（官方 `docs/can_user.md`；需要 sudo，拔插/重启后要重做）：

```bash
for i in $(ip -br link show type can | awk '{print $1}'); do
  sudo ip link set "$i" down
  sudo ip link set "$i" type can bitrate 1000000
  sudo ip link set "$i" up
done
ip -br link show type can        # 应全部是 UP
```

### 3.3 探测：固件 + 哪个口是哪条臂

机械臂上电，**等电源处指示灯变绿**（官方：Nero 需等绿灯）后：

```bash
python -m lerobot_robot_nero_dual.probe can0 can1 can2 can3
```

输出示例：

```
can0           固件 1.21 → firmware=v121
                 关节反馈   200Hz [+0.01, -0.35, ...]
                 主手控制帧 无
```

- **分主从**：主手处于 leader 模式时"主手控制帧"一行有数据；从臂只有"关节反馈"。
  （主手出厂可能还不是 leader 模式，那它看起来和从臂一样——插件连接时会把它设成 leader 模式。）
- **分左右**：跑的时候用手掰一下某条臂，看哪个口的数值在变。
- 同一口两行都有数据会打印 ⚠：那是硬件联动接法，见第 1 节。
- probe 是只读的：只发一帧固件查询，不使能、不发运动指令、不改主从模式。

### 3.4 固定口名（推荐）

`can0`…`can3` 的编号随插入顺序变，建议按 USB 物理口重命名成插件的默认名。先看每个模块的 USB 口：

```bash
for i in $(ip -br link show type can | awk '{print $1}'); do
  echo "$i $(ethtool -i "$i" | grep bus-info)"
done
```

再用 pyAgxArm 自带脚本按 USB 口激活并命名（脚本随 pyAgxArm 装进了 venv）：

```bash
S=$(python -c "import pyAgxArm,os;print(os.path.dirname(pyAgxArm.__file__))")/scripts/ubuntu
sudo bash $S/can_activate.sh can_left     1000000 "<左从臂的 bus-info>"
sudo bash $S/can_activate.sh can_right    1000000 "<右从臂的 bus-info>"
sudo bash $S/can_activate.sh can_l_leader 1000000 "<左主手的 bus-info>"
sudo bash $S/can_activate.sh can_r_leader 1000000 "<右主手的 bus-info>"
```

只要 USB 线不换口，每次开机/拔插后重跑这四行，名字就不变。口名最长 15 个字符。

**droplab2 现状**（2026-09-14：2 主手 + 2 从臂，4 个 USB-CAN 已插上，还没分配左右/主从）：

| 当前名 | bus-info | 模块序列号 |
|---|---|---|
| can0 | `1-4.2.1:1.0` | 0047003F4648571220363032 |
| can1 | `1-4.2.2:1.0` | 0027002B4148570B20343133 |
| can2 | `1-4.4.3:1.0` | 003000414148570D20343133 |
| can3 | `1-4.4.4:1.0` | 002300254148571420343133 |

---

## 4. 使用

下面默认口名已按 3.4 改好；固件自动识别，不用填。

### 4.1 软件遥操（先低速、不接相机）

```bash
lerobot-teleoperate \
  --robot.type=nero_dual \
  --robot.left_port=can_left --robot.right_port=can_right \
  --robot.speed_percent=20 \
  --robot.cameras='{}' \
  --teleop.type=nero_dual_leader \
  --teleop.left_port=can_l_leader --teleop.right_port=can_r_leader \
  --fps=30
```

开始前把主手摆到和从臂差不多的姿态：一连上，从臂就会以 `speed_percent` 的速度走到主手当前位置。

### 4.2 录数据

```bash
lerobot-record \
  --robot.type=nero_dual \
  --robot.left_port=can_left --robot.right_port=can_right \
  --teleop.type=nero_dual_leader \
  --teleop.left_port=can_l_leader --teleop.right_port=can_r_leader \
  --dataset.repo_id=local/nero_test \
  --dataset.single_task="pick up the cup" \
  --dataset.num_episodes=5 --dataset.fps=30 \
  --dataset.push_to_hub=false \
  --display_data=true
```

相机默认是 `wrist_left=/dev/video0`、`wrist_right=/dev/video2`、`top=/dev/video4`，
先用 `lerobot-find-cameras opencv` 确认编号，再覆盖，例如：

```bash
--robot.cameras='{top: {type: opencv, index_or_path: /dev/video4, width: 640, height: 480, fps: 30}}'
```

### 4.3 Python 里直接用（部署策略）

```python
from lerobot_robot_nero_dual import NeroDual, NeroDualConfig

robot = NeroDual(NeroDualConfig(
    left_port="can_left", right_port="can_right", speed_percent=30, cameras={},
))
robot.connect()
try:
    obs = robot.get_observation()          # dict，键见上面的特征名
    # 两种动作格式都接受：
    robot.send_action({k: obs[k] for k in robot.action_features})   # 按特征名
    # robot.send_action({"action": flat16})  # 扁平 16 维 [左 7 关节+夹爪, 右 7 关节+夹爪]
finally:
    robot.disconnect()
```

---

## 5. 配置项

`NeroDualConfig`（`--robot.*`）：

| 字段 | 默认 | 说明 |
|---|---|---|
| `left_port` / `right_port` | `can_left` / `can_right` | 从臂 CAN 口名 |
| `firmware` | `auto` | 自动识别；也可手动填 `default`(≤1.10) / `v111` / `v112` / `v120` / `v121`(≥1.21) |
| `can_interface` | `socketcan` | Linux 下不用改 |
| `read_only` | `False` | `True` = 不使能、只读，`send_action` 不下发 |
| `speed_percent` | `50` | 位置模式速度百分比，首次上机建议 20 |
| `control_hz` | `0` | 0 = 每次 `send_action` 直发一拍（控制器自带规划）；>0 开插值线程 |
| `policy_hz` | `30` | 插值窗口，仅 `control_hz>0` 时用 |
| `gripper_max_m` | `0.08` | 夹爪行程上限（m），动作会被裁到 `[0, gripper_max_m]` |
| `gripper_force_n` | `1.0` | 夹爪力（同 agx_arm_ros `gripper_default_effort`） |
| `cameras` | 3 路 OpenCV | 见 4.2 |

`NeroDualLeaderConfig`（`--teleop.*`）：

| 字段 | 默认 | 说明 |
|---|---|---|
| `left_port` / `right_port` | `can_l_leader` / `can_r_leader` | 主手 CAN 口名 |
| `firmware` | `auto` | 同上 |
| `set_leader_mode_on_connect` | `True` | 连接时把臂设成 leader（零力拖动）模式 |
| `connect_timeout_s` | `3.0` | 连接后等主手控制帧的超时 |

---

## 6. 安全要点

- **默认走位置模式（`move_j`）**：控制器自带轨迹规划；连接/使能时强制切回位置模式并设速度。
  **不默认用 MIT（`move_js`）**——SDK 标注"无平滑、可能剧烈震荡"，且模式会残留在控制器上。
- **主手收不到控制帧就报错**，绝不会把全零当动作发给从臂；偶发丢帧时沿用上一拍。
- 固件 ≤ 1.11 上电不推送 CAN 反馈，从臂连接时插件会调 `set_normal_mode()` 打开（官方首次使用指南方法二）。
- 夹爪指令变化 < 0.5 mm 不重发，避免刷爆 CAN。
- 急停：`robot.emergency_stop()`（电子急停）；物理急停按钮始终优先。

## 7. 排查

| 现象 | 可能原因 |
|---|---|
| `get_firmware 无应答` | 臂没上电/没绿灯；CAN 口没 UP；bitrate 不是 1 Mbps |
| `enable timeout` | 同上；或手动填的 `firmware` 与真实固件不符 |
| 主手 `无控制帧` 超时 | 口接的不是主手；主手不在 leader 模式 |
| 从臂不动 / 乱动 | 主手 CAN 还连在从臂上（第 1 节）；从臂之前被设成了主手（跑官方 `nero_mode_set_follower.py`） |
| probe 显示 `打不开` | 口没拉起（3.2）或名字不对（`ip -br link show type can`） |
| 自己看总线 | `candump can_left`（需要 can-utils），上电后应有持续数据 |

官方资料（本插件按这些实现）：

- [pyAgxArm](https://github.com/agilexrobotics/pyAgxArm)：`docs/nero/first_time_user_guide_can.md`（首次使用）、
  `docs/can_user.md`（CAN 模块）、`docs/nero/nero_api.md`（API）、`docs/nero/firmware_reference.md`（各固件差异）
- [PyAgxArm_demo](https://github.com/agilexrobotics/PyAgxArm_demo) `nero/`：单功能脚本，排查时可直接跑，例如
  `nero_read_firmware.py`、`nero_read_joint_angles.py`、`nero_read_leader_joint_angles.py`、
  `nero_mode_set_leader.py`、`nero_mode_set_follower.py`
- [agx_arm_ros](https://github.com/agilexrobotics/agx_arm_ros)：ROS2 驱动（软件遥操的参考实现、主从接线注意事项）
- [piper_sdk `asserts/double_piper.MD`](https://github.com/agilexrobotics/piper_sdk/blob/master/asserts/double_piper.MD)：
  主从臂原理（主手控制帧 ID、PC 控从臂时须断开主手）
