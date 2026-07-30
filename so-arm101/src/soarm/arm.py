"""机械臂连接与安全运动辅助。

所有运动一律通过 smooth_goto() 平滑插值执行——直接 send_action 跳变目标
会让舵机全速运动，禁止在其他代码里绕过本模块直发大幅目标。
"""

import json
import math
import os
import time
from pathlib import Path

TRACE = bool(os.environ.get("SOARM_TRACE"))  # 运动指令飞行记录(调试用)
_T0 = time.time()


def _trace(msg: str) -> None:
    if TRACE:
        print(f"    [t+{time.time()-_T0:6.1f}s] {msg}", flush=True)

from lerobot.robots.so_follower import SOFollower
from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig

PORT = "/dev/cu.usbmodem5AE70447161"
ARM_ID = "main_arm"

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

MAX_DEG_PER_S = 40.0  # 任何关节的角速度上限；duration 不足时自动拉长


POSES_FILE = Path(__file__).parent.parent.parent / "config" / "poses.json"


def load_pose(name: str) -> dict[str, float]:
    return json.loads(POSES_FILE.read_text())[name]


def _pin_goals_to_present() -> None:
    """使能扭矩前，把每个舵机 RAM 里的目标位置改写成当前位置。

    舵机会记住上次会话的目标；不清掉的话，lerobot 使能扭矩的瞬间
    臂会朝过期目标猛冲（上电砸桌面的根源）。
    """
    from lerobot.motors import Motor, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus

    motors = {
        name: Motor(id=i + 1, model="sts3215", norm_mode=MotorNormMode.RANGE_M100_100)
        for i, name in enumerate(JOINTS)
    }
    bus = FeetechMotorsBus(port=PORT, motors=motors)
    bus.connect(handshake=False)
    for name in motors:
        pos = bus.read("Present_Position", name, normalize=False)
        bus.write("Goal_Position", name, pos, normalize=False)
    bus.disconnect(disable_torque=False)


GRIPPER_TORQUE_LIMIT = 300  # 千分比。夹硬物(乐高)时防堵转过载；泡沫等软物可适当调高


def connect(calibrate: bool = False) -> SOFollower:
    """连接（带防冲击预处理 + 夹爪限扭）。"""
    _pin_goals_to_present()
    cfg = SOFollowerRobotConfig(port=PORT, cameras={}, id=ARM_ID)
    robot = SOFollower(cfg)
    robot.connect(calibrate=calibrate)
    robot.bus.write("Torque_Limit", "gripper", GRIPPER_TORQUE_LIMIT, normalize=False)
    return robot


def goto_exact(
    robot: SOFollower,
    target: dict[str, float],
    duration: float = 2.5,
    iters: int = 2,
    tol: float = 1.0,
) -> dict[str, float]:
    """平滑到达 target 后闭环修正重力垂度：读实际角，反向叠加差值重发。

    断电手扶采的样本在通电执行时会因稳态垂度而系统性偏低，此函数用
    舵机编码器消除该偏差。返回最终实际关节角。
    """
    import time as _t

    smooth_goto(robot, target, duration=duration)
    goal = dict(target)
    for _ in range(iters):
        _t.sleep(0.4)
        actual = read_joints(robot)
        errs = {j: target[j] - actual[j] for j in target}
        if max(abs(e) for e in errs.values()) <= tol:
            break
        goal = {j: goal[j] + errs[j] for j in target}
        smooth_goto(robot, goal, duration=0.8)
    return read_joints(robot)


def grip_load(robot: SOFollower) -> int:
    """夹爪负载幅值(0-1023)。读取异常按高负载处理。"""
    try:
        raw = int(robot.bus.read("Present_Load", "gripper", normalize=False))
        return raw & 0x3FF  # 低10位=幅值, bit10=方向
    except RuntimeError:
        return 1023


def grip_close(
    robot: SOFollower,
    load_thresh: int = 220,
    hold_min_opening: float = 4.0,
    hold_max_opening: float = 16.0,
    floor: float = 0.5,
    step: float = 1.2,
    hz: float = 18.0,
) -> tuple[bool, float, int]:
    """接触即停合爪: 逐步收拢, 负载超阈立即停在接触位并回让0.8锁定。

    舵机不持续堵转(不亮红灯不发热)。空夹会一路收到 floor。
    返回 (是否夹到东西, 接触时开合度, 触发负载)。
    """
    g = read_joints(robot)["gripper"]
    while g > floor:
        g = max(g - step, floor)
        robot.send_action({"gripper.pos": g})
        time.sleep(1.0 / hz)
        mag = grip_load(robot)
        if mag >= load_thresh:
            opening = read_joints(robot)["gripper"]
            # 向内再压1.2°咬住(扭矩上限已封顶力度)。曾用"+0.8回让"轻扶持,
            # 硬质物体运输途中会滑脱(实测), 勿改回。
            robot.send_action({"gripper.pos": opening - 1.2})
            return hold_min_opening <= opening <= hold_max_opening, opening, mag
    # 指令走完未触发负载阈值(限扭下负载读数可能到不了阈值)——以物理开合度定论:
    # 空夹会闭到 floor 附近; 被物体撑住则停在明显更开的位置 = 实际已夹住
    time.sleep(0.25)
    opening = read_joints(robot)["gripper"]
    held = hold_min_opening <= opening <= hold_max_opening
    if held:
        robot.send_action({"gripper.pos": opening - 1.2})  # 同样咬住
    return held, opening, grip_load(robot)


def grip_check(robot: SOFollower, grip_cmd: float, min_gap: float = 2.5) -> tuple[bool, float, int]:
    """合爪后判断是否夹住了东西。

    原理: 空夹时夹爪能收到指令位置附近; 夹住物体时被撑住, 停在明显更开的位置。
    返回 (是否夹住, 实际开合度, 原始负载; 读取异常时负载为-1)。
    过载错误不抛出——夹爪已限扭, 短暂过载标志按"位置差"正常判定即可。
    """
    try:
        g = read_joints(robot)["gripper"]
    except RuntimeError:
        time.sleep(0.3)
        g = read_joints(robot)["gripper"]
    try:
        load = int(robot.bus.read("Present_Load", "gripper", normalize=False))
    except RuntimeError:
        load = -1
    return (g - grip_cmd) >= min_gap, g, load


def joint_load(robot: SOFollower, joint: str) -> int:
    """任意关节负载幅值(0-1023), 读取异常返回-1。"""
    try:
        return int(robot.bus.read("Present_Load", joint, normalize=False)) & 0x3FF
    except RuntimeError:
        return -1


def descend_to_support(robot: SOFollower, start_sl: float, max_extra: float = 4.0,
                       step: float = 0.5) -> float:
    """承重感知缓降: 从 start_sl 逐步下降, 肩负载显著下降(物体触面承重)即停。

    到达 max_extra 上限也停(硬兜底保证足够低)。返回最终肩角。"""
    baseline: list[int] = []
    sl = start_sl
    while sl < start_sl + max_extra:
        sl += step
        smooth_goto(robot, {"shoulder_lift": sl}, duration=0.3)
        time.sleep(0.15)
        lo = joint_load(robot, "shoulder_lift")
        if len(baseline) >= 2:
            import statistics
            base = statistics.median(baseline)
            if lo >= 0 and base > 60 and lo < base * 0.55:
                break  # 负载骤降=已触面承重
        if lo >= 0:
            baseline.append(lo)
    return sl


def transport_to_drop(robot: SOFollower) -> None:
    """抓取后安全运输到回收位：垂直提肩 -> 高位收肘 -> 转体 -> 降落。

    顺序不可并线（实测）：低位收肘/直接插值都会让指尖刮桌。
    全程不发夹爪指令，保持夹压（夹爪目标由合爪时设定，动它就会泄压掉球）。
    """
    drop = load_pose("drop")
    cur = read_joints(robot)
    # 符号约定: shoulder_lift 增大=前倾伏低, 减小=抬起(观察位-85, 趴桌抓取0~+50)
    # 极简三步(用户定稿): 提离桌面 -> 转体 -> 整体平滑滑入罐口悬停位。
    # 投放位=罐口上方悬停(04示教, 肩38.9/肘-26.7), 终点高、肘展开量小,
    # 整体插值中段不会低于罐口高度。松爪由调用方负责。
    _trace("运输1: 提离")
    smooth_goto(robot, {"shoulder_lift": max(cur["shoulder_lift"] - 20.0, -25.0)}, duration=1.2)
    _trace("运输2: 转体")
    smooth_goto(robot, {"shoulder_pan": drop["shoulder_pan"]}, duration=2.0)
    _trace("运输3: 平滑滑入罐口悬停位")
    smooth_goto(robot, {j: drop[j] for j in
                        ("shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")},
                duration=3.5)


def descend_until_touch(robot: SOFollower, start_sl: float, max_extra: float = 18.0,
                        step: float = 0.6, wf_of=None) -> tuple[float, bool]:
    """空爪触觉找桌面: 逐步降肩, 肩负载骤降(指尖触面, 支撑面开始分担臂重)即停。

    返回 (触面肩角, 是否触到)。wf_of(sl) 给出每步应设的腕俯仰(垂直姿态补偿), 可为 None。
    这是深度控制的最终事实来源——不依赖任何映射/偏置算术链(那条链撞桌撞出来的教训)。"""
    import statistics
    baseline: list[int] = []
    sl = start_sl
    while sl < start_sl + max_extra:
        sl += step
        cmd = {"shoulder_lift": sl}
        if wf_of is not None:
            cmd["wrist_flex"] = wf_of(sl)
        smooth_goto(robot, cmd, duration=0.3)
        time.sleep(0.12)
        lo = joint_load(robot, "shoulder_lift")
        if len(baseline) >= 2:
            base = statistics.median(baseline)
            if lo >= 0 and base > 60 and lo < base * 0.55:
                return sl, True  # 负载骤降=指尖已触面
        if lo >= 0:
            baseline.append(lo)
    return sl, False


def hold_current(robot: SOFollower) -> None:
    """示教(扭矩释放)后安全接管：目标钉到当前位置再使能扭矩，臂原地保持。"""
    for name in JOINTS:
        pos = robot.bus.read("Present_Position", name, normalize=False)
        robot.bus.write("Goal_Position", name, pos, normalize=False)
    robot.bus.enable_torque()


def shutdown(robot: SOFollower, go_home: bool = True) -> None:
    """标准收尾：回 home 再断开（断开时扭矩自动释放）。

    home 的腕俯仰(45°)本身足够高、指尖不会先触桌，直接整体过渡即可；
    切勿额外抬腕分段——深折叠区抬腕会让夹爪/相机支架撞大臂（实测）。
    """
    if go_home:
        smooth_goto(robot, load_pose("home"), duration=3.5)
    robot.disconnect()


def read_joints(robot: SOFollower) -> dict[str, float]:
    obs = robot.get_observation()
    return {j: obs[f"{j}.pos"] for j in JOINTS}


def smooth_goto(
    robot: SOFollower,
    target: dict[str, float],
    duration: float = 2.0,
    hz: int = 50,
) -> None:
    """从当前位姿平滑过渡到 target。

    只控制 target 中显式给出的关节——未提及的关节保持原有舵机目标不变。
    （若把所有关节都重设为当前实际位置，会把夹爪的持续夹压泄掉导致掉球）
    余弦缓入缓出；若 duration 会使任一关节超过 MAX_DEG_PER_S，自动拉长时间。

    插值起点 = 目标位(Goal)与实测位(Present)中"离本次去处更近"的那个:
    - 吊着重物(垂度使实测比Goal深): 从Goal起, 避免先下坠一个垂度再抬(实测坑);
    - 被桌面顶住(Goal在桌面以下, 实测停在桌面): 从实测起, 立刻离桌,
      避免余弦前段指令仍在桌下、臂持续压桌(实测坑, 上一条修复的副作用)。
    """
    present = read_joints(robot)
    start = {}
    for j in target:
        try:
            g = float(robot.bus.read("Goal_Position", j, normalize=True))
        except Exception:
            g = present[j]
        start[j] = g if abs(target[j] - g) <= abs(target[j] - present[j]) else present[j]
    _trace("goto " + " ".join(f"{j}:{start[j]:.1f}->{target[j]:.1f}" for j in target) + f" ({duration:.1f}s)")
    max_delta = max(abs(target[j] - start[j]) for j in target)
    # 余弦轨迹峰值速度 = 平均速度 * pi/2，按峰值限速
    min_duration = (max_delta / MAX_DEG_PER_S) * (math.pi / 2)
    duration = max(duration, min_duration)
    steps = max(int(duration * hz), 1)
    for i in range(1, steps + 1):
        a = (1 - math.cos(math.pi * i / steps)) / 2  # 0->1 缓入缓出
        action = {f"{j}.pos": start[j] + (target[j] - start[j]) * a for j in target}
        robot.send_action(action)
        time.sleep(1.0 / hz)
