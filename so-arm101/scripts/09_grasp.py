"""抓取执行器：检测 -> 对位 -> 合爪 -> 提起 -> 送回收区 -> 投放 -> 复位。

用法:
    .venv/bin/python scripts/09_grasp.py            # 单次抓取
    .venv/bin/python scripts/09_grasp.py --loop     # 循环抓到视野内无目标为止
    .venv/bin/python scripts/09_grasp.py --grip 4   # 调整合爪紧度(默认5, 越小越紧)
    .venv/bin/python scripts/09_grasp.py --test 20  # 抓取成功率压测: 抓-提-验-原地放,
                                                    # 重复20次统计成功率(不运输)
"""

import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

import numpy as np

from soarm.arm import (
    connect, descend_until_touch, goto_exact, grip_close, grip_load, load_pose, read_joints,
    shutdown, smooth_goto, transport_to_drop,
)
from soarm.camera_client import get_frame
from soarm.mapping import MAP_JOINTS, PixelToJoints, _load
from soarm.vision import classify_blob, detect_blobs, servo_roi, target_param, workspace_roi

GRIPPER_OPEN = 30.0
SERVO_FILE = Path(__file__).parent.parent / "config" / "servo.json"
LOG_FILE = Path(__file__).parent.parent / "calibration" / "attempts_log.jsonl"


def log_attempt(**kw):
    kw["t"] = round(time.time(), 1)
    with LOG_FILE.open("a") as f:
        f.write(json.dumps(kw, ensure_ascii=False) + "\n")

SERVO_MAX_AREA = 400_000
SERVO_TOL_PX = 12
SERVO_ITERS = 10  # 引导期外推误差大, 给足收敛轮数

parser = argparse.ArgumentParser()
parser.add_argument("--loop", action="store_true")
parser.add_argument("--grip", type=float, default=float(target_param("grip", 5.0)))
parser.add_argument("--test", type=int, default=0, metavar="N", help="压测模式: 抓N次统计成功率")
parser.add_argument("--step", action="store_true", help="手动节拍: 每轮完成后按Enter才开下一轮(压测与--loop通用)")
args = parser.parse_args()

# 安全包络: 预测关节角不得超出样本集范围外太多
_, _joints = _load()
J_MIN = _joints.min(axis=0) - 12.0
J_MAX = _joints.max(axis=0) + 12.0

mapping = PixelToJoints()
observe = load_pose("observe")
drop = load_pose("drop")

robot = connect()
smooth_goto(robot, observe, duration=3.0)

MAX_RETRY = 3




def vertical_wf(cls, sl: float, el: float):
    """垂直姿态约束: 肩+肘+腕俯仰 ≈ 夹爪绝对俯仰(旧样本实测该和std仅3°)。
    从该类示教姿态取常数K, 返回当前肩/肘下应设的腕俯仰; 类未示教姿态则返回None。"""
    gj = cls.get("grasp_joints")
    if not gj:
        return None
    K = gj["shoulder_lift"] + gj["elbow_flex"] + gj["wrist_flex"]
    return K - sl - el


def cls_seg_kwargs(cls) -> dict:
    """类专属伺服分割: 有 servo_hsv 用色彩通道(如低饱和浅色物, notwhite 会失明);
    否则用默认"非白即物"。"""
    if cls.get("servo_hsv"):
        lo, hi = cls["servo_hsv"]
        return {"hsv_lo": tuple(lo), "hsv_hi": tuple(hi)}
    return {"seg": "notwhite"}


def servo_loop(robot, command, ref, servo, iters, seg_kw=None) -> bool:
    """悬停伺服至 ref, command 原地更新; 返回是否收敛(容差2倍内)。

    腕旋离开中立角时图像坐标系随之旋转, 雅可比同步旋转补偿(实测坑)。"""
    import math
    Jm = np.array(servo["jacobian"])
    sj = servo["joints"]
    gain = servo.get("roll_gain", 0.0)
    neutral = servo.get("roll_neutral", 0.0)
    err = None
    for it in range(iters):
        time.sleep(0.4)
        blobs2 = detect_blobs(get_frame(), seg="notwhite", max_area=SERVO_MAX_AREA, roi=servo_roi(ref))
        if not blobs2:
            print("  伺服: 腕相机未见目标")
            return False
        bb = min(blobs2, key=lambda x: (x.cx - ref[0]) ** 2 + (x.cy - ref[1]) ** 2)
        err = np.array([bb.cx, bb.cy]) - ref
        print(f"  伺服#{it}: 像素偏差 ({err[0]:+.0f},{err[1]:+.0f})")
        if np.hypot(*err) <= SERVO_TOL_PX:
            return True
        th = math.radians((command.get("wrist_roll", neutral) - neutral) * gain)
        R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
        dq = np.clip(np.linalg.solve(R @ Jm, -err) * 0.8, -4.0, 4.0)
        for j, d in zip(sj, dq):
            command[j] += float(d)
        smooth_goto(robot, command, duration=0.6)
    return err is not None and np.hypot(*err) <= SERVO_TOL_PX * 2

grabbed = 0
fails = 0
attempts = 0
successes = 0
while True:
    if args.test and attempts >= args.test:
        break
    time.sleep(0.5)
    for _try in range(3):  # 防臂未停稳的旧帧误判(同12号)
        frame = get_frame()
        blobs = detect_blobs(frame, roi=workspace_roi())
        if blobs:
            break
        time.sleep(1.2)
    if not blobs:
        print("安全区内无目标")
        if args.loop:
            print(f"任务完成，共抓取 {grabbed} 个")
        break

    classes = target_param("classes", {}) or {}
    if classes:  # 按类分参: 取第一个能识别的目标, 未定义类跳过不抓
        b = cls_name = None
        for cand in blobs:
            name = classify_blob(frame, cand)
            if name is not None:
                b, cls_name = cand, name
                break
            print(f"  未定义类目标 ({cand.cx:.0f},{cand.cy:.0f}) 面积{cand.area:.0f}，跳过")
        if b is None:
            print("视野内无已定义类别的目标")
            if args.loop:
                print(f"任务完成，共抓取 {grabbed} 个")
            break
        cls = classes.get(cls_name, {})
    else:  # 兼容旧单目标模式
        b, cls_name, cls = blobs[0], None, {}
    target = mapping(b.cx, b.cy)
    q = np.array([target[j] for j in MAP_JOINTS])
    if (q < J_MIN).any() or (q > J_MAX).any():
        print(f"目标 ({b.cx:.0f},{b.cy:.0f}) 预测位姿超出安全包络，跳过。请把球挪到工作区内侧")
        break

    servo = json.loads(SERVO_FILE.read_text()) if SERVO_FILE.exists() else None
    hover = servo.get("hover_delta", 0.0) if servo else 0.0
    # 抓取深度 = 映射预测 + 全局深度偏置(标定实测) + 类深度差(示教实测)
    # 偏置只在旧库引导期用: v3 新库样本自带实测深度, 再叠加就是双重扣深
    from soarm.mapping import SAMPLES_FILE
    stale_guide = SAMPLES_FILE.name != "samples_v3.json"
    depth_bias = servo.get("depth_bias", 0.0) if (servo and stale_guide) else 0.0
    grasp_sl = target["shoulder_lift"] + depth_bias + cls.get("depth_delta", 0.0)
    # 类抓取参数: ref=抓取点在腕相机中的定义(每类不同, 未示教则用全局标定值)
    ref_px = cls.get("ref") or (servo["ref"] if servo else None)
    angle_ref = cls.get("angle_ref") if classes else (servo.get("angle_ref") if servo else None)

    print(f"目标[{cls_name or '默认'}]({b.cx:.0f},{b.cy:.0f}) 面积{b.area:.0f}，执行抓取……")
    # 悬停对位: 指尖悬在目标上方数厘米, 伺服微调不会推动目标
    aim = {**target, "shoulder_lift": grasp_sl - hover, "gripper": GRIPPER_OPEN}
    wf_v = vertical_wf(cls, grasp_sl, aim["elbow_flex"])
    if wf_v is not None:
        aim["wrist_flex"] = wf_v  # 垂直姿态约束(见 vertical_wf)
    if servo and servo.get("roll_neutral") is not None:
        aim["wrist_roll"] = servo["roll_neutral"]  # 腕旋转中立起步, 对齐靠实测
    goto_exact(robot, aim, duration=3.0)

    if servo:  # 悬停高度视觉伺服
        ref = np.array(ref_px)
        command = read_joints(robot)
        converged = servo_loop(robot, command, ref, servo, SERVO_ITERS, cls_seg_kwargs(cls))
        # 未收敛不闭眼下潜(会夹到底板/扫飞目标), 回观察位重来
        if not converged:
            fails += 1
            print(f"  伺服未收敛，回观察位重试({fails}/{MAX_RETRY})")
            smooth_goto(robot, observe, duration=3.0)
            if fails >= MAX_RETRY:
                print("  连续未收敛，放弃")
                break
            continue

    if servo and servo.get("roll_gain") and angle_ref is not None:  # 朝向对齐: 主轴转回该类基准角
        blobs3 = detect_blobs(get_frame(), **cls_seg_kwargs(cls),
                              max_area=SERVO_MAX_AREA, roi=servo_roi(np.array(ref_px)))
        if blobs3:
            rr = np.array(ref_px)
            bb3 = min(blobs3, key=lambda x: (x.cx - rr[0]) ** 2 + (x.cy - rr[1]) ** 2)
            if bb3.elongation >= 1.5:
                aerr = (bb3.angle - angle_ref + 90) % 180 - 90
                if abs(aerr) >= 8.0:
                    # 上限90: 腕旋行程±180充足; 角差本身∈±90
                    droll = float(np.clip(aerr / servo["roll_gain"], -90.0, 90.0))
                    command["wrist_roll"] += droll
                    smooth_goto(robot, {"wrist_roll": command["wrist_roll"]},
                                duration=max(0.8, abs(droll) / 25))
                    print(f"  朝向对齐: 角差{aerr:+.0f}° -> 腕旋{droll:+.1f}°")
                    # 转腕会带偏指尖与像距, 必须复核伺服(12号同款, 09曾缺失导致空夹)
                    if not servo_loop(robot, command, np.array(ref_px), servo, 5, cls_seg_kwargs(cls)):
                        print("  转腕后伺服未收敛，本轮放弃")
                        fails += 1
                        smooth_goto(robot, {"gripper": GRIPPER_OPEN}, duration=0.5)
                        smooth_goto(robot, observe, duration=3.0)
                        if fails >= MAX_RETRY:
                            break
                        continue

    if hover:  # 垂直下潜到示教深度(深度=人工示教样本的平面拟合+类深度差, 无学习无算术链)
        el_now = command["elbow_flex"] if servo else aim["elbow_flex"]
        descend = {"shoulder_lift": grasp_sl}
        wf_v = vertical_wf(cls, grasp_sl, el_now)
        if wf_v is not None:
            descend["wrist_flex"] = wf_v  # 垂直姿态补偿
        smooth_goto(robot, descend, duration=1.2)
    hold = cls.get("hold", [4.0, 16.0])
    held, g, load = grip_close(robot, hold_min_opening=hold[0], hold_max_opening=hold[1])  # 接触即停合爪

    if args.test:  # 压测: 抓-提-验-降回-放
        attempts += 1
        held1, g1 = held, g
        cur_sl = read_joints(robot)["shoulder_lift"]
        smooth_goto(robot, {"shoulder_lift": max(cur_sl - 30.0, -60.0)}, duration=1.5)  # 提起
        time.sleep(0.5)
        load2 = grip_load(robot)
        held2 = load2 >= 80
        g2 = g
        ok = held1 and held2
        successes += ok
        print(f"  [{attempts}/{args.test}] {'成功' if ok else '失败'}"
              f"(合爪{'√' if held1 else '×'} 提起{'√' if held2 else '×'} 开合度{g2:.1f} 负载{load2})"
              f"  累计成功率 {successes}/{attempts} = {successes/attempts:.0%}")
        log_attempt(pixel=[round(b.cx), round(b.cy)],
                    result="success" if ok else ("slip" if held1 else "empty"),
                    opening=round(g2, 1), load=int(load2), cls=cls_name, mode="test")
        if held2:
            smooth_goto(robot, {"shoulder_lift": cur_sl}, duration=1.2)   # 降回抓取深度
        smooth_goto(robot, {"gripper": GRIPPER_OPEN}, duration=0.8)       # 贴面松爪(禁空中松手)
        smooth_goto(robot, observe, duration=3.0)
        if args.step and attempts < args.test:
            try:
                input("  调整位姿后按 Enter 开始下一轮 > ")
            except EOFError:
                pass
        continue

    if not held:
        fails += 1
        print(f"  验证: 空夹(开合度{g:.1f}, 负载{load})，第{fails}次失败" +
              ("，放弃" if fails >= MAX_RETRY else "，重试"))
        smooth_goto(robot, {"gripper": GRIPPER_OPEN}, duration=0.8)
        smooth_goto(robot, observe, duration=3.0)
        if fails >= MAX_RETRY:
            break
        continue                                                          # 重新检测(球滚了也能跟上)

    print(f"  验证: 已夹住(开合度{g:.1f}, 负载{load})")
    fails = 0
    transport_to_drop(robot)                                              # 分段运输(不碰夹爪)
    smooth_goto(robot, {"gripper": GRIPPER_OPEN}, duration=0.8)           # 投放
    time.sleep(0.3)
    grabbed += 1
    print(f"第 {grabbed} 个投放完成")
    smooth_goto(robot, observe, duration=3.5)                             # 复位再检测

    if not args.loop:
        break
    if args.step:  # 手动节拍: 每轮完成后等确认再开下一轮
        try:
            input("  按 Enter 开始下一轮 (Ctrl+C 结束) > ")
        except EOFError:
            pass

if args.test and attempts:
    print(f"\n压测结束: {successes}/{attempts} = {successes/attempts:.0%}")
shutdown(robot)
print("执行器退出")
