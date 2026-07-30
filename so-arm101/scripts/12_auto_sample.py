"""自动采样器：系统自己试抓、自己验证、自己积累样本（Waddle 式自采数据）。

前置: 已在当前工作面上跑过 10_servo_calib.py（悬停基准 + 雅可比）。
用法:
    .venv/bin/python scripts/12_auto_sample.py --n 10     # 目标采集10个验证样本

流程(每样本):
  观察位检测球 -> 引导映射预测 -> 悬停伺服对准 -> 从浅到深试探下潜+合爪
  -> 负载验证: 成功则存样本(像素,成功指令), 叼到随机位置丢下(随机化下一轮)
     失败则加深重试; 同一目标连败则放弃本轮
样本写入 calibration/samples_v3.json（2026-07-30 挪臂后的全新库, 从零攒），
映射即时重拟合、越采越准；v3≥3条后全系统自动切换到 v3, 旧库退役。
"""

import argparse
import json
import random
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

import numpy as np

from soarm.arm import (
    connect, descend_to_support, descend_until_touch, goto_exact, grip_close, grip_load,
    load_pose, read_joints, shutdown, smooth_goto,
)
from soarm.camera_client import get_frame
from soarm.mapping import MAP_JOINTS, PixelToJoints
from soarm.vision import classify_blob, detect_blobs, posture_ok, servo_roi, target_param, workspace_roi

CALIB = Path(__file__).parent.parent / "calibration"
V2_FILE = CALIB / "samples_v2.json"   # 旧臂位样本(深度带~8°系统差), 仅引导期兜底
V3_FILE = CALIB / "samples_v3.json"   # 新臂位净土库: 本脚本只写这里
SERVO_FILE = Path(__file__).parent.parent / "config" / "servo.json"

GRIPPER_OPEN = 30.0
GRIP = float(target_param("grip", 5.0))
SERVO_MAX_AREA = 400_000
SERVO_TOL_PX = 12
DEPTH_START_COLD = -8.0  # 无深度经验时的保守起点(负=偏浅)
DEPTH_START_WARM = -1.5  # 映射已含实测深度时的热启动起点
DEPTH_STEP = 1.5     # 每次失败加深量
DEPTH_MAX = 3.5      # 最深不超过预测+3.5°(覆盖区边缘外推偏浅需追赶; 夹底板检测兜底)
MAX_FAILS_PER_TARGET = 8
LOG_FILE = CALIB / "attempts_log.jsonl"

parser = argparse.ArgumentParser()
parser.add_argument("--n", type=int, default=10)
parser.add_argument("--wait", type=int, default=8, help="每轮之间的摆放时间(秒)")
args = parser.parse_args()

servo = json.loads(SERVO_FILE.read_text())
REF = np.array(servo["ref"])  # 全局兜底; 有类参数时每轮按类覆盖(见主循环)
CLASSES = target_param("classes", {}) or {}
ANGLE_REF = servo.get("angle_ref", 0.0)
DEPTH_DELTA = 0.0
SEG_KW = {"seg": "notwhite"}




def vertical_wf(cls, sl: float, el: float):
    """垂直姿态约束: 肩+肘+腕俯仰 ≈ 夹爪绝对俯仰(旧样本实测该和std仅3°)。
    从该类示教姿态取常数K, 返回当前肩/肘下应设的腕俯仰; 类未示教姿态则返回None。"""
    gj = cls.get("grasp_joints")
    if not gj:
        return None
    K = gj["shoulder_lift"] + gj["elbow_flex"] + gj["wrist_flex"]
    return K - sl - el


def cls_seg_kwargs(cls) -> dict:
    if cls.get("servo_hsv"):
        lo, hi = cls["servo_hsv"]
        return {"hsv_lo": tuple(lo), "hsv_hi": tuple(hi)}
    return {"seg": "notwhite"}
JAC = np.array(servo["jacobian"])
SJ = servo["joints"]
HOVER = servo.get("hover_delta", 8.0)

samples = json.loads(V3_FILE.read_text()) if V3_FILE.exists() else []


def bootstrap_mapping():
    for f in (V3_FILE, V2_FILE, CALIB / "samples_v2_desk.json", CALIB / "samples.json"):
        if f.exists() and len(json.loads(f.read_text())) >= 3:
            return PixelToJoints(f), f.name
    raise SystemExit("无可用引导样本")


def _jac_now(command):
    """腕旋离开中立角时图像坐标系随之旋转, 雅可比要同步旋转(实测坑)。"""
    import math
    th = math.radians((command.get("wrist_roll", servo.get("roll_neutral", 0.0))
                       - servo.get("roll_neutral", 0.0)) * servo.get("roll_gain", 0.0))
    R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
    return R @ JAC


def servo_converge(robot, command) -> bool:
    """悬停伺服, 收敛返回 True。command 原地更新。"""
    for _ in range(10):  # 覆盖区边缘初始偏差大, 弱轴(肘)每轮只追~30px, 给足轮数
        time.sleep(0.4)
        blobs = detect_blobs(get_frame(), **SEG_KW, max_area=SERVO_MAX_AREA, roi=servo_roi(REF))
        if not blobs:
            return False
        bb = min(blobs, key=lambda x: (x.cx - REF[0]) ** 2 + (x.cy - REF[1]) ** 2)
        err = np.array([bb.cx, bb.cy]) - REF
        if np.hypot(*err) <= SERVO_TOL_PX:
            return True
        dq = np.clip(np.linalg.solve(_jac_now(command), -err) * 0.8, -4.0, 4.0)
        for j, d in zip(SJ, dq):
            command[j] += float(d)
        smooth_goto(robot, command, duration=0.6)
    return bool(np.hypot(*err) <= SERVO_TOL_PX * 2)


def align_roll(robot, command) -> None:
    """朝向对齐: 把目标在腕相机里的主轴角度转回标定基准角。"""
    gain = servo.get("roll_gain", 0.0)
    if not gain:
        return
    blobs = detect_blobs(get_frame(), **SEG_KW,
                         max_area=SERVO_MAX_AREA, roi=servo_roi(REF))
    if not blobs:
        return
    bb = min(blobs, key=lambda x: (x.cx - REF[0]) ** 2 + (x.cy - REF[1]) ** 2)
    if bb.elongation < 1.5:
        return  # 视角近圆, 朝向无意义
    if ANGLE_REF is None:
        return  # 该类未定义朝向(近圆形)
    err = (bb.angle - ANGLE_REF + 90) % 180 - 90
    if abs(err) < 8.0:
        return
    delta = float(np.clip(err / gain, -90.0, 90.0))
    command["wrist_roll"] += delta
    smooth_goto(robot, {"wrist_roll": command["wrist_roll"]}, duration=max(0.8, abs(delta) / 25))
    print(f"  朝向对齐: 角差{err:+.0f}° -> 腕旋{delta:+.1f}°")


def log_attempt(**kw):
    kw["t"] = round(time.time(), 1)
    with LOG_FILE.open("a") as f:
        f.write(json.dumps(kw, ensure_ascii=False) + "\n")


def pick_sparse_spot(roi) -> tuple[float, float, float | None]:
    """有界新颖度放置: 偏向样本空白区, 但离最近样本不超过250px(外推误差有界)。

    返回 (x, y, 最近样本的实测肩深度) —— 放置深度用它锚定, 防止映射外推
    导致空中松手/怼纸。
    """
    import random as _r
    if not samples:
        return roi[0] + roi[2] * 0.5, roi[1] + roi[3] * 0.5, None
    pts = np.array([s["pixel"] for s in samples])
    depths = np.array([s["joints"]["shoulder_lift"] for s in samples])
    best, best_score, best_near = None, -1.0, None
    for _ in range(16):
        c = np.array([roi[0] + roi[2] * _r.uniform(0.25, 0.75),
                      roi[1] + roi[3] * _r.uniform(0.25, 0.75)])
        dists = np.linalg.norm(pts - c, axis=1)
        score = min(float(dists.min()), 250.0)  # 新颖度封顶
        if score > best_score:
            best, best_score, best_near = c, score, int(dists.argmin())
    return float(best[0]), float(best[1]), float(depths[best_near])


mapping, src = bootstrap_mapping()
print(f"引导映射: {src}，已有样本 {len(samples)}，目标 {args.n}")
observe = load_pose("observe")
robot = connect()
smooth_goto(robot, observe, duration=3.0)

import signal as _signal  # noqa: E402


def _estop(sig, frame):
    # 急停: 不释放扭矩(臂原地冻结不下坠), 直接退出; 用 13_recover.py 恢复
    print("\n【急停】扭矩保持, 臂已冻结。恢复: .venv/bin/python scripts/13_recover.py")
    sys.exit(130)


_signal.signal(_signal.SIGINT, _estop)

collected_this_run = 0
barren_rounds = 0  # 连续无收获轮数, 3轮自动收工(替代人工形态暂停)
round_no = 0
while collected_this_run < args.n:
    if round_no > 0 and args.wait > 0:  # 摆放间隙(预览页有绿框参照)
        for _s in range(args.wait, 0, -1):
            sys.stdout.write(f"\r  ⏳ 摆放时间 {_s}s（把目标放到绿框内新位置）   ")
            sys.stdout.flush()
            time.sleep(1)
        print()
    round_no += 1
    if barren_rounds >= 3:
        print("连续3轮未成功(目标可能倒伏/不可达), 自动收工")
        break
    time.sleep(0.5)
    # 单帧可能是臂未停稳的旧帧, 连看3帧才认"无目标"(实测: 一帧误判导致提前收工)
    for _try in range(3):
        frame = get_frame()
        blobs = detect_blobs(frame, roi=workspace_roi())
        if blobs:
            break
        time.sleep(1.2)
    if not blobs:
        print("安全区内无目标，结束（目标可能溜出检测区或检测失效，看一眼实物位置）")
        break
    b = blobs[0]
    # 按类取抓取参数(ref=该类抓取点的像素定义; 用错类的ref会系统性夹偏, 实测坑)
    cls_name = classify_blob(frame, b) if CLASSES else None
    cls = CLASSES.get(cls_name, {}) if cls_name else {}
    REF = np.array(cls["ref"]) if cls.get("ref") else np.array(servo["ref"])
    ANGLE_REF = cls.get("angle_ref") if cls_name else servo.get("angle_ref", 0.0)
    DEPTH_DELTA = cls.get("depth_delta", 0.0)
    SEG_KW = cls_seg_kwargs(cls)
    base = mapping(b.cx, b.cy)
    print(f"\n[样本{len(samples)}] 目标[{cls_name or '默认'}]({b.cx:.0f},{b.cy:.0f})")

    # 深度先验: 自采映射已含实测深度; 引导映射则叠加标定时测得的全局深度偏置
    if src == V3_FILE.name:
        DEPTH_START = DEPTH_START_WARM
    else:  # 旧库引导: 深度必须叠加标定实测的全局偏置 + 类深度差
        bias = servo.get("depth_bias", 0.0)
        base["shoulder_lift"] += bias + DEPTH_DELTA
        # 旧库+全局偏置的逐点残差可达数度, 一律浅起步逐级下探, 防悬停即怼桌(实测)
        DEPTH_START = DEPTH_START_COLD

    # 悬停对准(腕旋转用中立角起步, 对齐交给实测)
    hover_cmd = {**base, "shoulder_lift": base["shoulder_lift"] + DEPTH_START - HOVER,
                 "wrist_roll": servo.get("roll_neutral", base["wrist_roll"]),
                 "gripper": GRIPPER_OPEN}
    wf_v = vertical_wf(cls, base["shoulder_lift"] + DEPTH_START, base["elbow_flex"])
    if wf_v is not None:
        hover_cmd["wrist_flex"] = wf_v  # 垂直姿态: 腕俯仰由约束给出, 不用映射插值值
    goto_exact(robot, hover_cmd, duration=2.5)
    command = read_joints(robot)
    command["gripper"] = GRIPPER_OPEN
    if not servo_converge(robot, command):
        print("  伺服未收敛，放弃本轮（球可能贴沿）")
        smooth_goto(robot, observe, duration=3.0)
        continue
    align_roll(robot, command)      # 朝向对齐(转腕)
    servo_converge(robot, command)  # 旋转可能带偏质心, 快速复核

    # 触觉找桌 + 合爪: 深度的最终事实来源 = 肩负载触面检测
    # (映射/偏置/类差那条深度算术链只用于悬停对位, 不再决定抓取深度——撞桌教训)
    got = False
    fails = 0
    hover_sl = command["shoulder_lift"]
    first_conv = {j: command[j] for j in ("shoulder_pan", "elbow_flex")}  # 初次收敛基准
    TOUCH_BACK = 0.8  # 触面后回升量: 指尖离面一线合爪, 不刮纸
    while fails < 3:
        wf_of = ((lambda sl: vertical_wf(cls, sl, command["elbow_flex"]))
                 if cls.get("grasp_joints") else None)
        touch_sl, touched = descend_until_touch(robot, hover_sl, max_extra=HOVER + 12.0,
                                                step=0.6, wf_of=wf_of)
        if not touched:
            print(f"  下探{HOVER + 12.0:.0f}°未触面，放弃本轮")
            break
        grasp_sl = touch_sl - TOUCH_BACK
        cmd = {"shoulder_lift": grasp_sl}
        if wf_of is not None:
            cmd["wrist_flex"] = wf_of(grasp_sl)
            command["wrist_flex"] = cmd["wrist_flex"]
        smooth_goto(robot, cmd, duration=0.4)
        command["shoulder_lift"] = grasp_sl
        hold = cls.get("hold", [4.0, 16.0])
        held, g, load = grip_close(robot, hold_min_opening=hold[0], hold_max_opening=hold[1])
        if held:
            smooth_goto(robot, {"shoulder_lift": max(grasp_sl - 30.0, -60.0)}, duration=1.5)
            time.sleep(0.5)
            g2 = g
            held2 = grip_load(robot) >= 80  # 提起后负载仍在=东西还在爪里
            log_attempt(pixel=[round(b.cx), round(b.cy)], depth=round(grasp_sl, 1),
                        result="success" if held2 else "slip", n_samples=len(samples),
                        angle=round(b.angle), elong=round(b.elongation, 2))
            if held2:
                got = True
                drift = max(abs(command[j] - first_conv[j]) for j in first_conv)
                if drift <= 1.2:
                    final = {**{j: command[j] for j in MAP_JOINTS}, "shoulder_lift": grasp_sl}
                    samples.append({"pixel": [round(b.cx, 1), round(b.cy, 1)],
                                    "joints": {j: round(final[j], 2) for j in MAP_JOINTS}})
                    V3_FILE.write_text(json.dumps(samples, indent=2, ensure_ascii=False))
                    collected_this_run += 1
                    print(f"  ✓ 触面肩{touch_sl:.1f} 成功(开合{g2:.1f})，样本已存({len(samples)})")
                else:
                    print(f"  ✓ 抓住但目标漂移{drift:.1f}°(>1.2°)，样本作废不入库")
                break
            print(f"  提起后掉落(触面肩{touch_sl:.1f})，重试")
        else:
            result = "plate" if g > hold[1] else "empty"
            print(f"  合爪未成(开合{g:.1f}, 负载{load}, {result})，重新对位再试")
            log_attempt(pixel=[round(b.cx), round(b.cy)], depth=round(grasp_sl, 1),
                        result=result, n_samples=len(samples),
                        angle=round(b.angle), elong=round(b.elongation, 2))
        # 失败: 张爪, 回悬停, 重新伺服后再触觉找桌
        fails += 1
        smooth_goto(robot, {"gripper": GRIPPER_OPEN}, duration=0.6)
        back = {"shoulder_lift": hover_sl}
        if wf_of is not None:
            back["wrist_flex"] = wf_of(hover_sl)
        smooth_goto(robot, back, duration=1.0)
        if not servo_converge(robot, command):
            print("  目标移动后伺服失败，放弃本轮")
            break
        hover_sl = command["shoulder_lift"]

    barren_rounds = 0 if got else barren_rounds + 1
    if got:
        # 零预测放置: 复用一条已验证样本的完整关节位姿(指尖高度=物理复现),
        # 仅随机旋转底座换方位(底座旋转不改变指尖高度)。
        tmpl = dict(random.choice(samples)["joints"]) if samples else {
            j: command[j] for j in MAP_JOINTS} | {"shoulder_lift": grasp_sl}
        tmpl["shoulder_pan"] += random.uniform(-8.0, 8.0)
        place_sl = tmpl["shoulder_lift"]
        carry = {**tmpl, "shoulder_lift": max(place_sl - 30.0, -60.0)}  # 高位转移
        smooth_goto(robot, carry, duration=2.0)
        smooth_goto(robot, {"shoulder_lift": place_sl - 3.0}, duration=1.5)  # 先到位姿上方
        final_sl = descend_to_support(robot, place_sl - 3.0, max_extra=4.5)   # 承重感知缓降
        print(f"  放置: 触面于肩{final_sl:.1f}(样本深度{place_sl:.1f})")
        smooth_goto(robot, {"gripper": GRIPPER_OPEN}, duration=1.2)          # 缓开爪
        smooth_goto(robot, {"shoulder_lift": place_sl - HOVER}, duration=1.0)  # 抬离
        # 样本≥5后切换到自己的映射
        if len(samples) >= 3 and src != V3_FILE.name:
            mapping, src = PixelToJoints(V3_FILE), V3_FILE.name
            print("  引导映射已切换为新库 v3")
        elif src == V3_FILE.name:
            mapping = PixelToJoints(V3_FILE)  # 重拟合
    smooth_goto(robot, observe, duration=3.0)

print(f"\n本次自采 {collected_this_run} 个，样本库共 {len(samples)} 个")
shutdown(robot)
