"""像素坐标 -> 抓取关节角 的 RBF 插值映射。"""

import json
from pathlib import Path

import numpy as np
from scipy.interpolate import RBFInterpolator

_CALIB_DIR = Path(__file__).parent.parent.parent / "calibration"
_V3 = _CALIB_DIR / "samples_v3.json"
_V2 = _CALIB_DIR / "samples_v2.json"


def _v3_ready() -> bool:
    """v3(2026-07-30 挪臂后的新库) 攒够 3 条即整体切换, 旧库退役。"""
    try:
        return len(json.loads(_V3.read_text())) >= 3
    except (FileNotFoundError, json.JSONDecodeError):
        return False


# v3 = 新臂位净土库(首选); v2 = 旧臂位样本(深度带~8°系统差, 仅引导期兜底);
# v1 是断电手扶采的, 含垂度偏差, 最后兜底
SAMPLES_FILE = _V3 if _v3_ready() else (_V2 if _V2.exists() else _CALIB_DIR / "samples.json")

# 参与映射的关节（夹爪不插值，抓取时用固定开合策略）
MAP_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]

PIXEL_SCALE = 1000.0  # 像素坐标归一化因子，改善 RBF 条件数


def _load(samples_file: Path = SAMPLES_FILE):
    samples = json.loads(samples_file.read_text())
    px = np.array([s["pixel"] for s in samples]) / PIXEL_SCALE
    joints = np.array([[s["joints"][j] for j in MAP_JOINTS] for s in samples])
    return px, joints


class PixelToJoints:
    """样本≥6: 最小二乘平面拟合(小工作区内各关节≈像素的线性函数; 抓取成功深度
    自带±2°随机余量, 逐点插值会把噪声当真理, 深度面越采越抖——实测坑)。
    样本<6: 退回 RBF 逐点插值(点太少拟合平面欠定)。"""

    def __init__(self, samples_file: Path = SAMPLES_FILE, smoothing: float = 0.0):
        px, joints = _load(samples_file)
        if len(px) >= 6:
            A = np.hstack([np.ones((len(px), 1)), px])  # [1, x, y]
            self.coef, *_ = np.linalg.lstsq(A, joints, rcond=None)
            self.interp = None
        else:
            # linear 核经 LOO 对比最稳（thin_plate 会因近重复矛盾样本震荡）
            self.coef = None
            self.interp = RBFInterpolator(px, joints, kernel="linear", smoothing=smoothing)

    def __call__(self, pixel_x: float, pixel_y: float) -> dict[str, float]:
        p = np.array([pixel_x, pixel_y]) / PIXEL_SCALE
        if self.coef is not None:
            q = np.array([1.0, p[0], p[1]]) @ self.coef
        else:
            q = self.interp(p[None, :])[0]
        return dict(zip(MAP_JOINTS, q.tolist()))


def leave_one_out() -> None:
    """留一交叉验证：逐样本剔除后预测，报告各关节误差。"""
    px, joints = _load()
    n = len(px)
    errs = []
    for i in range(n):
        mask = np.arange(n) != i
        interp = RBFInterpolator(px[mask], joints[mask], kernel="thin_plate_spline")
        pred = interp(px[i : i + 1])[0]
        errs.append(pred - joints[i])
    errs = np.array(errs)
    print(f"留一交叉验证（{n} 样本）关节角误差（度）:")
    print(f"{'关节':<15}{'平均|误差|':>10}{'最大|误差|':>10}")
    for k, j in enumerate(MAP_JOINTS):
        print(f"{j:<15}{np.abs(errs[:, k]).mean():>10.2f}{np.abs(errs[:, k]).max():>10.2f}")
    worst = np.abs(errs[:, :3]).max(axis=1)  # 前三个关节主导定位
    print("\n各样本主关节最大误差:", " ".join(f"#{i}:{e:.1f}" for i, e in enumerate(worst)))


if __name__ == "__main__":
    leave_one_out()
