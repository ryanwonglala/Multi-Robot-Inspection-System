"""颜色目标检测：HSV 分割 + 形态学去噪 + 面积过滤。"""

import json
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


@dataclass
class Blob:
    cx: float          # 质心像素坐标
    cy: float
    area: float        # 像素面积
    bbox: tuple[int, int, int, int]  # x, y, w, h
    angle: float = 0.0  # 主轴朝向(度, 0-180, 相对图像x轴)
    elongation: float = 1.0  # 长宽比(≈1 则朝向无意义)


# 黄色发泡球的 HSV 范围（OpenCV H:0-179），根据实测可在 config 覆盖
YELLOW_LO = (20, 90, 90)
YELLOW_HI = (38, 255, 255)

_ROI_FILE = Path(__file__).parent.parent.parent / "config" / "roi.json"


def target_param(key: str, default):
    """读取 config/target.json 中目标物的附加参数(如 grip 合爪目标)。"""
    return _target_cfg().get(key, default)


def posture_ok(blob: Blob) -> bool:
    """判断目标形态是否正常(站姿)。倒伏/遮挡时面积或长宽比会显著偏离标准。"""
    cfg = _target_cfg()
    exp = cfg.get("expected_area")
    if exp:
        lo, hi = cfg.get("area_tolerance", [0.5, 2.0])
        if not (exp * lo <= blob.area <= exp * hi):
            return False
    max_e = cfg.get("posture_max_elong")
    if max_e and blob.elongation > max_e:
        return False
    return True


def servo_roi(ref, half: int = 420, frame_w: int = 1920, frame_h: int = 1080) -> tuple[int, int, int, int]:
    """伺服近景检测窗口: 只看基准点周围区域。

    近景全画面做全局阈值会把暗色桌面/臂影连成环形巨块(实测), 必须用窗口把
    视野限制在 目标+纸面 内。"""
    x = int(max(0, min(ref[0] - half, frame_w - 2 * half)))
    y = int(max(0, min(ref[1] - half, frame_h - 2 * half)))
    return (x, y, 2 * half, 2 * half)


def workspace_roi() -> tuple[int, int, int, int] | None:
    """config/roi.json 定义的观察位检测白名单区域(避开盆沿等禁区)。"""
    if _ROI_FILE.exists():
        r = json.loads(_ROI_FILE.read_text())
        return (r["x"], r["y"], r["w"], r["h"])
    return None


_TARGET_FILE = Path(__file__).parent.parent.parent / "config" / "target.json"
_REF_FILE = Path(__file__).parent.parent.parent / "calibration" / "observe_ref.jpg"
_REF_CACHE: dict = {}


def _target_cfg() -> dict:
    """config/target.json 定义当前目标物的分割参数(换目标改配置即可)。"""
    if _TARGET_FILE.exists():
        return json.loads(_TARGET_FILE.read_text())
    return {}


def _surface_anomaly_mask(
    frame: np.ndarray, roi, chroma_thresh: float = 14.0, edge_density_thresh: float = 0.07
) -> np.ndarray:
    """表面异常分割(抗阴影版): 色度异常 OR 边缘密度异常。

    - 色度通道: 与工作面色度(a,b)距离大 -> 彩色物体。阴影只降亮度不改色度, 免疫。
    - 纹理通道: 局部边缘密度高 -> 无彩色但有结构的物体(积木等)。纸面和阴影平滑, 免疫。
    亮度不参与判定, 光照/阴影天然自适应; 白色手指平滑同色, 归入背景。
    """
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB).astype(np.int16)
    if roi is not None:
        x, y, w, h = roi
        region = lab[y : y + h, x : x + w]
    else:
        region = lab
    bg = np.median(region.reshape(-1, 3), axis=0)
    chroma = np.sqrt((lab[:, :, 1] - bg[1]) ** 2.0 + (lab[:, :, 2] - bg[2]) ** 2.0)
    chroma_mask = chroma > chroma_thresh

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # 注: 勿加CLAHE——噪声放大会产生假块并把阴影区粘连成巨块(实测)
    med = float(np.median(gray))
    edges = cv2.Canny(gray, max(20, 0.66 * med), max(60, 1.33 * med))
    density = cv2.boxFilter(edges.astype(np.float32) / 255.0, -1, (51, 51))
    edge_mask = density > edge_density_thresh

    return ((chroma_mask | edge_mask).astype(np.uint8)) * 255


def _ref_diff_mask(frame: np.ndarray, l_thresh: float = 45.0, chroma_thresh: float = 12.0) -> np.ndarray:
    """参考帧差分(观察位固定+背景固定时的首选): 与空场参考照不同 = 异常物。

    手指/其影/纸面纹理都在参考照里, 天然免疫。彩色物走色度差(a,b, 抗阴影),
    黑色物走大幅亮度差(阈值远高于阴影的变暗量)。
    参考照由 scripts/14_capture_ref.py 在净空后重拍; 灯光或构图变了必须重拍。
    """
    if "img" not in _REF_CACHE:
        ref = cv2.imread(str(_REF_FILE))
        if ref is None:
            raise FileNotFoundError(
                f"缺少空场参考照 {_REF_FILE}: 清空工作面后跑 scripts/14_capture_ref.py"
            )
        _REF_CACHE["img"] = cv2.cvtColor(cv2.GaussianBlur(ref, (5, 5), 0), cv2.COLOR_BGR2LAB).astype(np.int16)
    ref_lab = _REF_CACHE["img"]
    lab = cv2.cvtColor(cv2.GaussianBlur(frame, (5, 5), 0), cv2.COLOR_BGR2LAB).astype(np.int16)
    if lab.shape != ref_lab.shape:
        raise ValueError(f"帧尺寸 {lab.shape} 与参考照 {ref_lab.shape} 不符, 重拍参考照")
    dl = np.abs(lab[:, :, 0] - ref_lab[:, :, 0])
    dc = np.sqrt((lab[:, :, 1] - ref_lab[:, :, 1]) ** 2.0 + (lab[:, :, 2] - ref_lab[:, :, 2]) ** 2.0)
    return (((dl > l_thresh) | (dc > chroma_thresh)).astype(np.uint8)) * 255


def _notwhite_mask(frame: np.ndarray, s_min: int = 90, v_max: int = 60) -> np.ndarray:
    """近景"非白即物"分割(类别无关): 饱和度高(彩色) OR 很暗(黑色)。

    白纸亮面 S≈13、阴影 S≈62、白手指 S≈15 都低于 s_min; 黑积木 V≈4 走暗通道。
    透明物本体测不到, 但其上的深色特征(如积木)可作伺服锚点。"""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    return ((hsv[:, :, 1] >= s_min) | (hsv[:, :, 2] <= v_max)).astype(np.uint8) * 255


def classify_blob(frame: np.ndarray, blob: Blob) -> str | None:
    """按 config classes 的 match 规则给观察位目标分类; None = 未定义类(不抓)。

    特征(全部光照弱敏感):
      solidity = 掩码像素数/外轮廓填充面积 (环状物如手串 ≈0.5, 实心物 ≈1.0)
      L/a/b    = 掩码内像素的 Lab 中值(a<0 偏绿, a>0 偏红, b>0 偏黄; 已减128居中)
    规则键: solidity_max, l_max, a_max, a_min, b_max, abs_a_max。按配置顺序首中即得。
    """
    classes = _target_cfg().get("classes")
    if not classes:
        return None
    x, y, w, h = blob.bbox
    m = _ref_diff_mask(frame)[y : y + h, x : x + w] > 0
    if m.sum() < 50:
        return None
    solidity = float(m.sum()) / max(blob.area, 1.0)
    lab = cv2.cvtColor(frame[y : y + h, x : x + w], cv2.COLOR_BGR2LAB).astype(np.int16)
    L, a, b = (float(v) for v in np.median(lab[m], axis=0) - [0, 128, 128])
    for name, cfg in classes.items():
        r = cfg.get("match", {})
        if "solidity_max" in r and solidity > r["solidity_max"]:
            continue
        if "l_max" in r and L > r["l_max"]:
            continue
        if "a_max" in r and a > r["a_max"]:
            continue
        if "a_min" in r and a < r["a_min"]:
            continue
        if "b_max" in r and b > r["b_max"]:
            continue
        if "abs_a_max" in r and abs(a) > r["abs_a_max"]:
            continue
        return name
    return None


def detect_blobs(
    frame: np.ndarray,
    hsv_lo: tuple | None = None,
    hsv_hi: tuple | None = None,
    min_area: float | None = None,
    max_area: float | None = None,
    roi: tuple[int, int, int, int] | None = None,  # x, y, w, h；None=全图
    seg: str | None = None,  # 调用方显式指定分割器: "notwhite"(近景伺服用)
) -> list[Blob]:
    """返回按面积降序的目标列表。

    分割模式: seg 显式指定优先; 否则由 config/target.json 的 mode 决定:
      "refdiff" = 参考帧差分(观察位固定+净空参考照, 首选)
      "surface" = 表面色异常分割(无参考照时的兜底)
      其他/缺省 = HSV 阈值分割(参数未显式给出时取 config, 兜底黄色)
    """
    cfg = _target_cfg()
    min_area = min_area if min_area is not None else cfg.get("min_area", 300)
    max_area = max_area if max_area is not None else cfg.get("max_area", 30000)
    if seg == "notwhite":
        mask = _notwhite_mask(frame)
    elif cfg.get("mode") == "refdiff" and hsv_lo is None:
        mask = _ref_diff_mask(
            frame,
            l_thresh=cfg.get("ref_l_thresh", 45.0),
            chroma_thresh=cfg.get("ref_chroma_thresh", 12.0),
        )
    elif cfg.get("mode") == "surface" and hsv_lo is None:
        mask = _surface_anomaly_mask(
            frame, roi,
            chroma_thresh=cfg.get("chroma_thresh", 14.0),
            edge_density_thresh=cfg.get("edge_density_thresh", 0.07),
        )
    else:
        hsv_lo = hsv_lo if hsv_lo is not None else tuple(cfg.get("hsv_lo", YELLOW_LO))
        hsv_hi = hsv_hi if hsv_hi is not None else tuple(cfg.get("hsv_hi", YELLOW_HI))
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(hsv_lo), np.array(hsv_hi))
    if roi is not None:
        x, y, w, h = roi
        m = np.zeros_like(mask)
        m[y : y + h, x : x + w] = mask[y : y + h, x : x + w]
        mask = m
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    max_elong = cfg.get("max_elongation")  # 细长物(线缆等)过滤, None=不过滤
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    blobs = []
    for c in contours:
        area = cv2.contourArea(c)
        if not (min_area <= area <= max_area):
            continue
        if max_elong is not None:
            (_, _), (ew, eh), _ = cv2.minAreaRect(c)
            if min(ew, eh) > 1e-6 and max(ew, eh) / min(ew, eh) > max_elong:
                continue
        m = cv2.moments(c)
        if m["m00"] == 0:
            continue
        (_, _), (rw, rh), rang = cv2.minAreaRect(c)
        if rw < rh:  # 归一化: angle 指向长轴, 0-180
            rw, rh = rh, rw
            rang += 90.0
        blobs.append(
            Blob(
                cx=m["m10"] / m["m00"],
                cy=m["m01"] / m["m00"],
                area=area,
                bbox=cv2.boundingRect(c),
                angle=rang % 180.0,
                elongation=(rw / rh) if rh > 1e-6 else 1.0,
            )
        )
    return sorted(blobs, key=lambda b: -b.area)


def annotate(frame: np.ndarray, blobs: list[Blob]) -> np.ndarray:
    out = frame.copy()
    for i, b in enumerate(blobs):
        x, y, w, h = b.bbox
        cv2.rectangle(out, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.drawMarker(out, (int(b.cx), int(b.cy)), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
        cv2.putText(
            out, f"#{i} ({int(b.cx)},{int(b.cy)}) a={int(b.area)}",
            (x, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2,
        )
    return out
