from __future__ import annotations

import re
import shutil
import struct
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from typing import List, Optional

import yaml


def default_report_dir() -> str:
    return str(Path.home() / 'roboinspec_ws' / 'reports')


def write_report(
    report: dict,
    report_dir: str | Path | None = None,
    filename: str | None = None,
) -> Path:
    target_dir = Path(report_dir or default_report_dir()).expanduser()
    target_dir.mkdir(parents=True, exist_ok=True)
    if filename is None:
        timestamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        area = str(report.get('target_area', 'unknown')).replace('/', '_')
        filename = f'inspection_{timestamp}_{area}.yaml'
    path = target_dir / filename
    with path.open('w', encoding='utf-8') as file:
        yaml.safe_dump(report, file, sort_keys=False, allow_unicode=True)
    return path


def write_markdown_report(
    summary: dict,
    report_dir: str | Path,
    filename: str = 'report.md',
) -> Path:
    """Render a simplified bilingual (中文 + English) Markdown report from a
    summary dict produced by inspection_runner.build_summary_report.

    Task 5.1 — new helper.
    """
    target_dir = Path(report_dir).expanduser()
    target_dir.mkdir(parents=True, exist_ok=True)
    path = target_dir / filename

    s = summary.get('summary') or {}
    status = summary.get('status', 'unknown')
    route = summary.get('route') or []
    areas = summary.get('areas') or []
    anomalies = summary.get('anomalies') or []
    return_home = summary.get('return_home') or {}
    details_file = summary.get('details_file', '')
    task = summary.get('task', '')

    # Build status label
    STATUS_MAP = {
        'completed': '已完成 / Completed',
        'completed_with_failures': '完成但有失败项 / Completed with failures',
        'completed_return_failed': '完成但回桩失败 / Completed, return failed',
        'aborted_unsafe_nav_state': '已中止 (不安全导航状态) / Aborted (unsafe nav state)',
        'dry_run': '演练 / Dry run',
    }
    status_label = STATUS_MAP.get(status, status)

    lines: list[str] = []
    lines.append('# RoboInspect 巡检报告 / Inspection Report\n')

    lines.append('## 概要 / Summary\n')
    lines.append(f'- **任务类型 / Task**: `{task}`')
    lines.append(f'- **整体状态 / Overall status**: `{status_label}`')
    checked = s.get('checked_count', 0)
    requested = s.get('requested_count', len(areas))
    lines.append(f'- **完成区域 / Areas checked**: {checked} / {requested}')
    lines.append(f'- **异常数量 / Anomaly count**: {s.get("anomaly_count", len(anomalies))}')
    lines.append('')

    lines.append('## 路线 / Route\n')
    if route:
        lines.append(', '.join(f'`{r}`' for r in route))
    else:
        lines.append('_(none)_')
    lines.append('')

    lines.append('## 各区域结果 / Per-area Results\n')
    if areas:
        lines.append('| 区域 / Area | 显示名 / Display Name | 状态 / Status | 照片数 / Photos | 异常 / Anomalies |')
        lines.append('|---|---|---|---|---|')
        for a in areas:
            area_key = a.get('area') or ''
            display = a.get('display_name') or area_key
            a_status = a.get('status') or 'unknown'
            photos = a.get('captured_image_count', 0)
            a_anomalies = len(a.get('anomalies') or [])
            lines.append(f'| `{area_key}` | {display} | `{a_status}` | {photos} | {a_anomalies} |')
    else:
        lines.append('_(no areas)_')
    lines.append('')

    lines.append('## 异常列表 / Anomalies\n')
    if anomalies:
        for idx, anm in enumerate(anomalies, 1):
            atype = anm.get('type', 'unknown')
            area_key = anm.get('area', '')
            desc = anm.get('description') or anm.get('summary') or ''
            lines.append(f'{idx}. `{area_key}` — type: `{atype}` {desc}')
    else:
        lines.append('_(无异常 / No anomalies detected)_')
    lines.append('')

    lines.append('## 回桩状态 / Return-home Status\n')
    rh_status = return_home.get('status') or 'unknown'
    rh_target = return_home.get('target')
    lines.append(f'- **状态 / Status**: `{rh_status}`')
    if rh_target:
        lines.append(f'- **目标桩位 / Target dock**: `{rh_target}`')
    lines.append('')

    lines.append('## 相关文件 / Related Files\n')
    if details_file:
        lines.append(f'- **完整机读报告 / Full machine report**: `{details_file}`')
    # Link to screenshot if it exists
    screenshot = target_dir / 'rviz_final.png'
    if screenshot.exists():
        lines.append(f'- **RViz 截图 / RViz screenshot**: `{screenshot}`')
    lines.append('')

    path.write_text('\n'.join(lines), encoding='utf-8')
    return path


def prune_report_dirs(
    parent: str | Path,
    pattern: str,
    keep: int = 10,
) -> list[Path]:
    """Keep the `keep` newest directories matching parent/pattern; delete the rest.

    Sort by directory name descending (names embed sortable UTC timestamps);
    fall back to st_mtime when names compare equal. Best-effort: ignore per-dir
    errors, NEVER raise. ONLY removes dirs matching the glob.

    Task 5.1 — new helper.
    """
    parent = Path(parent).expanduser()
    removed: list[Path] = []
    try:
        candidates = sorted(
            (d for d in parent.glob(pattern) if d.is_dir()),
            key=lambda d: (d.name, d.stat().st_mtime),
            reverse=True,
        )
    except Exception:
        return removed

    to_delete = candidates[keep:]
    for d in to_delete:
        try:
            shutil.rmtree(d)
            removed.append(d)
        except Exception:
            pass
    return removed


def _find_rviz_window_id() -> Optional[str]:
    """Return the X window id of the largest mapped window that looks like
    RViz (via xwininfo), or None. No wmctrl/xdotool needed."""
    try:
        tree = subprocess.run(
            ['xwininfo', '-root', '-tree'],
            capture_output=True, text=True, timeout=5).stdout
    except Exception:
        return None
    best_area = -1
    best_id = None
    for line in tree.splitlines():
        if 'rviz' not in line.lower():
            continue
        # e.g.  0x4800106 "cfg* - RViz": ("rviz2" "rviz2")  1545x1124+20+90  +162+476
        m = re.search(r'(0x[0-9a-fA-F]+).*?\s(\d+)x(\d+)\+-?\d+\+-?\d+', line)
        if not m:
            continue
        area = int(m.group(2)) * int(m.group(3))
        if area > best_area:
            best_area, best_id = area, m.group(1)
    return best_id


def _xwd_bytes_to_image(data: bytes):
    """Decode an XWD (X Window Dump) byte stream into a PIL RGB Image.

    Handles the common TrueColor 24/32-bpp ZPixmap dumps that `xwd -id`
    produces; channel order is derived from the header masks + byte order."""
    from PIL import Image  # optional dependency, imported lazily
    if len(data) < 100:
        raise ValueError('xwd stream too short')
    hdr = struct.unpack('>25I', data[:100])
    header_size = hdr[0]
    width, height = hdr[4], hdr[5]
    byte_order = hdr[7]            # 0 = LSBFirst, 1 = MSBFirst
    bpp = hdr[11]
    bytes_per_line = hdr[12] or (width * (bpp // 8))
    rmask, gmask, bmask = hdr[14], hdr[15], hdr[16]
    ncolors = hdr[19]
    # Pixel data follows the header (incl. window-name string) and the colormap
    # (ncolors * 12-byte XWDColor entries, present even for TrueColor visuals).
    pix = data[header_size + ncolors * 12:]
    nbytes = max(1, bpp // 8)
    pos = {}
    for ch, mask in zip('RGB', (rmask, gmask, bmask)):
        if not mask:
            continue
        value_byte = (mask.bit_length() - 1) // 8
        mem = value_byte if byte_order == 0 else (nbytes - 1 - value_byte)
        pos[mem] = ch
    rawmode = ''.join(pos.get(i, 'X') for i in range(nbytes))
    return Image.frombytes('RGB', (width, height), pix, 'raw',
                           rawmode, bytes_per_line, 1)


def capture_rviz_screenshot(
    target_dir: str | Path,
    filename: str = 'rviz_final.png',
) -> Optional[Path]:
    """Best-effort screenshot of the RViz window ONLY (not the full screen).

    Locates the RViz window via xwininfo and dumps its own pixels with
    `xwd -id` (works even when RViz is occluded by another window), then
    decodes the XWD stream to PNG. Returns the saved Path, or None if RViz is
    not found / X tooling is missing / any step fails. NEVER raises, and never
    falls back to a full-screen capture.

    Task 5.1 — RViz-window capture (replaces the earlier full-screen grab).
    """
    try:
        win_id = _find_rviz_window_id()
        if not win_id:
            return None
        dump = subprocess.run(
            ['xwd', '-silent', '-id', win_id],
            capture_output=True, timeout=10)
        if dump.returncode != 0 or not dump.stdout:
            return None
        img = _xwd_bytes_to_image(dump.stdout)
        target_dir = Path(target_dir).expanduser()
        target_dir.mkdir(parents=True, exist_ok=True)
        path = target_dir / filename
        img.save(str(path))
        return path
    except Exception:
        return None
