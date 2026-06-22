from __future__ import annotations

import shutil
from datetime import datetime, timezone
from pathlib import Path

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
