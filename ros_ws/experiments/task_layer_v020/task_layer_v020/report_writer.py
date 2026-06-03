from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path

import yaml


def default_report_dir() -> str:
    return '/home/ryan/tb4_ws/experiments/task_layer_v020/reports'


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
        yaml.safe_dump(report, file, sort_keys=False)
    return path
