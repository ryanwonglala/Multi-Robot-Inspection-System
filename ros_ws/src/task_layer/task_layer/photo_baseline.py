from __future__ import annotations

import math
from pathlib import Path
import shutil

import yaml


INDEX_VERSION = 1


def pose_distance(first: dict, second: dict) -> float:
    """Planar distance between two indexed observation poses."""
    return math.hypot(
        float(first['x']) - float(second['x']),
        float(first['y']) - float(second['y']),
    )


def pose_within_tolerance(first: dict, second: dict,
                          tolerance: float) -> bool:
    return pose_distance(first, second) <= float(tolerance) + 1e-9


class BaselineLibrary:
    """Structured photo baseline library keyed by area and view index."""

    def __init__(self, root: str | Path):
        self.root = Path(root).expanduser()
        self.index_path = self.root / 'index.yaml'
        self.data = self._load()

    def _load(self) -> dict:
        if not self.index_path.exists():
            return {'version': INDEX_VERSION, 'views': {}}
        with self.index_path.open('r', encoding='utf-8') as file:
            data = yaml.safe_load(file) or {}
        if data.get('version') != INDEX_VERSION:
            raise ValueError(
                f'unsupported baseline index version: {data.get("version")}')
        if not isinstance(data.get('views'), dict):
            raise ValueError('baseline index views must be a mapping')
        return data

    @staticmethod
    def _view_key(view_index: int) -> str:
        return str(int(view_index))

    def lookup(self, area: str, view_index: int) -> dict | None:
        entry = ((self.data.get('views') or {}).get(area) or {}).get(
            self._view_key(view_index))
        if not isinstance(entry, dict):
            return None
        resolved = dict(entry)
        resolved['image_path'] = str(self.root / str(entry['filename']))
        return resolved

    def record(self, area: str, view_index: int, source: str | Path,
               pose: dict) -> dict:
        source_path = Path(source)
        if not source_path.is_file():
            raise FileNotFoundError(f'baseline source image missing: {source}')

        relative = (Path('images') / area
                    / f'view_{int(view_index):02d}{source_path.suffix.lower()}')
        target = self.root / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source_path, target)

        entry = {
            'area': area,
            'view_index': int(view_index),
            'pose': {
                'x': round(float(pose['x']), 4),
                'y': round(float(pose['y']), 4),
                'yaw': round(float(pose['yaw']), 4),
            },
            'filename': relative.as_posix(),
        }
        self.data.setdefault('views', {}).setdefault(area, {})[
            self._view_key(view_index)] = entry
        self._write()
        return self.lookup(area, view_index)

    def _write(self):
        self.root.mkdir(parents=True, exist_ok=True)
        temporary = self.index_path.with_suffix('.yaml.tmp')
        with temporary.open('w', encoding='utf-8') as file:
            yaml.safe_dump(self.data, file, sort_keys=False)
        temporary.replace(self.index_path)
