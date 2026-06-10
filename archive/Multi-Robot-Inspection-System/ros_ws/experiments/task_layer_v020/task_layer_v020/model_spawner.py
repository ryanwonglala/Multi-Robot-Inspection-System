import random
import re
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
import yaml


PACKAGE_NAME = 'task_layer_v020'


def package_share_dir() -> Path:
    return Path(get_package_share_directory(PACKAGE_NAME))


def default_world_model_path() -> Path:
    return package_share_dir() / 'config' / 'world_model.yaml'


def models_dir() -> Path:
    return package_share_dir() / 'models'


def load_world_model(path: str | Path | None = None) -> dict:
    source = Path(path).expanduser() if path else default_world_model_path()
    if not source.is_file():
        raise FileNotFoundError('world model does not exist: %s' % source)
    with source.open('r', encoding='utf-8') as file:
        return yaml.safe_load(file) or {}


def list_builtin_models() -> list[dict]:
    all_paths = sorted(models_dir().rglob('*.sdf'))
    nested_stems = {
        path.stem
        for path in all_paths
        if path.parent != models_dir()
    }
    selected = [
        path
        for path in all_paths
        if path.parent != models_dir() or path.stem not in nested_stems
    ]
    stem_counts = {}
    for path in selected:
        stem_counts[path.stem] = stem_counts.get(path.stem, 0) + 1

    entries = []
    for path in selected:
        rel = path.relative_to(models_dir()).with_suffix('')
        key = path.stem if stem_counts[path.stem] == 1 else rel.as_posix()
        entries.append({
            'key': key,
            'path': str(path),
            'label': key,
            'aliases': sorted({path.stem, path.parent.name, key}),
        })
    return entries


def resolve_model_file(model_key: str, file_param: str = '') -> str:
    if file_param:
        candidate = Path(file_param).expanduser()
        if not candidate.is_file():
            raise FileNotFoundError('model file does not exist: %s' % candidate)
        return str(candidate.resolve())

    candidate = Path(model_key).expanduser()
    if candidate.is_file():
        return str(candidate.resolve())

    for entry in list_builtin_models():
        if model_key == entry['key'] or model_key in entry['aliases']:
            return entry['path']

    available = ', '.join(model['key'] for model in list_builtin_models())
    raise FileNotFoundError(
        'unknown model "%s"; use file:=/path/to/model.sdf or one of: %s'
        % (model_key, available)
    )


def area_items(world_model: dict) -> list[tuple[str, dict]]:
    return list((world_model.get('areas') or {}).items())


def resolve_area(world_model: dict, query: str) -> tuple[str, dict]:
    query = str(query).strip()
    items = area_items(world_model)
    if not query:
        raise ValueError('area is empty')

    if query.isdigit():
        index = int(query)
        if index < 1 or index > len(items):
            raise ValueError('area number must be 1..%d' % len(items))
        return items[index - 1]

    normalized = normalize_text(query)
    for key, area in items:
        names = {
            normalize_text(key),
            normalize_text(area.get('display_name', key)),
            normalize_text(area.get('marker_model', '')),
        }
        if normalized in names:
            return key, area

    raise ValueError('unknown area: %s' % query)


def normalize_text(value: str) -> str:
    return value.strip().lower().replace(' ', '_').replace('-', '_')


def area_center(area: dict) -> tuple[float, float]:
    center = area.get('center')
    if not center or len(center) < 2:
        raise ValueError('area is missing center: [x, y]')
    return float(center[0]), float(center[1])


def area_random(area: dict, margin: float = 0.2) -> tuple[float, float]:
    bounds = area.get('bounds') or {}
    required = ['x_min', 'x_max', 'y_min', 'y_max']
    if not all(key in bounds for key in required):
        raise ValueError('area is missing bounds')

    x_min = float(bounds['x_min']) + margin
    x_max = float(bounds['x_max']) - margin
    y_min = float(bounds['y_min']) + margin
    y_max = float(bounds['y_max']) - margin
    if x_min > x_max or y_min > y_max:
        return area_center(area)
    return random.uniform(x_min, x_max), random.uniform(y_min, y_max)


def make_spawn_command(params: dict) -> list[str]:
    command = ['ros2', 'run', 'ros_gz_sim', 'create', '--ros-args']
    for key, value in params.items():
        command.extend(['-p', f'{key}:={value}'])
    return command


def prepare_spawn_params(params: dict) -> dict:
    prepared = dict(params)
    prepared['file'] = prepare_sdf_file(Path(str(params['file'])))
    return prepared


def prepare_sdf_file(source: Path) -> str:
    text = source.read_text(encoding='utf-8')
    rewritten = re.sub(
        r'(<uri>)(.*?)(</uri>)',
        lambda match: match.group(1) + resolve_asset_uri(match.group(2).strip(), source) + match.group(3),
        text,
        flags=re.DOTALL,
    )
    temp = tempfile.NamedTemporaryFile(
        mode='w',
        encoding='utf-8',
        suffix='_' + safe_filename(source.name),
        prefix='task_layer_v020_',
        delete=False,
    )
    with temp:
        temp.write(rewritten)
    return temp.name


def resolve_asset_uri(uri: str, sdf_path: Path) -> str:
    if uri.startswith(('http://', 'https://', 'file://', 'data:')):
        return uri

    if uri.startswith('model://'):
        rest = uri[len('model://'):]
        parts = rest.split('/', 1)
        model_name = parts[0]
        tail = parts[1] if len(parts) > 1 else ''
        candidates = []
        if tail:
            candidates.extend([
                models_dir() / model_name / tail,
                sdf_path.parent / tail,
            ])
            candidates.extend(sorted(models_dir().glob('*/' + tail)))
        for candidate in candidates:
            if candidate.is_file():
                return candidate.resolve().as_uri()
        return uri

    if uri.startswith('/'):
        candidate = Path(uri)
        return candidate.resolve().as_uri() if candidate.is_file() else uri

    candidate = sdf_path.parent / uri
    if candidate.is_file():
        return candidate.resolve().as_uri()

    matches = sorted(models_dir().glob('*/' + uri))
    if matches:
        return matches[0].resolve().as_uri()

    return uri


def safe_filename(value: str) -> str:
    return re.sub(r'[^A-Za-z0-9_.-]+', '_', value)


def run_spawn(params: dict) -> int:
    return subprocess.run(make_spawn_command(prepare_spawn_params(params)), check=False).returncode


def validate_model_assets(sdf_file: str) -> list[str]:
    source = Path(sdf_file)
    text = source.read_text(encoding='utf-8')
    issues = []
    for uri in re.findall(r'<uri>(.*?)</uri>', text, flags=re.DOTALL):
        uri = uri.strip()
        resolved = resolve_asset_uri(uri, source)
        if resolved == uri and not uri.startswith(('http://', 'https://', 'file://', 'data:')):
            issues.append(uri)
    return issues


def unique_entity_name(model_key: str, index: int | None = None) -> str:
    base = re.sub(r'[^A-Za-z0-9_]+', '_', Path(model_key).stem).strip('_')
    if not base:
        base = 'model'
    suffix = '%03d' % index if index is not None else str(int(time.time() * 1000))
    return '%s_%s' % (base, suffix)


class SpawnModel(Node):
    def __init__(self):
        super().__init__('spawn_model')
        self.declare_parameter('world', 'map')
        self.declare_parameter('world_model_path', '')
        self.declare_parameter('model', 'small_box_obstacle')
        self.declare_parameter('file', '')
        self.declare_parameter('name', '')
        self.declare_parameter('allow_renaming', False)
        self.declare_parameter('area', '')
        self.declare_parameter('placement', 'manual')
        self.declare_parameter('random_margin', 0.2)
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.25)
        self.declare_parameter('roll', 0.0)
        self.declare_parameter('pitch', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('dry_run', False)

    def run(self):
        model_key = self.get_parameter('model').value
        file_param = self.get_parameter('file').value
        model_file = resolve_model_file(model_key, file_param)
        entity_name = self.get_parameter('name').value or unique_entity_name(Path(model_file).stem)
        x = float(self.get_parameter('x').value)
        y = float(self.get_parameter('y').value)
        area_query = self.get_parameter('area').value
        placement = self.get_parameter('placement').value

        if area_query:
            world_model_path = self.get_parameter('world_model_path').value or None
            world_model = load_world_model(world_model_path)
            area_key, area = resolve_area(world_model, area_query)
            if placement == 'center':
                x, y = area_center(area)
            elif placement == 'random':
                margin = float(self.get_parameter('random_margin').value)
                x, y = area_random(area, margin)
            elif placement != 'manual':
                raise ValueError('placement must be manual, center, or random')
            self.get_logger().info('Resolved area=%s placement=%s' % (area_key, placement))

        params = {
            'world': self.get_parameter('world').value,
            'file': model_file,
            'name': entity_name,
            'allow_renaming': bool(self.get_parameter('allow_renaming').value),
            'x': x,
            'y': y,
            'z': float(self.get_parameter('z').value),
            'R': float(self.get_parameter('roll').value),
            'P': float(self.get_parameter('pitch').value),
            'Y': float(self.get_parameter('yaw').value),
        }

        self.get_logger().info(
            'Spawning model %s from %s into world=%s at x=%.3f y=%.3f z=%.3f yaw=%.3f'
            % (
                entity_name,
                model_file,
                params['world'],
                params['x'],
                params['y'],
                params['z'],
                params['Y'],
            )
        )

        if self.get_parameter('dry_run').value:
            command = make_spawn_command(prepare_spawn_params(params))
            self.get_logger().info('dry_run=true, command: %s' % ' '.join(command))
            return 0

        return_code = run_spawn(params)
        if return_code != 0:
            self.get_logger().error('ros_gz_sim create failed with code %d' % return_code)
        return return_code


def main(args=None):
    rclpy.init(args=args)
    node = SpawnModel()
    try:
        return_code = node.run()
    except Exception as exc:
        node.get_logger().error(str(exc))
        return_code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(return_code)
