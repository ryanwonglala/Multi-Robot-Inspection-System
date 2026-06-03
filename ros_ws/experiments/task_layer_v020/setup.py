from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'task_layer_v020'


def data_files_for_tree(destination, source):
    data_files = []
    for root, _dirs, files in os.walk(source):
        if not files:
            continue
        target = os.path.join(destination, os.path.relpath(root, source))
        data_files.append((target, [os.path.join(root, file) for file in files]))
    return data_files


setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
    ] + data_files_for_tree('share/' + package_name + '/models', 'models'),
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='ryan',
    maintainer_email='ryanwong1379@gmail.com',
    description='Task layer experiment v0.2: single-robot semantic-area-to-Nav2 navigation.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'go_to_area = task_layer_v020.go_to_area_node:main',
            'inspect_area = task_layer_v020.inspection_runner:main',
            'task_gui = task_layer_v020.task_gui_node:main',
            'scene_builder_gui = task_layer_v020.scene_builder_gui_node:main',
            'spawn_model = task_layer_v020.model_spawner:main',
            'set_initial_pose = task_layer_v020.set_initial_pose_node:main',
        ],
    },
)
