from glob import glob
from setuptools import find_packages, setup

package_name = 'task_layer_v010'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='ryan',
    maintainer_email='ryanwong1379@gmail.com',
    description='Task layer experiment v0.1: single-robot semantic-area-to-Nav2 navigation.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'go_to_area = task_layer_v010.go_to_area_node:main',
            'nav_gui = task_layer_v010.nav_gui_node:main',
            'set_initial_pose = task_layer_v010.set_initial_pose_node:main',
        ],
    },
)
