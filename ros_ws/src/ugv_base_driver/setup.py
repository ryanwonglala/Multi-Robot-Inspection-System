from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'ugv_base_driver'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roboinspec',
    maintainer_email='1011072@mymail.sutd.edu.sg',
    description='Namespaced ROS 2 driver for the RoboInspect Waveshare UGV base.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'ugv_base_node = ugv_base_driver.ugv_base_node:main',
        ],
    },
)
