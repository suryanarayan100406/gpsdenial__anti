from setuptools import setup
import os
from glob import glob

package_name = 'perception_layer'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lenovo',
    maintainer_email='lenovo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ekf_fusion_node = perception_layer.ekf_fusion_node:main',
            'noise_injection_node = perception_layer.noise_injection_node:main',
            'sensor_health_monitor = perception_layer.sensor_health_monitor:main'
        ],
    },
)
