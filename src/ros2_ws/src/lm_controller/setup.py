from setuptools import setup
from glob import glob
import os

package_name = 'lm_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='LM null-space controller for uFactory xArm',
    license='MIT',
    entry_points={
        'console_scripts': [
            'xarm_interface_node = lm_controller.xarm_interface_node:main',
        ],
    },
)