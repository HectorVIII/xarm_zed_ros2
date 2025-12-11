from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'xarm_zed_handover'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='huitao',
    maintainer_email='jszdhyjs@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
                'handover_node = xarm_zed_handover.handover_node:main',
                'zed_right_hand_node = xarm_zed_handover.zed_left_hand_node:main',
                'handover_gui = xarm_zed_handover.handover_gui_node:main'
        ],
    },
)
