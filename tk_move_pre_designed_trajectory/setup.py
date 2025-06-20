from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tk_move_pre_designed_trajectory'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tk',
    maintainer_email='takuchanapp@gmail.com',
    description='Pre-designed trajectory movement package for autonomous robot navigation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sin_trajectory = tk_move_pre_designed_trajectory.sin_trajectory:main',
            'tan_trajectory = tk_move_pre_designed_trajectory.tan_trajectory:main',
            'circle_trajectory = tk_move_pre_designed_trajectory.circle_trajectory:main',
            'ellipse_trajectory = tk_move_pre_designed_trajectory.ellipse_trajectory:main',
            'triangle_trajectory = tk_move_pre_designed_trajectory.triangle_trajectory:main',
            'square_trajectory = tk_move_pre_designed_trajectory.square_trajectory:main',
            'pentagon_trajectory = tk_move_pre_designed_trajectory.pentagon_trajectory:main',
            'hexagon_trajectory = tk_move_pre_designed_trajectory.hexagon_trajectory:main',
            'rectangle_trajectory = tk_move_pre_designed_trajectory.rectangle_trajectory:main',
        ],
    },
)
