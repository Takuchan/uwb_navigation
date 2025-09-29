import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'tk_uwb_ekf'

setup(
    name=package_name,
    version='0.0.0',
    # パッケージ内のPythonモジュールを自動的に検索
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launchファイルをインストール対象に追加
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tk',
    maintainer_email='takuchanapp@gmail.com',
    description='UWB EKF localization package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    # console_scriptsに実行可能ファイルを登録
    entry_points={
        'console_scripts': [
            # '実行可能ファイル名 = パッケージ名.スクリプト名:main'
            'uwb_ekf_node = tk_uwb_ekf.uwb_ekf_node:main',
        ],
    },
)
