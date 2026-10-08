import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'cryvex_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'web'), glob('web/*.html')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='main',
    maintainer_email='volkanimre34@gmail.com',
    description='Cryvex gercek donanim baslatma paketi (STM32 koprusu, YDLIDAR, EKF)',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stm32_bridge = cryvex_bringup.stm32_bridge:main',
            'cafe_ui_server = cryvex_bringup.cafe_ui_server:main',
            'patrol = cryvex_bringup.patrol:main',
        ],
    },
)
