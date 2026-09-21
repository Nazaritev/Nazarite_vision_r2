from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'stm32_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ljx',
    maintainer_email='ljx@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'stm32_driver_node=stm32_driver.stm32_driver_node:main',
            'serial_script_launcher=stm32_driver.serial_script_launcher:main',
            'nav2_agent=stm32_driver.nav_agent:main',
            'r2_mode1=stm32_driver.r2_mode1:main',
            'r2_mode1_blue=stm32_driver.r2_mode1_blue:main',
            'r1_mode1_challenge_blue=stm32_driver.r1_mode1_challenge_blue:main',
            'r1_mode1_challenge_red=stm32_driver.r1_mode1_challenge_red:main',
            'r1_mode1_challenge_blue_test=stm32_driver.r1_mode1_challenge_blue_test:main',
            'r1_mode1_challenge_red_test=stm32_driver.r1_mode1_challenge_red_test:main',
            'r2_mode1_final_test_blue=stm32_driver.r2_mode1_final_test_blue:main',
            'odom_cmd_vel_fuser=stm32_driver.odom_cmd_vel_fuser:main',
        ],
    },
)
