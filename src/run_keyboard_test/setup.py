from setuptools import find_packages, setup
from glob import glob
package_name = 'run_keyboard_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name+'/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fzx',
    maintainer_email='fzx@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'get_and_pub=run_keyboard_test.get_and_pub:main',
            'camera=run_keyboard_test.camera:main',
            'run_slope=run_keyboard_test.run_slope:main',
            'scan_get=run_keyboard_test.scan_get:main',
            'nav_to_pose=run_keyboard_test.nav_to_pose:main',
            'yolo_control=run_keyboard_test.yolo_control:main',
        ],
    },
)

