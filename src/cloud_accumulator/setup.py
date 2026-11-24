from setuptools import setup

package_name = 'cloud_accumulator'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fish',
    maintainer_email='fish@todo.todo',
    description='ROI-bounded voxel point cloud accumulator',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'accumulate = cloud_accumulator.accumulate_node:main',
        ],
    },
)
