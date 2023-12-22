import os
from glob import glob
from setuptools import setup

package_name: str = 'gps_slam_converter'
node_package_name: str = package_name + '.node'
position_package_name: str = package_name + '.position'

packages_list: list = [
    package_name,
    node_package_name,
    position_package_name
]

setup(
    name=package_name,
    version='0.1.3',
    packages=packages_list,
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='reidlo',
    maintainer_email='naru5135@wavem.net',
    description='GPS <-> SLAM Coordinates Converting Package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_slam_converter = gps_slam_converter.main:main'
        ],
    },
)
