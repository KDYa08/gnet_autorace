import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'gnet_autorace'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kdya08',
    maintainer_email='kdya08@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'odom_test = gnet_autorace.missions.odom_test:main',
            'hsv_trackbar = gnet_autorace.missions.hsv_trackbar_ros:main',
            'line_tracing = gnet_autorace.missions.line_tracing:main',
            'core = gnet_autorace.core.core:main',
            'traffic_light = gnet_autorace.missions.traffic_light:main',
            'intersection = gnet_autorace.missions.intersection:main',
            'construction_sign = gnet_autorace.missions.construction_sign:main',
            'construction = gnet_autorace.missions.construction:main',
            'parking_sign = gnet_autorace.missions.parking_sign:main',
            'parking = gnet_autorace.missions.parking:main',
            'blockbar = gnet_autorace.missions.blockbar:main',
            'tunnel_sign = gnet_autorace.missions.tunnel_sign:main',
            'tunnel_right_hand = gnet_autorace.missions.tunnel_right_hand:main',
            'tunnel_bug2 = gnet_autorace.missions.tunnel_bug2:main'
        ],
    },
)
