from setuptools import find_packages, setup

package_name = 'astromoon_missions'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/mission.launch.py',
        ]),

        ('share/' + package_name + '/missions', [
            'missions/m1_waypoint_traverse.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='emilien',
    maintainer_email='emilienghazal@gmail.com',
    description='TODO: Mission design',
    license='Apache 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "mission_manager = astromoon_missions.mission_manager:main",
            "mission_referee = astromoon_missions.mission_referee:main",
        ],
    },
)
