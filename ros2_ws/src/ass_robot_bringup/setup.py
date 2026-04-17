from setuptools import setup

package_name = 'ass_robot_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'cmd_vel_bridge = ass_robot_bringup.cmd_vel_bridge:main',
            'path_follower = ass_robot_bringup.path_follower:main',
        ],
    },
)
