from setuptools import find_packages, setup

package_name = 'teleop_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ETW III',
    maintainer_email='TODO@example.com',
    description=(
        'Bridges geometry_msgs/Twist (e.g. from teleop_twist_keyboard) into '
        'freenove_driver.motor calls, with a watchdog stop if /cmd_vel goes quiet.'
    ),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_bridge = teleop_bridge.cmd_vel_bridge:main',
        ],
    },
)
