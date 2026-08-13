from setuptools import find_packages, setup

package_name = 'safety_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='etw3',
    maintainer_email='alitaseeb@gmail.com',
    description='Safety and motor control nodes for lane following',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'estop_node = safety_nodes.estop_node:main',
            'lane_follower = safety_nodes.lane_follower:main',
            'experimental_lane_follower = safety_nodes.experimental_lane_follower:main',
        ],
    },
)
