from setuptools import find_packages, setup

package_name = 'vision_nodes'

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
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'frame_saver = vision_nodes.frame_saver:main',
            'lane_offset_publisher = vision_nodes.lane_offset_publisher:main',
        ],
    },
)
