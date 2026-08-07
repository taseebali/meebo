from setuptools import find_packages, setup

package_name = 'freenove_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    package_data={package_name: ['NOTICE.md']},
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ETW III',
    maintainer_email='TODO@example.com',
    description=(
        'Vendored Freenove 4WD Smart Car hardware driver (motor, PCA9685, '
        'ultrasonic, servo), wrapped as an ament_python package so ROS 2 '
        'nodes can import it directly. See freenove_driver/NOTICE.md.'
    ),
    license='CC-BY-NC-SA-3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
