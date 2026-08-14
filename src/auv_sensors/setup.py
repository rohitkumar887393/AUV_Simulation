from setuptools import find_packages, setup

package_name = 'auv_sensors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='coratia',
    maintainer_email='ec23b1065@iiitdm.ac.in',
    description='AUV Sensor Nodes',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_node = auv_sensors.imu_node:main',
            'depth_sensor = auv_sensors.depth_sensor:main',
            'leak_sensor = auv_sensors.leak_sensor:main',
            'battery_monitor = auv_sensors.battery_monitor:main',
            'dvl_sensor = auv_sensors.dvl_sensor:main',
        ],
    },
)
