from setuptools import find_packages, setup

package_name = 'auv_autonomy'

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
    maintainer='coratia',
    maintainer_email='ec23b1065@iiitdm.ac.in',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [

            'depth_hold = auv_autonomy.depth_hold:main',
            'roll_stabilizer = auv_autonomy.roll_stabilizer:main',
            'heading_hold = auv_autonomy.heading_hold:main',

            'distance_hold = auv_autonomy.distance_hold:main',

            'mission_manager = auv_autonomy.mission_manager:main',
            'inspection_mission = auv_autonomy.inspection_mission:main',
        ],
    },
)

