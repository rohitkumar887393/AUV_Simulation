from setuptools import find_packages, setup

package_name = 'auv_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    
        ('share/auv_bringup/launch',
            ['launch/sensors.launch.py',
             'launch/teleop.launch.py',
             'launch/autonomy.launch.py']),
        ('share/auv_bringup/config',
            ['config/control_params.yaml']),

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
         'teleop = auv_bringup.teleop:main',

        ],
    },
)
