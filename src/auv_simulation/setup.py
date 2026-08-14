from setuptools import find_packages, setup

package_name = 'auv_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/auv_simulation/launch',
            ['launch/simulation.launch.py']),
        ('share/auv_simulation/config',
            ['config/ideal_auv.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='coratia',
    maintainer_email='ec23b1065@iiitdm.ac.in',
    description='AUV Simulation Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'actuator_simulator = auv_simulation.actuator_simulator:main',
            'ideal_auv = auv_simulation.ideal_auv:main',
            'sensor_simulator = auv_simulation.sensor_simulator:main',
            'visualizer = auv_simulation.visualizer:main',
            'visualizer_3d = auv_simulation.visualizer_3d:main',
            'web_visualizer = auv_simulation.web_visualizer:main',
        ],
    },
)
