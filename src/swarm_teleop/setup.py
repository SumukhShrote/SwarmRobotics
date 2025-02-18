from setuptools import find_packages, setup

package_name = 'swarm_teleop'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/swarm_teleop.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='janhavi',
    maintainer_email='janhavic.2604@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'velocity_control = swarm_teleop.velocity_control:main',
            'teleop_control = swarm_teleop.teleop_control:main',
        ],
    },
)
