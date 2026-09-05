from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'agt_gazebo_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AGT',
    maintainer_email='contact@aldoubt.com',
    description='Gazebo Classic integration harness for AGT mapping and navigation.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'imu_unit_adapter = agt_gazebo_sim.imu_unit_adapter:main',
        'mapping_drive = agt_gazebo_sim.mapping_drive:main',
        'nav_goal_probe = agt_gazebo_sim.nav_goal_probe:main',
    ]},
)
