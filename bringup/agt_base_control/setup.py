from setuptools import find_packages, setup

package_name = 'agt_base_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/cmd_vel_guard.yaml']),
        ('share/' + package_name + '/launch', ['launch/cmd_vel_guard.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AGT Robotics',
    maintainer_email='dev@agt.local',
    description='50 Hz velocity guard for the Bunker base.',
    license='Apache-2.0',
    entry_points={'console_scripts': ['cmd_vel_guard = agt_base_control.cmd_vel_guard:main']},
)
