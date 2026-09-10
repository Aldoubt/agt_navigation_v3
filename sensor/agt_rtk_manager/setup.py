from setuptools import find_packages, setup

package_name = 'agt_rtk_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/rtk_manager.yaml', 'config/map_origin.example.yaml']),
        ('share/' + package_name + '/launch', ['launch/rtk_manager.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AGT Robotics',
    maintainer_email='dev@agt.local',
    description='Quality-gated RTK/INS integration manager for AGT navigation.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'rtk_manager = agt_rtk_manager.rtk_manager:main',
        ],
    },
)
