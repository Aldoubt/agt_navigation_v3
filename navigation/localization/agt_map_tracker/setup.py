from setuptools import find_packages, setup

package_name = 'agt_map_tracker'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/map_tracker.yaml']),
        ('share/' + package_name + '/launch', ['launch/map_tracker.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={'console_scripts': ['map_tracker = agt_map_tracker.map_tracker:main',
                                     'map_tracker_artifact_analyzer = agt_map_tracker.artifact_analyzer:main']},
)
