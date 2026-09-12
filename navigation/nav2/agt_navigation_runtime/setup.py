from setuptools import find_packages, setup

package_name = 'agt_navigation_runtime'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/runtime.yaml', 'config/mission_example.yaml']),
        ('share/' + package_name + '/launch', ['launch/runtime.launch.py']),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='AGT Robotics',
    maintainer_email='dev@agt.local',
    description='AGT inspection mission runtime',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'mission_runtime = agt_navigation_runtime.mission_runtime:main',
        'validate_records = agt_navigation_runtime.validate_records:main',
        'generate_demo_report = agt_navigation_runtime.generate_demo_report:main',
        'demo_preflight = agt_navigation_runtime.demo_preflight:main',
    ]},
)
