from setuptools import setup

package_name = 'agt_map_runtime_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/runtime_bridge.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'map_runtime_bridge = agt_map_runtime_bridge.runtime_bridge_node:main',
        ],
    },
)
