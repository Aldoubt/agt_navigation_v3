from setuptools import find_packages, setup

package_name = 'agt_map_manager'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/map_manager.yaml',
            'config/metadata.example.yaml',
        ]),
        ('share/' + package_name + '/launch', ['launch/map_manager.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AGT',
    maintainer_email='contact@aldoubt.com',
    description='Versioned AGT Map Package manager.',
    license='Apache-2.0',
    entry_points={'console_scripts': ['map_manager = agt_map_manager.map_manager:main']},
)
