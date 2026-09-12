from setuptools import find_packages, setup

package_name = 'agt_localization_manager'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/localization_manager.yaml']),
        ('share/' + package_name + '/launch', ['launch/localization_manager.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AGT',
    maintainer_email='contact@aldoubt.com',
    description='Single-owner map-to-odom localization manager.',
    license='Apache-2.0',
    entry_points={'console_scripts': ['localization_manager = agt_localization_manager.localization_manager:main']},
)
