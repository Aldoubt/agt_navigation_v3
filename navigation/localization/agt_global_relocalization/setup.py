from setuptools import find_packages, setup

package_name = 'agt_global_relocalization'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/global_relocalization.yaml']),
        ('share/' + package_name + '/launch', ['launch/global_relocalization.launch.py']),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='AGT',
    maintainer_email='contact@aldoubt.com',
    description='External 3D map localization SDK adapter for AGT.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'global_relocalization = agt_global_relocalization.global_relocalization:main',
    ]},
)
