from setuptools import find_packages, setup

package_name = 'agt_map_converter'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'PyYAML'],
    zip_safe=True,
    maintainer='AGT',
    maintainer_email='contact@aldoubt.com',
    description='Convert PCD maps into Nav2 occupancy and terrain layers.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'pcd_to_nav_map = agt_map_converter.pcd_to_nav_map:main',
        'validate_nav_map = agt_map_converter.validate_nav_map:main',
    ]},
)
