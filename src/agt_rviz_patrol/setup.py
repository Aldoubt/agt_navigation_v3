from setuptools import find_packages, setup

package_name = 'agt_rviz_patrol'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/front_sky_three_views.yaml']),
        ('share/' + package_name + '/launch', ['launch/rviz_patrol.launch.py']),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='AGT',
    maintainer_email='contact@aldoubt.com',
    description='Collect RViz goals, generate an inspection mission, and return home.',
    license='Apache-2.0',
    entry_points={'console_scripts': ['rviz_patrol = agt_rviz_patrol.rviz_patrol:main']},
)
