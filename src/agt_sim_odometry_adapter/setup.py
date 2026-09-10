from setuptools import setup

package_name = 'agt_sim_odometry_adapter'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sim_odometry_adapter.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AGT',
    maintainer_email='contact@aldoubt.com',
    description='Normalizes Gazebo truth odometry to the AGT local-odometry contract.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'sim_odometry_adapter = agt_sim_odometry_adapter.sim_odometry_adapter:main',
    ]},
)
