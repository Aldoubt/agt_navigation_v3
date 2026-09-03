from setuptools import find_packages, setup

package_name = 'agt_bunker_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/command_guard.yaml']),
        ('share/' + package_name + '/launch', ['launch/command_guard.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AGT',
    maintainer_email='contact@aldoubt.com',
    description='50 Hz command guard and watchdog for Bunker tracked base.',
    license='Apache-2.0',
    entry_points={'console_scripts': ['command_guard = agt_bunker_control.command_guard:main']},
)
