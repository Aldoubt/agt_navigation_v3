from setuptools import find_packages, setup

package_name = 'agt_fastlio_adapter'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/adapter.yaml']),
        ('share/' + package_name + '/launch', ['launch/adapter.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AGT',
    maintainer_email='contact@aldoubt.com',
    description='Frame-checked FAST-LIO2 odometry adapter.',
    license='Apache-2.0',
    entry_points={'console_scripts': ['fastlio_adapter = agt_fastlio_adapter.fastlio_adapter:main']},
)
