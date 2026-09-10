from setuptools import find_packages, setup

package_name = 'agt_batch_lio_adapter'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/batch_lio_adapter.yaml']),
        ('share/' + package_name + '/launch', ['launch/batch_lio_adapter.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AGT',
    maintainer_email='contact@aldoubt.com',
    description='Batch-LIO to AGT canonical odometry frame adapter',
    license='Apache-2.0',
    entry_points={'console_scripts': ['batch_lio_adapter = agt_batch_lio_adapter.batch_lio_adapter:main']},
)
