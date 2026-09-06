from setuptools import find_packages, setup

package_name = 'agt_mapping_session'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mapping_session.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AGT',
    maintainer_email='contact@aldoubt.com',
    description='AGT mapping lifecycle session manager.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mapping_session = agt_mapping_session.mapping_session:main',
        ],
    },
)
