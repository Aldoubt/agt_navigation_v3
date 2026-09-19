from setuptools import find_packages, setup


package_name = 'agt_localization_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/localization_shadow.launch.py']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='AGT',
    maintainer_email='contact@aldoubt.com',
    description='Shadow-only compatibility layer for AGT Localization v1.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'shadow_manager = agt_localization_ros.shadow_manager_node:main',
        ],
    },
)
