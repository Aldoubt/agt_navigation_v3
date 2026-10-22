from glob import glob
from setuptools import find_packages, setup


package_name = 'agt_operator_console'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AGT',
    maintainer_email='contact@aldoubt.com',
    description='Interactive operator orchestration for AGT field modes.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'operator_console = agt_operator_console.operator_console:main',
            'field_test_gui = agt_operator_console.field_test_gui:main',
            'agt_field_console = agt_operator_console.field_console:main',
            'replay_audit = agt_operator_console.replay_audit:main',
            'deterministic_seed_injector = agt_operator_console.deterministic_seed_injector:main',
            'pose_chain_recorder = agt_operator_console.pose_chain_recorder:main',
        ],
    },
)
