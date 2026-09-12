from setuptools import find_packages, setup

package_name = 'agt_relocalization_benchmark'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/relocalization_sweep.yaml']),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='AGT',
    maintainer_email='contact@aldoubt.com',
    description='Capture mapping replay queries and sweep relocalization parameters.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'capture_cases = agt_relocalization_benchmark.capture_cases:main',
            'import_pgo_cases = agt_relocalization_benchmark.import_pgo_cases:main',
            'sweep = agt_relocalization_benchmark.sweep:main',
        ],
    },
)
