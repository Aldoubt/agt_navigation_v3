from setuptools import find_packages, setup


package_name = 'agt_navigation_benchmark'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'configs/default.yaml',
            'configs/recorder_profile.yaml',
        ]),
        ('share/' + package_name + '/planner_benchmark/configs', [
            'planner_benchmark/configs/navfn.yaml',
            'planner_benchmark/configs/smac.yaml',
            'planner_benchmark/configs/theta_star.yaml',
        ]),
    ],
    install_requires=['setuptools', 'numpy', 'PyYAML', 'matplotlib', 'Pillow'],
    zip_safe=True,
    maintainer='AGT',
    maintainer_email='contact@aldoubt.com',
    description='Offline Nav2 rosbag analysis for planner, costmap and controller oscillation.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'analyze_nav_bag = agt_nav_benchmark.analyze_nav_bag:main',
        ],
    },
)
