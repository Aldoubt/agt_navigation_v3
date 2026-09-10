from setuptools import find_packages, setup


package_name = 'agt_demo_task'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/navigation_debug.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AGT',
    maintainer_email='contact@aldoubt.com',
    description='Trigger a camera capture after RViz NavigateToPose success.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'navigate_capture_task = agt_demo_task.navigate_capture_task:main',
        ],
    },
)
