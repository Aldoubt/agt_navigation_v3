from setuptools import find_packages, setup


package_name = 'agt_capability_camera'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AGT',
    maintainer_email='contact@aldoubt.com',
    description='Camera capture capability for the Autolabor-C1 image stream.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'camera_capture_service = agt_capability_camera.camera_capture_service:main',
        ],
    },
)
