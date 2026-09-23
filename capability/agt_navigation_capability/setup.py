from setuptools import setup

setup(
    name='agt_navigation_capability', version='0.4.0',
    packages=['agt_navigation_capability'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/agt_navigation_capability']),
        ('share/agt_navigation_capability', ['package.xml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='AGT', maintainer_email='contact@aldoubt.com',
    description='Guarded Navigation V4 capability action', license='Apache-2.0',
    entry_points={'console_scripts': [
        'navigation_capability = agt_navigation_capability.capability:main',
    ]},
)
