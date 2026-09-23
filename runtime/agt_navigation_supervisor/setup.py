from setuptools import setup

setup(
    name='agt_navigation_supervisor', version='0.4.0',
    packages=['agt_navigation_supervisor'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/agt_navigation_supervisor']),
        ('share/agt_navigation_supervisor', ['package.xml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='AGT', maintainer_email='contact@aldoubt.com',
    description='Navigation V4 health supervisor', license='Apache-2.0',
    entry_points={'console_scripts': [
        'navigation_supervisor = agt_navigation_supervisor.supervisor:main',
        'wait_navigation_ready = agt_navigation_supervisor.wait_ready:main',
    ]},
)
