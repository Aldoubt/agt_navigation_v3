from glob import glob
from setuptools import setup
setup(name='agt_task_continuity', version='0.1.0', packages=['agt_task_continuity'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/agt_task_continuity']),
                  ('share/agt_task_continuity', ['package.xml']),
                  ('share/agt_task_continuity/config', glob('config/*.yaml')),
                  ('share/agt_task_continuity/launch', glob('launch/*.launch.py'))],
      install_requires=['setuptools'], zip_safe=True, maintainer='AGT',
      maintainer_email='dev@agt.local', description='Bounded research task continuity',
      license='Apache-2.0', entry_points={'console_scripts': [
          'task_continuity = agt_task_continuity.node:main']})
