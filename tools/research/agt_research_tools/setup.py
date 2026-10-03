from setuptools import find_packages, setup
from glob import glob
setup(name='agt_research_tools', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/agt_research_tools']),
                  ('share/agt_research_tools', ['package.xml']),
                  ('share/agt_research_tools/config', glob('config/*.yaml'))],
      install_requires=['setuptools', 'PyYAML'], zip_safe=True,
      maintainer='AGT', maintainer_email='contact@aldoubt.com', license='Apache-2.0',
      entry_points={'console_scripts': [
          'freeze_experiment = agt_research_tools.manifest:freeze_main',
          'verify_experiment = agt_research_tools.manifest:verify_main',
          'export_row_bag = agt_research_tools.bag_export:main',
          'evaluate_continuity = agt_research_tools.evaluate:main',
          'geometry_from_urdf = agt_research_tools.geometry:main',
          'build_stable_submap = agt_research_tools.stable_submap:main',
          'record_experiment = agt_research_tools.recorder:main',
          'inject_localization_fault = agt_research_tools.fault_injector:main']})
