from setuptools import setup
from glob import glob

setup(
    name="agt_mission_executor",
    version="1.0.0",
    packages=["agt_mission_executor"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/agt_mission_executor"]),
        ("share/agt_mission_executor", ["package.xml"]),
        ("share/agt_mission_executor/launch", glob("launch/*.py")),
    ],
    entry_points={"console_scripts": ["mission_executor=agt_mission_executor.entry:main"]},
)
