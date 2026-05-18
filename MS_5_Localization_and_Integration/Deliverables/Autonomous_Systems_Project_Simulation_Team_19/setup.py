import os
import glob
from setuptools import setup

package_name = 'autonomous_systems_project_team_19'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'Worlds'), glob.glob('Worlds/*.world')),
    ],
    install_requires=['setuptools', 'numpy', 'matplotlib'],
    zip_safe=True,
    maintainer='Team 19',
    maintainer_email='team19@example.com',
    description='MS5 Gazebo simulation package for tracks 1, 2 and 3.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'Autonomous_Systems_MS_5_Localization_Team_19 = autonomous_systems_project_team_19.Autonomous_Systems_MS_5_Localization_Team_19:main',
            'Autonomous_Systems_MS_5_Planning_Team_19 = autonomous_systems_project_team_19.Autonomous_Systems_MS_5_Planning_Team_19:main',
            'Autonomous_Systems_MS_5_Speed_Controller_Team_19 = autonomous_systems_project_team_19.Autonomous_Systems_MS_5_Speed_Controller_Team_19:main',
            'Autonomous_Systems_MS_5_Stanley_Lateral_Team_19 = autonomous_systems_project_team_19.Autonomous_Systems_MS_5_Stanley_Lateral_Team_19:main',
            'Autonomous_Systems_MS_5_Gazebo_Command_Mux_Team_19 = autonomous_systems_project_team_19.Autonomous_Systems_MS_5_Gazebo_Command_Mux_Team_19:main',
            'Autonomous_Systems_MS_5_Graph_Recorder_Team_19 = autonomous_systems_project_team_19.Autonomous_Systems_MS_5_Graph_Recorder_Team_19:main',
            'Autonomous_Systems_MS_5_Plot_Graphs_Team_19 = autonomous_systems_project_team_19.Autonomous_Systems_MS_5_Plot_Graphs_Team_19:main',
        ],
    },
)
