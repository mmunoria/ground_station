"""
Launch file for the ground_station command GUI.

    ros2 launch ground_station command_gui.launch.py
    ros2 launch ground_station command_gui.launch.py \
        config_file:=/path/to/your/drones.yaml

    # Or skip the config file entirely and give the domains/namespaces you
    # want directly on the command line (YAML list syntax, no spaces needed
    # around commas but quote the whole thing so the shell leaves it alone):
    ros2 launch ground_station command_gui.launch.py \
        domains:="[1,2]" namespaces:="[drone1,drone2]"

By default this loads config/drones.yaml (installed alongside this package),
which supplies the drone_domains / drone_namespaces parameters that
command_gui.py (and command_node.py) read at startup -- see that file's
comments for the expected format. The `domains` / `namespaces` launch
arguments, when non-empty, override whatever config_file provides -- handy
for pointing the GUI at a different subset of drones without editing or
copying drones.yaml.

`ros2 run ground_station command_gui` still works for a quick manual test
with default (no-domain, no-namespace) parameters, but it does not load
drones.yaml -- use this launch file whenever you want the GUI wired up to
your configured fleet.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    config_file = LaunchConfiguration('config_file').perform(context)
    domains_text = LaunchConfiguration('domains').perform(context).strip()
    namespaces_text = LaunchConfiguration('namespaces').perform(context).strip()

    parameters = [config_file]

    overrides = {}
    if domains_text:
        overrides['drone_domains'] = yaml.safe_load(domains_text)
    if namespaces_text:
        overrides['drone_namespaces'] = yaml.safe_load(namespaces_text)
    if overrides:
        parameters.append(overrides)

    command_gui_node = Node(
        package='ground_station',
        executable='command_gui',
        name='command_gui',
        output='screen',
        parameters=parameters,
    )
    return [command_gui_node]


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('ground_station'), 'config', 'drones.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='YAML file providing drone_domains / drone_namespaces (see config/drones.yaml).',
        ),
        DeclareLaunchArgument(
            'domains',
            default_value='',
            description='Override drone_domains, e.g. "[1,2,3]". Leave empty to use config_file.',
        ),
        DeclareLaunchArgument(
            'namespaces',
            default_value='',
            description='Override drone_namespaces, e.g. "[drone1,drone2,drone3]". Leave empty to use config_file.',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
