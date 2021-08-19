#!/usr/bin/env python3

import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

package_name = "assignment2"
game_master_namespace = "/z0000000"

joy_node_config_yaml =  os.path.join(
        get_package_share_directory(package_name),
        "launch", "joy_node_config.yaml"
    )
print("config.yaml location: ", joy_node_config_yaml);
def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package="joy",
            node_executable="joy_node",
	    parameters=[joy_node_config_yaml],
            output="screen",
            node_namespace = game_master_namespace,
        ),
    ])
