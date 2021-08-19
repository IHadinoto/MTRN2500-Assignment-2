#!/usr/bin/env python3

import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

package_name = "assignment2"

rviz2_config =  os.path.join(
        get_package_share_directory(package_name),
        "launch", "rviz2_config.rviz"
    )

print("rviz2 config location: ", rviz2_config)
rviz2_launch_param = '\-\-display-config ' + rviz2_config

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package="rviz2",
            node_executable="rviz2",
            output="screen",
            arguments = [rviz2_launch_param],
        ),
    ])
