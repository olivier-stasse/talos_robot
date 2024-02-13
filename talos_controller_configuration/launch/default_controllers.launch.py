# Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_pal.include_utils import include_launch_py_description


def generate_launch_description():
    robot_model_arg = DeclareLaunchArgument(
        'robot', default_value='upper_body',
        description='Robot model'
    )
    walking_controllers_launch = include_launch_py_description(
        'talos_controller_configuration', [
            'launch', 'walking_controllers.launch.py'],
    )

    # TO COMPLETE
    # walk_pose_launch = include_launch_py_description(
    #     'talos_controller_configuration', [
    #         'launch', 'walk_pose.launch.py'],
    # )

    position_controllers_launch = include_launch_py_description(
        'talos_controller_configuration', [
            'launch', 'position_controllers.launch.py'],
        launch_arguments={robot_model_arg}
    )

    ld = LaunchDescription()

    ld.add_action(robot_model_arg)

    ld.add_action(walking_controllers_launch)
    ld.add_action(position_controllers_launch)
 #   ld.add_action(walk_pose_launch)

    return ld
