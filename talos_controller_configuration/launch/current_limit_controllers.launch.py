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
from ament_index_python.packages import get_package_share_directory
from controller_manager.launch_utils import generate_load_controller_launch_description


def generate_launch_description():

    yaml_file = os.path.join(
        get_package_share_directory('talos_controller_configuration'), 'config', 'current_limit_controllers.yaml')

    torso_limit_controller_launch = generate_load_controller_launch_description(
        controller_name='torso_current_limit_controller',
        controller_type='pal_ros_controllers/CurrentLimitController',
        controller_params_file=yaml_file
    )

    head_limit_controller_launch = generate_load_controller_launch_description(
        controller_name='head_current_limit_controller',
        controller_type='pal_ros_controllers/CurrentLimitController',
        controller_params_file=yaml_file
    )
    arm_left_limit_controller_launch = generate_load_controller_launch_description(
        controller_name='arm_left_current_limit_controller',
        controller_type='pal_ros_controllers/CurrentLimitController',
        controller_params_file=yaml_file
    )

    arm_right_limit_controller_launch = generate_load_controller_launch_description(
        controller_name='arm_right_current_limit_controller',
        controller_type='pal_ros_controllers/CurrentLimitController',
        controller_params_file=yaml_file
    )

    gripper_left_limit_controller_launch = generate_load_controller_launch_description(
        controller_name='gripper_left_current_limit_controller',
        controller_type='pal_ros_controllers/CurrentLimitController',
        controller_params_file=yaml_file
    )

    gripper_right_limit_controller_launch = generate_load_controller_launch_description(
        controller_name='gripper_right_current_limit_controller',
        controller_type='pal_ros_controllers/CurrentLimitController',
        controller_params_file=yaml_file
    )

    leg_left_limit_controller_launch = generate_load_controller_launch_description(
        controller_name='leg_left_current_limit_controller',
        controller_type='pal_ros_controllers/CurrentLimitController',
        controller_params_file=yaml_file
    )

    leg_right_limit_controller_launch = generate_load_controller_launch_description(
        controller_name='leg_right_current_limit_controller',
        controller_type='pal_ros_controllers/CurrentLimitController',
        controller_params_file=yaml_file
    )

    ld = LaunchDescription()

    ld.add_action(torso_limit_controller_launch)
    ld.add_action(head_limit_controller_launch)
    ld.add_action(arm_left_limit_controller_launch)
    ld.add_action(arm_right_limit_controller_launch)
    ld.add_action(gripper_left_limit_controller_launch)
    ld.add_action(gripper_right_limit_controller_launch)
    ld.add_action(leg_left_limit_controller_launch)
    ld.add_action(leg_right_limit_controller_launch)

    return ld
