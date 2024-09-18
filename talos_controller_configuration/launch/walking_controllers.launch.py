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
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    pkg_share_folder = os.path.join(
        get_package_share_directory('talos_controller_configuration'), 'config')

    walking_params_config = DeclareLaunchArgument(
        'walking_params',
        default_value=os.path.join(pkg_share_folder, 'walking_params.yaml'),
        description='Walking parameters file'
    )

    walking_specific_offsets = DeclareLaunchArgument(
        'walking_specific_offsets',
        default_value=os.path.join(
            pkg_share_folder, 'walking_specific_offsets.yaml'),
        description='Walking specific offsets parameters file'
    )
    walking_specific_params = DeclareLaunchArgument(
        'walking_specific_params',
        default_value=os.path.join(
            pkg_share_folder, 'walking_specific_params.yaml'),
        description='Walking specific parameters file'
    )
    walking_controller_launch = generate_load_controller_launch_description(
        controller_name='walking_controller',
        controller_params_file=os.path.join(
            pkg_share_folder,
            'walking_controller.yaml'))

    ld = LaunchDescription()
    ld.add_action(walking_specific_offsets)
    ld.add_action(walking_specific_params)
    ld.add_action(walking_params_config)
    ld.add_action(walking_controller_launch)

    return ld
