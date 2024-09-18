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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_pal.include_utils import include_launch_py_description
from launch.conditions import UnlessCondition
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    default_controllers = include_launch_py_description(
        pkg_name='talos_controller_configuration',
        paths=['launch', 'talos_default_controllers.launch.py'])

    full_body_position_controllers = include_launch_py_description(
        pkg_name='talos_controller_configuration',
        paths=['launch', 'full_body_position_controllers.launch.py'])

    walking_controller = include_launch_py_description(
        pkg_name='talos_walking',
        paths=['launch', 'walking_controller.launch.py'])

    ld = LaunchDescription()
    ld.add_action(default_controllers)
    ld.add_action(full_body_position_controllers)
    ld.add_action(walking_controller)

    return ld
