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
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_pal.include_utils import include_launch_py_description
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    talos_common_hardware_path = os.path.join(
        get_package_share_directory('talos_bringup'), 'config', 'talos_common_hardware.yaml')

    declare_hardware_comp = DeclareLaunchArgument(
        'talos_common_hardware', default_value=talos_common_hardware_path,
        description='Hardware configuration file')

    bringup_controllers = include_launch_py_description(
        pkg_name='talos_controller_configuration',
        paths=['launch', 'bringup_controllers.launch.py'])

    play_motion2 = include_launch_py_description(
        "talos_bringup", ["launch", "talos_play_motion2.launch.py"]
    )

    twist_mux = include_launch_py_description(
        "talos_bringup", ["launch", "twist_mux.launch.py"]
    )

    talos_state_publisher = include_launch_py_description(
        "talos_description", ["launch", "robot_state_publisher.launch.py"],
    )

    bringup_controllers_hardware = include_launch_py_description(
        "talos_controller_configuration", [
            "launch", "bringup_controllers_hardware.launch.py"],
    )

    # movegroup launch passing the decription

    ld = LaunchDescription()
    ld.add_action(talos_state_publisher)

   # ld.add_action(declare_hardware_comp)
    ld.add_action(bringup_controllers)
  #  ld.add_action(bringup_controllers_hardware)
    ld.add_action(play_motion2)
    # ld.add_action(twist_mux)

    return ld
