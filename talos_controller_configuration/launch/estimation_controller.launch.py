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

from launch import LaunchDescription

from launch.actions import TimerAction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

from controller_manager.launch_utils import generate_load_controller_launch_description
from pathlib import Path
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    estimation_controller_ros_yaml = str(
        Path(get_package_share_directory("talos_controller_configuration")) /
        "config" /
        "estimation_controller_ros.yaml"
    )

    est_ctrl_spawner = generate_load_controller_launch_description(
        controller_name='state_estimation_controller',
        controller_params_file=estimation_controller_ros_yaml
    )

    est_ctrl_bc_spawner = generate_load_controller_launch_description(
        controller_name='state_estimation_controller_broadcaster',
        controller_params_file=estimation_controller_ros_yaml
    )

    delay_est_ctrl_spawner_after_est_ctrl_bc_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=est_ctrl_bc_spawner.entities[2],
            on_exit=[TimerAction(
                period=2.0,
                actions=[est_ctrl_spawner],
            )],
        )
    )

    ld = LaunchDescription()
    ld.add_action(est_ctrl_bc_spawner)
    ld.add_action(delay_est_ctrl_spawner_after_est_ctrl_bc_spawner)
    return ld
