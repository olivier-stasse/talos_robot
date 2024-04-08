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
from launch.actions import GroupAction
from ament_index_python.packages import get_package_share_directory
from controller_manager.launch_utils import generate_load_controller_launch_description
from launch.actions import DeclareLaunchArgument
from launch_pal.include_utils import include_launch_py_description
import yaml


def generate_launch_description():

    pkg_share_folder = os.path.join(
        get_package_share_directory('talos_controller_configuration'), 'config')

    # Joint state controller
    joint_state_broadcaster_launch = generate_load_controller_launch_description(
        controller_name='joint_state_broadcaster',
        controller_type='joint_state_broadcaster/JointStateBroadcaster',
        controller_params_file=os.path.join(
            pkg_share_folder, 'joint_state_broadcaster.yaml')
    )

    # Force-torque sensors controller for the wrists
    force_torque_sensor_left_launch = generate_load_controller_launch_description(
        controller_name='ft_sensor_left_controller',
        controller_type='force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster',
        controller_params_file=os.path.join(
            pkg_share_folder,
            'ft_sensor_left_controller.yaml'))

    force_torque_sensor_right_launch = generate_load_controller_launch_description(
        controller_name='ft_sensor_right_controller',
        controller_type='force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster',
        controller_params_file=os.path.join(
            pkg_share_folder,
            'ft_sensor_right_controller.yaml'))

    # IMU sensors controller
    imu_sensor_controller_launch = generate_load_controller_launch_description(
        controller_name='imu_sensor_broadcaster',
        controller_type='imu_sensor_broadcaster/IMUSensorBroadcaster',
        controller_params_file=os.path.join(
            pkg_share_folder,
            'imu_sensor_broadcaster.yaml'))

    ld = LaunchDescription()

    ld.add_action(joint_state_broadcaster_launch)
    ld.add_action(force_torque_sensor_right_launch)
    ld.add_action(force_torque_sensor_left_launch)
    # TODO imu_sensor
    # ld.add_action(imu_sensor_controller_launch)

    return ld
