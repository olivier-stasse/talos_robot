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


def generate_launch_description():
    enable_unloading_workaround_arg = DeclareLaunchArgument(
        'enable_unloading_workaround', default_value='False')

    # TO REVIEW
    extra_args = GroupAction([
        DeclareLaunchArgument(
            'extra_args',
            default_value='',
            description='Extra arguments for controller spawner node'),
        DeclareLaunchArgument(
            'extra_args_unloading_workaround',
            default_value='--shutdown-timeout 1.0',
            description='Extra arguments for controller spawner node if unloading workaround is enabled')
    ])

    pkg_share_folder = os.path.join(
        get_package_share_directory('talos_controller_configuration'), 'config')

    # Joint state controller
    joint_state_broadcaster_launch = generate_load_controller_launch_description(
        controller_name='joint_state_broadcaster',
        controller_type='joint_state_broadcaster/JointStateBroadcaster',
        controller_params_file=os.path.join(
            pkg_share_folder, 'joint_state_broadcaster.yaml')
    )
    # Force-torque sensor controller
    force_torque_sensor_controller_launch = generate_load_controller_launch_description(
        controller_name='force_torque_sensor_controller',
        controller_type='force_torque_sensor_controller/ForceTorqueSensorController',
        controller_params_file=os.path.join(
            get_package_share_directory('force_torque_sensor_controller'), 'force_torque_sensor_controller.yaml'))

    # IMU sensors controller
    imu_sensor_controller_launch = generate_load_controller_launch_description(
        controller_name='imu_sensor_controller',
        controller_type='imu_sensor_controller/ImuSensorController',
        controller_params_file=os.path.join(
            get_package_share_directory('imu_sensor_controller'),
            'imu_sensor_controller.yaml'))

    ld = LaunchDescription()

    ld.add_action(extra_args)

    ld.add_action(joint_state_broadcaster_launch)
    ld.add_action(enable_unloading_workaround_arg)
    ld.add_action(force_torque_sensor_controller_launch)
    ld.add_action(imu_sensor_controller_launch)

    return ld
