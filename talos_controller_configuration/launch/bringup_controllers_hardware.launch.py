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
from launch_pal.include_utils import include_launch_py_description


def generate_launch_description():

    pkg_share_folder = os.path.join(
        get_package_share_directory('talos_controller_configuration'), 'config')

    # Joint torque state controller
    joint_torque_sensor_launch = generate_load_controller_launch_description(
        controller_name='joint_torque_sensor_state_controller',
        controller_type='joint_torque_sensor_state_controller/JointStateTorqueSensorController',
        controller_params_file=os.path.join(
            get_package_share_directory('joint_torque_sensor_state_controller'), 'joint_torque_sensor_state_controller.yaml')
    )
    # Temperature sensor controller
    temperature_sensor_controller_launch = generate_load_controller_launch_description(
        controller_name='temperature_sensor_controller',
        controller_type='temperature_sensor_controller/TemperatureSensorController',
        controller_params_file=os.path.join(
            get_package_share_directory('temperature_sensor_controller'), 'temperature_sensor_controller.yaml'))

    # Mode state controller
    mode_state_controller_launch = generate_load_controller_launch_description(
        controller_name='mode_state_controller',
        controller_type='mode_state_controller/ModeStateController',
        controller_params_file=os.path.join(
            get_package_share_directory('mode_state_controller'),
            'mode_state_controller.yaml'))

    current_limit_controllers_launch = include_launch_py_description(
        'talos_controller_configuration', [
            'launch', 'current_limit_controllers.launch.py'],
    )

    ld = LaunchDescription()

    ld.add_action(joint_torque_sensor_launch)
    ld.add_action(temperature_sensor_controller_launch)
    ld.add_action(mode_state_controller_launch)
    ld.add_action(current_limit_controllers_launch)

    return ld
