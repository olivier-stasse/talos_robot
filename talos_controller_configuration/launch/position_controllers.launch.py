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

from launch_pal.robot_utils import get_robot_name
from ament_index_python.packages import get_package_share_directory
from controller_manager.launch_utils import generate_load_controller_launch_description
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, LaunchConfigurationEquals, AnyCondition


def generate_launch_description():
    robot_model_arg = DeclareLaunchArgument(
        'robot_model', default_value='upper_body',
        description='Robot model'
    )
    enable_unloading_workaround_arg = DeclareLaunchArgument(
        'enable_unloading_workaround', default_value='False')

    pkg_share_folder = get_package_share_directory(
        'talos_controller_configuration')

    joint_state_broadcaster_launch = GroupAction(
        [generate_load_controller_launch_description(
            controller_name='joint_state_broadcaster',
            controller_type='joint_state_broadcaster/JointStateBroadcaster',
            controller_params_file=os.path.join(
                pkg_share_folder,
                'config/joint_trajectory_controllers', 'joint_state_broadcaster.yaml'))
         ],
        forwarding=False)

    torso_controller_launch = GroupAction(
        [
            generate_load_controller_launch_description(
                controller_name='torso_controller',
                controller_type='joint_trajectory_controller/JointTrajectoryController',
                controller_params_file=os.path.join(
                    pkg_share_folder,
                    'config/joint_trajectory_controllers', 'torso.yaml'
                )
            )
        ],
        forwarding=False,
        condition=IfCondition(
            AnyCondition([
                LaunchConfigurationEquals('robot_model', 'full_no_grippers'),
                LaunchConfigurationEquals('robot_model', 'lower_body_torso'),
                LaunchConfigurationEquals('robot_model', 'full_v2'),
                LaunchConfigurationEquals('robot_model', 'lower_body'),
                LaunchConfigurationEquals(
                    'robot_model', 'lower_body_torso_head'),
                LaunchConfigurationEquals('robot_model', 'torso_leg_right'),
                LaunchConfigurationEquals('robot_model', 'torso_leg_left'),
                LaunchConfigurationEquals('robot_model', 'upper_body'),
                LaunchConfigurationEquals(
                    'robot_model', 'upper_body_leg_left'),
                LaunchConfigurationEquals(
                    'robot_model', 'upper_body_leg_right'),
            ])
        )
    )
    head_controller_launch = GroupAction(
        [generate_load_controller_launch_description(
            controller_name='head_controller',
            controller_type='joint_trajectory_controller/JointTrajectoryController',
            controller_params_file=os.path.join(
                pkg_share_folder,
                'config/joint_trajectory_controllers', 'head_controller.yaml'))
         ],
        forwarding=False,
        condition=IfCondition(
            AnyCondition([
                LaunchConfigurationEquals('robot_model', 'full_no_grippers'),
                LaunchConfigurationEquals(
                    'robot_model', 'lower_body_head_arms'),
                LaunchConfigurationEquals('robot_model', 'full_v2'),
                LaunchConfigurationEquals(
                    'robot_model', 'lower_body_torso_head'),
                LaunchConfigurationEquals('robot_model', 'lower_body_head'),
                LaunchConfigurationEquals('robot_model', 'upper_body'),
                LaunchConfigurationEquals(
                    'robot_model', 'upper_body_leg_left'),
                LaunchConfigurationEquals(
                    'robot_model', 'upper_body_leg_right'),
            ])
        ))

    arm_right_controller_launch = GroupAction(
        [generate_load_controller_launch_description(
            controller_name='arm_right_controller',
            controller_type='joint_trajectory_controller/JointTrajectoryController',
            controller_params_file=os.path.join(
                pkg_share_folder,
                'config/joint_trajectory_controllers', 'arm_right_controller.yaml'))
         ],
        forwarding=False,
        condition=IfCondition(
            AnyCondition([
                LaunchConfigurationEquals('robot_model', 'full_no_grippers'),
                LaunchConfigurationEquals('robot_model', 'arm_right'),
                LaunchConfigurationEquals('robot_model', 'full_v2'),
                LaunchConfigurationEquals(
                    'robot_model', 'lower_body_head_arms'),
                LaunchConfigurationEquals(
                    'robot_model', 'upper_body_leg_left'),
                LaunchConfigurationEquals(
                    'robot_model', 'upper_body_leg_right'),
            ])
        )
    )

    arm_left_controller_launch = GroupAction(
        [generate_load_controller_launch_description(
            controller_name='arm_left_controller',
            controller_type='joint_trajectory_controller/JointTrajectoryController',
            controller_params_file=os.path.join(
                pkg_share_folder,
                'config/joint_trajectory_controllers', 'arm_left_controller.yaml'))
         ],
        forwarding=False,
        condition=IfCondition(
            AnyCondition([
                LaunchConfigurationEquals('robot_model', 'full_no_grippers'),
                LaunchConfigurationEquals('robot_model', 'full_v2'),
                LaunchConfigurationEquals(
                    'robot_model', 'lower_body_head_arms'),
                LaunchConfigurationEquals(
                    'robot_model', 'upper_body_leg_left'),
                LaunchConfigurationEquals(
                    'robot_model', 'upper_body_leg_right'),
            ])
        ))

    end_effector_right_controller_launch = GroupAction(
        [generate_load_controller_launch_description(
            controller_name='gripper_right_controller',
            controller_type='joint_trajectory_controller/JointTrajectoryController',
            controller_params_file=os.path.join(
                pkg_share_folder,
                'config/joint_trajectory_controllers', 'gripper_right_controller.yaml'))
         ],
        forwarding=False,
        condition=IfCondition(
            AnyCondition([
                LaunchConfigurationEquals(
                    'robot_model', 'lower_body_head_arms'),
                LaunchConfigurationEquals('robot_model', 'full_v2'),
                LaunchConfigurationEquals('robot_model', 'upper_body'),
                LaunchConfigurationEquals(
                    'robot_model', 'lower_body_head_arms'),
                LaunchConfigurationEquals(
                    'robot_model', 'upper_body_leg_left'),
                LaunchConfigurationEquals(
                    'robot_model', 'upper_body_leg_right'),
                LaunchConfigurationEquals(
                    'robot_model', 'arm_right'),
            ])
        )
    )

    end_effector_left_controller_launch = GroupAction(
        [generate_load_controller_launch_description(
            controller_name='gripper_left_controller',
            controller_type='joint_trajectory_controller/JointTrajectoryController',
            controller_params_file=os.path.join(
                pkg_share_folder,
                'config/joint_trajectory_controllers', 'gripper_left_controller.yaml'))
         ],
        forwarding=False,
        condition=IfCondition(
            AnyCondition([
                LaunchConfigurationEquals(
                    'robot_model', 'lower_body_head_arms'),
                LaunchConfigurationEquals('robot_model', 'full_v2'),
                LaunchConfigurationEquals('robot_model', 'upper_body'),
                LaunchConfigurationEquals(
                    'robot_model', 'lower_body_head_arms'),
                LaunchConfigurationEquals(
                    'robot_model', 'upper_body_leg_left'),
                LaunchConfigurationEquals(
                    'robot_model', 'upper_body_leg_right'),
            ])
        )
    )

    ld = LaunchDescription()

    ld.add_action(get_robot_name('talos'))

    ld.add_action(robot_model_arg)
    ld.add_action(enable_unloading_workaround_arg)

    ld.add_action(joint_state_broadcaster_launch)
    ld.add_action(torso_controller_launch)
    ld.add_action(head_controller_launch)
    ld.add_action(arm_right_controller_launch)
    ld.add_action(arm_left_controller_launch)
    ld.add_action(end_effector_right_controller_launch)
    ld.add_action(end_effector_left_controller_launch)

    return ld
