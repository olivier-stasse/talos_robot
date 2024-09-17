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
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_pal.robot_arguments import CommonArgs


def generate_launch_description():

    pkg_share_folder = get_package_share_directory(
        'talos_controller_configuration')

    torso_controller_launch = GroupAction(
        [
            generate_load_controller_launch_description(
                controller_name='torso_controller',
                controller_params_file=os.path.join(
                    pkg_share_folder,
                    'config/joint_trajectory_controllers', 'torso_controller.yaml'
                )
            )
        ],
        forwarding=False,
        condition=IfCondition(
            PythonExpression(
                ["'", LaunchConfiguration('robot_model'), "' == 'full_no_grippers' or '",
                 LaunchConfiguration(
                     'robot_model'), "' == 'lower_body_torso' or '",
                 LaunchConfiguration('robot_model'), "' == 'full_v2' or '",
                 LaunchConfiguration(
                     'robot_model'), "' == 'lower_body_torso_head' or '",
                 LaunchConfiguration(
                     'robot_model'), "' == 'torso_leg_right' or '",
                 LaunchConfiguration(
                     'robot_model'), "' == 'torso_leg_left' or '",
                 LaunchConfiguration(
                     'robot_model'), "' == 'upper_body' or '",
                 LaunchConfiguration(
                     'robot_model'), "' == 'upper_body_leg_left' or '",
                 LaunchConfiguration(
                     'robot_model'), "' == 'upper_body_leg_right'"
                 ]
            )
        )
    )

    leg_left_controller_launch = GroupAction(
        [
            generate_load_controller_launch_description(
                controller_name='leg_left_controller',
                controller_params_file=os.path.join(
                    pkg_share_folder,
                    'config/joint_trajectory_controllers', 'leg_left_controller.yaml'
                )
            )
        ],
        forwarding=False,
        condition=IfCondition(
            PythonExpression(
                ["'", LaunchConfiguration('robot_model'), "' == 'full_no_grippers' or '",
                 LaunchConfiguration(
                     'robot_model'), "' == 'lower_body_torso' or '",
                 LaunchConfiguration('robot_model'), "' == 'full_v2' or '",
                 LaunchConfiguration(
                     'robot_model'), "' == 'lower_body' or '",
                 LaunchConfiguration(
                     'robot_model'), "' == 'lower_body_torso_head' or '",
                 LaunchConfiguration(
                     'robot_model'), "' == 'torso_leg_left' or '",
                 LaunchConfiguration(
                     'robot_model'), "' == 'upper_body_leg_left'"
                 ]
            )
        )
    )

    leg_right_controller_launch = GroupAction(
        [
            generate_load_controller_launch_description(
                controller_name='leg_right_controller',
                controller_params_file=os.path.join(
                    pkg_share_folder,
                    'config/joint_trajectory_controllers', 'leg_right_controller.yaml'
                )
            )
        ],
        forwarding=False,
        condition=IfCondition(
            PythonExpression(
                ["'", LaunchConfiguration('robot_model'), "' == 'full_no_grippers' or '",
                 LaunchConfiguration(
                     'robot_model'), "' == 'lower_body_torso' or '",
                 LaunchConfiguration('robot_model'), "' == 'full_v2' or '",
                 LaunchConfiguration(
                     'robot_model'), "' == 'lower_body' or '",
                 LaunchConfiguration(
                     'robot_model'), "' == 'lower_body_torso_head' or '",
                 LaunchConfiguration(
                     'robot_model'), "' == 'torso_leg_right' or '",
                 LaunchConfiguration(
                     'robot_model'), "' == 'upper_body_leg_right'"
                 ]
            )
        )
    )

    head_controller_launch = GroupAction(
        [generate_load_controller_launch_description(
            controller_name='head_controller',
            controller_params_file=os.path.join(
                pkg_share_folder,
                'config/joint_trajectory_controllers', 'head_controller.yaml'))
         ],
        forwarding=False,
        condition=IfCondition(
            PythonExpression(
                ["'", LaunchConfiguration('robot_model'), "' == 'full_no_grippers' or '",
                 LaunchConfiguration(
                     'robot_model'), "' == 'lower_body_head_arms' or '",
                 LaunchConfiguration('robot_model'), "' == 'full_v2' or '",
                 LaunchConfiguration(
                     'robot_model'), "' == 'lower_body_torso_head' or '",
                 LaunchConfiguration(
                     'robot_model'), "' == 'lower_body_head' or '",
                 LaunchConfiguration(
                     'robot_model'), "' == 'upper_body' or '",
                 LaunchConfiguration(
                     'robot_model'), "' == 'upper_body_leg_left' or '",
                 LaunchConfiguration(
                     'robot_model'), "' == 'upper_body_leg_right'"
                 ]
            )
        )
    )

    arm_right_controller_launch = GroupAction(
        [generate_load_controller_launch_description(
            controller_name='arm_right_controller',
            controller_params_file=os.path.join(
                pkg_share_folder,
                'config/joint_trajectory_controllers', 'arm_right_controller.yaml'))
         ],
        forwarding=False,
        condition=IfCondition(
            PythonExpression(
                ["'", LaunchConfiguration('robot_model'), "' == 'full_no_grippers' or '",
                 LaunchConfiguration(
                     'robot_model'), "' == 'arm_right' or '",
                 LaunchConfiguration('robot_model'), "' == 'full_v2' or '",
                 LaunchConfiguration(
                     'robot_model'), "' == 'lower_body_head_arms' or '",
                 LaunchConfiguration(
                     'robot_model'), "' == 'upper_body_leg_left' or '",
                 LaunchConfiguration(
                     'robot_model'), "' == 'upper_body_leg_right'",
                 ]
            )
        )
    )

    arm_left_controller_launch = GroupAction(
        [generate_load_controller_launch_description(
            controller_name='arm_left_controller',
            controller_params_file=os.path.join(
                pkg_share_folder,
                'config/joint_trajectory_controllers', 'arm_left_controller.yaml'))
         ],
        forwarding=False,
        condition=IfCondition(
            PythonExpression(
                ["'", LaunchConfiguration('robot_model'), "' == 'full_no_grippers' or '",
                 LaunchConfiguration('robot_model'), "' == 'full_v2' or '",
                 LaunchConfiguration(
                     'robot_model'), "' == 'lower_body_head_arms' or '",
                 LaunchConfiguration(
                     'robot_model'), "' == 'upper_body_leg_left' or '",
                 LaunchConfiguration(
                     'robot_model'), "' == 'upper_body_leg_right'",
                 ]
            )
        )
    )

    end_effector_right_controller_launch = GroupAction(
        [generate_load_controller_launch_description(
            controller_name='gripper_right_controller',
            controller_params_file=os.path.join(
                pkg_share_folder,
                'config/joint_trajectory_controllers', 'gripper_right_controller.yaml'))
         ],
        forwarding=False,
        condition=IfCondition(
            PythonExpression(
                ["'", LaunchConfiguration('robot_model'), "' == 'upper_body' or '",
                 LaunchConfiguration('robot_model'), "' == 'full_v2' or '",
                 LaunchConfiguration(
                     'robot_model'), "' == 'lower_body_head_arms' or '",
                 LaunchConfiguration(
                     'robot_model'), "' == 'upper_body_leg_left' or '",
                 LaunchConfiguration(
                     'robot_model'), "' == 'upper_body_leg_right' or '",
                 LaunchConfiguration(
                     'robot_model'), "' == 'arm_right'",
                 ]
            )
        )
    )

    end_effector_left_controller_launch = GroupAction(
        [generate_load_controller_launch_description(
            controller_name='gripper_left_controller',
            controller_params_file=os.path.join(
                pkg_share_folder,
                'config/joint_trajectory_controllers', 'gripper_left_controller.yaml'))
         ],
        forwarding=False,
        condition=IfCondition(
            PythonExpression(
                ["'", LaunchConfiguration('robot_model'), "' == 'upper_body' or '",
                 LaunchConfiguration('robot_model'), "' == 'full_v2' or '",
                 LaunchConfiguration(
                     'robot_model'), "' == 'lower_body_head_arms' or '",
                 LaunchConfiguration(
                     'robot_model'), "' == 'upper_body_leg_left' or '",
                 LaunchConfiguration(
                     'robot_model'), "' == 'upper_body_leg_right'",
                 ]
            )
        )
    )

    ld = LaunchDescription()

    ld.add_action(CommonArgs.robot_name)

    ld.add_action(torso_controller_launch)
    ld.add_action(head_controller_launch)
    ld.add_action(arm_right_controller_launch)
    ld.add_action(arm_left_controller_launch)
    ld.add_action(leg_left_controller_launch)
    ld.add_action(leg_right_controller_launch)
    ld.add_action(end_effector_right_controller_launch)
    ld.add_action(end_effector_left_controller_launch)

    return ld
