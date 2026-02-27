# Copyright 2025 Enactic, Inc.
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

"""
MoveIt2 demo launch for OpenArm single arm (no gripper).

Starts ros2_control bringup (robot_state_publisher + controller_manager +
joint_state_broadcaster + joint_trajectory_controller) and then MoveIt2
move_group + RViz2 for trajectory planning on the real arm.

Usage (real hardware, CAN 2.0, can0):
  ros2 launch openarm_single_moveit_config demo.launch.py

Usage (fake/simulation):
  ros2 launch openarm_single_moveit_config demo.launch.py use_fake_hardware:=true

Override CAN interface or mode:
  ros2 launch openarm_single_moveit_config demo.launch.py can_interface:=can0 can_fd:=false
"""

import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _generate_robot_description(context: LaunchContext,
                                 use_fake_hardware,
                                 can_interface,
                                 can_fd):
    """Process v10.urdf.xacro for single arm, no gripper, with ros2_control."""
    use_fake_hardware_str = context.perform_substitution(use_fake_hardware)
    can_interface_str = context.perform_substitution(can_interface)
    can_fd_str = context.perform_substitution(can_fd)

    xacro_path = os.path.join(
        get_package_share_directory("openarm_description"),
        "urdf", "robot", "v10.urdf.xacro",
    )

    return xacro.process_file(
        xacro_path,
        mappings={
            "arm_type": "v10",
            "bimanual": "false",
            "hand": "false",
            "ee_type": "none",
            "ros2_control": "true",
            "use_fake_hardware": use_fake_hardware_str,
            "can_interface": can_interface_str,
            "can_fd": can_fd_str,
            "arm_prefix": "",
            # Include the body so the arm mounts on the rod and world link exists
            "with_body": "true",
            # Single arm is right-arm hardware (calibrated as right_arm).
            # Use right-arm xyz/rpy: arm_base_rpy=+1.5708 (positive roll, same
            # as bimanual right arm).  reflect is now +1 for empty arm_prefix.
            "arm_base_xyz": "0.0 -0.031 0.698",
            "arm_base_rpy": "1.5708 0 0",
        },
    ).toprettyxml(indent="  ")


def _bringup_nodes(context: LaunchContext,
                   use_fake_hardware,
                   can_interface,
                   can_fd,
                   controllers_file):
    """Return robot_state_publisher + ros2_control_node."""
    robot_description = _generate_robot_description(
        context, use_fake_hardware, can_interface, can_fd
    )
    controllers_file_str = context.perform_substitution(controllers_file)
    robot_description_param = {"robot_description": robot_description}

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description_param],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[robot_description_param, controllers_file_str],
    )

    return [robot_state_pub, control_node]


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def generate_launch_description():

    declared_arguments = [
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Use fake/mock hardware (simulation) instead of real CAN hardware.",
        ),
        DeclareLaunchArgument(
            "can_interface",
            default_value="can0",
            description="SocketCAN interface name (e.g. can0, can1).",
        ),
        DeclareLaunchArgument(
            "can_fd",
            default_value="false",
            description="Use CAN FD frames (true) or standard CAN 2.0 frames (false).",
        ),
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="openarm_bringup",
            description="Package that contains the ros2_control controllers YAML.",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="openarm_v10_controllers.yaml",
            description="Controllers YAML file inside runtime_config_package/config/v10_controllers/.",
        ),
    ]

    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    can_interface = LaunchConfiguration("can_interface")
    can_fd = LaunchConfiguration("can_fd")
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file_name = LaunchConfiguration("controllers_file")

    controllers_file = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config",
         "v10_controllers", controllers_file_name]
    )

    # --- ros2_control bringup (RSP + controller_manager) ---
    bringup_func = OpaqueFunction(
        function=_bringup_nodes,
        args=[use_fake_hardware, can_interface, can_fd, controllers_file],
    )

    # --- Controller spawners (delayed to let controller_manager start) ---
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    jtc_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )

    delayed_jsb = TimerAction(period=2.0, actions=[jsb_spawner])
    delayed_jtc = TimerAction(period=3.0, actions=[jtc_spawner])

    # --- MoveIt2 configuration ---
    # Use .robot_description() to pass mappings explicitly — avoids the
    # xacro_args whitespace-splitting bug that breaks values containing spaces
    # (like arm_base_xyz="0.0 0.031 0.698").  ros2_control is false here
    # because move_group only needs the geometry, not the hardware plugin.
    _moveit_urdf = os.path.join(
        get_package_share_directory("openarm_description"),
        "urdf", "robot", "v10.urdf.xacro",
    )
    moveit_config = (
        MoveItConfigsBuilder("openarm", package_name="openarm_single_moveit_config")
        .robot_description(
            file_path=_moveit_urdf,
            mappings={
                "arm_type": "v10",
                "bimanual": "false",
                "hand": "false",
                "ee_type": "none",
                "ros2_control": "false",
                "use_fake_hardware": "false",
                "arm_prefix": "",
                "with_body": "true",
                "arm_base_xyz": "0.0 -0.031 0.698",
                "arm_base_rpy": "1.5708 0 0",
            },
        )
        .to_moveit_configs()
    )
    moveit_params = moveit_config.to_dict()

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_params],
    )

    # --- RViz2 with MoveIt2 MotionPlanning panel ---
    rviz_cfg = os.path.join(
        get_package_share_directory("openarm_single_moveit_config"),
        "config", "moveit.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_cfg],
        parameters=[moveit_params],
    )

    return LaunchDescription(
        declared_arguments + [
            bringup_func,
            delayed_jsb,
            delayed_jtc,
            move_group_node,
            rviz_node,
        ]
    )
