from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("moveit_resources_panda").to_moveit_configs()

    # MoveGroupInterface demo executable
    move_group_demo = Node(
        # name="mg_com_server",
        package="mv2_com",
        executable="mg_com_server",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"planning_group": "panda_arm"}
        ],
    )

    return LaunchDescription([move_group_demo])