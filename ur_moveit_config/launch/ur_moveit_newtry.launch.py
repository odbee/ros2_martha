import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("ur")
        .robot_description(file_path="srdf/ur.srdf.xacro")
        .trajectory_execution(file_path="config/controllers.yaml")
        .moveit_cpp(
            file_path=get_package_share_directory("mv2_com")
            + "/config/moveit_cpp.yaml"
        )
        .to_moveit_configs()
    )
    # # MoveItCpp demo executable
    # moveit_cpp_node = Node(
    #     name="mpserver",
    #     package="mv2_com",
    #     executable="mpserver",
    #     output="screen",
    #     parameters=[moveit_config.to_dict()],
    # )

    # moveit_cpp_node = Node(
    #     name="mpserver2",
    #     package="mv2_com",
    #     executable="mpserver2",
    #     output="screen",
    #     parameters=[moveit_config.to_dict()],
    # )

    # RViz
    rviz_config_file = (
        get_package_share_directory("mv_com")
        + "/launch/mpserver.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "panda_link0"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("ur_moveit_config"),
        "config",
        "controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="both",
    )

    # Load controllers
    load_controllers = []
    for controller in [
        "panda_arm_controller",
        "panda_hand_controller",
        "joint_state_broadcaster",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    return LaunchDescription(
        [
            static_tf,
            robot_state_publisher,
            rviz_node,
            # moveit_cpp_node,
            ros2_control_node,
        ]
        + load_controllers
    )
