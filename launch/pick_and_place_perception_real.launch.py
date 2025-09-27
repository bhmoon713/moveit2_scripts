import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ur5", package_name="my_moveit_config").to_moveit_configs()
    rviz_config_path = os.path.join(
        get_package_share_directory('object_detection'),
        'rviz',
        'sensor_data_real.rviz'
    )
    # First two nodes
    object_detection_real_node = Node(
        package='object_detection',
        executable='object_detection_real',
        name='object_detection_real',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # static_transform_publisher_node = Node(
    #     package='object_detection',
    #     executable='static_transform_publisher',
    #     name='static_transform_publisher',
    #     output='screen',
    #     parameters=[{'use_sim_time': False}]
    # )

    rviz_real_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen',
            parameters=[{'use_sim_time': False}]
    )

    # Final node (delayed)
    moveit_cpp_node = Node(
        name="pick_and_place_real",
        package="moveit2_scripts",
        executable="pick_and_place_perception_real",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': False}
        ]
    )

    # Wrap final node in TimerAction (wait 2 seconds)
    delayed_moveit_node = TimerAction(
        period=5.0,
        actions=[moveit_cpp_node]
    )

    return LaunchDescription([
        object_detection_real_node,
        # static_transform_publisher_node,
        rviz_real_node,
        moveit_cpp_node,
        # delayed_moveit_node
    ])
