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
        'sensor_data.rviz'
    )
    # First two nodes
    object_detection_node = Node(
        package='object_detection',
        executable='object_detection',
        name='object_detection',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    static_transform_publisher_node = Node(
        package='object_detection',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen',
            parameters=[{'use_sim_time': True}]
    )

    # Final node (delayed)
    moveit_cpp_node = Node(
        name="pick_and_place",
        package="moveit2_scripts",
        executable="pick_and_place_perception",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True}
        ]
    )

    # Wrap final node in TimerAction (wait 2 seconds)
    delayed_moveit_node = TimerAction(
        period=5.0,
        actions=[moveit_cpp_node]
    )

    return LaunchDescription([
        object_detection_node,
        static_transform_publisher_node,
        rviz_node,
        delayed_moveit_node
    ])
