import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # urdf_file_name = 'EurocMavFirefly.urdf'
    # urdf = os.path.join(
    #     get_package_share_directory('dsso_ros'),
    #     'urdf',
    #     urdf_file_name)
    # with open(urdf, 'r') as infp:
    #     robot_desc = infp.read()

    return LaunchDescription([
	# map -> odom whould be from SLAM..
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id',
                       'map', '--child-frame-id', 'odom']),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments=['--x', '-0.0216', '--y', '-0.0647', '--z', '0.0098', '--roll', '-0.0037594', '--pitch', '0.025777', '--yaw', '1.5559946', '--frame-id',
        #                'base_link', '--child-frame-id', 'dsso_cam']),
        # Node(
        #     package='joint_state_publisher',
        #     executable='joint_state_publisher'),
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     parameters=[{'robot_description': robot_desc}]),
        Node(
            package='dsso_ros',
            executable='dsso_ros',
            parameters=['/home/tp/ros2_ws/src/dsso_ros/config/EuRoC_params.yaml'],
            # prefix=['xterm -e gdb -ex run --args'],
            # output='screen'
        )
    ])
