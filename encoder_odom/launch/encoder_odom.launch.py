from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Nodes
    encoder_odom = Node(
        package='encoder_odom',
        executable='encoder_odom',
        name='encoder_odom',
        remappings=[
            ('/robot/left_encoder_counts', '/kohm/left_encoder_counts'),
            ('/robot/right_encoder_counts', '/kohm/right_encoder_counts'), # To allow for easy swaps with pcpl based 3D LiDAR pipelines
        ],
    )

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument('use_sim_time',
                              default_value='false',
                              description='Use simulation clock if true'),

        # Nodes
        encoder_odom,
    ])