from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'odom_topic',
            # default_value='/car_state/odom',
            # default_value='/sim/ego_racecar/odom',
            # default_value='/odom',
            default_value='/car_state/odom',
            description='Odometry topic'
        ),
        DeclareLaunchArgument(
            'ackermann_cmd_topic',
            # default_value='/vesc/high_level/ackermann_cmd_mux/input/nav_1',
            # default_value='/sim/drive',
            default_value='/drive',
            description='Ackermann command topic'
        ),
        DeclareLaunchArgument(
            'save_dir',
            default_value='/home/misys/forza_ws/race_stack/controller/map/lut',
            description='Directory to save LUT csv'
        ),
        DeclareLaunchArgument(
            'save_LUT_name',
            default_value='F1TENTH_Pacejka',
            description='Name for saving LUT'
        ),
        DeclareLaunchArgument(
            'plot_model',
            default_value='True',
            description='Enable model plotting'
        ),
        DeclareLaunchArgument(
            'racecar_version',
            # default_value='SIM',
            default_value='Orinnano',
            description='Racecar version'
        ),

        Node(
            package='on_track_sys_id',
            executable='on_track_sys_id',
            name='on_track_sys_id',
            output='screen',
            parameters=[{
                'odom_topic': LaunchConfiguration('odom_topic'),
                'ackermann_cmd_topic': LaunchConfiguration('ackermann_cmd_topic'),
                'save_LUT_name': LaunchConfiguration('save_LUT_name'),
                'save_dir': LaunchConfiguration('save_dir'),
                'plot_model': LaunchConfiguration('plot_model'),
                'racecar_version': LaunchConfiguration('racecar_version'),
            }]
        )
    ])
