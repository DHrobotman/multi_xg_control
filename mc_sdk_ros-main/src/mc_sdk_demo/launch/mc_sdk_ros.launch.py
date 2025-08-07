from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 键盘控制节点（唯一输入源）
        Node(
            package='mc_sdk_ros',
            executable='keyboard_teleop',
            name='keyboard_teleop',
            output='screen',
            prefix='xterm -e'  
        ),

        # 机器人1
        Node(
            package='mc_sdk_ros',
            executable='mc_sdk_ros',
            name='mc_sdk_demo_1',
            namespace='/xg_101',
            output='screen',
            parameters=[{}],
            prefix='xterm -e'  
        ),

        # 机器人2
        Node(
            package='mc_sdk_ros',
            executable='mc_sdk_ros',
            name='mc_sdk_demo_2',
            namespace='/xg_102',
            output='screen',
            parameters=[{}],
            prefix='xterm -e'  
        ),

        # 可继续添加更多机器人...
    ])