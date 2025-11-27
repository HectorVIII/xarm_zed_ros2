# arm_handover/launch/xarm_zed_handover.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ===== Launch parameter =====
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.1.225',
        description='IP address of xArm robot'
    )

    left_hand_topic_arg = DeclareLaunchArgument(
        'left_hand_topic',
        default_value='/left_hand/point',
        description='Topic name for stable left hand point from ZED'
    )

    robot_ip = LaunchConfiguration('robot_ip')
    left_hand_topic = LaunchConfiguration('left_hand_topic')

    # ===== ZED 左手检测节点 =====
    zed_left_hand_node = Node(
        package='xarm_zed_handover',
        executable='zed_left_hand_node',
        name='zed_left_hand_node',
        output='screen',
        parameters=[{
            'topic_name': left_hand_topic,    # 和节点里 declare_parameter('topic_name', ...) 对应
        }]
    )

    # ===== 机械臂 handover 节点 =====
    handover_node = Node(
        package='xarm_zed_handover',
        executable='handover_node',
        name='handover_node',
        output='screen',
        parameters=[{
            'robot_ip': robot_ip,               # 节点里 declare_parameter('robot_ip', ...) 用
            'left_hand_topic': left_hand_topic  # 节点里 declare_parameter('left_hand_topic', ...) 用
        }]
    )

    # 返回 LaunchDescription
    return LaunchDescription([
        robot_ip_arg,
        left_hand_topic_arg,
        zed_left_hand_node,
        handover_node,
    ])

