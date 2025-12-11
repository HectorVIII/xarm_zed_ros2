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

    right_hand_topic_arg = DeclareLaunchArgument(
        'right_hand_topic',
        default_value='/right_hand/point',
        description='Topic name for stable right hand point from ZED'
    )

    robot_ip = LaunchConfiguration('robot_ip')
    right_hand_topic = LaunchConfiguration('right_hand_topic')

    # ===== ZED 右手检测节点 =====
    zed_right_hand_node = Node(
        package='xarm_zed_handover',
        executable='zed_right_hand_node',
        name='zed_right_hand_node',
        output='screen',
        parameters=[{
            'topic_name': right_hand_topic,    # 和节点里 declare_parameter('topic_name', ...) 对应
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
            'right_hand_topic': right_hand_topic  # 节点里 declare_parameter('right_hand_topic', ...) 用
        }]
    )

    # 返回 LaunchDescription
    return LaunchDescription([
        robot_ip_arg,
        right_hand_topic_arg,
        zed_right_hand_node,
        handover_node,
    ])

