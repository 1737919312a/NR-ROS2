#!/usr/bin/env python3
"""
ROS 2 启动文件 - 边缘端节点
启动车道检测和AprilTag定位节点
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 声明启动参数
    declare_lane_detection = DeclareLaunchArgument(
        'lane_detection',
        default_value='true',
        description='是否启动车道检测节点'
    )
    
    declare_apriltag = DeclareLaunchArgument(
        'apriltag',
        default_value='true',
        description='是否启动AprilTag定位节点'
    )
    
    declare_camera_topic = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/image_raw',
        description='摄像头话题名称'
    )
    
    declare_rear_camera_topic = DeclareLaunchArgument(
        'rear_camera_topic',
        default_value='/camera/rear/image_raw',
        description='后置摄像头话题名称'
    )
    
    # 车道检测节点
    lane_detection_node = Node(
        package='ros2_robot_system',
        executable='lane_detection_node',
        name='lane_detection_node',
        parameters=[{
            'width': 640,
            'height': 480,
            'roi_top': 300,
            'roi_bottom': 480,
            'kp_angular': 0.8,
            'kp_linear': 0.3,
            'max_angular': 1.0,
            'max_linear': 0.5,
        }],
        remappings=[
            ('/camera/image_raw', LaunchConfiguration('camera_topic')),
        ],
        condition=IfCondition(LaunchConfiguration('lane_detection')),
        output='screen'
    )
    
    # AprilTag定位节点
    apriltag_node = Node(
        package='ros2_robot_system',
        executable='apriltag_node',
        name='apriltag_node',
        parameters=[{
            'tag_size': 0.05,
            'fx': 309.0,
            'fy': 309.0,
            'cx': 320.0,
            'cy': 240.0,
            'target_distance': 0.6,
            'kp_distance': 0.8,
            'kp_angle': 2.0,
            'max_linear': 0.3,
            'max_angular': 1.0,
        }],
        remappings=[
            ('/camera/rear/image_raw', LaunchConfiguration('rear_camera_topic')),
        ],
        condition=IfCondition(LaunchConfiguration('apriltag')),
        output='screen'
    )
    
    return LaunchDescription([
        declare_lane_detection,
        declare_apriltag,
        declare_camera_topic,
        declare_rear_camera_topic,
        lane_detection_node,
        apriltag_node,
    ])
