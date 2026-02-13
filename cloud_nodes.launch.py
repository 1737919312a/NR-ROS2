#!/usr/bin/env python3
"""
ROS 2 启动文件 - 云端节点
启动YOLO检测和决策通信节点
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 声明启动参数
    declare_yolo = DeclareLaunchArgument(
        'yolo_detection',
        default_value='true',
        description='是否启动YOLO检测节点'
    )
    
    declare_decision_comm = DeclareLaunchArgument(
        'decision_comm',
        default_value='true',
        description='是否启动决策通信节点'
    )
    
    declare_remote_ip = DeclareLaunchArgument(
        'remote_ip',
        default_value='192.168.1.100',
        description='边缘端IP地址'
    )
    
    declare_model_path = DeclareLaunchArgument(
        'model_path',
        default_value='yolov8n.engine',
        description='YOLO模型路径'
    )
    
    # YOLO检测节点
    yolo_detection_node = Node(
        package='ros2_robot_system',
        executable='yolo_detection_node',
        name='yolo_detection_node',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'conf_threshold': 0.5,
            'nms_threshold': 0.4,
        }],
        condition=IfCondition(LaunchConfiguration('yolo_detection')),
        output='screen'
    )
    
    # 决策通信节点
    decision_comm_node = Node(
        package='ros2_robot_system',
        executable='decision_comm_node',
        name='decision_comm_node',
        parameters=[{
            'remote_ip': LaunchConfiguration('remote_ip'),
            'remote_port': 8889,
            'local_port': 8890,
        }],
        condition=IfCondition(LaunchConfiguration('decision_comm')),
        output='screen'
    )
    
    return LaunchDescription([
        declare_yolo,
        declare_decision_comm,
        declare_remote_ip,
        declare_model_path,
        yolo_detection_node,
        decision_comm_node,
    ])
