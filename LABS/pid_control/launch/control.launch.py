#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # Конфигурация Stage
    stage_dir = get_package_share_directory('stage_ros2')
    control_dir = get_package_share_directory('pid_control')
    
    # Аргументы запуска
    task_arg = DeclareLaunchArgument(
        'task',
        default_value='line',
        description='Task type: line, circle or oval',
        choices=['line', 'circle', 'oval'])
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty',
        description='World file name without extension')
    
    # Параметры для разных задач
    pid_profiles = {
        'line': {
            'kp': 0.25,
            'ki': 0.0,
            'kd': 7.0,
            'params': {
                'line_y': -7.0,
                'task_vel': 0.5,
                'min_obstacle_range': 0.5
            }
        },
        'circle': {
            'kp': 0.3,
            'ki': 0.001,
            'kd': 5.0,
            'params': {
                'cx': -6.0,
                'cy': 0.0,
                'R': 6.0,
                'task_vel': 0.4,
                'min_obstacle_range': 0.7
            }
        },
        'oval': {
            'kp': 0.35,
            'ki': 0.002,
            'kd': 6.5,
            'params': {
                'oval_center_x': -6.0,
                'oval_center_y': 0.0,
                'oval_radius': 6.0,
                'oval_straight_len': 12.0,
                'task_vel': 0.35,
                'min_obstacle_range': 1.0
            }
        }
    }

    def configure_control_node(context):
        task_type = context.launch_configurations['task']
        profile = pid_profiles[task_type]
        
        params = {
            'task': task_type,
            'kp': profile['kp'],
            'ki': profile['ki'],
            'kd': profile['kd']
        }
        params.update(profile['params'])
        
        return [Node(
            package='pid_control',
            executable='pid_node',
            name='pid_node',
            parameters=[params],
            output='screen'
        )]

    return LaunchDescription([
        # Аргументы
        task_arg,
        world_arg,
        DeclareLaunchArgument(
            'enforce_prefixes',
            default_value='false',
            description='Use prefixes for single robot env'),
        DeclareLaunchArgument(
            'use_static_transformations',
            default_value='true',
            description='Use static TF for sensors'),
        DeclareLaunchArgument(
            'one_tf_tree',
            default_value='false',
            description='Publish all TF in one tree'),
        
        # Конфигурация Stage
        OpaqueFunction(function=lambda context: [
            SetLaunchConfiguration('world_file', os.path.join(
                stage_dir,
                'world',
                LaunchConfiguration('world').perform(context) + '.world'
            ))
        ]),
        
        # Запуск Stage
        Node(
            package='stage_ros2',
            executable='stage_ros2',
            name='stage',
            parameters=[{
                'one_tf_tree': LaunchConfiguration('one_tf_tree'),
                'enforce_prefixes': LaunchConfiguration('enforce_prefixes'),
                'use_static_transformations': LaunchConfiguration('use_static_transformations'),
                'world_file': LaunchConfiguration('world_file')
            }]
        ),
        
        # Динамическая конфигурация управляющего узла
        OpaqueFunction(function=configure_control_node)
    ])