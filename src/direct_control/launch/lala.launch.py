#!/usr/bin/ python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

    # micro_ros_agent = ExecuteProcess(
    #     cmd=[[
    #         'micro-ros-agent udp4 --port 8888 -v '
    #     ]],
    #     shell=True
    # )

    test_actuator = Node(
        package='direct_control',
        executable='asdasd',
        output='screen',
        shell=True,
    )

    return LaunchDescription([
        test_actuator
    ])