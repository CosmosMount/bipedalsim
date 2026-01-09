import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

def generate_launch_description():

    # 获取工作空间路径
    ws_path = os.path.expanduser('~/Desktop/gazebo_ws')  # 根据你的实际路径修改

    # 启动 ros_gz_bridge
    bridge = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '--ros-args', '-p', 'config_file:=' + os.path.join(ws_path, 'src/RM26_Gazebo/bridge.yaml')
        ],
        output='screen'
    )
    
    # 启动 Gazebo 并加载 SDF
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '/home/cosmosmount/Desktop/RM26_Gazebo/model/bipedal.sdf'],
        output='screen'
    )

    # 启动 lowlevel 节点
    lowlevel = Node(
        package='lowlevel',
        executable='lowlevel_msg',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 启动 controller 节点
    controller = Node(
        package='controller',
        executable='controller',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        bridge,
        lowlevel,
        controller
    ])
