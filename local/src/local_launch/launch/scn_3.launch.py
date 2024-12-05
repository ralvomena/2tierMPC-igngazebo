from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os
import math

def generate_launch_description():
    
    ld = LaunchDescription()

    world_file_path = PathJoinSubstitution([get_package_share_directory('local_launch'), 'worlds','scenario3.sdf'])
    
    agv_file_path = PathJoinSubstitution([get_package_share_directory('local_launch'), 'models','tugbot', 'model.sdf'])

    ign_gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_ign_gazebo'), 'launch'), '/ign_gazebo.launch.py']),
                    launch_arguments=[('ign_args',world_file_path)])
    ld.add_action(ign_gazebo)
    
    n = 6
    agv_ids = ['agv_' + str(i+1) for i in range(n)]
    poses = {'agv_1': [45.0, 95.0, math.radians(180)], 
                    'agv_2': [45.0, 90.0, math.radians(180)],
                    'agv_3': [45.0, 2.5, math.radians(180)], 
                    'agv_4': [45.0, -2.5, math.radians(180)],
                    'agv_5': [45.0, -90.0, math.radians(180)],
                    'agv_6': [45.0, -95.0, math.radians(180)]
                }

    for agv in agv_ids:
        start_gazebo_ros_spawner_cmd = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=[
            '-name', agv,
            '-file', agv_file_path,
            '-x', str(poses[agv][0]),
            '-y', str(poses[agv][1]),
            '-z', '0.0',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', str(poses[agv][2])
        ],
        output='screen',
        )
        ld.add_action(start_gazebo_ros_spawner_cmd)

    cmd_vel_args = ['/model/agv_' + str(i) + '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist' for i in range(1, n+1)]
    odometry_args = ['/model/agv_' + str(i) + '/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry' for i in range(1, n+1)]
    pose_args = ['/model/agv_' + str(i) + '/pose@geometry_msgs/msg/Pose@ignition.msgs.Pose' for i in range(1, n+1)]
    bridge_node_args = cmd_vel_args + odometry_args + pose_args

    remappings = []
    for i in range(1, n + 1):
        remappings.extend([
            (f'/model/agv_{i}/cmd_vel', f'/agv_{i}/cmd_vel'),
            (f'/model/agv_{i}/odometry', f'/agv_{i}/odom'),
            (f'/model/agv_{i}/pose', f'/agv_{i}/pose')
    ])
    
    ros_bridge = Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
            name='bridge_node',
            arguments=bridge_node_args,
            remappings=remappings,
            output='screen',
            parameters=[
                {"use_sim_time": True},
            ],
            )
    ld.add_action(ros_bridge)

    return ld