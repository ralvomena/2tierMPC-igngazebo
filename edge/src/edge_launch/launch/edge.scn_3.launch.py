from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    agvs = ['agv_1', 'agv_2', 'agv_3', 'agv_4', 'agv_5', 'agv_6']
    priorities = {'agv_1': 'high', 'agv_2': 'high', 'agv_3': 'high', 'agv_4': 'high',
                    'agv_5': 'high', 'agv_6': 'high'}
    sim_name = 'scn_3'
    save_sim_data = False
    agv_type = 1
    agv_diam = 0.75
    d_safe = 1.0
    d_safe_obs = 1.0
    high_vel = 1.5
    medium_vel = 1.0
    low_vel = 0.75
    scenario = 3
    obstacles = 5
    scn_size = [-100, -200, 100, 200] #-x, -y, +x, +y

    mpc_planning_nodes = []

    for agv_id in agvs:
        mpc_planning_nodes.append(Node(
            package="edge_tier_pkg",
            executable="mpc_node",
            name=agv_id.lower() + '_mpc',
            parameters=[{"agv_id": agv_id},
                        {"priority": priorities[agv_id]},
                        {"sim_name": sim_name},
                        {"save_sim_data": save_sim_data},
                        {"agv_type": agv_type},
                        {"agv_diam": agv_diam},
                        {"d_safe": d_safe},
                        {"d_safe_obs": d_safe_obs},
                        {"high_vel": high_vel},
                        {"medium_vel": medium_vel},
                        {"low_vel": low_vel},
                        {"scenario": scenario},
                        {"obstacles": obstacles},
                        {"scn_size": scn_size}
            ]
        ))

    supervisor_node = Node(
            package="edge_tier_pkg",
            executable="supervisor",
        )
    ld.add_action(supervisor_node)

    for node in mpc_planning_nodes:
        ld.add_action(node)
    return ld
