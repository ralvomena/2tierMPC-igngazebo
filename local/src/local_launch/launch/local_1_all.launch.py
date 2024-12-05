from launch import LaunchDescription
from launch_ros.actions import Node
from math import radians

def generate_launch_description():
    ld = LaunchDescription()

    agvs_quantity = 4
    agv_ids = ['agv_' + str(i+1) for i in range(agvs_quantity)]
    mpc_local_nodes = []
    netem_nodes = []

    priority = 'high'
    sim_name = 'scn_1'
    save_sim_data = False
    high_vel = 1.0
    medium_vel = 0.75
    low_vel = 0.5
    scenario = 1
    scn_size = [-25, -25, 25, 25] #-x, -y, +x, +y
    netem_node_enable = False

    for agv_id in agv_ids:
        mpc_local_nodes.append(Node(
            package="local_tier_pkg",
            executable="mpc_tracking_node",
            name=agv_id.lower() + '_mpc_local_tracking',
            parameters=[{"agv_id": agv_id},   
                    {"priority": priority},
                    {"sim_name": sim_name},
                    {"save_sim_data": save_sim_data},
                    {"high_vel": high_vel},
                    {"medium_vel": medium_vel},
                    {"low_vel": low_vel},
                    {"scenario": scenario},
                    {"scn_size": scn_size}]))
        
        if netem_node_enable:
            netem_nodes.append(Node(
                package="local_tier_pkg",
                executable="netem_node",
                name=agv_id.lower() + '_netem_node',
                parameters=[{"agv_id": agv_id}]))

    for mpc_node in mpc_local_nodes:
        ld.add_action(mpc_node)
        
    if netem_node_enable:
        for netem_node in netem_nodes:
            ld.add_action(netem_node)
    
    return ld