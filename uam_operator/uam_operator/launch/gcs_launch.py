import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

vicon_server_ip = '192.168.10.2'
vicon_buffer_size = 200

number_of_vehicles = 2

def generate_launch_description():

    vicon_receiver_node = Node(
        package= 'vicon_receiver',
        executable= 'vicon_client',
        output= 'screen',
        parameters=[{'hostname': vicon_server_ip, 'buffer_size': vicon_buffer_size, 'namespace': 'vicon'}]
    )

    odom_bridge_node = []
    for i in range(number_of_vehicles):
        subject = "/NASA_ULI_" + str(i+1)
        odom_bridge_node.append(Node(
            package = 'uam_vehicle_interface',
            executable= 'vicon_odometry_bridge',
            output= 'screen',
            namespace='uav_' + str(i+1),
            remappings= [("mocap_pose","/vicon" + subject + subject)]
        ))
        
    ld = LaunchDescription()
    ld.add_action(vicon_receiver_node)
    for i in range(number_of_vehicles):
        ld.add_action(odom_bridge_node[i])
    return ld