import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    NameSpace = LaunchConfiguration('namespace')
    id = LaunchConfiguration('mav_id')

    NameSpace_arg = DeclareLaunchArgument('namespace', default_value='')
    id_arg = DeclareLaunchArgument('mav_id', default_value='1')

    flight_manager_node = Node(
        package = 'uam_flightmanager',
        name= 'flightmanager',
        executable='flightmanager',
        namespace = NameSpace,
        
    )

    vehicle_interface_node =Node(
        package = 'uam_vehicle_interface',
        name = 'vehicle_interface',
        executable ='vehicle_interface',
        namespace = NameSpace,
        parameters = [{"mav_id": id}],
    )

    ld = LaunchDescription()
    ld.add_action(NameSpace_arg)
    ld.add_action(id_arg)
    ld.add_action(vehicle_interface_node)
    ld.add_action(flight_manager_node)
    return ld