import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    operator_dir = get_package_share_directory('uam_operator')
    params_file = os.path.join(operator_dir,'params','tube_trajectory_params.yaml')
    operator_node = Node(
        package='uam_operator',
        name='trajectory_server',
        executable='trajectory_server',
        output='screen',
        parameters = [params_file]
    )

    flightmanager_node =Node(
        package = 'uam_flightmanager',
        name= 'flightmanager',
        executable='flightmanager',
        output='screen',
        arguments=["--ros-args","--log-level","flightmanager:=debug"]
    )

    vehicle_interface_node =Node(
        package = 'uam_vehicle_interface',
        name = 'vehicle_interface',
        executable ='vehicle_interface',
        output='screen',
        arguments=["--ros-args","--log-level","vehicle_interface:=debug"]
    )

    ld = LaunchDescription()
    ld.add_action(operator_node)
    ld.add_action(flightmanager_node)
    ld.add_action(vehicle_interface_node)
    return ld