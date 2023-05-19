import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

number_of_vehicles = 1

def generate_launch_description():

    operator_dir = get_package_share_directory('uam_operator')
    params_file = os.path.join(operator_dir,'params','test_params.yaml')

    operator_container = ComposableNodeContainer(
        name='uam_operator',
        namespace = '',
        package = 'rclcpp_components',
        executable = 'component_container',
        composable_node_descriptions=[
            ComposableNode(
                package = 'uam_operator',
                plugin='uam_operator::TubeTrajectoryServer',
                name = 'trajectory_server',
                parameters = [params_file]
            ),
            ComposableNode(
                package = 'uam_operator',
                plugin='uam_operator::VehicleInterface',
                name='operator_vehicle_interface',
                namespace = "/uav_1",
                parameters = [{"vehicle_id":"1"}],
            ),
        ],
        output = 'screen'
    )

    flightmanager_node = Node(
        package = 'uam_flightmanager',
        name= 'flightmanager',
        executable='flightmanager',
        namespace = "/uav_1",
        #arguments=["--ros-args","--log-level","flightmanager:=debug"],
        output='screen'
    )

    vehicle_interface_node = Node(
        package = 'uam_vehicle_interface',
        name = 'vehicle_interface',
        executable ='vehicle_interface',
        namespace = "/uav_1",
        parameters = [{"mav_id": 1}],
        #arguments=["--ros-args","--log-level","vehicle_interface:=debug"],
        output='screen'
    )



    ld = LaunchDescription()
    ld.add_action(vehicle_interface_node)
    ld.add_action(flightmanager_node)
    ld.add_action(operator_container)
    return ld