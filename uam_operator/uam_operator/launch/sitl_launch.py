import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

number_of_vehicles = 3

def generate_launch_description():

    operator_dir = get_package_share_directory('uam_operator')
    params_file = os.path.join(operator_dir,'params','tube_trajectory_params.yaml')

    operator_components = [
        ComposableNode(
                package = 'uam_operator',
                plugin='uam_operator::TubeServer',
                name = 'tube_server',
                parameters = [params_file]
            )
    ]

    for i in range(number_of_vehicles):
        operator_components.append(
             ComposableNode(
                package = 'uam_operator',
                plugin='uam_operator::OperatorInterface',
                name='operator_vehicle_interface',
                namespace = 'uav_' + str(i+1),
                parameters = [{"vehicle_id":i+1}],
            )
        )

    operator_container = ComposableNodeContainer(
        name='uam_operator',
        namespace = '',
        package = 'rclcpp_components',
        executable = 'component_container',
        composable_node_descriptions = operator_components,
        output = 'screen'
    )

    flight_manager_nodes = []
    vehicle_interface_nodes = []
    ros_gz_bridges = []
    vehicle_odom_bridges = []
    for i in range(number_of_vehicles):
        NameSpace = 'uav_' + str(i+1)
        flight_manager_nodes.append(
            Node(
                package = 'uam_flightmanager',
                name= 'flightmanager',
                executable='flightmanager',
                namespace = NameSpace,
                output='screen'
            )
        )
        vehicle_interface_nodes.append(
            Node(
                package = 'uam_vehicle_interface',
                name = 'vehicle_interface',
                executable ='vehicle_interface',
                namespace = NameSpace,
                parameters = [{"mav_id": i+2}],
                output='screen'
            )
        )
        ros_gz_bridges.append(
            Node(
                package = 'ros_gz_bridge',
                executable = 'parameter_bridge',
                remappings = [('/model/x500_vision_'+str(i+1)+'/pose',NameSpace+'/mocap_pose')],
                arguments = ['model/x500_vision_'+str(i+1)+'/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V']
            )  
        )
        vehicle_odom_bridges.append(
            Node(
                package = 'uam_util',
                executable = 'vehicle_odometry_bridge',
                name = 'vehicle_odometry_bridge',
                namespace = NameSpace,
                output = 'screen'
            )
        )

    rviz2_node = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d' + os.path.join(operator_dir, 'rviz', 'tube_trajectory_config.rviz')]
    )

    tf2_static_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['--x', '0', '--y', '0', '--z', '0',
                      '--qx', '0', '--qy', '0', '--qz', '0', '--qw','1',
                      '--frame-id', 'world', '--child-frame-id', 'map']
    )

    tf2_static_NED = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['--x', '0', '--y', '0', '--z', '0',
                    '--qx', '0.70710678118', '--qy', '0.70710678118', '--qz', '0', '--qw','0',
                    '--frame-id', 'world', '--child-frame-id', 'NED']
    )

    ld = LaunchDescription()
    ld.add_action(tf2_static_NED)
    ld.add_action(tf2_static_map)
    ld.add_action(rviz2_node)
    for i in range(number_of_vehicles):
        ld.add_action(vehicle_interface_nodes[i])
        ld.add_action(flight_manager_nodes[i])
        ld.add_action(vehicle_odom_bridges[i])
        ld.add_action(ros_gz_bridges[i])

    ld.add_action(operator_container)

    return ld