import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    operator_dir = get_package_share_directory('uam_operator')
    params_file = os.path.join(operator_dir,'params','tube_trajectory_params.yaml')

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
            ComposableNode(
                package = 'uam_operator',
                plugin='uam_operator::VehicleInterface',
                name='operator_vehicle_interface',
                namespace = "/uav_2",
                parameters = [{"vehicle_id":"2"}],
            )
        ],
        output = 'screen'
    )

    flightmanager_node_1 =Node(
        package = 'uam_flightmanager',
        name= 'flightmanager',
        executable='flightmanager',
        namespace = "/uav_1",
        #arguments=["--ros-args","--log-level","flightmanager:=debug"],
        output='screen'
    )

    flightmanager_node_2 =Node(
        package = 'uam_flightmanager',
        name= 'flightmanager',
        executable='flightmanager',
        namespace = "/uav_2",
        #arguments=["--ros-args","--log-level","flightmanager:=debug"],
        output='screen'
    )

    vehicle_interface_node_1 =Node(
        package = 'uam_vehicle_interface',
        name = 'vehicle_interface',
        executable ='vehicle_interface',
        namespace = "/uav_1",
        parameters = [{"mav_id": 2}],
        #arguments=["--ros-args","--log-level","vehicle_interface:=debug"],
        output='screen'
    )
    vehicle_interface_node_2 =Node(
        package = 'uam_vehicle_interface',
        name = 'vehicle_interface',
        executable ='vehicle_interface',
        namespace = "/uav_2",
        parameters = [{"mav_id": 3}],
        #arguments=["--ros-args","--log-level","vehicle_interface:=debug"],
        output='screen'
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
                    '--qx', '1', '--qy', '0', '--qz', '0', '--qw','0',
                    '--frame-id', 'world', '--child-frame-id', 'NED']
    )

    ros_gz_bridge_1 = Node(
        package = 'ros_gz_bridge',
        executable = 'parameter_bridge',
        remappings = [("/model/x500_vision_1/pose","/uav_1/mocap_pose")],
        arguments = ['model/x500_vision_1/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V']    
    )

    ros_gz_bridge_2 = Node(
        package = 'ros_gz_bridge',
        executable = 'parameter_bridge',
        remappings = [("/model/x500_vision_2/pose","/uav_2/mocap_pose")],
        arguments = ['model/x500_vision_2/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V']    
    )

    vehicle_odom_bridge_1 = Node(
        package = 'uam_vehicle_interface',
        executable = 'vehicle_odometry_bridge',
        name = 'vehicle_odometry_bridge',
        namespace = '/uav_1',
        output = 'screen'
    )

    vehicle_odom_bridge_2 = Node(
        package = 'uam_vehicle_interface',
        executable = 'vehicle_odometry_bridge',
        name = 'vehicle_odometry_bridge',
        namespace = '/uav_2',
        output = 'screen'
    )


    ld = LaunchDescription()
    ld.add_action(ros_gz_bridge_1)
    ld.add_action(ros_gz_bridge_2)
    ld.add_action(vehicle_odom_bridge_1)
    ld.add_action(vehicle_odom_bridge_2)
    ld.add_action(operator_container)
    ld.add_action(flightmanager_node_1)
    ld.add_action(flightmanager_node_2)
    ld.add_action(vehicle_interface_node_1)
    ld.add_action(vehicle_interface_node_2)
    ld.add_action(rviz2_node)
    ld.add_action(tf2_static_map)
    ld.add_action(tf2_static_NED)
    return ld