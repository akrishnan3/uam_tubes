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
        output='screen'
        #arguments=["--ros-args","--log-level","flightmanager:=debug"]
    )

    vehicle_interface_node =Node(
        package = 'uam_vehicle_interface',
        name = 'vehicle_interface',
        executable ='vehicle_interface',
        output='screen'
        #arguments=["--ros-args","--log-level","vehicle_interface:=debug"]
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

    ros_gz_bridge = Node(
        package = 'ros_gz_bridge',
        executable = 'parameter_bridge',
        arguments = ['model/x500_vision_0/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V']
    )

    vehicle_odom_bridge = Node(
        package = 'uam_vehicle_interface',
        executable = 'vehicle_odometry_bridge',
        name = 'vehicle_odometry_bridge',
        output = 'screen'
    )


    ld = LaunchDescription()
    ld.add_action(ros_gz_bridge)
    ld.add_action(vehicle_odom_bridge)
    ld.add_action(operator_node)
    ld.add_action(flightmanager_node)
    ld.add_action(vehicle_interface_node)
    ld.add_action(rviz2_node)
    ld.add_action(tf2_static_map)
    ld.add_action(tf2_static_NED)
    return ld