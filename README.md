# uam_tubes
Implementation of Safe Trajectory Tubes NASA Secure Safe Assured Autonomy for Urban Air Mobility

## Dependencies

### 1.Download the latest version of PX4-Autopilot from github and run setup script.
> git clone git@github.com:PX4/PX4-Autopilot.git
> 
> ./PX4-Autopilot/Tools/setup/ubuntu.sh

Follow Instructions on https://docs.px4.io/main/en/sim_gazebo_gz/ to run SITL Simulation

### 2.Prepare The Ros2 workspace clone the following repositories
(Note These  directories have been added to .gitignore)
> git clone git@github.com:PX4/px4_msgs.git
> 
> git clone git@github.com:PX4/px4_ros_com.git
> git clone git@github.com:andriyukr/ros2-vicon-receiver.git

### 3. Install Vicon SDK 
Find and run the script install_libs.sh within the package "ros2-vicon-receiver"

### 4.Install ros_gz bridge. Obtain pose data directly from gazebo sim. simulates MOCAP system
Download and build ros_gz from source Follow instructions on https://index.ros.org/r/ros_gz/
Need to have gz-garden https://gazebosim.org/docs/garden/install.
Don't forget to rosdep install
I had to manually install actuator_msgs https://github.com/rudislabs/actuator_msgs

## Run The SITL imulation
First run the PX4 Simulation
Navigate to the PX4-Autopilot Directory Use the following Command
>PX4_SYS_AUTOSTART=4005 PX4_GZ_MODEL_POSE="-1.06,1.52" PX4_GZ_MODEL=x500_vision PX4_MICRODDS_NS=uav_1 ./PX4-Autopilot/build/px4_sitl_default/bin/px4 -i 1
Then from a terminal *NOT VS Code* Run the following launch file.
>ros2 launch uam_operator sitl_launch.py