
#include "uam_vehicle_interface/vehicle_interface.hpp"


VehicleInterface::VehicleInterface() : rclcpp::Node("vehicle_interface")
{

  using namespace std::placeholders;
  

  rclcpp::QoS px4_qos_pub = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().transient_local();
  rclcpp::QoS px4_qos_sub = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
  //----------------------- Parameters ------------------------
  this-> declare_parameter("mav_id", 1);

  //----------------------- Publisher --------------------------
  
  vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
    "fmu/in/vehicle_command",
    px4_qos_pub
  );

  offboard_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
    "fmu/in/offboard_control_mode",
    px4_qos_pub
  );

  trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
    "fmu/in/trajectory_setpoint",
    px4_qos_pub
  );

  vehicle_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "vehicle_interface/vehicle_pose",
    10
  );
  
  //------------------------ Subscriptions ------------------------

    vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
    "fmu/out/vehicle_status",
    px4_qos_sub,
    std::bind(& VehicleInterface::vehicle_status_callback, this, _1)
  );

  vehicle_odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
    "fmu/out/vehicle_odometry",
    px4_qos_sub,
    std::bind(&VehicleInterface::vehicle_odometry_callback, this, _1)
  );

  position_setpoint_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "vehicle_interface/position_setpoint",
    10,
    std::bind(& VehicleInterface::position_setpoint_callback, this, _1)
  );

  //---------------------------Services---------------------------------

  arm_service_ = this->create_service<vehicle_interface_msgs::srv::ArmService>(
    "vehicle_interface/arm_service",
    std::bind(& VehicleInterface::arm_routine, this, _1, _2)
  );

  //--------------------------------------------------------------------

  mav_id = this->get_parameter("mav_id").as_int();
}

void VehicleInterface::arm_routine(const std::shared_ptr<vehicle_interface_msgs::srv::ArmService::Request> request,
                      std::shared_ptr<vehicle_interface_msgs::srv::ArmService::Response> response)
{
  
  rclcpp::Rate rate_timer(std::chrono::milliseconds(100));
  if(current_arming_state != px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED  
      && request->command == vehicle_interface_msgs::srv::ArmService::Request::ARM){
    RCLCPP_INFO(this->get_logger(),"Recieved Request to arm");
    for(int i = 10; i > 0; i--){
      publish_position_setpoint(
        current_position[0],
        current_position[1],
        current_position[2],
        yaw_from_quaternion(current_quaternion)
      );
      rate_timer.sleep();
    }
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
    RCLCPP_INFO(this->get_logger(),"Set vehicle flight mode: offboard");
    arm();
    rate_timer.sleep();
  }else{
    RCLCPP_INFO(this->get_logger(),"Recieved Request to disarm");
    disarm();
    rate_timer.sleep();
  }
  if(current_arming_state != px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED){
    RCLCPP_INFO(this->get_logger(),"Vehicle Armed");
    response->response = vehicle_interface_msgs::srv::ArmService::Response::VEHICLE_ARMED;
  }else{
    RCLCPP_INFO(this->get_logger(),"Vehicle Disarmed");
    response->response = vehicle_interface_msgs::srv::ArmService::Response::VEHICLE_DISARMED;
  }
}

void VehicleInterface::vehicle_status_callback(const px4_msgs::msg::VehicleStatus &msg)
{
  //RCLCPP_INFO(this->get_logger(),"Recieved vehicle_status");
  current_arming_state = msg.arming_state;
  current_nav_state = msg.nav_state;
  //RCLCPP_INFO(this->get_logger(),"Nav State: %i", current_nav_state);
}

void VehicleInterface::vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry &msg)
{
  //RCLCPP_INFO(this->get_logger(), "Recieved vehicle_odometry");
  for(int i = 0; i<3; i++) current_position[i] = msg.position[i];
  for(int i=0; i<4; i++) current_quaternion[i] = msg.q[i];

  publish_vehicle_pose();
}


void VehicleInterface::position_setpoint_callback(const geometry_msgs::msg::PoseStamped &msg)
{
  using namespace std::chrono_literals;

  // TODO Handle Frame Transform
  float q[4] = {
    msg.pose.orientation.w, 
    msg.pose.orientation.x,
    msg.pose.orientation.y,
    msg.pose.orientation.z};

  publish_position_setpoint(
    msg.pose.position.x,
    msg.pose.position.y,
    msg.pose.position.z,
    yaw_from_quaternion(q)
  );

}


/**
 * @brief Send a command to Arm the vehicle
 */
void VehicleInterface::arm()
{
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0);
	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to disarm the vehicle
 */
void VehicleInterface::disarm()
{
  publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 4);
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0);
	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}


/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void VehicleInterface::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	px4_msgs::msg::VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = mav_id;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_pub_->publish(msg);
}

/**
 * @brief Publish the offboard control mode.
 */
void VehicleInterface::publish_offboard_control_mode()
{
	px4_msgs::msg::OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_mode_pub_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 * @param x x-coordinate in meters
 * @param y y-coordinate in meters
 * @param z z-coordinate in meters
 * @param yaw Yaw angle [-PI,PI]
 */
void VehicleInterface::publish_position_setpoint(float x, float y, float z, float yaw)
{
  publish_offboard_control_mode();
	px4_msgs::msg::TrajectorySetpoint msg{};
	msg.position = {x, y, z};
	msg.yaw = yaw; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_pub_->publish(msg);
  RCLCPP_DEBUG(this->get_logger(), "Publish trajectory setpoint [%f, %f, %f]", x,y,z);
}

float VehicleInterface::yaw_from_quaternion(float* q)
{
  return atan2(2.0*(q[1]*q[2]+q[0]*q[3]),1.0- 2.0*(q[2]*q[2] + q[3]*q[3]));
}


void VehicleInterface::publish_vehicle_pose()
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id = "NED";
  msg.header.stamp = this->get_clock()->now();
  msg.pose.position.x = double(current_position[0]);
  msg.pose.position.y = double(current_position[1]);
  msg.pose.position.z = double(current_position[2]);
  msg.pose.orientation.w = double(current_quaternion[0]);
  msg.pose.orientation.x = double(current_quaternion[1]);
  msg.pose.orientation.y = double(current_quaternion[2]);
  msg.pose.orientation.z = double(current_quaternion[3]);
  vehicle_pose_pub_->publish(msg);
}


int main(int argc, char ** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VehicleInterface>());
  rclcpp::shutdown();
  return 0;
}
