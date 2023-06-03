#include "uam_flightmanager/flightmanager.hpp"


using namespace uam_flightmanager;

FlightManagerServer::FlightManagerServer() : rclcpp::Node("flightmanager")
{
  using namespace std::placeholders;
  
  //---------------------- Action Server -----------------------

  this->Path_Server_ = rclcpp_action::create_server<PathToPose>(
    this,
    "path_to_pose",
    std::bind(&FlightManagerServer::handle_goal, this, _1, _2),
    std::bind(&FlightManagerServer::handle_cancel, this, _1),
    std::bind(&FlightManagerServer::handle_accepted, this, _1)
  );

  //---------------------- Publishers --------------------------
  position_setpoint_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "vehicle_interface/position_setpoint",10);

  //----------------------- Subscriptions ----------------------
  vehicle_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "vehicle_interface/vehicle_pose",
    10,
    std::bind(&FlightManagerServer::vehicle_pose_callback, this, _1)
  );
  //-----------------------ServiceClient ----------------------
  arm_service_client_ = this->create_client<vehicle_interface_msgs::srv::ArmService>(
    "vehicle_interface/arm_service"
  );
 
  //-----------------------Timers -----------------------------

  loiter_timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&FlightManagerServer::send_loiter_pose,this));

  loiter_timer_->cancel();

}

//---------------------memberfunctions--------------------

rclcpp_action::GoalResponse FlightManagerServer::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const PathToPose::Goal> goal)
{
// TODO: Check if position of first point is close to current position

  RCLCPP_INFO(this->get_logger(),"Recieved goal request");
  n_waypts = goal->path.poses.size();
  time_t0 = goal->path.header.stamp.sec;
  start_action = goal->start_action;
  end_action = goal->end_action;
  //RCLCPP_DEBUG(this->get_logger(),"goal->path.header.stamp.sec = %i",time_t0);
  path.clear();
    //RCLCPP_DEBUG(this->get_logger(),"Recieved %d waypoints", n_waypts);
  for (int n = 0; n<n_waypts; n++){
    waypoint waypt;
    waypt.t = time_in_microseconds(goal->path.poses[n].header.stamp);
    //RCLCPP_DEBUG(this->get_logger(),"path[%i].t = %f", n, waypt.t);
    // Change from ENU to NED
    waypt.x = goal->path.poses[n].pose.position.y;
    waypt.y = goal->path.poses[n].pose.position.x;
    waypt.z = -goal->path.poses[n].pose.position.z;
    path.push_back(waypt);
  }
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse FlightManagerServer::handle_cancel(
  const std::shared_ptr<GoalHandlePathToPose> goal_handle)
{
  RCLCPP_INFO(this->get_logger(),"Recieved request To cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}
  
void FlightManagerServer::handle_accepted(const std::shared_ptr<GoalHandlePathToPose> goal_handle)
{
  using namespace std::placeholders;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread(std::bind(&FlightManagerServer::execute_goal, this, _1), goal_handle).detach();
}


void FlightManagerServer::execute_goal(const std::shared_ptr<GoalHandlePathToPose> goal_handle)
{

  rclcpp::Rate command_rate(std::chrono::milliseconds(100));
  auto result_msg = std::make_shared<PathToPose::Result>();

  // Takeoff
  double yaw;
  if(start_action == uam_flightmanager_msgs::action::PathToPose::Goal::TAKEOFF){
    auto request = std::make_shared<vehicle_interface_msgs::srv::ArmService::Request>();
    request->command = request->ARM;
    if(!arm_service_client_->wait_for_service(std::chrono::seconds(1))){
      result_msg->final_state = result_msg->FAIL;
      RCLCPP_ERROR(this->get_logger(),"vehicle_interface arm_service not available");
      goal_handle->abort(result_msg);
      return;
    }
    auto result = arm_service_client_->async_send_request(request);
      if(result.get()->response != vehicle_interface_msgs::srv::ArmService::Response::VEHICLE_ARMED){
        result_msg->final_state = result_msg->FAIL;
        goal_handle->abort(result_msg);
        return;
      }
    yaw = yaw_from_quaternion(vehicle_quaternion);
    if(!takeoff_to_altitude(path[0].z, yaw)){
      land(yaw);
      RCLCPP_ERROR(this->get_logger(),"Takeoff Failed");
      return;
    }
  }else{
    loiter_timer_->cancel();
  }

  // Turn Vehicle
    yaw = atan2(path[1].y - path[0].y, path[1].x - path[0].x);
    loiter_position[0] = path[0].x;
    loiter_position[1] = path[0].y;
    loiter_position[2] = path[0].z;
    turn_to_heading(yaw,1.0);

  // Fly Path  
  for(int n = 1; n < n_waypts; n++){
    double progress = 0.0;
     yaw = atan2(path[n].y - path[n-1].y, path[n].x - path[n-1].x);
    while (progress < 1.0){
      double t_now = time_in_microseconds(this->get_clock()->now());
      progress = (t_now-path[n-1].t)/(path[n].t - path[n-1].t);
      
      if(progress >= 0.0 && progress <=1){
        float x = progress * path[n].x + (1.0 -progress) *path[n-1].x;
        float y = progress * path[n].y + (1.0 - progress)* path[n-1].y;
        float z = progress * path[n].z + (1.0 - progress) * path[n-1].z;
        publish_setpoint(x,y,z,yaw);
        //RCLCPP_DEBUG(this->get_logger(),"Tracking %i progress = %f setpoint = [%f, %f, %f]", n, progress, x, y, z);
      }else if (progress > 1.0){
        publish_setpoint(path[n].x, path[n].y,path[n].z,yaw);
        //RCLCPP_DEBUG(this->get_logger(),"Tracking %i progress = %f", n, progress);   
      }else{
        publish_setpoint(path[n-1].x, path[n-1].y,path[n-1].z,yaw);
        //RCLCPP_DEBUG(this->get_logger(),"Tracking %i progress = %f", n-1, progress);
      }
      
      if (goal_handle->is_canceling()){
        land(yaw);
        result_msg->final_state = result_msg->FAIL;
        goal_handle->canceled(result_msg);
        RCLCPP_INFO(this->get_logger(), "Goal Canceled");
        return;
      }
      command_rate.sleep();
    }
  }
  
  if(end_action == PathToPose::Goal::LOITER){
    loiter_position[0] = path[n_waypts-1].x;
    loiter_position[1] = path[n_waypts-1].y;
    loiter_position[2] = path[n_waypts-1].z;
    loiter_yaw = yaw;
    loiter_timer_-> reset();
  }else{
    if(land(yaw)){
      RCLCPP_INFO(this->get_logger(),"Vehicle Landed");
    }
  }
  if (rclcpp::ok()){
    result_msg->final_state = result_msg->SUCCESS;
    RCLCPP_INFO(this->get_logger(),"Goal Succeeded");
    goal_handle->succeed(result_msg);
  }
  return;
}


bool FlightManagerServer::takeoff_to_altitude(double height, double yaw)
{
  rclcpp::Rate wait_rate(std::chrono::milliseconds(100));
  
  int timeout_counter = 30000;
  double pos_x = vehicle_position[0];
  double pos_y = vehicle_position[1];

  while((vehicle_position[2] - height) > 0.3 && timeout_counter-- > 0 )
  {
    publish_setpoint(pos_x, pos_y, height, yaw);
    wait_rate.sleep();
  }

  if ((vehicle_position[2] - height) < 0.3){
    return true;
  }

  return false;

}

bool FlightManagerServer::land(double yaw)
{
  loiter_timer_->cancel();
  rclcpp::Rate wait_rate(std::chrono::milliseconds(200));
  double pos_x = vehicle_position[0];
  double pos_y = vehicle_position[1];
  double previous_height;
  double height_cmd = vehicle_position[2];
  int timeout_counter = 3000;
  do{
    previous_height = vehicle_position[2];
    for(int i = 0; i<10; i++)
    {
      height_cmd += 0.05;
      publish_setpoint(pos_x, pos_y, height_cmd, yaw);
      wait_rate.sleep();
    }
  } while ((vehicle_position[2]-previous_height)>0.05 && --timeout_counter > 0);

  if((vehicle_position[2]-previous_height)<0.05){
    auto request = std::make_shared<vehicle_interface_msgs::srv::ArmService::Request>();
    request->command = request->DISARM;
    auto result = arm_service_client_->async_send_request(request);
    if(result.get()->response == vehicle_interface_msgs::srv::ArmService::Response::VEHICLE_DISARMED){
      return true;
    }
  }
  return false;
}

void FlightManagerServer::turn_to_heading(double desired_yaw, double yaw_rate)
{
  
  double start_yaw = yaw_from_quaternion(vehicle_quaternion);

  double delta_yaw = desired_yaw-start_yaw;

  if(delta_yaw > 3.1415926){
    delta_yaw -= 2.0*3.1415926;
  }else if (delta_yaw < -3.1415926){
    delta_yaw += 2.0*3.1415926;
  }
  std::cout << "Turn to heading " << desired_yaw << "from " << start_yaw <<std::endl;
  std::cout << "Delta Yaw "<< delta_yaw << std::endl;
  if(abs(delta_yaw)<0.3){
    return;
  }

  rclcpp::Rate wait_rate(std::chrono::milliseconds(100));
  
  double n_steps = (abs(delta_yaw)/yaw_rate)/0.1;
  double yaw = start_yaw;
  std::cout << "n_steps " << n_steps <<std::endl;
  for (int n=0; n<n_steps; n++){
    yaw = start_yaw + (delta_yaw*n)/n_steps;

    if(yaw < -3.1415926){
      yaw += 2*3.1415926;
    }else if(yaw > 3.1415926){
      yaw -= 2*3.1415926;
    }
    //std::cout << "yaw: " << yaw << std::endl;
    publish_setpoint(loiter_position[0], loiter_position[1], loiter_position[2], yaw);
    wait_rate.sleep();
  }
  
}

void FlightManagerServer::send_loiter_pose()
{
  //RCLCPP_DEBUG(this->get_logger(),"Publish Loiter Pose [%3.2f, %3.2f, %3.2f]", loiter_position[0], loiter_position[1], loiter_position[2]);
  publish_setpoint(loiter_position[0], loiter_position[1], loiter_position[2], loiter_yaw);
}


void FlightManagerServer::publish_setpoint(double x, double y, double z, double yaw)
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id = "NED";
  msg.header.stamp = this->get_clock()->now();
  msg.pose.position.x = x;
  msg.pose.position.y = y;
  msg.pose.position.z = z;
  msg.pose.orientation. w = cos(0.5*yaw);
  msg.pose.orientation. z = sin(0.5*yaw);
  position_setpoint_pub_->publish(msg);
}

double FlightManagerServer::time_in_microseconds(builtin_interfaces::msg::Time t)
{
  return double(t.sec-time_t0)*1e6 + double(t.nanosec)/1000.0;
}

void FlightManagerServer::vehicle_pose_callback(const geometry_msgs::msg::PoseStamped &msg)
{
  vehicle_position[0] = msg.pose.position.x;
  vehicle_position[1] = msg.pose.position.y;
  vehicle_position[2] = msg.pose.position.z;
  vehicle_quaternion[0] = msg.pose.orientation.w;
  vehicle_quaternion[1] = msg.pose.orientation.x;
  vehicle_quaternion[2] = msg.pose.orientation.y;
  vehicle_quaternion[3] = msg.pose.orientation.z;
}

double FlightManagerServer::yaw_from_quaternion(double* q)
{
  return atan2(2.0*(q[1]*q[2]+q[0]*q[3]),1.0- 2.0*(q[2]*q[2] + q[3]*q[3]));
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FlightManagerServer>());
  rclcpp::shutdown();
  return 0;
}
