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
  this->position_setpoint_pub_ = rclcpp::create_publisher<geometry_msgs::msg::PoseStamped>(
    this,"/vehicle_interface/position_setpoint",10);

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
    RCLCPP_DEBUG(this->get_logger(),"goal->path.header.stamp.sec = %i",time_t0);
    path.clear();
      RCLCPP_DEBUG(this->get_logger(),"Recieved %d waypoints", n_waypts);
    for (int n = 0; n<n_waypts; n++){
      waypoint waypt;
      waypt.t = time_in_microseconds(goal->path.poses[n].header.stamp);
      RCLCPP_DEBUG(this->get_logger(),"path[%i].t = %f", n, waypt.t);
      waypt.x = goal->path.poses[n].pose.position.x;
      waypt.y = -goal->path.poses[n].pose.position.y;
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
    
    for(int n = 1; n < n_waypts; n++){
      double progress = 0.0;
      double yaw = atan2(path[n].y - path[n-1].x, path[n].x - path[n-1].x);
      while (progress < 1.0){
        double t_now = time_in_microseconds(this->get_clock()->now());
        progress = (t_now-path[n-1].t)/(path[n].t - path[n-1].t);
        
        if(progress >= 0.0 && progress <=1){
          float x = progress * path[n].x + (1.0 -progress) *path[n-1].x;
          float y = progress * path[n].y + (1.0 - progress)* path[n-1].y;
          float z = progress * path[n].z + (1.0 - progress) * path[n-1].z;
          publish_setpoint(x,y,z,yaw);
          RCLCPP_DEBUG(this->get_logger(),"Tracking %i progress = %f setpoint = [%f, %f, %f]", n, progress, x, y, z);
        }else if (progress > 1.0){
          publish_setpoint(path[n].x, path[n].y,path[n].z,yaw);
          RCLCPP_DEBUG(this->get_logger(),"Tracking %i progress = %f", n, progress);   
        }else{
          publish_setpoint(path[n-1].x, path[n-1].y,path[n-1].z,yaw);
          RCLCPP_DEBUG(this->get_logger(),"Tracking %i progress = %f", n-1, progress);
        }
        
        if (goal_handle->is_canceling()){
          uam_flightmanager_msgs::action::PathToPose::Result::SharedPtr msg;
          msg->final_state = msg->FAIL;
          goal_handle->canceled(msg);
          RCLCPP_INFO(this->get_logger(), "Goal Canceled");
          return;
        }
        command_rate.sleep();
    }

    }

    uam_flightmanager_msgs::action::PathToPose::Result::SharedPtr msg;
    msg->final_state = msg->SUCCESS;
    goal_handle->succeed(msg);
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


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FlightManagerServer>());
  rclcpp::shutdown();
  return 0;
}
