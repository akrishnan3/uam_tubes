#include "uam_operator/uam_operator.hpp"


using namespace uam_operator;

VehicleInterface::VehicleInterface(const rclcpp::NodeOptions & options) : rclcpp::Node("vehicle_interface", options)
{

    this->declare_parameter("vehicle_id", "1");
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/trajectory_server/path",
        10,
        std::bind(& VehicleInterface::send_path_goal, this, std::placeholders::_1)
    );
    path_client_ptr_ = rclcpp_action::create_client<PathToPose>(this,"path_to_pose");
    vehicle_id = this->get_parameter("vehicle_id").as_string();
    RCLCPP_INFO(this->get_logger(),"Launched Component uam_operator::vehicle_interface with id: %s", vehicle_id.c_str());
}


void VehicleInterface::send_path_goal(nav_msgs::msg::Path path)
{
    using namespace std::placeholders;
    RCLCPP_INFO(this->get_logger(),"Recieved path message");
    if(path.header.frame_id != vehicle_id){
        return;
    }
    auto goal_msg = PathToPose::Goal();
    goal_msg.start_action = goal_msg.TAKEOFF;
    goal_msg.end_action = goal_msg.LAND;
    goal_msg.path.header.frame_id = "map";
    goal_msg.path.header.stamp = this->get_clock()->now();
    goal_msg.path.poses = path.poses;

    auto send_goal_options = rclcpp_action::Client<PathToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback=
        std::bind(&VehicleInterface::goal_response_callback, this, _1);
    send_goal_options.result_callback=
        std::bind(&VehicleInterface::result_callback,this,_1);
    this->path_client_ptr_->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(this->get_logger(),"Sent Goal to flight manager server");
}

void VehicleInterface::goal_response_callback(const GoalHandlePathToPose::SharedPtr & goal_handle)
{
    if(!goal_handle){
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    }
    else{
        RCLCPP_INFO(this->get_logger(),"Goal Accepted, Waiting for result");
    }
}
void VehicleInterface::result_callback(const GoalHandlePathToPose::WrappedResult & result)
{
    switch (result.code){
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(this->get_logger(), "Goal was Aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(),"Goal was Camceled");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(),"Unknown result code");
            return;
    }
    RCLCPP_INFO(this->get_logger(),"Goal Returned with success");
}


RCLCPP_COMPONENTS_REGISTER_NODE(uam_operator::VehicleInterface)


