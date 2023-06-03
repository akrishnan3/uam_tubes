#include "uam_operator/operator_interface.hpp"


using namespace uam_operator;

OperatorInterface::OperatorInterface(const rclcpp::NodeOptions & options) : rclcpp::Node("operator_interface", options)
{

    this->declare_parameter("vehicle_id", 1);
    flight_plan_sub_ = this->create_subscription<uam_operator_msgs::msg::FlightPlan>(
        "/trajectory_server/flight_plan",
        10,
        std::bind(& OperatorInterface::send_path_goal, this, std::placeholders::_1)
    );
    path_client_ptr_ = rclcpp_action::create_client<PathToPose>(this,"path_to_pose");
    vehicle_id = this->get_parameter("vehicle_id").as_int();
    RCLCPP_INFO(this->get_logger(),"Launched Component uam_operator::vehicle_interface with id: %i", vehicle_id);
}


void OperatorInterface::send_path_goal(uam_operator_msgs::msg::FlightPlan flight_plan_msg)
{
    using namespace std::placeholders;
    RCLCPP_INFO(this->get_logger(),"Recieved path message");
    if(flight_plan_msg.vehicle_id != vehicle_id){
        return;
    }
    auto goal_msg = PathToPose::Goal();
    switch(flight_plan_msg.start_command){
        case flight_plan_msg.TAKEOFF : 
            goal_msg.start_action = goal_msg.TAKEOFF;
            break;
        case flight_plan_msg.LOITER : 
            goal_msg.start_action = goal_msg.LOITER;
            break;
    }
    switch(flight_plan_msg.end_command){
        case flight_plan_msg.LOITER : 
            goal_msg.end_action = goal_msg.LOITER;
            break;
        case flight_plan_msg.LAND : 
            goal_msg.end_action = goal_msg.LAND;
            break;
    }
    goal_msg.path.header.frame_id = "map";
    goal_msg.path.header.stamp = this->get_clock()->now();
    goal_msg.path.poses = flight_plan_msg.waypts.poses;

    auto send_goal_options = rclcpp_action::Client<PathToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback=
        std::bind(&OperatorInterface::goal_response_callback, this, _1);
    send_goal_options.result_callback=
        std::bind(&OperatorInterface::result_callback,this,_1);
    this->path_client_ptr_->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(this->get_logger(),"Sent Goal to flight manager server");
}

void OperatorInterface::goal_response_callback(const GoalHandlePathToPose::SharedPtr & goal_handle)
{
    if(!goal_handle){
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    }
    else{
        RCLCPP_INFO(this->get_logger(),"Goal Accepted, Waiting for result");
    }
}
void OperatorInterface::result_callback(const GoalHandlePathToPose::WrappedResult & result)
{
    switch (result.code){
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(this->get_logger(), "Goal was Aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(),"Goal was Canceled");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(),"Unknown result code");
            return;
    }
    RCLCPP_INFO(this->get_logger(),"Goal Returned with success");
}


RCLCPP_COMPONENTS_REGISTER_NODE(uam_operator::OperatorInterface)


