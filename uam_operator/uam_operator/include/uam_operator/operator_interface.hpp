#include <chrono>
#include <functional>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "uam_operator_msgs/msg/flight_plan.hpp"
#include "uam_flightmanager_msgs/action/path_to_pose.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace uam_operator{
using PathToPose = uam_flightmanager_msgs::action::PathToPose;
using GoalHandlePathToPose = rclcpp_action::ClientGoalHandle<PathToPose>;

class OperatorInterface : public rclcpp::Node
{
    public :
        OperatorInterface(const rclcpp::NodeOptions & options);
    private:
        uint8_t vehicle_id;
        rclcpp::Subscription<uam_operator_msgs::msg::FlightPlan>::SharedPtr flight_plan_sub_;
        rclcpp_action::Client<PathToPose>::SharedPtr path_client_ptr_;
        void send_path_goal(uam_operator_msgs::msg::FlightPlan flight_plan_msg);
        void goal_response_callback(const GoalHandlePathToPose::SharedPtr & goal_handle);
        void result_callback(const GoalHandlePathToPose::WrappedResult & result);
};

}