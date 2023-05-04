#include <chrono>
#include <functional>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "uam_flightmanager_msgs/action/path_to_pose.hpp"

namespace uam_operator{
using PathToPose = uam_flightmanager_msgs::action::PathToPose;
using GoalHandlePathToPose = rclcpp_action::ClientGoalHandle<PathToPose>;

class TubeTrajectoryServer : public rclcpp::Node
{
    public:
        TubeTrajectoryServer();

    private:
    rclcpp_action::Client<PathToPose>::SharedPtr path_client_ptr_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_viz_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr paths_viz_pub_;
    rclcpp::TimerBase::SharedPtr obstacle_pub_timer_;
    rclcpp::TimerBase::SharedPtr send_goal_timer_;
    
    //-----------------Callback Functions------------
    void send_path_goal();
    void goal_response_callback(const GoalHandlePathToPose::SharedPtr & goal_handle);
    void result_callback(const GoalHandlePathToPose::WrappedResult & result);

     
    void publish_vizualization();
    void publish_obstacles();
    void publish_paths();

};
}