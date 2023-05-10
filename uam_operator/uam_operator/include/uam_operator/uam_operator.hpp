#include <chrono>
#include <functional>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "uam_flightmanager_msgs/action/path_to_pose.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace uam_operator{
using PathToPose = uam_flightmanager_msgs::action::PathToPose;
using GoalHandlePathToPose = rclcpp_action::ClientGoalHandle<PathToPose>;

class TubeTrajectoryServer : public rclcpp::Node
{
    public:
        TubeTrajectoryServer(const rclcpp::NodeOptions & options);

    private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_viz_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr paths_viz_pub_;
    rclcpp::TimerBase::SharedPtr obstacle_pub_timer_;
    rclcpp::TimerBase::SharedPtr send_goal_timer_;
    
    //-----------------Callback Functions------------
     
    void publish_vizualization();
    void publish_obstacles();
    void publish_paths();
    void publish_goals();

};

class VehicleInterface : public rclcpp::Node
{
    public :
        VehicleInterface(const rclcpp::NodeOptions & options);
    private:
        std::string vehicle_id;
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
        rclcpp_action::Client<PathToPose>::SharedPtr path_client_ptr_;
        void send_path_goal(nav_msgs::msg::Path path);
        void goal_response_callback(const GoalHandlePathToPose::SharedPtr & goal_handle);
        void result_callback(const GoalHandlePathToPose::WrappedResult & result);
};

}