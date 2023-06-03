#include <chrono>
#include <functional>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "uam_flightmanager_msgs/action/path_to_pose.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace uam_operator{

class TrajectoryServer : public rclcpp::Node
{
    public:
        TrajectoryServer(const rclcpp::NodeOptions & options);

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

}