#include <chrono>
#include <functional>
#include <vector>
#include <memory>
#include <thread>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
//#include "tf2_ros/transform_broadcaster.h"
#include "uam_flightmanager_msgs/action/path_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "vehicle_interface_msgs/srv/arm_service.hpp"

namespace uam_flightmanager
{
    using PathToPose = uam_flightmanager_msgs::action::PathToPose;
    using GoalHandlePathToPose = rclcpp_action::ServerGoalHandle<PathToPose>;

struct waypoint{
    double t;
    double x;
    double y;
    double z;
};


class FlightManagerServer : public rclcpp::Node 
{

    public:
        
        FlightManagerServer(); //Constructor

    private:

        //-------------------- Action Server -------------------
        rclcpp_action::Server<PathToPose>::SharedPtr Path_Server_;
        //-------------------- Publishers ----------------------
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr position_setpoint_pub_;
        //std::unique_ptr<tf2_ros::TransformBrodcaster()> tf_dynamic_brodcaster_;

        //--------------------Subscriptions ---------------------
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vehicle_pose_sub_;

        //--------------------Service Clients------------------

        rclcpp::Client<vehicle_interface_msgs::srv::ArmService>::SharedPtr arm_service_client_;

        //---------- Callback Function prototypes --------------

        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const PathToPose::Goal> goal
        );

        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandlePathToPose> goal_handle
        );

        void handle_accepted(const std::shared_ptr<GoalHandlePathToPose> goal_handle);

        void execute_goal(const std::shared_ptr<GoalHandlePathToPose> goal_handle);

        void vehicle_pose_callback(const geometry_msgs::msg::PoseStamped &msg);

        //-------------- Vehicle Action Modes ---------------

        bool takeoff_to_altitude(double height, double yaw);
        bool land(double yaw);

        //---------------Helper Function Prototypes-----------
        void publish_setpoint(double x, double y, double z, double yaw);
        double time_in_microseconds(builtin_interfaces::msg::Time t);
        //---------------------Variables --------------------
        
        int n_waypts;
        int time_t0;
        std::vector<waypoint> path;
        uint8_t start_action;
        uint8_t end_action;
        double vehicle_position[3];
        double vehicle_quaternion[4];

};
}