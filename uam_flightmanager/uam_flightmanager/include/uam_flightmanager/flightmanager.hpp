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

        //-----------------------Timers-------------------------

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

        //void vehicle_command_ack_callback(const px4_msgs::msg::VehicleCommandAck &msg);

        //---------------Helper Function Prototypes-----------
        void publish_setpoint(double x, double y, double z, double yaw);
        double time_in_microseconds(builtin_interfaces::msg::Time t);
        //---------------------Variables --------------------
        
        int n_waypts;
        int time_t0;
        std::vector<waypoint> path;

};
}