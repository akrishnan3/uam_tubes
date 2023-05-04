#include <chrono>
#include <functional>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"


class VehicleInterface : public rclcpp::Node
{

    public:
        VehicleInterface(); //Constructor
    private:
        //-------------------Publishers-----------------
        rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
        rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
        rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;

        //-----------------Subscriptions----------------
        rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
        rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr position_setpoint_sub_;
        
        //------------------- Timers --------------------
            rclcpp::TimerBase::SharedPtr timer_;

        //-------------------Call Backs------------------

        void vehicle_status_callback(const px4_msgs::msg::VehicleStatus &msg);
        void vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry &msg);
        void position_setpoint_callback(const geometry_msgs::msg::PoseStamped &msg);

        // ----------------- State ----------------------

        uint8_t current_arming_state;
        uint8_t current_nav_state;
        float current_position[3];
        float current_quaternion[4];


        // --------------- Helper Functions --------------
        
        void arm();
        void disarm();
        void publish_offboard_control_mode();
        void publish_vehicle_command(uint16_t command, float param1, float param2);
        void publish_position_setpoint(float x, float y, float z, float yaw);
        float yaw_from_quaternion(float* q);
};      