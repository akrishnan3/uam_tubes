#include<functional>
#include<memory>
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/timesync_status.hpp"
#include "px4_ros_com/frame_transforms.h"

class VehicleOdometryBridge : public rclcpp::Node
{

    public:
        rclcpp::QoS px4_qos_pub = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().transient_local();
        VehicleOdometryBridge() : Node("vehicle_odometry_bridge"){
            this->odometry_publisher_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
                "/fmu/in/vehicle_visual_odometry",
                px4_qos_pub
            );
            this->odometry_subscription_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
                "/model/x500_vision_0/pose",
                10,
                std::bind(& VehicleOdometryBridge::odometry_bridge,this,std::placeholders::_1)
            );
            RCLCPP_INFO(this->get_logger(),"Launched vehicle_odometry_bridge");
        }

    private:
        rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_publisher_;
        rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr odometry_subscription_;
        rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr timesync_subscription_;
        int64_t time_offset;
        void timesync_callback_(px4_msgs::msg::TimesyncStatus msg){
            time_offset = msg.timestamp + msg.observed_offset;
        }

        void odometry_bridge(tf2_msgs::msg::TFMessage msg_sub){
            using namespace px4_ros_com::frame_transforms;
            px4_msgs::msg::VehicleOdometry msg_pub;
            msg_pub.timestamp = this->get_clock()->now().nanoseconds()/1000;
            msg_pub.timestamp_sample = msg_pub.timestamp - time_offset;
            msg_pub.pose_frame = msg_pub.POSE_FRAME_FRD;
            auto translation = msg_sub.transforms[0].transform.translation;
            Eigen::Vector3d position_BaseLink = {translation.x, translation.y, translation.z};
            auto position_Aircraft = transform_static_frame(position_BaseLink, StaticTF::ENU_TO_NED);
            msg_pub.position =  {float(position_Aircraft.x()),
                                 float(position_Aircraft.y()),
                                 float(position_Aircraft.z())};
            
            auto rotation = msg_sub.transforms[0].transform.rotation;
            Eigen::Quaterniond quaternion_BaseLink(rotation.w, rotation.x, rotation.y, rotation.z);
            auto quaternion_Aircraft = ros_to_px4_orientation(quaternion_BaseLink);
            
            msg_pub.q ={float(quaternion_Aircraft.w()),
                        float(quaternion_Aircraft.x()), 
                        float(quaternion_Aircraft.y()),
                        float(quaternion_Aircraft.z())};
            msg_pub.quality = 100;
            odometry_publisher_->publish(msg_pub);
        }
};

        

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VehicleOdometryBridge>());
    rclcpp::shutdown();
    return 0;
}