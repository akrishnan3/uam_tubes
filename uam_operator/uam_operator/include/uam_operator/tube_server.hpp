#include <chrono>
#include <functional>
#include <vector>
#include <memory>
#include <random>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_stl_containers/eigen_stl_containers.h>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "uam_operator_msgs/msg/flight_plan.hpp"
#include "uam_operator/tube_trajectory.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace uam_operator
{
using namespace tube_trajectory;

enum class vehicle_states{ready_for_takeoff,flying,landing_requested};

class TubeServer : public rclcpp::Node
{
    public:
        TubeServer(const rclcpp::NodeOptions & options);

    private:

        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_viz_pub_;
        rclcpp::Publisher<uam_operator_msgs::msg::FlightPlan>::SharedPtr flight_plan_pub_;
        rclcpp::TimerBase::SharedPtr obstacle_pub_timer_;
        rclcpp::TimerBase::SharedPtr path_timer_;
        
        void publish_obstacles();
        void generate_paths();

        void load_obstacles();
        void load_initial_positions();

        void add_random_path(int id);

        std::vector<std::shared_ptr<obstacle>> obstacles;
        std::vector<double> vertiports_x;
        std::vector<double> vertiports_y;
        double takeoff_height;
        
        int num_static_obstacles;
        int num_vehicles;
        int vehicle_id = -1;

        EigenSTL::vector_Vector3d vehicle_positions;
        std::vector<vehicle_states> vehicle_status;
        int segments_flown = 0;
        std::uniform_real_distribution<double> unif;
        std::default_random_engine re;

        double bounds_x_min;
        double bounds_y_min;
        double bounds_z_min;
        double bounds_x_max;
        double bounds_y_max;
        double bounds_z_max;

};

}