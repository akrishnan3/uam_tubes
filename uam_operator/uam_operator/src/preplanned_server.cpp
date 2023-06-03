#include "uam_operator/preplanned_server.hpp"


using namespace uam_operator;

TrajectoryServer::TrajectoryServer(const rclcpp::NodeOptions & options) : rclcpp::Node("preplanned_server", options)
{

using namespace std::chrono_literals;
//--------------------------- Parameters -------------------------

std::vector<double> default_obs{0.0};
std::vector<int> default_path_n{2};
std::vector<double> default_path{0.0,1.0};

this->declare_parameter("num_vehicles",1);

this->declare_parameter("num_obstacles", 0);
this->declare_parameter("obstacles_x",default_obs);
this->declare_parameter("obstacles_y",default_obs);
this->declare_parameter("obstacles_h", default_obs);
this->declare_parameter("obstacles_d",default_obs);
this->declare_parameter("obstacles_w", default_obs);

this->declare_parameter("num_paths", 0);
this->declare_parameter("paths_n",default_path_n);
this->declare_parameter("paths_x",default_path);
this->declare_parameter("paths_y", default_path);
this->declare_parameter("paths_z", default_path);

//------------------------- Timers ------------------------------

obstacle_pub_timer_ = this->create_wall_timer(5s, std::bind(&TrajectoryServer::publish_vizualization, this));
send_goal_timer_ = this->create_wall_timer(7s, std::bind(&TrajectoryServer::publish_goals,this));

//-------------------------- Publishers ------------------------
path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/trajectory_server/path",10);
obstacle_viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/tube_trajectory_server/obstacle_markers",10);
paths_viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/tube_trajectory_server/path_markers",10);

RCLCPP_INFO(this->get_logger(),"Launched Component: trajectory_server");

}

void TrajectoryServer::publish_goals()
{
    this->send_goal_timer_->cancel();

    int num_paths = this->get_parameter("num_paths").as_int();
    auto paths_n = this->get_parameter("paths_n").as_integer_array();
    auto paths_x = this->get_parameter("paths_x").as_double_array();
    auto paths_y = this->get_parameter("paths_y").as_double_array();
    auto paths_z = this->get_parameter("paths_z").as_double_array();
    int j = 0;
    for (int n=0; n<num_paths; n++){
        nav_msgs::msg::Path msg;
        msg.header.frame_id = std::to_string(n+1);
        int t0 = this->get_clock()->now().seconds() + 5;
        for(int i = 0; i<paths_n[n]; i++){
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.header.stamp.sec = t0 + 5 *(i+1);
            pose.pose.position.x = paths_x[j+i];
            pose.pose.position.y = paths_y[j+i];
            pose.pose.position.z = paths_z[j+i];
            msg.poses.push_back(pose);
        }
        path_pub_->publish(msg);
        j += paths_n[n];
    }

}

void TrajectoryServer::publish_vizualization(){
    this->publish_obstacles();
    this->publish_paths();
}

void TrajectoryServer::publish_obstacles()
{
        int nObs = this->get_parameter("num_obstacles").as_int();
        auto obstacles_x = this->get_parameter("obstacles_x").as_double_array();
        auto obstacles_y = this->get_parameter("obstacles_y").as_double_array();
        auto obstacles_h = this->get_parameter("obstacles_h").as_double_array();
        auto obstacles_d = this->get_parameter("obstacles_d").as_double_array();
        auto obstacles_w = this->get_parameter("obstacles_w").as_double_array();

        visualization_msgs::msg::MarkerArray obstacle_array;

        for(int n = 0; n < nObs; n++){
            visualization_msgs::msg::Marker obstacle;
            obstacle.header.frame_id = "map";
            obstacle.header.stamp = this->get_clock()->now();
            obstacle.id = n;
            obstacle.pose.position.x = obstacles_x[n];
            obstacle.pose.position.y = obstacles_y[n];
            obstacle.pose.position.z = 0.5 * obstacles_h[n];
            obstacle.pose.orientation.w = 1.0;
            obstacle.pose.orientation.x = 0.0;
            obstacle.pose.orientation.y = 0.0;
            obstacle.pose.orientation.z = 0.0;
            obstacle.type = obstacle.CUBE;
            obstacle.scale.x = obstacles_d[n];
            obstacle.scale.y = obstacles_w[n];
            obstacle.scale.z = obstacles_h[n];
            obstacle.color.r = 0.8;
            obstacle.color.g = 0.8;
            obstacle.color.b = 0.0;
            obstacle.color.a = 1.0;
            obstacle.action = obstacle.ADD;
            obstacle.lifetime.sec = 5.0;
            obstacle_array.markers.push_back(obstacle);
        }
        obstacle_viz_pub_->publish(obstacle_array);
        RCLCPP_DEBUG(this->get_logger(),"Published obstacle markers");
}

void TrajectoryServer::publish_paths()
{

    int num_paths = this->get_parameter("num_paths").as_int();
    auto paths_n = this->get_parameter("paths_n").as_integer_array();
    auto paths_x = this->get_parameter("paths_x").as_double_array();
    auto paths_y = this->get_parameter("paths_y").as_double_array();
    auto paths_z = this->get_parameter("paths_z").as_double_array();

    visualization_msgs::msg::MarkerArray paths_markers;

    int p = 0;

    for(int k = 0; k < num_paths; k++){

        visualization_msgs::msg::Marker trajectory;

        trajectory.header.frame_id = "map";
        trajectory.header.stamp = this->get_clock()->now();
        trajectory.id = 100 + k;
        trajectory.pose.position.x = 0.0;
        trajectory.pose.position.y = 0.0;
        trajectory.pose.position.z = 0.0;
        trajectory.pose.orientation.w = 1.0;
        trajectory.pose.orientation.x = 0.0;
        trajectory.pose.orientation.y = 0.0;
        trajectory.pose.orientation.z = 0.0;
        trajectory.type = trajectory.LINE_STRIP;
        trajectory.scale.x = 0.1;

        for(int i=0; i< paths_n[k]; i++){
            geometry_msgs::msg::Point point;
            std_msgs::msg::ColorRGBA color;
            point.x =  paths_x[p+i];
            point.y =  paths_y[p+i];
            point.z =  paths_z[p+i];

            color.a = 1.0;
            color.r = 1.0;

            trajectory.points.push_back(point);
            trajectory.colors.push_back(color);
        }
        
        p += paths_n[k];

        trajectory.action = trajectory.ADD;
        trajectory.lifetime.sec = 5.0;

        paths_markers.markers.push_back(trajectory);
    }
    paths_viz_pub_->publish(paths_markers);
    RCLCPP_DEBUG(this->get_logger(),"Published Path markers");
}

RCLCPP_COMPONENTS_REGISTER_NODE(uam_operator::TrajectoryServer)


