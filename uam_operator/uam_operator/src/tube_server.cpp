#include "uam_operator/tube_server.hpp"

using namespace tube_trajectory;
using namespace uam_operator;

TubeServer::TubeServer(const rclcpp::NodeOptions & options) : rclcpp::Node("tube_server", options)
{
    using namespace std::chrono_literals;
    //-------------------Parameters-----------------------
    this->declare_parameter("F_star", 1.0);
    this->declare_parameter("d_star",1.0);
    this->declare_parameter("alpha", 1.0);
    this->declare_parameter("spring_const",100.0);
    this->declare_parameter("damp_const",10.0);

    std::vector<double> default_obs{0.0};
    this->declare_parameter("num_obstacles", 0);
    this->declare_parameter("obstacles_x",default_obs);
    this->declare_parameter("obstacles_y",default_obs);
    this->declare_parameter("obstacles_h",default_obs);
    this->declare_parameter("obstacles_w",default_obs);
    this->declare_parameter("obstacles_d",default_obs);

    this->declare_parameter("num_walls", 0);
    this->declare_parameter("walls_a", default_obs);
    this->declare_parameter("walls_b", default_obs);
    this->declare_parameter("walls_c", default_obs);
    this->declare_parameter("walls_d", default_obs);


    std::vector<double> default_vertiport{0.0, 3.0};
    this->declare_parameter("num_vertiports", 1);
    this->declare_parameter("vertiports_x", default_vertiport);
    this->declare_parameter("vertiports_y", default_vertiport);
    this->declare_parameter("takeoff_height", 1.0);

    std::vector<double> default_bounds_x{-2.0, 2.0};
    std::vector<double> default_bounds_y{-4.0, 4.0};
    std::vector<double> default_bounds_z{1.0, 3.0};

    this->declare_parameter("bounds_x", default_bounds_x);
    this->declare_parameter("bounds_y", default_bounds_y);
    this->declare_parameter("bounds_z", default_bounds_z);

    this->declare_parameter("num_total_segments", 5);

    //-------------------- Timers ------------------------
    obstacle_pub_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1100), 
        std::bind(&TubeServer::publish_obstacles, this)
    );
    path_timer_ = this->create_wall_timer(
        15s,
        std::bind(&TubeServer::generate_paths, this)
    );

    re.seed(11);

    //------------------ Publishers ----------------------
    obstacle_viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("trajectory_server/obstacle_markers",10);
    flight_plan_pub_ = this->create_publisher<uam_operator_msgs::msg::FlightPlan>("/trajectory_server/flight_plan",10);
    load_obstacles();
    load_initial_positions();
}

void TubeServer::load_initial_positions()
{
    num_vehicles = this->get_parameter("num_vertiports").as_int();
    vertiports_x = this->get_parameter("vertiports_x").as_double_array();
    vertiports_y = this->get_parameter("vertiports_y").as_double_array();
    takeoff_height = this->get_parameter("takeoff_height").as_double();
    for (int i = 0; i<num_vehicles; i++){
        vehicle_positions.push_back(Eigen::Vector3d(vertiports_x[i],vertiports_y[i],takeoff_height));
        vehicle_status.push_back(vehicle_states::ready_for_takeoff);
    }
}

void TubeServer::add_random_path(int id)
{

    auto F_star = this->get_parameter("F_star").as_double();
    auto d_star = this->get_parameter("d_star").as_double();
    auto alpha = this->get_parameter("alpha").as_double();
    auto spring_const = this->get_parameter("spring_const").as_double();
    auto damp_const = this->get_parameter("damp_const").as_double();

    Eigen::Vector3d start = vehicle_positions[id];
    Eigen::Vector3d end;
    bool point_inside_obstacle = false;
    double dist;
    std::shared_ptr<trajectory> path;
    for(int i=0; i<10; i++){
        
        if (vehicle_status[id] == vehicle_states::landing_requested){
            double approach_z = bounds_z_min + unif(this->re)*(bounds_z_max - bounds_z_min);
            end = Eigen::Vector3d(vertiports_x[id], vertiports_y[id], approach_z);
        } else {
            end = start;
        
            dist = (end-start).norm();
            point_inside_obstacle = false;

            while(dist< 2.5 || dist > 5 || point_inside_obstacle){
                
                end = Eigen::Vector3d(
                    bounds_x_min + unif(this->re)*(bounds_x_max - bounds_x_min),
                    bounds_y_min + unif(this->re)*(bounds_y_max - bounds_y_min),
                    bounds_z_min + unif(this->re)*(bounds_z_max - bounds_z_min)
                );

                dist = (end-start).norm();

                point_inside_obstacle = false;
                for(auto obs: obstacles)
                {
                    if(obs->check_inclusion(end))
                    {
                        point_inside_obstacle = true;
                    }
                }
            }

        }

        int n_beads = int((end-start).norm()/0.5);
        if (n_beads < 4)
        {
            n_beads = 4;
        }
        
        path = std::make_shared<trajectory>(start, end, this->obstacles, n_beads, spring_const, damp_const, F_star,alpha,d_star);
        path->converge_trajectory();
        if(path->check_trajectory()){
            break;
        }
    }

    path->set_color(0.0, 1.0 - double(id)/num_vehicles, double(id)/num_vehicles);

    RCLCPP_INFO(this->get_logger(),"Generate Path for vehicle: %i from:[%3.2f, %3.2f, %3.2f] to: [%3.2f, %3.2f, %3.2f] ", id, start[0], start[1], start[2], end[0], end[1], end[2]);

    obstacles.push_back(path);
    vehicle_positions[id]=end;

    double t_padd = 0.0;
    uam_operator_msgs::msg::FlightPlan flight_plan_msg;
    switch(vehicle_status[id])
    {
        case vehicle_states::ready_for_takeoff: 
            flight_plan_msg.start_command = flight_plan_msg.TAKEOFF;
            flight_plan_msg.end_command = flight_plan_msg.LOITER;
            t_padd = 10.0;
            vehicle_status[id] = vehicle_states::flying;
            break;
        case vehicle_states::flying:
            flight_plan_msg.start_command = flight_plan_msg.LOITER;
            flight_plan_msg.end_command = flight_plan_msg.LOITER;
            vehicle_status[id] = vehicle_states::flying;
            t_padd = 5.0;
            break;
        case vehicle_states::landing_requested:
            flight_plan_msg.start_command = flight_plan_msg.LOITER;
            flight_plan_msg.end_command = flight_plan_msg.LAND;
            t_padd = 5.0;
            vehicle_status[id] = vehicle_states::ready_for_takeoff;
            break;
    }

    auto t0 = this->get_clock()->now().seconds()+ t_padd;
    path->constructPathMsgFromObject(flight_plan_msg.waypts,t0,0.25);
    flight_plan_msg.vehicle_id = uint8_t(id+1);

    flight_plan_pub_->publish(flight_plan_msg);

}


void TubeServer::load_obstacles()
{
    auto F_star = this->get_parameter("F_star").as_double();
    auto d_star = this->get_parameter("d_star").as_double();
    auto alpha = this->get_parameter("alpha").as_double();

    int nObs = this->get_parameter("num_obstacles").as_int();
    auto obstacles_x = this->get_parameter("obstacles_x").as_double_array();
    auto obstacles_y = this->get_parameter("obstacles_y").as_double_array();
    auto obstacles_h = this->get_parameter("obstacles_h").as_double_array();
    auto obstacles_d = this->get_parameter("obstacles_d").as_double_array();
    auto obstacles_w = this->get_parameter("obstacles_w").as_double_array();
    RCLCPP_INFO(this->get_logger(),"Loading %i Obstacles", nObs);
    for(int i=0; i<nObs; i++){
        auto bldg = std::make_shared<building>(obstacles_x[i],obstacles_y[i], obstacles_h[i], obstacles_d[i], obstacles_w[i],F_star,alpha,d_star);
        bldg->set_color(0.8,0.8,0.0);
        bldg->set_padding(d_star/2);
        obstacles.push_back(bldg);
    }

    int nWalls = this->get_parameter("num_walls").as_int();
    auto walls_a = this->get_parameter("walls_a").as_double_array();
    auto walls_b = this->get_parameter("walls_b").as_double_array();
    auto walls_c = this->get_parameter("walls_c").as_double_array();
    auto walls_d = this->get_parameter("walls_d").as_double_array();
    RCLCPP_INFO(this->get_logger(),"Loading %i Walls", nWalls);
    for(int i=0; i<nWalls; i++){
        auto wall = std::make_shared<plane>(walls_a[i],walls_b[i],walls_c[i],walls_d[i],F_star,alpha,d_star);
        obstacles.push_back(wall);
    }

    
    num_static_obstacles = nObs+nWalls;

    auto bounds_x = this->get_parameter("bounds_x").as_double_array();
    auto bounds_y = this->get_parameter("bounds_y").as_double_array();
    auto bounds_z = this->get_parameter("bounds_z").as_double_array();
    bounds_x_min = bounds_x[0];
    bounds_x_max = bounds_x[1];
    bounds_y_min = bounds_y[0];
    bounds_y_max = bounds_y[1];
    bounds_z_min = bounds_z[0];
    bounds_z_max = bounds_z[1];


}


void TubeServer::publish_obstacles()
{
    visualization_msgs::msg::MarkerArray obstacle_array;
    int n = 0;
    for(auto obstacle : obstacles){
        visualization_msgs::msg::Marker marker;
        obstacle->constructMarkersFromObject(marker);
        marker.header.stamp = this->get_clock()->now();
        marker.header.frame_id = "map";
        marker.id = 10+n;
        marker.action = marker.ADD;
        marker.lifetime.sec = 5.0;
        obstacle_array.markers.push_back(marker);
        n++;
    }

    obstacle_viz_pub_ ->publish(obstacle_array);

}

void TubeServer::generate_paths()
{

    if (segments_flown > this->get_parameter("num_total_segments").as_int()){
        vehicle_status[vehicle_id] = vehicle_states::landing_requested;
        if (segments_flown > this->get_parameter("num_total_segments").as_int()+ num_vehicles+1){
            path_timer_->cancel();
        }
    }



    if(++vehicle_id >= num_vehicles){
        vehicle_id = 0;
    }

    if (obstacles.size() >= num_static_obstacles+num_vehicles){
        auto itr = obstacles.begin();
        std::advance(itr,num_static_obstacles);
        obstacles.erase(itr);
    }



    add_random_path(vehicle_id);

    segments_flown++;

}

RCLCPP_COMPONENTS_REGISTER_NODE(uam_operator::TubeServer);

//int main(int argc, char ** argv)
//{
//  rclcpp::init(argc, argv);
//  rclcpp::spin(std::make_shared<TubeServer>());
//  rclcpp::shutdown();
//  return 0;
//}
