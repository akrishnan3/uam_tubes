#include "uam_operator/tube_trajectory.hpp"

using namespace tube_trajectory;

//---------------------------------------- Obstacle ---------------------------------------

double obstacle::sigmoid_function(double d)
{
    return F_star * (1.0 - 1.0/(1.0+std::exp(-alpha*(d-d_star))));
}

Eigen::Vector3d obstacle::compute_repulsive_force(const Eigen::Vector3d &query_point)
{
    //std::cout << "obstacle::compute_repulsive_force" <<std::endl;
    return {0.0,0.0,0.0};
}

bool obstacle::check_intersection(const Eigen::Matrix3Xd &path)
{
    return false;
}
bool obstacle::check_inclusion(const Eigen::Vector3d &point)
{
    return false;
}

void obstacle::constructMarkersFromObject(visualization_msgs::msg::Marker &marker){}

void obstacle::set_color (double r, double g, double b)
{
    color[0] = r;
    color[1] = g;
    color[2] = b;
}

//---------------------------------------- Building ---------------------------------------

building::building(double x=0, double y=0, double h=1.0, double w=1.0, double d=1.0,
                double FStar=1.0, double Alpha=1.0, double DStar=1.0) : obstacle(FStar,Alpha,DStar)
{
    const shapes::Shape* box = new shapes::Box(w,d,h);
    bounding_box = new bodies::Box(box);
    bounding_box->setPose(Eigen::Translation3d(x,y,h/2)*Eigen::Quaterniond(1,0,0,0));
    geometric_box = new bodies::Box(*(bounding_box));
    centroids.push_back(Eigen::Vector3d(x,y,h));
    centroids.push_back(Eigen::Vector3d(x,y,h/2));
}

Eigen::Vector3d building::compute_repulsive_force(const Eigen::Vector3d &query_point)
{   
    Eigen::Vector3d repulsive_force = {0,0,0};
    for (auto centroid : centroids){
        double d = (query_point-centroid).norm();
        auto F = this->sigmoid_function(d);
        repulsive_force += (query_point-centroid).normalized()*F;
    }
    //std::cout << "building repulsive_force " << F <<std::endl;
    return repulsive_force;
    
}
 
bool building::check_intersection(const Eigen::Matrix3Xd &path)
{

    int size_of_path = path.outerSize();
    EigenSTL::vector_Vector3d intersects;
    int intersect_count = 1;
    for(int i=0; i+1<size_of_path; i++){
        auto origin = path.col(i);
        auto r_vec = path.col(i+1)-path.col(i);
        if(bounding_box->intersectsRay(origin,r_vec, &intersects,intersect_count)){
            if((intersects[0]-origin).norm() < r_vec.norm()){
                return true;
            }
        }
    }
    return false;
}

bool building::check_inclusion(const Eigen::Vector3d &point)
{
    return bounding_box->containsPoint(point);
}

void building::constructMarkersFromObject(visualization_msgs::msg::Marker &marker)
{
    //std::cout << "Visualize Building" << std::endl;
    bodies::constructMarkerFromBody(this->geometric_box, marker);
    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = 1.0;
}

void building::set_padding(double padding)
{
    bounding_box->setPadding(padding);
}



// ------------------------------------- plane --------------------------------------------


Eigen::Vector3d plane::compute_repulsive_force(const Eigen::Vector3d &query_point)
{
    auto unit_normal = Eigen::Vector3d(A,B,C).normalized();
    auto origin = -unit_normal*D;
    double d = (query_point-origin).dot(unit_normal);
    return unit_normal * this->sigmoid_function(d);
}

bool plane::check_intersection(const Eigen::Matrix3Xd &path)
{
    
    for (auto point : path.colwise()){
        if (A*point(0) + B*point(1) + C*point(2) + D < 0){
            return true;
        }
    }
        
    return false;
}

// ---------------------------------- trajectory -------------------------------------------

trajectory::trajectory(Eigen::Vector3d &start, Eigen::Vector3d &end,  std::vector<std::shared_ptr<obstacle>> obstacle_list, int n_beads=3,
double spring_const=1.0, double damping_const=1.0,double FStar=1.0, double Alpha=1.0, double DStar=1.0) : 
obstacle(FStar,Alpha,DStar), obstacleList{obstacle_list}, springConst{spring_const}, dampingConst{damping_const}
{
    //std::cout << "Recieved Call to Trajectory Constructor with "<< n_beads << " beads" << std::endl;
    position.setConstant(3,n_beads,0.0);
    velocity.setConstant(3,n_beads,0.0);
    acceleration.setConstant(3,n_beads,0.0);
    //std::cout << "Initialized position, velocity, acceleration with zeros" << std::endl;

    for(int i = 0; i<n_beads; i++){
        double eta = double(i)/double(n_beads-1);
        position.col(i) = start*(1.0-eta) + end*eta;
    }
    //std::cout << "Linearly Interpolated Bead positions" << std::endl;

}

trajectory::trajectory()
{
    position.setConstant(3,2,0.0);
    velocity.setConstant(3,2,0.0);
    acceleration.setConstant(3,2,0.0);
    springConst = 1.0;
    dampingConst = 1.0;
}


Eigen::Vector3d trajectory::compute_repulsive_force(const Eigen::Vector3d &query_point)
{
    Eigen::Vector3d force(0,0,0);
    int n_beads = position.outerSize();
    for (int i = 0; i<n_beads; i++){
        double d = (position.col(i) - query_point).norm();
        force += (query_point - position.col(i))*(this->sigmoid_function(d)/d);
    }

    return force;
}


bool trajectory::check_intersection(const Eigen::Matrix3Xd &path)
{

    for (auto waypt : path.colwise()){
        for (auto bead : position.colwise()){
            if((waypt - bead).squaredNorm() < (this->d_star *  this->d_star)){
                return true;
            }
        }

    }
    return false;
    
}

bool trajectory::check_inclusion(const Eigen::Vector3d &point)
{
    for (auto bead : position.colwise()){
        if((point - bead).squaredNorm() < (this->d_star *  this->d_star)){
            return true;
        }
    }
    return false;
}

bool trajectory::check_trajectory()
{
    for(auto obs: obstacleList){
        if(obs->check_intersection(position)){
            return false;
        }
    }
    return true;
}

void trajectory::converge_trajectory()
{
    int max_iter = 1000;
    double step_size = 0.001;
    double vel_tol = 0.1;
    double acc_tol = 10;
    for(int i = 0; i<max_iter; i++)
    {
        rk4_step(step_size);
        if((velocity.colwise().norm()).maxCoeff() < vel_tol && (acceleration.colwise().norm()).maxCoeff() < acc_tol){
            std::cout << "converged in " << i << " iterations" << std::endl;
            return;
        }
    }

    std::cout << "convergence reached iteration limit" << std::endl;

}


Eigen::Matrix3Xd trajectory::rubber_band_ode(const Eigen::Matrix3Xd &X, const Eigen::Matrix3Xd &Xdot)
{   
    //std::cout << "Rubber Band ODE" << std::endl;
    int n_beads = X.outerSize();
    //std::cout << "n_beads = " << n_beads << std::endl;
    Eigen::Matrix3Xd Xddot;
    Xddot.setConstant(3,n_beads,0.0);

    for(int i=0; i < n_beads; i++){
        // External Forces
        for(auto obs : obstacleList){
            Xddot.col(i) += obs->compute_repulsive_force(X.col(i));
        }
        // Internal Forces

        if (i==0)
        {
            //std::cout << "spring forces bead " << i << std::endl;
            Xddot.col(i) += (position.col(0) + X.col(i+1) - X.col(i)*2 )*springConst;
        }
        else if (i==n_beads-1)
        {
            //std::cout << "spring forces bead " << i << std::endl;
            Xddot.col(i) += (X.col(i-1) + position.rightCols(1) - X.col(i)*2 )*springConst;
        }
        else
        {
            //std::cout << "spring forces bead " << i << std::endl;
            Xddot.col(i) += (X.col(i+1) + X.col(i-1) - X.col(i)*2 )*springConst;
        }
        //std::cout << "damping force bead " << i  << std::endl;
        Xddot.col(i) += Xdot.col(i) * -(Xdot.col(i).norm()*dampingConst);

    }
    
    return Xddot;
}

void trajectory::rk4_step(double step_size)
{   
    //std::cout << "RK4 Step" << std::endl;
    double rkAlfa[] = {1.0, 2.0, 2.0, 1.0};
    double rkTau[] = {0, 0.5, 0.5, 1.0};
    double rkEta = 6.0;

    Eigen::Matrix3Xd dX;
    Eigen::Matrix3Xd dXdot;

    auto X_minus = position.middleCols(1,position.outerSize()-2);
    auto X = X_minus;
    auto Xdot_minus = velocity.middleCols(1,position.outerSize()-2);
    auto Xdot = Xdot_minus;
    auto Xdot_copy = Xdot;
    auto Xddot = rubber_band_ode(X,Xdot);
    //std::cout << "Start Iteration";
    for (int k = 0; k<4; k++){
        //std::cout << " " << k;
        if (k>0){
            Xdot_copy = Xdot;
            Xdot =  Xdot_minus + Xddot*rkTau[k]*step_size;
            Xddot = rubber_band_ode(X_minus+Xdot_copy*rkTau[k]*step_size,Xdot);
        }

        position.middleCols(1,position.outerSize()-2) += Xdot * (step_size*rkAlfa[k]/rkEta);
        velocity.middleCols(1,position.outerSize()-2) += Xddot * (step_size*rkAlfa[k]/rkEta);
    }
    //std::cout << std::endl;

    acceleration.middleCols(1,position.outerSize()-2) = Xddot;

}

void trajectory::constructMarkersFromObject(visualization_msgs::msg::Marker &path_marker)
{
    int n_beads = position.outerSize();
    //std::cout << "Visualize " << n_beads << " beads" << std::endl;
    for(int i=0; i<n_beads; i++){
        geometry_msgs::msg::Point pt;
        pt.x = position(0,i);
        pt.y = position(1,i);
        pt.z = position(2,i);
        path_marker.points.push_back(pt);
    }
    path_marker.type = path_marker.LINE_STRIP;
    path_marker.scale.x = 0.1;
    path_marker.color.a = 1.0;
    path_marker.color.r = color[0];
    path_marker.color.g = color[1];
    path_marker.color.b = color[2];

}

void trajectory::constructPathMsgFromObject(nav_msgs::msg::Path &path_msg, int t0, double speed)
{
    int n_beads = position.outerSize();
    auto t = t0;
    for(int i=0; i<n_beads; i++){
        if (i>0){
            t += int((position.col(i) - position.col(i-1)).norm()/speed);
        }
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = position(0,i);
        pose.pose.position.y = position(1,i);
        pose.pose.position.z = position(2,i);
        pose.header.stamp.sec = t;
        path_msg.poses.push_back(pose);
    }
}

//------------------------------------------------------------------------------------------