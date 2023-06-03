#include <vector> 
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_stl_containers/eigen_stl_containers.h>

#include "geometric_shapes/bodies.h"
#include "geometric_shapes/body_operations.h"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/path.hpp"

namespace tube_trajectory
{

class obstacle
{
    public:
        obstacle(double FStar=1.0, double Alpha=1.0, double DStar=1.0): F_star{FStar}, alpha{Alpha}, d_star{DStar}{};
        virtual Eigen::Vector3d compute_repulsive_force(const Eigen::Vector3d &query_point);
        virtual bool check_intersection(const Eigen::Matrix3Xd &path);
        virtual bool check_inclusion(const Eigen::Vector3d &point);
        virtual void constructMarkersFromObject(visualization_msgs::msg::Marker &marker);
        void set_color (double r, double g, double b);

    protected:
        double sigmoid_function(double d);
        double color[3] = {0,0,0};
        double F_star;
        double alpha;
        double d_star;
        
};

class building : public obstacle
{
    public:
        building(double x, double y, double h, double w, double d,
                double FStar, double Alpha, double DStar);
        Eigen::Vector3d compute_repulsive_force(const Eigen::Vector3d &query_point);
        bool check_intersection(const Eigen::Matrix3Xd &path);
        bool check_inclusion(const Eigen::Vector3d &point);
        void constructMarkersFromObject(visualization_msgs::msg::Marker &marker);
        void set_padding(double padding);
    private:
        EigenSTL::vector_Vector3d centroids;
        bodies::Box* bounding_box;
        bodies::Box* geometric_box;
};

class plane : public obstacle
{
    public:

        plane(double a=0.0, double b=0.0, double c=1.0, double d=0.0,
         double FStar=1.0, double Alpha=1.0, double DStar=1.0):
         obstacle(FStar,Alpha,DStar), A{a}, B{b}, C{c}, D{d}{};

        Eigen::Vector3d compute_repulsive_force(const Eigen::Vector3d &query_point);
        bool check_intersection(const Eigen::Matrix3Xd &path);
        
    private:

        double A;
        double B;
        double C;
        double D;

};

class trajectory : public obstacle
{
    public:
        trajectory();
        trajectory(Eigen::Vector3d &start, Eigen::Vector3d &end, std::vector<std::shared_ptr<obstacle>> obstacle_list, int n_beads,
        double spring_const, double damping_const, double FStar, double Alpha, double DStar);
        
        Eigen::Vector3d compute_repulsive_force(const Eigen::Vector3d &query_point);
        bool check_intersection(const Eigen::Matrix3Xd &path);
        bool check_inclusion(const Eigen::Vector3d &point);
        void converge_trajectory();
        bool check_trajectory();
        Eigen::Matrix3Xd rubber_band_ode(const Eigen::Matrix3Xd &X, const Eigen::Matrix3Xd &Xdot);
        void rk4_step(double step_size);
        void constructMarkersFromObject(visualization_msgs::msg::Marker &path_marker);
        void constructPathMsgFromObject(nav_msgs::msg::Path &path_msg, int t0, double speed);

    private:
        Eigen::Matrix3Xd position;
        Eigen::Matrix3Xd velocity;
        Eigen::Matrix3Xd acceleration;
        std::vector<std::shared_ptr<obstacle>> obstacleList;
        double springConst;
        double dampingConst;


};

}


