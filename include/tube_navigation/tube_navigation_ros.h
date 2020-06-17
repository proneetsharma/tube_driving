#ifndef TUBE_NAVIGATION_ROS_H
#define TUBE_NAVIGATION_ROS_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/server.h>
#include <visualization_msgs/Marker.h>
#include <tube_navigation/TubeNavigationConfig.h>
#include <ropod_ros_msgs/RoutePlannerActionResult.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <string>
#include <vector>
#include <iostream>
#define vectormag(x,y) sqrt(pow(x,2) + pow(y,2))
struct GeometryVertex
{
    float x; float y;
};

struct Area
{
    GeometryVertex p1; GeometryVertex p2; GeometryVertex p3; GeometryVertex p4; int label; std::string type;
};

double pose_x, pose_y, right_front_wheel_x, right_front_wheel_y, left_front_wheel_x, left_front_wheel_y;
double poseAMCL_x = 0.0, poseAMCL_y = 0.0, poseAMCL_a = 0.0 , ropod_x = 0.0, ropod_y = 0.0, ropod_theta = 0.0;
double odom_x = 0.0, odom_y = 0.0, odom_theta= 0.0, odom_phi = 0.0, odom_v = 0.0;
float distance = 0.0, wall_theta = 0.0, delta_theta = 0.0, robot_vel = 0.0;
float prev_x = 0.0, prev_y = 0.0;
geometry_msgs::Point feeler_point; 

bool start_navigation = false, run_exe = false;

class TubeNavigationROS
{
    public:
        TubeNavigationROS();
	    void run(){__run();};
        void getFeeler();
        virtual ~TubeNavigationROS();


    private:
        int current_label;
        ros::NodeHandle nh;
        ros::Subscriber amcl_sub;
        ros::Subscriber odom_sub;
        ros::Subscriber laserscan_sub;
        ros::Subscriber goal_route_sub;
        ros::Publisher cmd_vel_pub;
        ros::Publisher markers_pub;
        ros::Publisher wallmarker_pub;
        ros::Publisher outer_wallmarker_pub;
        ros::Publisher feeler_pub;
        visualization_msgs::Marker points;
        visualization_msgs::Marker offset_wall;
        visualization_msgs::Marker outer_offset_wall;
        visualization_msgs::Marker feeler;
        std::vector<GeometryVertex> vertex_list;
        std::vector<GeometryVertex> right_vertex_list;
        std::vector<Area> area_list;
        ropod_ros_msgs::RoutePlannerResult goal_result;


        tf::TransformListener tf_listener;

        dynamic_reconfigure::Server<tube_navigation::TubeNavigationConfig> dyn_recon_srv;
        void getOdomVelCallback(const nav_msgs::Odometry::ConstPtr& odom_vel);
        void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& sensormsg);
        void poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgAMCL);
        void visualizeMarker();
        void getCurrentAndNextState(float x, float y);
        void goalRouteCallback(const ropod_ros_msgs::RoutePlannerActionResult::ConstPtr& goalroutemsg);
        void dynamicReconfigureCallback(tube_navigation::TubeNavigationConfig &dyn_config, uint32_t level);
        void __run();
};

#endif