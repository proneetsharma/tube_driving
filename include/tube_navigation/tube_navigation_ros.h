#ifndef TUBE_NAVIGATION_ROS_H
#define TUBE_NAVIGATION_ROS_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
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
double prev_x, prev_y, wall_theta;


class TubeNavigationROS
{
    public:
        TubeNavigationROS();
        void run();
        virtual ~TubeNavigationROS();
        void getFeeler();


    private:
        int current_label;
        ros::NodeHandle nh;
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


        // tf::TransformListener tf_listener;

        dynamic_reconfigure::Server<tube_navigation::TubeNavigationConfig> dyn_recon_srv;
        void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& sensormsg);
        void visualizeMarker();
        void getCurrentAndNextState(float x, float y);
        void goalRouteCallback(const ropod_ros_msgs::RoutePlannerActionResult::ConstPtr& goalroutemsg);
        void dynamicReconfigureCallback(tube_navigation::TubeNavigationConfig &dyn_config, uint32_t level);

};

#endif