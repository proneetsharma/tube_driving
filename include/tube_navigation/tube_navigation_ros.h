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

class TubeNavigationROS
{
    public:
        TubeNavigationROS();
        virtual ~TubeNavigationROS();
        void run();

    private:
        ros::NodeHandle nh;
        ros::Subscriber laserscan_sub;
        ros::Subscriber goal_route_sub;
        ros::Publisher cmd_vel_pub;
        ros::Publisher markers_pub;

        // tf::TransformListener tf_listener;

        dynamic_reconfigure::Server<tube_navigation::TubeNavigationConfig> dyn_recon_srv;

        void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& sensormsg);

        void goalRouteCallback(const ropod_ros_msgs::RoutePlannerActionResult::ConstPtr& goalroutemsg);
          
        void dynamicReconfigureCallback(tube_navigation::TubeNavigationConfig &dyn_config, uint32_t level);




};
#endif