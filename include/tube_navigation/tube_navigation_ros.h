#ifndef TUBE_NAVIGATION_ROS_H
#define TUBE_NAVIGATION_ROS_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/server.h>
#include <visualization_msgs/Marker.h>
#include <tube_navigation/TubeNavigationConfig.h>
#include <ropod_ros_msgs/RoutePlannerActionResult.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

class TubeNavigationROS
{
    public:
        TubeNavigationROS();
	    void run(){__run();};
        void getFeeler();
        virtual ~TubeNavigationROS();

        

    private:
        ros::NodeHandle nh;
        ros::Subscriber laserscan_sub;
        ros::Subscriber goal_route_sub;
        ros::Subscriber amcl_sub;
        ros::Subscriber odom_sub;
        ros::Publisher cmd_vel_pub;
        ros::Publisher markers_pub;
        ros::Publisher feeler_pub;
        visualization_msgs::Marker feeler;
        ropod_ros_msgs::RoutePlannerResult goal_result;
        

        tf::TransformListener tf_listener;

        dynamic_reconfigure::Server<tube_navigation::TubeNavigationConfig> dyn_recon_srv;

        void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& sensormsg);
        void getOdomVelCallback(const nav_msgs::Odometry::ConstPtr& odom_vel);
        void goalRouteCallback(const ropod_ros_msgs::RoutePlannerActionResult::ConstPtr& goalroutemsg);
        void poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgAMCL);
          
        void dynamicReconfigureCallback(tube_navigation::TubeNavigationConfig &dyn_config, uint32_t level);
	    void __run();



};
#endif
