#include "ros/ros.h"
#include "tube_navigation/tube_navigation_ros.h"

TubeNavigationROS::TubeNavigationROS() : nh("~")
{
    laserscan_sub = nh.subscribe("laser_scan", 1, &TubeNavigationROS::laserScanCallback, this);
    goal_route_sub = nh.subscribe("goal_route", 1, &TubeNavigationROS::goalRouteCallback, this);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    markers_pub = nh.advertise<visualization_msgs::Marker>("areas", 1);

    dynamic_reconfigure::Server<tube_navigation::TubeNavigationConfig>::CallbackType f;
    f = boost::bind(&TubeNavigationROS::dynamicReconfigureCallback, this, _1, _2);
    dyn_recon_srv.setCallback(f);
}

void TubeNavigationROS::run()
{
    geometry_msgs::Twist cmd_vel;
    cmd_vel_pub.publish(cmd_vel);
}
void TubeNavigationROS::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& sensormsg)
{

}
void TubeNavigationROS::goalRouteCallback(const ropod_ros_msgs::RoutePlannerActionResult::ConstPtr& goalroutemsg)

{
    ROS_INFO_STREAM("Received goal"); 
}
    
void TubeNavigationROS::dynamicReconfigureCallback(tube_navigation::TubeNavigationConfig &dyn_config, uint32_t level)

{

}

TubeNavigationROS::~TubeNavigationROS()
{

}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "tube_navigation");
    TubeNavigationROS tube_navigtion_ros;
    
    
    
    ros::Rate loop_rate(10.0);
    while (ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}

        