#ifndef TUBE_NAVIGATION_H
#define TUBE_NAVIGATION_H

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/server.h>
#include <visualization_msgs/Marker.h>
#include <tube_navigation/TubeNavigationConfig.h>
#include <ropod_ros_msgs/RoutePlannerActionResult.h>

class TubeNavigation
{
    public:
        TubeNavigation();
        virtual ~TubeNavigation();
        void run();

    private:
        void cruising();
        void idle();
        void accelDecel();
        void entryCorner();


};
#endif