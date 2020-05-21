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

/*using namespace std;
#include <string>
#include <vector>

//class PointID;
class Point {
public:
    double x, y;
    Point();
    Point(double xval, double yval);
    Point sub(Point b);
    Point add(Point b);
    Point sub(PointID b);
    Point add(PointID b);
};

class PointID {
public:
    double x, y;
    string id;
    PointID();
    PointID(double xval, double yval, string id);
    PointID sub(Point b);
    PointID add(Point b);
    PointID sub(PointID b);
    PointID add(PointID b);
};

class AreaQuad {
public:
    Point p0, p1, p2, p3;
    AreaQuad();
    AreaQuad(Point p0val, Point p1val, Point p2val, Point p3val);
    bool contains(Point q);
};


class Rectangle {
public:
    double width;
    double depth;
    double x;
    double y;
};


class AreaQuadID {
public:
    PointID p0, p1, p2, p3;
    int id;
    string type;
    AreaQuadID();
    AreaQuadID(PointID p0val, PointID p1val, PointID p2val, PointID p3val, int idval, string typeval);
    vector<string> getPointIDs();
    bool contains(Point q);
    Point center();
};*/


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
