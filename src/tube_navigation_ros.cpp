#include "ros/ros.h"
#include "tube_navigation/tube_navigation_ros.h"


std::vector<geometry_vertex> vertex_list;

TubeNavigationROS::TubeNavigationROS() : nh("~")
{
    laserscan_sub = nh.subscribe("laser_scan", 1, &TubeNavigationROS::laserScanCallback, this);
    goal_route_sub = nh.subscribe<ropod_ros_msgs::RoutePlannerActionResult>("goal_route", 1, &TubeNavigationROS::goalRouteCallback, this);

    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

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

void TubeNavigationROS::visualizeMarker()
{

    markers_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    while (ros::ok())
    {
        // Set the frame ID and timestamp.
        points.header.frame_id = "/map";
        points.header.stamp = ros::Time::now();

        points.id = 0;
        points.type = visualization_msgs::Marker::LINE_STRIP;
        points.action = visualization_msgs::Marker::ADD;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        points.scale.x = 0.3;
        points.scale.y = 0.3;
        points.scale.z = 0;

        // Set the color
        points.color.a = 0.7;
        points.color.g = 0.3;
        points.color.b = 0.5;
        points.color.a = 1.0;

        points.lifetime = ros::Duration();
        markers_pub.publish(points);
    }
}

void TubeNavigationROS::goalRouteCallback(const ropod_ros_msgs::RoutePlannerActionResult::ConstPtr& goalroutemsg)
{
    ROS_INFO_STREAM("Received goal"); 

    ropod_ros_msgs::RoutePlannerResult goal_result = goalroutemsg->result;
    std::vector<ropod_ros_msgs::Area> planner_areas = goal_result.areas;

    geometry_msgs::Point p;  

    for (int i = 0; i < planner_areas.size(); i++)
    {
        for (int j = 0; j < planner_areas[i].sub_areas.size(); j++)
        {
            for (int k = 0; k < planner_areas[i].sub_areas[j].geometry.vertices.size(); k++)
            {
                geometry_vertex coordinate;
                coordinate.x = planner_areas[i].sub_areas[j].geometry.vertices[k].x;
                coordinate.y = planner_areas[i].sub_areas[j].geometry.vertices[k].y;
                vertex_list.push_back(coordinate);

            }

        // Draw a rectangle
        int points_size = vertex_list.size();

        if (points_size >= 4)
            {
                for(int i = 0; i <= 4; i++)
                {
                    if (i == 4)
                    {
                        p.x = vertex_list[points_size-4].x;
                        p.y = vertex_list[points_size-4].y;
                    }
                    else
                    {
                        p.x = vertex_list[points_size-(4-i)].x;
                        p.y = vertex_list[points_size-(4-i)].y;
                    }
                    points.points.push_back(p);
                }
            }
        }
    }

    visualizeMarker();

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

        