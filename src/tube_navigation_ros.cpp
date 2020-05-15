#include "ros/ros.h"
#include "tube_navigation/tube_navigation_ros.h"

// std::vector<PointID> vertex_list;
// std::vector<AreaQuadID> arealist;

std::vector<int> assignment;

std::vector<string> area_names;
std::vector<string> area_ids;
std::vector<string> sub_area_names;
std::vector<string> sub_area_ids;




std::list<geometry_vertex> vertex_list;


TubeNavigationROS::TubeNavigationROS() : nh("~")
{
    laserscan_sub = nh.subscribe("laser_scan", 1, &TubeNavigationROS::laserScanCallback, this);
    goal_route_sub = nh.subscribe<ropod_ros_msgs::RoutePlannerActionResult>("goal_route", 1, &TubeNavigationROS::goalRouteCallback, this);

    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    markers_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

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

    ropod_ros_msgs::RoutePlannerResult goal_result = goalroutemsg->result;
    std::vector<ropod_ros_msgs::Area> planner_areas = goal_result.areas;

    // int intermediate_area_id_counter = 10000;
    for (int i = 0; i < planner_areas.size(); i++)
    {
        for (int j = 0; j < planner_areas[i].sub_areas.size(); j++)
        {
            for (int k = 0; k < planner_areas[i].sub_areas[j].geometry.vertices.size(); k++)
            {
                geometry_vertex p;
                p.x = planner_areas[i].sub_areas[j].geometry.vertices[k].x;
                p.y = planner_areas[i].sub_areas[j].geometry.vertices[k].y;
                p.id = std::to_string(planner_areas[i].sub_areas[j].geometry.vertices[k].id);
                vertex_list.push_back(p);
            }
        
        }
    }

    visualizeMarker(&vertex_list);
}
    
void TubeNavigationROS::dynamicReconfigureCallback(tube_navigation::TubeNavigationConfig &dyn_config, uint32_t level)
{

}

void TubeNavigationROS::visualizeMarker(std::list<geometry_vertex>* vertices)
{
    markers_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    points.header.frame_id = "/map";
    points.header.stamp = ros::Time::now();
    points.id = 1;
    points.type = visualization_msgs::Marker::CUBE;
    points.action = visualization_msgs::Marker::ADD;
    points.scale.x = 0.2;
    points.scale.y = 0.2;
    points.color.g = 1.0f;
    points.color.a = 1.0;


    std::list<geometry_vertex>::iterator it;
    static uint32_t count =0;
    for (it = vertices->begin(); it != vertices->end(); ++it)
    {
        // std::cout << it->x << it->y;
        points.pose.position.x = it->x;
        points.pose.position.y = it->y;
        markers_pub.publish(points);
        ROS_INFO_STREAM("Point Published"); 

       points.lifetime = ros::Duration();
    }
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

        