#include "ros/ros.h"
#include "tube_navigation/tube_navigation_ros.h"

TubeNavigationROS::TubeNavigationROS() : nh("~")
{
    laserscan_sub = nh.subscribe("laser_scan", 1, &TubeNavigationROS::laserScanCallback, this);
    goal_route_sub = nh.subscribe<ropod_ros_msgs::RoutePlannerActionResult>("goal_route", 1, &TubeNavigationROS::goalRouteCallback, this);

    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    markers_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    wallmarker_pub = nh.advertise<visualization_msgs::Marker>("wall/visualization_marker", 1);
    outer_wallmarker_pub = nh.advertise<visualization_msgs::Marker>("outerwall/visualization_marker", 1);


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
    // Set the frame ID and timestamp.
    points.header.frame_id = "/map";
    points.header.stamp = ros::Time::now();

    points.id = 0;
    points.type = visualization_msgs::Marker::LINE_STRIP;
    points.action = visualization_msgs::Marker::ADD;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    points.scale.x = 0.05;
    points.scale.y = 0.05;
    points.scale.z = 0;

    // Set the color
    points.color.a = 0.7;
    points.color.g = 0.3;
    points.color.b = 0.5;

    points.lifetime = ros::Duration();
    markers_pub.publish(points);


    // Visualize right wall
    // Set the frame ID and timestamp.
    offset_wall.header.frame_id = "/map";
    offset_wall.header.stamp = ros::Time::now();
    offset_wall.id = 0;
    offset_wall.type = visualization_msgs::Marker::LINE_STRIP;
    offset_wall.action = visualization_msgs::Marker::ADD;
    offset_wall.scale.x = 0.05;
    offset_wall.scale.y = 0.05;
    offset_wall.scale.z = 0;
    offset_wall.color.a = 0.7;
    offset_wall.color.g = 0.8;
    offset_wall.color.b = 0.0;
    offset_wall.lifetime = ros::Duration();
    wallmarker_pub.publish(offset_wall);


    // Visualize outer right wall
    // Set the frame ID and timestamp.
    outer_offset_wall.header.frame_id = "/map";
    outer_offset_wall.header.stamp = ros::Time::now();
    outer_offset_wall.id = 0;
    outer_offset_wall.type = visualization_msgs::Marker::LINE_STRIP;
    outer_offset_wall.action = visualization_msgs::Marker::ADD;
    outer_offset_wall.scale.x = 0.05;
    outer_offset_wall.scale.y = 0.05;
    outer_offset_wall.scale.z = 0;
    outer_offset_wall.color.a = 0.7;
    outer_offset_wall.color.g = 0.8;
    outer_offset_wall.color.b = 0.0;
    outer_offset_wall.lifetime = ros::Duration();
    outer_wallmarker_pub.publish(outer_offset_wall);


}

void TubeNavigationROS::goalRouteCallback(const ropod_ros_msgs::RoutePlannerActionResult::ConstPtr& goalroutemsg)
{
    ROS_INFO_STREAM("Received goal"); 
    int points_size;
    goal_result = goalroutemsg->result;
    std::vector<ropod_ros_msgs::Area> planner_areas = goal_result.areas;

    geometry_msgs::Point p;  
    geometry_msgs::Point right_p;  


    for (int i = 0; i < planner_areas.size(); i++)
    {
        for (int j = 0; j < planner_areas[i].sub_areas.size(); j++)
        {
            for (int k = 0; k < planner_areas[i].sub_areas[j].geometry.vertices.size(); k++)
            {
                GeometryVertex coordinate;
                coordinate.x = planner_areas[i].sub_areas[j].geometry.vertices[k].x;
                coordinate.y = planner_areas[i].sub_areas[j].geometry.vertices[k].y;
                vertex_list.push_back(coordinate);

                if ((planner_areas[i].type != "junction") && (k == 0 || k == 1))
                {
                    right_vertex_list.push_back(coordinate);
                }
            }

        int right_points_size = right_vertex_list.size();

        if (right_points_size >= 2)
            {
                float x1 = right_vertex_list[right_points_size - 2].x;
                float y1 = right_vertex_list[right_points_size - 2].y;
                float x2 = right_vertex_list[right_points_size - 1].x;
                float y2 = right_vertex_list[right_points_size - 1].y;

                int L = sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
                float offsetPixels = -0.1;
                float outer_offsetPixels = -0.7;

                // Parallel line points for right wall
                if (L != 0)
                {
                    right_p.x = x1 + offsetPixels * (y2-y1) / L;
                    right_p.y = y1 + offsetPixels * (x1-x2) / L;
                    offset_wall.points.push_back(right_p);

                    right_p.x = x1 + outer_offsetPixels * (y2-y1) / L;
                    right_p.y = y1 + outer_offsetPixels * (x1-x2) / L;
                    outer_offset_wall.points.push_back(right_p);

                    right_p.x = x2 + offsetPixels * (y2-y1) / L;
                    right_p.y = y2 + offsetPixels * (x1-x2) / L;
                    offset_wall.points.push_back(right_p);

                    right_p.x = x2 + outer_offsetPixels * (y2-y1) / L;
                    right_p.y = y2 + outer_offsetPixels * (x1-x2) / L;
                    outer_offset_wall.points.push_back(right_p);
                }
                
            }    

        // Draw a rectangle
        points_size = vertex_list.size();

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

        