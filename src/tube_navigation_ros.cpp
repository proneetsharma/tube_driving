#include "ros/ros.h"
#include "tube_navigation/tube_navigation_ros.h"

TubeNavigationROS::TubeNavigationROS() : nh("~")
{
    current_label = 1;
    laserscan_sub = nh.subscribe("laser_scan", 1, &TubeNavigationROS::laserScanCallback, this);
    goal_route_sub = nh.subscribe<ropod_ros_msgs::RoutePlannerActionResult>("goal_route", 1, &TubeNavigationROS::goalRouteCallback, this);

    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    markers_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    wallmarker_pub = nh.advertise<visualization_msgs::Marker>("wall/visualization_marker", 1);
    outer_wallmarker_pub = nh.advertise<visualization_msgs::Marker>("outerwall/visualization_marker", 1);
    feeler_pub = nh.advertise<visualization_msgs::Marker>("feeler/visualization_marker", 1);


    dynamic_reconfigure::Server<tube_navigation::TubeNavigationConfig>::CallbackType f;
    f = boost::bind(&TubeNavigationROS::dynamicReconfigureCallback, this, _1, _2);
    dyn_recon_srv.setCallback(f);
}

void TubeNavigationROS::run()
{
    // geometry_msgs::Twist cmd_vel;
    // cmd_vel_pub.publish(cmd_vel);


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
    offset_wall.scale.x = 0.3;
    offset_wall.scale.y = 0.3;
    offset_wall.scale.z = 0;
    offset_wall.color.a = 0.7;
    offset_wall.color.g = 0.0;
    offset_wall.color.b = 0.0;
    offset_wall.color.r = 1.0;
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

void TubeNavigationROS::getFeeler()
{
	geometry_msgs::Point feeler_point; 

	tf::TransformListener tf_listener;
	tf::StampedTransform right_front_wheel_pose, left_front_wheel_pose, robot_centre_pose;
	std::string right_front_wheel = std::string("ropod_001/right_front_wheel_link");
	std::string left_front_wheel = std::string("ropod_001/left_front_wheel_link");
	std::string robot_centre = std::string("ropod_001/base_link");
	tf_listener.waitForTransform(right_front_wheel, "/map", ros::Time(0), ros::Duration(3.0));
	// tf_listener.waitForTransform(left_front_wheel, "/map", ros::Time(0), ros::Duration(3.0));
		
	tf_listener.lookupTransform("map", right_front_wheel, ros::Time(0), right_front_wheel_pose);
	tf_listener.lookupTransform("map", left_front_wheel, ros::Time(0), left_front_wheel_pose);
	tf_listener.lookupTransform("map", robot_centre, ros::Time(0), robot_centre_pose);


	float x = robot_centre_pose.getOrigin().getX();
	float y = robot_centre_pose.getOrigin().getY();
	float x1 = right_front_wheel_pose.getOrigin().getX();
	float y1 = right_front_wheel_pose.getOrigin().getY();
	float x2 = left_front_wheel_pose.getOrigin().getX();
	float y2 = left_front_wheel_pose.getOrigin().getY();

	float L = sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
	float offsetPixels = 0.5;

	// fillers at some offset from robot
	if (L != 0)
	{	
		feeler.points.clear();
		feeler_point.x = x1 + offsetPixels * (y2-y1) / L;
		feeler_point.y = y1 + offsetPixels * (x1-x2) / L;
		feeler.points.push_back(feeler_point);

		feeler_point.x = x2 + offsetPixels * (y2-y1) / L;
		feeler_point.y = y2 + offsetPixels * (x1-x2) / L;
		feeler.points.push_back(feeler_point);
	}

	feeler.header.frame_id = "/map";
    feeler.header.stamp = ros::Time::now();
    feeler.id = 0;
    feeler.type = visualization_msgs::Marker::POINTS;
    feeler.action = visualization_msgs::Marker::ADD;
    feeler.scale.x = 0.1;
    feeler.scale.y = 0.1;
    feeler.scale.z = 0;
    feeler.color.a = 0.7;
    feeler.color.g = 0.0;
    feeler.color.b = 0.0;
    feeler.color.r = 1.0;
    feeler.lifetime = ros::Duration();
    feeler_pub.publish(feeler);

    getCurrentAndNextState(x,y);
}

void TubeNavigationROS::getCurrentAndNextState(float x, float y)
{
    std::vector<Area>::iterator ait;
    for (ait = area_list.begin() ; ait != area_list.end(); ait++)         
    {
        if (ait->label == current_label)
        {
            float right_x = ait->p1.x;
            float right_y = ait->p1.y;
            float left_x = ait->p2.x;
            float left_y = ait->p2.y;

            float v1_x = (left_x - right_x);
            float v1_y = (left_y - right_y);                                                                                                            

            float v2_x = (left_x - x);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
            float v2_y = (left_y - y);

            float dot_product = (v1_x*v2_x) + (v1_y*v2_y);
            float magnitude = vectormag(v1_x,v1_y) * vectormag(v2_x,v2_y); 

            float angle = atan2((v1_y*v2_y), (v1_x*v2_x));
            std::cout<<"Angle: "<<angle << "label: " << ait->label<<"\n";

            if (angle > 0)
            {
                current_label += 1;
            }
        }                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
    }
}

void TubeNavigationROS::goalRouteCallback(const ropod_ros_msgs::RoutePlannerActionResult::ConstPtr& goalroutemsg)
{
    ROS_INFO_STREAM("Received goal"); 
    int points_size;
    std::string sub_area_type;
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

        points_size = vertex_list.size();
        sub_area_type = planner_areas[i].type;
        if (sub_area_type == "junction")
        {
            sub_area_type = "inter";
        }
        else
        {
            sub_area_type = "hallway";
        }

        // Draw a rectangle
        if (points_size >= 4)
            {
                for(int n = 0; n <= 4; n++)
                {
                    if (n == 4)
                    {
                        p.x = vertex_list[points_size-4].x;
                        p.y = vertex_list[points_size-4].y;
                    }
                    else
                    {
                        p.x = vertex_list[points_size-(4-n)].x;
                        p.y = vertex_list[points_size-(4-n)].y;
                    }
                    points.points.push_back(p);
                }

            }

        int right_points_size = right_vertex_list.size();
        if (right_points_size >= 2)
            {
                float x1 = right_vertex_list[right_points_size - 2].x;
                float y1 = right_vertex_list[right_points_size - 2].y;
                float x2 = right_vertex_list[right_points_size - 1].x;
                float y2 = right_vertex_list[right_points_size - 1].y;

                float L = sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
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
        }
    Area areainfo;
    areainfo.p1 = vertex_list[(i*4)+1];
    areainfo.p2 = vertex_list[(i*4)+2];
    areainfo.p3 = vertex_list[(i*4)+3];
    areainfo.p4 = vertex_list[(i*4)+4];
    areainfo.label = i+1;
    areainfo.type = sub_area_type;
    area_list.push_back(areainfo);
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
		tube_navigtion_ros.getFeeler();
    }
    return 0;
}



// # ifdef 0
//     std::vector<Area>::iterator ait;    
//     for (ait = area_list.begin() ; ait != area_list.end(); ait++)         
//     {
//         std::cout << ait->type << std::endl;
//     }
// # endif