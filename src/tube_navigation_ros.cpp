#include "ros/ros.h"
#include "tube_navigation/tube_navigation_ros.h"

// structure contains state and corresponding function pointer
typedef struct {
    RobotState state;
    void (TubeNavigationROS::*func)();
}StateFunction;

// state is mapped to corresponding function pointer
static StateFunction transition_map[] = {
    {idle , &TubeNavigationROS::Idle},
    {cruising , &TubeNavigationROS::cruising},
    {cornering,&TubeNavigationROS::entry_corner}
};

bool side = false;

//Function to handle the subscriber and publisher
TubeNavigationROS::TubeNavigationROS() : nh("~")
{
    maximum_states = maxi_states;
    current_state = idle;
    current_label = 1;
    laserscan_sub = nh.subscribe("laser_scan", 1, &TubeNavigationROS::laserScanCallback, this);
    amcl_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("robot_pose", 1, &TubeNavigationROS::poseAMCLCallback, this);
    goal_route_sub = nh.subscribe<ropod_ros_msgs::RoutePlannerActionResult>("goal_route", 1, &TubeNavigationROS::goalRouteCallback, this);
    odom_sub = nh.subscribe<nav_msgs::Odometry>("odom_pose", 100, &TubeNavigationROS::getOdomVelCallback, this); 
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    markers_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    wallmarker_pub = nh.advertise<visualization_msgs::Marker>("wall/visualization_marker", 1);
    outer_wallmarker_pub = nh.advertise<visualization_msgs::Marker>("outerwall/visualization_marker", 1);
    feeler_pub = nh.advertise<visualization_msgs::Marker>("feeler/visualization_marker", 1);
    nextState_pub = nh.advertise<visualization_msgs::Marker>("next_state", 1);

    dynamic_reconfigure::Server<tube_navigation::TubeNavigationConfig>::CallbackType f;
    f = boost::bind(&TubeNavigationROS::dynamicReconfigureCallback, this, _1, _2);
    dyn_recon_srv.setCallback(f);
}

// inactive state of robot, required during initialization 
void TubeNavigationROS::Idle(){

}
// add one more paramter for event data
void TubeNavigationROS::event(RobotEvent new_event){
    event_generated = true;
    current_state = state_transition_table[new_event].next;
    while (event_generated){
        event_generated = false;
        assert(current_state < maxi_states); 
        // call the respective function using for the state
        (this->*transition_map[current_state].func)();
    //delete event data once used
    }
}

void TubeNavigationROS::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& sensormsg)
{
    //code here
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
  
void TubeNavigationROS::getFeeler()
{
	geometry_msgs::Point feeler_point; 

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

    float m = x2 + offsetPixels * (y2-y1) / L;
    float n = y2 + offsetPixels * (x1-x2) / L;

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

//Calculate the distance between the feeler and wall
double distBetweenFeelerTube(geometry_msgs::Point fp, GeometryVertex tfc, GeometryVertex tbc)
{
	// https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
	double numerator =  (tbc.y-tfc.y)*fp.x - (tbc.x-tfc.x)*fp.y + tbc.x*tfc.y - tbc.y*tfc.x;
	double denominator = sqrt( (tbc.y-tfc.y)*(tbc.y-tfc.y) + (tbc.x-tfc.x)*(tbc.x-tfc.x) );
	return numerator/denominator;
}

void TubeNavigationROS::getCurrentAndNextState(float x, float y)
{
    geometry_msgs::Point nextpoint;
    std::vector<Area>::iterator ait;

    GeometryVertex left_point;
    GeometryVertex right_point;
    geometry_msgs::Point robot_c;


    for (ait = area_list.begin() ; ait != area_list.end(); ait++)         
    {
        if ((ait->label == current_label))
        {
            left_point.x = ait->p2.x;
            left_point.y = ait->p2.y;
            right_point.x = ait->p3.x;
            right_point.y = ait->p3.y;

            robot_c.x = x;
            robot_c.y = y;

            nextState.points.clear();
            nextpoint.x = ait->p2.x;
            nextpoint.y = ait->p2.y;
            nextState.points.push_back(nextpoint);
            nextpoint.x = ait->p3.x;
            nextpoint.y = ait->p3.y;
            nextState.points.push_back(nextpoint);

            // visualize a point on the front vertices of current area
            nextState.header.frame_id = "/map";
            nextState.header.stamp = ros::Time::now();
            nextState.id = 0;
            nextState.type = visualization_msgs::Marker::POINTS;
            nextState.action = visualization_msgs::Marker::ADD;
            nextState.scale.x = 0.2;
            nextState.scale.y = 0.2;
            nextState.scale.z = 0;
            nextState.color.a = 0.7;
            nextState.color.g = 0.3;
            nextState.color.b = 0.0;
            nextState.color.r = 0.5;
            nextState.lifetime = ros::Duration();
            nextState_pub.publish(nextState);

            // Distance between robot center and front wall of a area
            float distance = distBetweenFeelerTube(robot_c, left_point, right_point);

            currentIdLabel.label = ait->label;
            currentIdLabel.type = ait->type;

            // update the state when robot crosses an area.
            if(distance < 0) 
            {
                current_label++;   
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
    int count = 1;

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

            // Store relevent information of an area
            Area areainfo;
            areainfo.p1 = vertex_list[points_size-1];
            areainfo.p2 = vertex_list[points_size-2];
            areainfo.p3 = vertex_list[points_size-3];
            areainfo.p4 = vertex_list[points_size-4];
            areainfo.label = count;
            areainfo.type = sub_area_type;
            area_list.push_back(areainfo);
            count++;
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

    }
    // Make the bool true after storing the area to navigate and 
    // Robot parameters are properly defined
    start_navigation = true;
    visualizeMarker();
}

// Function to get the pose of the robot from AMCL
void TubeNavigationROS::poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgAMCL)
{
    poseAMCL_x = msgAMCL->pose.pose.position.x;
    poseAMCL_y = msgAMCL->pose.pose.position.y;
    poseAMCL_a = tf::getYaw(msgAMCL->pose.pose.orientation); 
    ROS_INFO_STREAM("Received pose");   
}
// Function to get the robot pose from the odom tf from Rviz
void TubeNavigationROS::getOdomVelCallback(const nav_msgs::Odometry::ConstPtr& odom_vel)
{
    odom_x = odom_vel->twist.twist.linear.x;
    odom_y = odom_vel->twist.twist.linear.y;
    odom_theta = odom_vel->twist.twist.angular.z;
    odom_phi = atan2(odom_y, odom_x);
    odom_v = sqrt(odom_x*odom_x+odom_y*odom_y);
}

// Function that handle the whole execution of the navigation
void TubeNavigationROS::__run()
{
	if (start_navigation == true)
	{	    
	    ros::Rate rate(10.0);
		bool start = true;
	    for (int i=0; i<area_list.size(); i++)
	    {
			while (currentIdLabel.label != area_list[i+1].label)
			{
				getFeeler();				
				// get robot pose from tf
				tf::TransformListener tf_listener;
				tf::StampedTransform t_ropod_pose;
				std::string target_frame = std::string("ropod_001/base_link");
				tf_listener.waitForTransform(target_frame, "/map", ros::Time(0), ros::Duration(3.0));
											
				tf_listener.lookupTransform("map", target_frame, ros::Time(0), t_ropod_pose);

				ropod_x = t_ropod_pose.getOrigin().getX();
				ropod_y = t_ropod_pose.getOrigin().getY();
				ropod_theta = tf::getYaw(t_ropod_pose.getRotation());
                //Calculate the angle of the wall
				prev_x = right_vertex_list[(i*2)+0].x, prev_y = right_vertex_list[(i*2)+0].y;   
				wall_theta = atan2(right_vertex_list[(i*2)+1].y-prev_y, right_vertex_list[(i*2)+1].x-prev_x);
				wall_next = atan2(right_vertex_list[(i*2)+3].y-right_vertex_list[(i*2)+2].y, right_vertex_list[(i*2)+3].x-right_vertex_list[(i*2)+2].x);
                //Publish the velocity																
				cmd_vel.linear.x = robot_velocity;
				phi_des_l = atan2(ropod_y, right_vertex_list[(i*2)+1].y);
                phi_des_r = atan2(ropod_x, right_vertex_list[(i*2)+1].x);
                //Calculate the distance from feeler to wall
				distance_feeler_wall = distBetweenFeelerTube(feeler.points[0], right_vertex_list[(i*2)+1], right_vertex_list[(i*2)+0]);
				cmd_vel_pub.publish(cmd_vel);

				if (currentIdLabel.type == "hallway")
				{                  
                    //Check for the feeler distance with wall
                    if ((fabs(distance_feeler_wall)>0.5) && (fabs(fabs(wall_theta) - fabs(ropod_theta))>=1e-1) && !side)
                    {
                        // steering angle calculation
                        delta_theta = atan2(sin(wall_theta-ropod_theta), cos(wall_theta-ropod_theta));
                        cmd_vel.angular.z = delta_theta-phi_des_l;
                        cmd_vel_pub.publish(cmd_vel);
                    }
                    else
                    {
                        // Correct the angle based on the transition entry of the feeler
                        delta_theta = atan2(sin(wall_theta-ropod_theta), cos(wall_theta-ropod_theta));
                        if (side && wall_theta<ropod_theta)//(distance<0) && (ropod_theta < 0))
                        {
                            cmd_vel.angular.z = delta_theta+(phi_des_r*smoothing_factor);
                        }
                        else
                        {
                            cmd_vel.angular.z = delta_theta;
                        }
                        //publish the correction angle
                        cmd_vel_pub.publish(cmd_vel);
                    }
				} 
				else
				{
                    event(event_1);
				}				
			}
		}
		run_exe = true;
	}
}
// state machine for cornering
void TubeNavigationROS::entry_corner()
{
	geometry_msgs::Twist cmd_vel;
    //Publish the robot angle and velocity for the turn, left and right direction
    //is changed based on the sign
    cmd_vel.linear.x = robot_velocity*decel_step;
    cmd_vel.angular.z = -(ropod_theta+M_PI);
    cmd_vel_pub.publish(cmd_vel);
    side = true;
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
        //Check for new navigation
		if (run_exe == false)
		{
            tube_navigtion_ros.event(event_0);
			// tube_navigtion_ros.run();  
		}
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}

