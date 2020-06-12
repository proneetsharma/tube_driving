#include "ros/ros.h"
#include "tube_navigation/tube_navigation_ros.h"
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <string>
#include <vector>
#include <math.h>

double poseAMCL_x = 0.0, poseAMCL_y = 0.0, poseAMCL_a = 0.0 , ropod_x = 0.0, ropod_y = 0.0, ropod_theta = 0.0;
double odom_x = 0.0, odom_y = 0.0, odom_theta= 0.0, odom_phi = 0.0, odom_v = 0.0;
float distance = 0.0, wall_theta = 0.0, delta_theta = 0.0, robot_vel = 0.0;
float prev_x = 0.0, prev_y = 0.0;
geometry_msgs::Point feeler_point; 

bool start_navigation = false, run_exe = false;


struct point{
    float x; float y; 
};

std::vector<point> pointlist;

//Constructor 
TubeNavigationROS::TubeNavigationROS() : nh("~")
{
    
    laserscan_sub = nh.subscribe("laser_scan", 1, &TubeNavigationROS::laserScanCallback, this);
    amcl_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("robot_pose", 1, &TubeNavigationROS::poseAMCLCallback, this);
    goal_route_sub = nh.subscribe<ropod_ros_msgs::RoutePlannerActionResult>("goal_route", 1, &TubeNavigationROS::goalRouteCallback, this); 
	odom_sub = nh.subscribe<nav_msgs::Odometry>("odom_pose", 100, &TubeNavigationROS::getOdomVelCallback, this);   
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	feeler_pub = nh.advertise<visualization_msgs::Marker>("feeler/visualization_marker", 1);
    markers_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    dynamic_reconfigure::Server<tube_navigation::TubeNavigationConfig>::CallbackType f;
    f = boost::bind(&TubeNavigationROS::dynamicReconfigureCallback, this, _1, _2);
    dyn_recon_srv.setCallback(f);    
     
}

void TubeNavigationROS::getFeeler()
{
	

	tf::TransformListener tf_listener;
	tf::StampedTransform right_front_wheel_pose, left_front_wheel_pose, robot_centre_pose;
	std::string right_front_wheel = std::string("ropod_001/right_front_wheel_link");
	std::string left_front_wheel = std::string("ropod_001/left_front_wheel_link");
	std::string robot_centre = std::string("ropod_001/base_link");
	tf_listener.waitForTransform(right_front_wheel, "/map", ros::Time(0), ros::Duration(3.0));
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
		//Right feeler
		feeler_point.x = x1 + offsetPixels * (y2-y1) / L;
		feeler_point.y = y1 + offsetPixels * (x1-x2) / L;
		feeler.points.push_back(feeler_point);
		//Left feeler
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

    
}

double distBetweenFeelerTube(geometry_msgs::Point fp, point tfc, point tbc)
{
	// https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
	// P point, v, w points of line segmens
	double numerator =  (tbc.y-tfc.y)*fp.x - (tbc.x-tfc.x)*fp.y + tbc.x*tfc.y - tbc.y*tfc.x;
	double denominator = sqrt( (tbc.y-tfc.y)*(tbc.y-tfc.y) + (tbc.x-tfc.x)*(tbc.x-tfc.x) );
	return numerator/denominator;
}

//Move the robot
void TubeNavigationROS::__run()
{
	if (start_navigation == true)
	{
	   std::vector<ropod_ros_msgs::Area> planner_areas = goal_result.areas;
	      
	    for (int i = 0; i < planner_areas.size(); i++)
	    {
			for (int j = 0; j < planner_areas[i].sub_areas.size(); j++)
			{
				for (int k = 0; k < planner_areas[i].sub_areas[j].geometry.vertices.size(); k++)
				{
					point p;
					p.x = planner_areas[i].sub_areas[j].geometry.vertices[k].x;
					p.y = planner_areas[i].sub_areas[j].geometry.vertices[k].y;
					pointlist.push_back(p);
				}
		
			}
	    }

	    

		/*if ((planner_areas[i].type != "junction") && (k == 0 || k == 1))
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
                }*/

	    
	    geometry_msgs::Twist cmd_vel;
	    
	    
	    ros::Rate rate(10.0);
		bool start = true;
	    for (int i=0; i<pointlist.size()/4; i++)//pointlist.size()/4
	    {
			
			//Check for frame parallel
			
			
			while (fabs(pointlist[(i*4)+1].y-ropod_y)>=1e-1)
			{
				TubeNavigationROS::getFeeler();				
				float phi_des_0 = 0; 
				tf::TransformListener tf_listener;
				tf::StampedTransform t_ropod_pose;
				std::string target_frame = std::string("ropod_001/base_link");
				tf_listener.waitForTransform(target_frame, "/map", ros::Time(0), ros::Duration(3.0));
											
				tf_listener.lookupTransform("map", target_frame, ros::Time(0), t_ropod_pose);

				ropod_x = t_ropod_pose.getOrigin().getX();
				ropod_y = t_ropod_pose.getOrigin().getY();
				ropod_theta = tf::getYaw(t_ropod_pose.getRotation());
				std::cout<<"\n Feeler_point"<<feeler.points[0].x<<feeler.points[0].y<<feeler.points[1].x<<feeler.points[1].y;
				std::cout<<"\n robot_angle from tf"<<ropod_theta;
				prev_x = pointlist[(i*4)+0].x, prev_y = pointlist[(i*4)+0].y;   
				wall_theta = atan2(pointlist[(i*4)+1].y-prev_y, pointlist[(i*4)+1].x-prev_x);
				double wall_1 = atan2(pointlist[(i*4)+5].y-pointlist[(i*4)+4].y, pointlist[(i*4)+5].x-pointlist[(i*4)+4].x);
				std::cout<<"\n Difference in hallway"<<wall_1-wall_theta;
				std::cout<<"\n wall point"<<pointlist[(i*4)+2].y; 
				std::cout <<"\n robot y"<<ropod_y;
				std::cout <<"\n wall angle"<<wall_theta;
				
				cmd_vel.linear.x = 1.1;//cos(robot_vel);
				phi_des_0 = atan2(ropod_y, pointlist[(i*4)+2].y);
				std::cout <<"\n robot velocity"<<cos(robot_vel);
				double distance = distBetweenFeelerTube(feeler.points[0], pointlist[(i*4)+1], pointlist[(i*4)+0]);
				std::cout <<"\n distance"<<distance;				
				cmd_vel_pub.publish(cmd_vel);
				
				if (abs(wall_1-wall_theta)<=1)
				{
					if (fabs(wall_theta - ropod_theta)>=1e-1)
					{		
						if (fabs(distance)>0.8)		
						{
							delta_theta = wall_theta-ropod_theta; //poseAMCLa - wall_theta;
							std::cout <<"\n change in angle"<<delta_theta;
							
							cmd_vel.angular.z = delta_theta-phi_des_0-1;
							std::cout <<"\n steering angle"<<cmd_vel.angular.z;
							cmd_vel_pub.publish(cmd_vel);
							start = false;

						}	 
						else
						{
							delta_theta = wall_theta-ropod_theta; //poseAMCLa - wall_theta;
							std::cout <<"\n change in angle"<<delta_theta;
							
							cmd_vel.angular.z = delta_theta;
							std::cout <<"\n steering angle"<<cmd_vel.angular.z;
							cmd_vel_pub.publish(cmd_vel);
						}
									
						
						
					}

				} 
								
				
				else
				{
					cmd_vel.angular.z = wall_theta - ropod_theta;
					std::cout <<"\n steering angle"<<cmd_vel.angular.z;
					cmd_vel_pub.publish(cmd_vel);
					
				}				

				

			}
			
					
				//}
		        //Left turn
				/*if (wall_theta >0)
				{
					delta_theta = ropod_theta - wall_theta;
		            std::cout <<"\n change in angle"<<delta_theta;
					
					delta_theta = -1*delta_theta;
					
		                	
					cmd_vel.angular.z = delta_theta;
		            std::cout <<"\n steering angle greater"<<cmd_vel.angular.z;
		            cmd_vel_pub.publish(cmd_vel);
		                	


				}*/

			
			//colliding with wall since obstacle avoidance is not implented
			//So forcefully breaking from the loop
		

		
	
			//prev_x = pointlist[(i*4)+2].x, prev_y = pointlist[(i*4)+2].y;
			/*x1 = it->x;
			y1 = it->y;	
			std::cout <<"\n goal"<<x1 << y1;	
			std::cout<<"\nAMCL_pose"<<atan2(pointlist[2].y-pointlist[3].y,\
											pointlist[2].x-pointlist[3].x);
			//x_linear_velocity = robot_velocity*cos(steering angle)*cos(orientation of vechile)
			//proportional controller
			distance = sqrt(pow((x1-poseAMCLx), 2) + pow((y1-poseAMCLy), 2));
			cmd_vel.linear.x = 1;//distance;
			//y_linear_velocity = robot_velocity*cos(steering angle)*sin(orientation of vechile)
			cmd_vel.linear.y = 0;
			cmd_vel.linear.z = 0;
			std::cout<<"\nDistance"<<distance;
			cmd_vel_pub.publish(cmd_vel);
			cmd_vel.angular.x = 0;
			cmd_vel.angular.y = 0;
			//rotational velocity = 1/distance(frontaxle,rearaxle)*sin(steering angle)
			cmd_vel.angular.z = atan2(y1-poseAMCLy, x1-poseAMCLx);
			cmd_vel_pub.publish(cmd_vel);
			break;*/
		
		 

		}
		run_exe = true;

	}

	

}

void TubeNavigationROS::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& sensormsg)
{

}

//Get the initial pose
void TubeNavigationROS::poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgAMCL)
{
    poseAMCL_x = msgAMCL->pose.pose.position.x;
    poseAMCL_y = msgAMCL->pose.pose.position.y;
    poseAMCL_a = tf::getYaw(msgAMCL->pose.pose.orientation); 
	//std::cout<<poseAMCL_a;
    ROS_INFO_STREAM("Received pose");   
     
    
}

//Get the goal areas
void TubeNavigationROS::goalRouteCallback(const ropod_ros_msgs::RoutePlannerActionResult::ConstPtr& goalroutemsg)
{
    ROS_INFO_STREAM("Received goal");        
    goal_result = goalroutemsg->result;
    start_navigation = true;
    
     
}

void TubeNavigationROS::getOdomVelCallback(const nav_msgs::Odometry::ConstPtr& odom_vel)
{
    odom_x = odom_vel->twist.twist.linear.x;
    odom_y = odom_vel->twist.twist.linear.y;
    odom_theta = odom_vel->twist.twist.angular.z;
    odom_phi = atan2(odom_y, odom_x);
    odom_v = sqrt(odom_x*odom_x+odom_y*odom_y);
	//ROS_INFO_STREAM("Received Odom values");
    
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
		if (run_exe == false)
		{
			tube_navigtion_ros.run();  

		}
              
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
