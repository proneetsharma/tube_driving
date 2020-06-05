#include "ros/ros.h"
#include "tube_navigation/tube_navigation_ros.h"
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <string>
#include <vector>
#include <math.h>

double poseAMCL_x, poseAMCL_y, poseAMCL_a, ropod_x, ropod_y, ropod_theta;
double odom_x, odom_y, odom_theta, odom_phi, odom_v;
float distance, wall_theta, delta_theta;
float prev_x = 0.0, prev_y = 0.0;

bool start_navigation = false;


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
	
    markers_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    dynamic_reconfigure::Server<tube_navigation::TubeNavigationConfig>::CallbackType f;
    f = boost::bind(&TubeNavigationROS::dynamicReconfigureCallback, this, _1, _2);
    dyn_recon_srv.setCallback(f);    
     
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

	    
	    
	    geometry_msgs::Twist cmd_vel;
	    prev_x = pointlist[3].x, prev_y = pointlist[3].y;
	    
	    ros::Rate rate(10.0);
	    for (int i=0; i<pointlist.size(); i++)//pointlist.size()
	    {
			tf::TransformListener tf_listener;
			tf::StampedTransform t_ropod_pose;
			std::string target_frame = std::string("ropod_001/base_link");
			tf_listener.waitForTransform(target_frame, "/map", ros::Time(0), ros::Duration(3.0));
			//tf_listener.lookupTransform("map", target_frame, ros::Time(0), t_ropod_pose);

			//ropod_x = t_ropod_pose.getOrigin().getX();
			//ropod_y = t_ropod_pose.getOrigin().getY();
			//ropod_theta = tf::getYaw(t_ropod_pose.getRotation());
			//std::cout<<"\n robot_angle from tf"<<ropod_theta;
		
		
		        
		    tf_listener.lookupTransform("map", target_frame, ros::Time(0), t_ropod_pose);

			ropod_x = t_ropod_pose.getOrigin().getX();
            ropod_y = t_ropod_pose.getOrigin().getY();
            ropod_theta = tf::getYaw(t_ropod_pose.getRotation());
            std::cout<<"\n robot_angle from tf"<<ropod_theta;
		        
			//wall_theta = atan2(pointlist[(i*4)+2].y-prev_y, pointlist[(i*4)+2].x-prev_x);
			//std::cout<<"\n wall point"<<pointlist[(i*4)+2].y; 
			//std::cout <<"\n robot y"<<poseAMCLy;
			//std::cout <<"\n wall angle"<<wall_theta;
			//std::cout <<"\n robot angle from AMCL"<<poseAMCLa;
			//Publish constant Velocity
			cmd_vel.linear.x = 1;
							
			cmd_vel_pub.publish(cmd_vel);
			//Check for frame parallel
			/*if (wall_theta != ropod_theta)
			{
				//Right turn
		        if (wall_theta<0)
				{  			
					delta_theta = wall_theta-ropod_theta; //poseAMCLa - wall_theta;
					std::cout <<"\n change in angle"<<delta_theta;
					
					cmd_vel.angular.z = delta_theta;
					std::cout <<"\n steering angle"<<cmd_vel.angular.z;
					cmd_vel_pub.publish(cmd_vel);
					
				}
		        //Left turn
				if (wall_theta >0)
				{
					delta_theta = ropod_theta - wall_theta;
		            std::cout <<"\n change in angle"<<delta_theta;
					
					delta_theta = -1*delta_theta;
					
		                	
					cmd_vel.angular.z = delta_theta;
		            std::cout <<"\n steering angle greater"<<cmd_vel.angular.z;
		            cmd_vel_pub.publish(cmd_vel);
		                	


				}

			}*/
			//colliding with wall since obstacle avoidance is not implented
			//So forcefully breaking from the loop
		

		
	
			prev_x = pointlist[(i*4)+2].x, prev_y = pointlist[(i*4)+2].y;
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
	ROS_INFO_STREAM("Received Odom values");
    
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
        tube_navigtion_ros.run();        
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
