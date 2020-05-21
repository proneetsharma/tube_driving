#include "ros/ros.h"
#include "tube_navigation/tube_navigation_ros.h"
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <string>
#include <vector>
#include <math.h>

double poseAMCLx, poseAMCLy, poseAMCLa;
float distance, wall_theta, delta_theta;
float prev_x = 0.0, prev_y = 0.0;


struct point{
    float x; float y; 
};

std::vector<point> pointlist;


TubeNavigationROS::TubeNavigationROS() : nh("~")
{
    laserscan_sub = nh.subscribe("laser_scan", 1, &TubeNavigationROS::laserScanCallback, this);
    sub_amcl = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("ropod_001_amcl/pose", 1, &TubeNavigationROS::poseAMCLCallback, this);
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

void TubeNavigationROS::poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgAMCL)
{
    poseAMCLx = msgAMCL->pose.pose.position.x;
    poseAMCLy = msgAMCL->pose.pose.position.y;
    poseAMCLa = tf::getYaw(msgAMCL->pose.pose.orientation); 
    std::cout<<"called";    
     
    
}



void TubeNavigationROS::goalRouteCallback(const ropod_ros_msgs::RoutePlannerActionResult::ConstPtr& goalroutemsg)
{
    
    ROS_INFO_STREAM("Received goal"); 
    
    ropod_ros_msgs::RoutePlannerResult goal_result = goalroutemsg->result;
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
    
    
    for (int i=0; i<10; i++)//pointlist.size()
    {
        
        std::cout <<"\n robot angle"<<poseAMCLa;
        std::cout <<"\n loop"<<i;
        std::cout <<"\n goal"<<pointlist[(i*4)+2].y;
        while (round(pointlist[(i*4)+2].y) != round(poseAMCLy))
	{
                
                //Subscribing to the pose
                sub_amcl = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("ropod_001_amcl/pose", 1, &TubeNavigationROS::poseAMCLCallback, this);
                ros::spinOnce();
		wall_theta = atan2(pointlist[(i*4)+2].y-prev_y,\
                                         pointlist[(i*4)+2].x-prev_x);
                std::cout<<"\n wall point"<<pointlist[(i*4)+2].y; 
                std::cout <<"\n robot y"<<poseAMCLy;
                std::cout <<"\n wall angle"<<wall_theta;
                std::cout <<"\n robot angle"<<poseAMCLa;
		//Publish constant Velocity
                cmd_vel.linear.x = 1.1;
                              
                cmd_vel_pub.publish(cmd_vel);
                //Check for frame parallel
                if (wall_theta != poseAMCLa)
                {
			//Right turn
                        if (wall_theta<0)
			{  			
				delta_theta = wall_theta-poseAMCLa; //poseAMCLa - wall_theta;
		        	std::cout <<"\n change in angle"<<delta_theta;
		        	
				cmd_vel.angular.z = delta_theta;
		        	std::cout <<"\n steering angle"<<cmd_vel.angular.z;
		        	cmd_vel_pub.publish(cmd_vel);
		        	
			}
                        //Left turn
			if (wall_theta >0)
			{
				delta_theta = poseAMCLa - wall_theta;
                        	std::cout <<"\n change in angle"<<delta_theta;
				if (wall_theta >= 3.1)
				{
					delta_theta = -1*delta_theta;	
				}
                        	
		        	cmd_vel.angular.z = delta_theta;
                        	std::cout <<"\n steering angle greater"<<cmd_vel.angular.z;
                        	cmd_vel_pub.publish(cmd_vel);
                        	


			}

		}
		//colliding with wall since obstacle avoidance is not implented
		//So forcefully breaking from the loop
		
		if (i==9)
		{
			break;
		}

	}
	
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

