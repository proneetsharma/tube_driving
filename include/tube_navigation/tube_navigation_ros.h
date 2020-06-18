#ifndef TUBE_NAVIGATION_ROS_H
#define TUBE_NAVIGATION_ROS_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/server.h>
#include <visualization_msgs/Marker.h>
#include <tube_navigation/TubeNavigationConfig.h>
#include <ropod_ros_msgs/RoutePlannerActionResult.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <string>
#include <vector>
#include <iostream>
#define vectormag(x,y) sqrt(pow(x,2) + pow(y,2))
#define XOR(a,b) (!a != !b)
struct GeometryVertex
{
    float x; float y;
};

struct AreaType
{
    std::string type; int label;
};

struct Area
{
    GeometryVertex p1; GeometryVertex p2; GeometryVertex p3; GeometryVertex p4; int label; std::string type;
};

double pose_x, pose_y, right_front_wheel_x, right_front_wheel_y, left_front_wheel_x, left_front_wheel_y;
double poseAMCL_x = 0.0, poseAMCL_y = 0.0, poseAMCL_a = 0.0 , ropod_x = 0.0, ropod_y = 0.0, ropod_theta = 0.0;
double odom_x = 0.0, odom_y = 0.0, odom_theta= 0.0, odom_phi = 0.0, odom_v = 0.0;
float distance = 0.0, wall_theta = 0.0, delta_theta = 0.0, robot_vel = 0.0;
float prev_x = 0.0, prev_y = 0.0;
geometry_msgs::Point feeler_point; 

bool start_navigation = false, run_exe = false;

typedef enum{
    idle,
    cruising,
    entry_corner,
    align_accel,
    align_decel,
    turn,
    entry_straight,
    inter_straight,
    overtake_spacious,
    overtake_tight,
    maxi_states,
} RobotState;

// define all external and internal events
// assign proper names to events
typedef enum{
    event_0,event_1,event_2,event_3,event_4,event_5,event_6,event_7,event_8, \
    event_9,event_10,event_11,event_12,event_13,event_14,event_15,event_16,event_17
}RobotEvent;

typedef struct {
    RobotState current;
    RobotEvent event;
    RobotState next;
} StateTransitionElement;

// assign proper names to event
static StateTransitionElement state_transition_table[] = {
    {idle, event_0 , cruising},
    {cruising, event_1 ,entry_corner},
    {entry_corner, event_2, align_accel},
    {align_accel, event_3 , align_decel},
    {align_decel, event_4 , turn},
    {align_accel, event_5 , turn},
    {turn, event_6 , cruising},
    {cruising, event_7 , entry_straight},
    {entry_straight, event_8 , inter_straight},
    {inter_straight, event_9 , cruising},
    {cruising, event_10 , overtake_spacious},
    {overtake_spacious, event_11 , cruising},
    {cruising, event_12 , overtake_tight},
    {overtake_tight, event_13 , cruising},
    {inter_straight, event_14 , overtake_spacious},
    {turn, event_15 , overtake_tight},
    {inter_straight, event_16 ,overtake_tight},
    {turn, event_17 , overtake_spacious}
};

class TubeNavigationROS
{
    public:
        TubeNavigationROS();
        void event(RobotEvent new_event);
	    void run(){__run();};
        void getFeeler();
        void Idle();
        virtual ~TubeNavigationROS();
        


    private:
        bool event_generated;
        RobotState current_state;
        unsigned char maximum_states;
        int current_label;
        ros::NodeHandle nh;
        ros::Subscriber amcl_sub;
        ros::Subscriber odom_sub;
        ros::Subscriber laserscan_sub;
        ros::Subscriber goal_route_sub;
        ros::Publisher cmd_vel_pub;
        ros::Publisher markers_pub;
        ros::Publisher wallmarker_pub;
        ros::Publisher outer_wallmarker_pub;
        ros::Publisher feeler_pub;
        ros::Publisher nextState_pub;
        visualization_msgs::Marker points;
        visualization_msgs::Marker offset_wall;
        visualization_msgs::Marker outer_offset_wall;
        visualization_msgs::Marker feeler;
        visualization_msgs::Marker nextState;
        std::vector<GeometryVertex> vertex_list;
        std::vector<GeometryVertex> right_vertex_list;
        std::vector<Area> area_list;
        AreaType currentIdLabel;

        ropod_ros_msgs::RoutePlannerResult goal_result;


        tf::TransformListener tf_listener;
        void __run();
        dynamic_reconfigure::Server<tube_navigation::TubeNavigationConfig> dyn_recon_srv;
        void getOdomVelCallback(const nav_msgs::Odometry::ConstPtr& odom_vel);
        void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& sensormsg);
        void poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgAMCL);
        void visualizeMarker();
        void getCurrentAndNextState(float x, float y);
        void goalRouteCallback(const ropod_ros_msgs::RoutePlannerActionResult::ConstPtr& goalroutemsg);
        void dynamicReconfigureCallback(tube_navigation::TubeNavigationConfig &dyn_config, uint32_t level);
};

#endif