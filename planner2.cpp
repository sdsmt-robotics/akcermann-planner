#define _USE_MATH_DEFINES
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "ackermann_msgs/AckermannDrive.h"
#include "tf/transform_datatypes.h"
#include <math.h>

#define max_steer_angle 1.5
#define GOAL_THRESHOLD 3.0

void pathCallback(const nav_msgs::Path::ConstPtr& msg);
void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

static int current_waypoint = -1;
static nav_msgs::Path waypoints;

static ros::Publisher ackermann_cmd_pub;

int main(int argc, char **argv)
{
    std::string robot_pose_topic;
    std::string waypoints_topic;
    ros::init(argc, argv, "ackermann_local_planner");
    ros::NodeHandle n;
    ros::NodeHandle nh_priv( "~" );

    nh_priv.param("robot_pose_topic",robot_pose_topic, (const std::string)"/current_pose" );
    nh_priv.param("waypoints_topic",waypoints_topic,(const std::string)"/path");

    ros::Subscriber path_sub = n.subscribe(waypoints_topic,1,pathCallback);
    ros::Subscriber pose_sub = n.subscribe(robot_pose_topic,1,poseCallback);
    ackermann_cmd_pub = n.advertise<ackermann_msgs::AckermannDrive>("ackermann_cmd",1);
    ros::Rate loop_rate(1);

    ros::spin();
}

void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    ackermann_msgs::AckermannDrive cmd;
    if(-1 == current_waypoint || current_waypoint == waypoints.poses.size())
    {
        cmd.steering_angle = 0;
        cmd.steering_angle_velocity = 0;
        cmd.speed = 0;
        cmd.acceleration = 0;
        cmd.jerk = 0;
        ackermann_cmd_pub.publish(cmd);
        return;
    }

    double xdiff = waypoints.poses[current_waypoint].pose.position.x - (*msg).pose.pose.position.x;
    double ydiff = waypoints.poses[current_waypoint].pose.position.y - (*msg).pose.pose.position.y;
    double dist_to_goal = sqrt(xdiff*xdiff + ydiff*ydiff);
    double desired_orientation = atan2(ydiff,xdiff);   
    while(desired_orientation < 0 )
    desired_orientation += M_PI;
    while(desired_orientation > 2*M_PI)
    desired_orientation -= M_PI;

    double actual_orientation = tf::getYaw((*msg).pose.pose.orientation);
    while(actual_orientation < 0)
    actual_orientation += M_PI;
    while(actual_orientation > 2*M_PI)
    actual_orientation -= M_PI;
    
    double steer_angle = desired_orientation - actual_orientation;
    if(steer_angle > M_PI)
        steer_angle = 2*M_PI - steer_angle;
    if(steer_angle < -M_PI)
        steer_angle = steer_angle + 2*M_PI;

    if(steer_angle > max_steer_angle)
        steer_angle = max_steer_angle;
    if(steer_angle < -max_steer_angle)
        steer_angle = -max_steer_angle;

    cmd.steering_angle = steer_angle;
    cmd.steering_angle_velocity = 0;
    cmd.speed = 0.2;
    cmd.acceleration = 0;
    cmd.jerk = 0;
    ackermann_cmd_pub.publish(cmd);

    if(dist_to_goal < GOAL_THRESHOLD)
        current_waypoint++;

    return;
}

void pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    waypoints = *msg;
    current_waypoint = 0;

    //Path Debug
    /*for(int i = 0; i < waypoints.poses.size(); i++)
    {
        ROS_INFO("Waypoint %d:\n",i);
        ROS_INFO("X: %lf\n",waypoints.poses[i].pose.position.x);    
        ROS_INFO("Y: %lf\n",waypoints.poses[i].pose.position.y);    
        ROS_INFO("Z: %lf\n",waypoints.poses[i].pose.position.z);
    }*/
}
