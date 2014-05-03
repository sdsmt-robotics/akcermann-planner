#define _USE_MATH_DEFINES
#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_datatypes.h"
#include "nav_msgs/OccupancyGrid.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include <sstream>
#include <fstream>
#include <math.h>
//map origin is start point
//lidar has 30 meter range, 270 degrees
//Try 0.1 m resolution on the map

double dist_to_gaol();
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

//Global AckermannDriveStamped message to be published at regular intervals.
//Recalculated whenever a pose is received.
ackermann_msgs::AckermannDriveStamped cmd;
bool goal_received = false;
geometry_msgs::PoseStamped goal;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ackermann_local_planner");
  ros::NodeHandle n;

  ros::Publisher  cmd_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("ackermann_cmd",1000);
  ros::Subscriber pose_sub = n.subscribe("pose_topic", 1000, poseCallback);
  ros::Subscriber map_sub = n.subscribe("map",1000, mapCallback);

  ros::Rate loop_rate(1);
  std_servs::Empty srv;
  cmd.drive.steering_angle = 0;
  cmd.drive.steering_angle_velocity = 0;
  cmd.drive.speed = 0;
  cmd.drive.acceleration = 0;
  cmd.drive.jerk = 0;
  
  ros::serviceClient next_goal_srv = n.serviceClient("goal_srv");
  while (ros::ok())
  {
    cmd_pub.publish(cmd);
    if(dist_to_goal() < 1)
      next_goal_srv.call(srv); 
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  goal_received = true;
  goal = *msg;
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  ROS_INFO("Got a map!");
}

//calculate new circle/steering angle
void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  if(!goal_received)
    return;

  double xdiff = goal.pose.position.x - msg.pose.position.x;
  double ydiff = goal.pose.position.y - msg.pose.position.y;
  double desired_orientation = atan2(ydiff,xdiff);
  
  while(desired_orientation < 0 )
    desired_orientation += PI;
  while(desired_orienation > 2*PI)  
    desired_orienation -= PI;

  double actual_orientation = tf::getYaw(goal.pose.orientation);
  while(actual_orientation < 0)
    actual_orienation += PI;
  while(actual_orienation > 2*PI)
    actual_orienation -= PI;
 
  double steer_angle = desired_orientation - actual_orientation;
  if(steer_angle > max_steer_angle)
    steer_angle = max_steer_angle;
  if(steer_angle < - max_steer_angle)
    steer_angle = - max_steer_angle;

  cmd.drive.steering_angle = steer_angle;
  cmd.drive.speed = 0.1;  
}

