#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_datatypes.h"
#include "nav_msgs/OccupancyGrid.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_pose");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("test_pose_topic", 1000);
  ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("test_map_topic", 1000);
ros::Rate loop_rate(1);

  geometry_msgs::PoseStamped p;
  geometry_msgs::PoseStamped goal;
  nav_msgs::OccupancyGrid map;
  
  p.pose.position.x = 0;
  p.pose.position.y = 0;
  p.pose.position.z = 0;
  p.pose.orientation = tf::createQuaternionMsgFromYaw(0);
  p.header.stamp = ros::Time::now();
  p.header.frame_id = "position";

  goal = p;
  goal.header.frame_id = "goal";


  while (ros::ok())
  {
    chatter_pub.publish(p);
    ros::spinOnce();
    chatter_pub.publish(goal);
    ros::spinOnce();
    map_pub.publish(map);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
