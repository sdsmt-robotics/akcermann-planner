#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_datatypes.h"
#include "nav_msgs/Path.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_pose");
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<nav_msgs::Path>("path", 1000);
	ros::Rate loop_rate(1);
	nav_msgs::Path myPath;
	for(int i = 0; i < 5; i++)
	{
		geometry_msgs::PoseStamped temppose;
		temppose.pose.position.x = 5;
		temppose.pose.position.y = 6;
		temppose.pose.position.z = 7;
		temppose.pose.orientation = tf::createQuaternionMsgFromYaw(3.1);
		myPath.poses.push_back(temppose);
	}	

	while (ros::ok())
    {
      chatter_pub.publish(myPath);
      ros::spinOnce();
      loop_rate.sleep();
    }
    return 0;
}
