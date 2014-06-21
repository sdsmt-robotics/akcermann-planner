#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "nav_msgs/Path.h"
#include <fstream>

using namespace std;

int add_pose(ifstream &fin);
bool serviceCallback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);

ros::Publisher path_pub;
nav_msgs::Path mypath;

int main(int argc, char ** argv)
{
	ros::init(argc,argv,"path_publisher");
	ros::NodeHandle n;
	ros::NodeHandle n_priv("~");
	string waypoint_file;
	string waypoints_topic;

	n_priv.param("waypoint_file",waypoint_file,(const std::string)"waypoints.txt");
	n_priv.param("waypoints_topic",waypoints_topic,(const std::string) "/path");
	//read file
	ifstream fin;
	fin.open(waypoint_file.c_str());
	if(!fin)
	{
		ROS_ERROR("Failed to open file %s\n",waypoint_file.c_str());
		return 0;
	}
	
	//build path
	while(1)
	{
		int status = add_pose(fin);
		if(-1 == status)
		{
			ROS_WARN("Unexpected failure in read.\n");
			return 0;
		}
		if(1 == status)
			break;
	}

	path_pub = n.advertise<nav_msgs::Path>(waypoints_topic,1);
	ros::ServiceServer service = n.advertiseService("go_service", serviceCallback);

	ros::spin();	
    return 0;
}


bool serviceCallback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
	path_pub.publish(mypath);
	return true;
}

int add_pose(ifstream &fin)
{
	geometry_msgs::PoseStamped tempPose;
	if(!(fin >> tempPose.pose.position.x))
		return 1;
	if(!(fin >> tempPose.pose.position.y))
		return -1;
	
	mypath.poses.push_back(tempPose);
	return 0;
}


