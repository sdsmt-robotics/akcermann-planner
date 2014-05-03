#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "tf/transform_datatypes.h"
#include <fstream>

nav_msgs::OccupancyGrid grid;
void load_map();

int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_pub");
  
  ros::NodeHandle n;
  ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("map",1000);
  ros::Publisher goal_pub = n.advertise<geometry_msgs::PoseStamped>("goal",1000);
ros::Rate loop_rate(1);
  grid.info.resolution = .1;
  grid.info.height = 4;
  grid.info.width = 4;
  grid.info.origin.position.x=0;
  grid.info.origin.position.y=0;
  grid.info.origin.position.z=0;
  grid.info.origin.orientation = tf::createQuaternionMsgFromYaw(0);
load_map();
  while (ros::ok())
  {
    map_pub.publish(grid);
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  ros::spin();
  return 0;
}

void load_map()
{
  int r,g,b;
  int rows = 4, cols = 4;
  char line[1000];
  std::ifstream mapin;
  mapin.open("testmap3.ppm");
  if(!mapin)
  {
	ROS_INFO("Failed to open file.");
	return;
  }

  //ignore format line
  mapin.getline(line,1000);
//ignore comment line
  mapin.getline(line,1000);
//size line
  mapin.getline(line,1000);
//max line
  mapin.getline(line,1000);

  grid.data.resize(rows*cols);
  for(int i = 0; i < rows; i++)
  { 
    for(int j = 0; j < cols; j++)
    {
      
      mapin >> r >> g >> b;
ROS_INFO("Row: %d Col: %d %d %d %d\n",i,j,r,g,b);
      if( r < 127 )
	grid.data[i*cols+j] = 100;
      else
        grid.data[i*cols+j] = 0;
    }
  } 
}
