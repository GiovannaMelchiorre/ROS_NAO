#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/point_cloud_conversion.h>
#include <sstream>

ros::Publisher newPC_pub;


void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_in)
{
  sensor_msgs::PointCloud output;
  sensor_msgs::convertPointCloud2ToPointCloud(*cloud_in, output);
  newPC_pub.publish(output);
}
 

 
int main(int argc, char **argv)
{
  ros::init(argc, argv, "pointCloudTransformation");
  
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("pointCloud", 1000, cloud_cb);

  newPC_pub = nh.advertise<sensor_msgs::PointCloud>("PointCloudChannel", 1000);
  
  ros::spin (); //where she stops nobody knows

  return 0;
}



