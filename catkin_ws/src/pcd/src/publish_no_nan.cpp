#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/buffer.h>
// PCL includes
#include "pcl_ros/point_cloud.h"
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
using namespace pcl;
ros::Publisher result_publisher;
// declare point cloud
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
PointCloudXYZRGB::Ptr cloud_in (new PointCloudXYZRGB); 

void  cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input_pcl)
{
  sensor_msgs::PointCloud2 pcl_to_ros_pointcloud2;
  pcl::fromROSMsg (*input_pcl, *cloud_in);
  // Remove NaN point
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, indices);
  pcl::toROSMsg(*cloud_in, *pcl_to_ros_pointcloud2);
  result_publisher.publish(pcl_to_ros_pointcloud2);
  ROS_INFO("Success output");
}


int   main (int argc, char** argv)
{
     // Initialize ROS
     std::cout << "START TO TRANSFORM";
     ros::init (argc, argv, "remove_nan");
     ros::NodeHandle nh;
     // Create a ROS subscriber for the input point cloud
     ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth_registered/points", 1, cloud_cb);
     result_publisher = nh.advertise<sensor_msgs::PointCloud2> ("result_out", 1);
     // Spin
     ros::spin ();
}