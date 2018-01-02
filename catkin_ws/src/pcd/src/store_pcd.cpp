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

// declare point cloud
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudXYZRGBA;
PointCloudXYZRGBA::Ptr cloud_in (new PointCloudXYZRGBA); 

void  cloud_cb (const PointCloudXYZRGB::ConstPtr& input_pcl)
{
  copyPointCloud (*input_pcl, *cloud_in);

  // Remove NaN point
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, indices);
  // Save .pcd file
  int vec = pcl::io::savePCDFile("test3.pcd", *cloud_in);
  ROS_INFO("Success output");  
}


int   main (int argc, char** argv)
{
     // Initialize ROS
     std::cout << "START TO TRANSFORM";
     ros::init (argc, argv, "store_PCD");
     ros::NodeHandle nh;
     
     // Create a ROS subscriber for the input point cloud
     ros::Subscriber sub = nh.subscribe<PointCloudXYZRGB> ("/tf_pcl", 1, cloud_cb);
     
     // Spin
     ros::spin ();
}