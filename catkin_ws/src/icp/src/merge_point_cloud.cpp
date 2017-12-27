#include <ros/ros.h>
#include <string>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl_ros/point_cloud.h"
#include <pcl/registration/icp.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/ply_io.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <image_transport/image_transport.h>

ros::Publisher input_publisher;
ros::Publisher output_publisher;
ros::Publisher icp_publisher;
ros::Publisher result_publisher;

bool first = true;

using namespace pcl;

// declare point cloud
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
PointCloudXYZRGB::Ptr cloud_in (new PointCloudXYZRGB);
PointCloudXYZRGB::Ptr cloud_out(new PointCloudXYZRGB);
PointCloudXYZRGB::Ptr icp_out (new PointCloudXYZRGB); 
PointCloudXYZRGB::Ptr result (new PointCloudXYZRGB); 

void  cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input_point2)
{
  // declare icp
  pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
  
  // convert from PointCloud2 to pcl_rgb
  if(first)
  {
    pcl::fromROSMsg ( *input_point2, *result);
    pcl::fromROSMsg ( *input_point2, *cloud_out);
    std::vector<int> a;
    pcl::removeNaNFromPointCloud(*result, *result, a);
    std::vector<int> b;
    pcl::removeNaNFromPointCloud(*cloud_out, *cloud_out, b);
    first = false;
  }

  pcl::fromROSMsg ( *input_point2, *cloud_in);

  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, indices);

  icp.setInputSource(cloud_in);
  icp.setInputTarget(cloud_out);
    
  icp.align( *icp_out);


  std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;
  
  *result += *icp_out;

  cloud_in->header.frame_id = "camera_rgb_optical_frame";
  cloud_out->header.frame_id = "camera_rgb_optical_frame";
  icp_out->header.frame_id = "camera_rgb_optical_frame";
  result->header.frame_id = "camera_rgb_optical_frame";

  // publish point cloud
  input_publisher.publish(cloud_in);
  output_publisher.publish(cloud_out);
  icp_publisher.publish(icp_out);
  result_publisher.publish(result);
  ROS_INFO("Success output");//cout
}

int   main (int argc, char** argv)
{
     // Initialize ROS
     std::cout << "START TO TRANSFORM";
     ros::init (argc, argv, "my_pcl_tutorial");
     ros::NodeHandle nh;
     
     // Create a ROS subscriber for the input point cloud
     ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth_registered/points", 1, cloud_cb);
     
     // Create a ROS publisher for the output point cloud
     input_publisher = nh.advertise<PointCloudXYZRGB> ("cloud_in", 1);
     output_publisher = nh.advertise<PointCloudXYZRGB> ("cloud_out", 1);
     icp_publisher = nh.advertise<PointCloudXYZRGB> ("icp_out", 1);
     result_publisher = nh.advertise<PointCloudXYZRGB> ("result_out", 1);
     
     // Spin
     ros::spin ();
}
