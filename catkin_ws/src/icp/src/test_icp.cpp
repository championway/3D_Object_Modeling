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
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
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
typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudXYZRGBA;
PointCloudXYZRGB::Ptr cloud_in (new PointCloudXYZRGB);
PointCloudXYZRGB::Ptr cloud_out(new PointCloudXYZRGB);
PointCloudXYZRGB::Ptr icp_out (new PointCloudXYZRGB); 
PointCloudXYZRGB::Ptr result (new PointCloudXYZRGB);
PointCloudXYZRGBA::Ptr rgba (new PointCloudXYZRGBA); 

void  cloud_cb (const PointCloudXYZRGB::ConstPtr& input_pcl)
{
  std::cout<< "recieve data" << std::endl;
  // declare icp
  pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree1 (new pcl::search::KdTree<pcl::PointXYZRGB>);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGB>);
  // convert from PointCloud2 to pcl_rgb
  if(first)
  {
    copyPointCloud (*input_pcl, *result);
    copyPointCloud (*input_pcl, *cloud_out);
    std::vector<int> a;
    pcl::removeNaNFromPointCloud(*result, *result, a);
    std::vector<int> b;
    pcl::removeNaNFromPointCloud(*cloud_out, *cloud_out, b);
    first = false;
  }

  copyPointCloud (*input_pcl, *cloud_in);

  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, indices);

  std::cout<< "start caculate icp" << std::endl;

  tree1->setInputCloud(cloud_in); 
  tree2->setInputCloud(cloud_out); 
  icp.setInputSource(cloud_in);
  icp.setInputTarget(cloud_out);
  icp.setMaxCorrespondenceDistance(1500);
  icp.setTransformationEpsilon(1e-10);
  icp.setEuclideanFitnessEpsilon(0.001);
  icp.setMaximumIterations(10); 
  icp.align( *icp_out);

  std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
  //std::cout << icp.getFinalTransformation() << std::endl;
  if (icp.getFitnessScore()<0.0001)
  {
  *result += *icp_out;

  cloud_in->header.frame_id = "camera_rgb_optical_frame";
  cloud_out->header.frame_id = "camera_rgb_optical_frame";
  icp_out->header.frame_id = "camera_rgb_optical_frame";
  result->header.frame_id = "camera_rgb_optical_frame";
  copyPointCloud (*result, *rgba);

  //int vec1 = pcl::io::savePCDFile ("savePCDFile.pcd", *result);
  //int vec2 = pcl::io::savePCDFileASCII ("savePCDFileASCII.pcd", *result);
  //int vec3 = pcl::io::savePLYFile ("savePLYFile.ply", *result);
  int vec4 = pcl::io::savePCDFile ("1500_0.001_10_kdtree.pcd", *rgba);
  //int vec5 = pcl::io::savePCDFileASCII ("rgbaASCII.pcd", *rgba);
  std::cout << "Success output" << std::endl;//cout
  std::cout << "=====================" << std::endl << std::endl;//cout
  }
  else{std::cout<<"SKIP"<<std::endl;}
  std::cout << "=====================" << std::endl << std::endl;//cout
}

int   main (int argc, char** argv)
{
     // Initialize ROS
     std::cout << "START TO TRANSFORM";
     ros::init (argc, argv, "merge");
     ros::NodeHandle nh;
     
     // Create a ROS subscriber for the input point cloud
     ros::Subscriber sub = nh.subscribe<PointCloudXYZRGB> ("/tf_pcl", 1, cloud_cb);
     
     // Create a ROS publisher for the output point cloud
     input_publisher = nh.advertise<PointCloudXYZRGB> ("cloud_in", 1);
     output_publisher = nh.advertise<PointCloudXYZRGB> ("cloud_out", 1);
     icp_publisher = nh.advertise<PointCloudXYZRGB> ("icp_out", 1);
     result_publisher = nh.advertise<PointCloudXYZRGB> ("result_out", 1);
     
     // Spin
     ros::spin ();
}
