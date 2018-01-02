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
using namespace pcl;

// declare point cloud
typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudXYZRGBA;
PointCloudXYZRGBA::Ptr cloud_in (new PointCloudXYZRGBA); 

int main (int argc, char** argv)
{
  PointCloudXYZRGBA::Ptr test (new PointCloudXYZRGBA);
  PointCloudXYZRGBA::Ptr test1(new PointCloudXYZRGBA);
  PointCloudXYZRGBA::Ptr result(new PointCloudXYZRGBA);
  pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
  pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree1 (new pcl::search::KdTree<pcl::PointXYZRGBA>);
  pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBA>);
  // Load .pcd file from argv[1]
  int ret = pcl::io::loadPCDFile ("test.pcd", *test);
  int ret1 = pcl::io::loadPCDFile ("test3.pcd", *test1);
  if (ret < 0 || ret1 < 0) {
      PCL_ERROR("Couldn't read file %s\n", argv[1]);
    return -1;
  }

  // Remove NaN point
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*test, *test, indices);
  std::vector<int> indices1;
  pcl::removeNaNFromPointCloud(*test1, *test1, indices1);
  tree1->setInputCloud(test); 
  tree2->setInputCloud(test1); 
  icp.setInputSource(test);
  icp.setInputTarget(test1);
  icp.setMaxCorrespondenceDistance(1500);
  icp.setTransformationEpsilon(1e-10);
  icp.setEuclideanFitnessEpsilon(0.001);
  icp.setMaximumIterations(10); 
  icp.align( *result);
  // Save .pcd file
  *result = *result + *test1;
  int vec = pcl::io::savePCDFile("result.pcd", *result);
  return 0;
}