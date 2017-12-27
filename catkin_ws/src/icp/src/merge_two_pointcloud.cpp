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
#include <pcl/filters/radius_outlier_removal.h>
#include <image_transport/image_transport.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

ros::Publisher input_publisher;
ros::Publisher output_publisher;
ros::Publisher icp_publisher;

using namespace pcl;
using namespace message_filters;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
const PointCloudXYZRGB::ConstPtr& input, const ImageConstPtr& rgb
void  cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input_point2)
{
  // declare icp
  pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
  // declare point cloud
  PointCloudXYZRGB::Ptr cloud_in (new PointCloudXYZRGB);
  PointCloudXYZRGB::Ptr cloud_out(new PointCloudXYZRGB);
  PointCloudXYZRGB::Ptr icp_out (new PointCloudXYZRGB); 
  // convert from PointCloud2 to pcl_rgb
  pcl::fromROSMsg ( *input_point2, *cloud_in);

  icp.setInputSource(cloud_in);
  icp.setInputTarget(cloud_out);
    
  icp.align( *icp_out);

  std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;
  
  cloud_in->header.frame_id = "icp";
  cloud_out->header.frame_id = "icp";
  Final->header.frame_id = "icp";

  // publish point cloud
  input_publisher.publish(cloud_in);
  output_publisher.publish(cloud_out);
  icp_publisher.publish(icp_out);
  ROS_INFO("Success output");//cout
}

int   main (int argc, char** argv)
{
  // Initialize ROS
  std::cout << "START TO TRANSFORM";
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;
     
  // Create a ROS subscriber for the input point cloud
  //ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth_registered/points", 1, cloud_cb);
  message_filters::Subscriber<PointCloudXYZRGB> camera1_sub(nh, "/camera1/tf_pcl", 1);
  message_filters::Subscriber<sensor_msgs::Image> camera2_sub(nh, "/camera2/tf_pcl", 1);
  typedef sync_policies::ApproximateTime<PointCloudXYZRGB, Image> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), camera1_sub, camera2_sub);
  //TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::Image > sync(pcl_sub, rgb_sub, 10);
  sync.registerCallback(boost::bind(&cloud_cb, _1, _2));

  // Create a ROS publisher for the output point cloud
  input_publisher = nh.advertise<PointCloudXYZRGB> ("cloud_in", 1);
  output_publisher = nh.advertise<PointCloudXYZRGB> ("cloud_out", 1);
  icp_publisher = nh.advertise<PointCloudXYZRGB> ("icp_out", 1);

  // Spin
  ros::spin ();
}
