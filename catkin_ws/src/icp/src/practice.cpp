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
ros::Publisher tf_pcl_publisher;
ros::Publisher tf_ros_publisher;
ros::Publisher orig_ros_publisher;
using namespace pcl;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;


int   main (int argc, char** argv)
{
    // Initialize ROS
    std::cout << "START TO TRANSFORM";
    ros::init (argc, argv, "my_pcl_tutorial");
    ros::NodeHandle nh;  

    // Create a ROS publisher for the output point cloud
    tf_pcl_publisher = nh.advertise<PointCloudXYZRGB> ("tf_pcl", 1);
    tf_ros_publisher = nh.advertise<PointCloudXYZRGB> ("tf_ros", 1);
    orig_ros_publisher = nh.advertise<PointCloudXYZRGB> ("orig_ros", 1);
    //shit();
  PointCloudXYZRGB::Ptr cloud_in (new PointCloudXYZRGB);
  PointCloudXYZRGB::Ptr cloud_out (new PointCloudXYZRGB);

  // Fill in the CloudIn data
  cloud_in->width    = 5;
  cloud_in->height   = 1;
  cloud_in->is_dense = false;
  cloud_in->points.resize (cloud_in->width * cloud_in->height);
  for (size_t i = 0; i < cloud_in->points.size (); ++i)
  {
    cloud_in->points[i].r = 0;
    cloud_in->points[i].g = 255;
    cloud_in->points[i].b = 0;
    cloud_in->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_in->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_in->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }
    std::cout << "Saved " << cloud_in->points.size () << " data points to input:"
        << std::endl;
    for (size_t i = 0; i < cloud_in->points.size (); ++i) std::cout << "    " <<
        cloud_in->points[i].x << " " << cloud_in->points[i].y << " " <<
        cloud_in->points[i].z << std::endl;
    *cloud_out = *cloud_in;
    std::cout << "size:" << cloud_out->points.size() << std::endl;
    for (size_t i = 0; i < cloud_in->points.size (); ++i)
      cloud_out->points[i].x = cloud_in->points[i].x + 0.7f;
    std::cout << "Transformed " << cloud_in->points.size () << " data points:"
        << std::endl;
    for (size_t i = 0; i < cloud_out->points.size (); ++i)
      std::cout << "    " << cloud_out->points[i].x << " " <<
        cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    icp.setInputSource(cloud_in);
    icp.setInputTarget(cloud_out);
    PointCloudXYZRGB::Ptr Final (new PointCloudXYZRGB);   
    icp.align(*Final);
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
    icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;
    cloud_in->header.frame_id = "camera_link";
    cloud_out->header.frame_id = "camera_link";
    Final->header.frame_id = "camera_link";
    while(1)
    {
    orig_ros_publisher.publish(cloud_in);
    tf_pcl_publisher.publish(cloud_out);
    tf_ros_publisher.publish(Final);
    ROS_INFO("Success output");//cout
    }
    // Spin
    ros::spin ();
  }

