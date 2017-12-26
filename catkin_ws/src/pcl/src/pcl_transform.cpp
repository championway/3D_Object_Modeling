#include <ros/ros.h>
// PCL specific includes
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <std_msgs/Header.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/common/transforms.h>

ros::Publisher tf_pcl_publisher;
ros::Publisher tf_ros_publisher;
tf::TransformListener* lr;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

void  cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  sensor_msgs::PointCloud2 tf_ros;
  pcl::PointCloud<pcl::PointXYZRGB> tf_pcl;  
  tf::TransformBroadcaster br;
  tf::StampedTransform trans;
  lr->waitForTransform("/viewed_tag_1", input->header.frame_id, ros::Time::now(), ros::Duration(10.0) );
  pcl_ros::transformPointCloud("/viewed_tag_1", *input, tf_ros, *lr); //tf transform to apriltag frame

  pcl::fromROSMsg (tf_ros, tf_pcl);//convert from PointCloud2 to pcl_rgb
  
  //publish to topics
  tf_pcl_publisher.publish(tf_pcl);
  tf_ros_publisher.publish(tf_ros);
  ROS_INFO("Success output");  
}

int   main (int argc, char** argv)
{
     // Initialize ROS
     std::cout << "START TO TRANSFORM";
     ros::init (argc, argv, "my_pcl_tutorial");
     ros::NodeHandle nh;

     // Create tf transform listener  
     tf::TransformListener listener(ros::Duration(10));
     lr = &listener;
     
     // Create a ROS subscriber for the input point cloud
     ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth_registered/points", 1, cloud_cb);
     
     // Create a ROS publisher for the output point cloud
     tf_pcl_publisher = nh.advertise<PointCloudRGB> ("tf_pcl", 1);
     tf_ros_publisher = nh.advertise<sensor_msgs::PointCloud2> ("tf_ros", 1);
     
     // Spin
     ros::spin ();
  }

