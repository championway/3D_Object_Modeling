#include <ros/ros.h>
// PCL specific includes
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Header.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <math.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/common/transforms.h>
ros::Publisher tf_pcl_publisher;
ros::Publisher tf_ros_publisher;
ros::Publisher orig_ros_publisher;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

void  cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  sensor_msgs::PointCloud2 tf_ros;
  sensor_msgs::PointCloud2 orig_ros;
  pcl::PointCloud<pcl::PointXYZRGB> tf_pcl;  
  tf::TransformListener lr;
  tf::TransformBroadcaster br;
  tf::StampedTransform trans;
  orig_ros = *input;
  //tf::Stamped<tf::Transform> transform;
  //transform.setOrigin( tf::Vector3(0.0, 2.0, 0.0) );
  //transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
  //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), output1.header.frame_id, "aa"));
  try{
      lr.waitForTransform("/camera_link", orig_ros.header.frame_id, ros::Time(0), ros::Duration(10.0) );
      //lr.lookupTransform("viewed_tag_1", output1.header.frame_id, ros::Time(0), trans);
      //pcl_ros::transformPointCloud("viewed_tag_1", trans, output1, tf_ros);
      pcl_ros::transformPointCloud("/camera_link", *input, tf_ros, lr);
    }
    catch( tf::TransformException ex)
    {
      ROS_ERROR("transfrom exception : %s",ex.what());
  }

  pcl::fromROSMsg (tf_ros, tf_pcl);//convert from PointCloud2 to pcl_rgb
  //publish to topics
  orig_ros_publisher.publish(orig_ros);
  tf_pcl_publisher.publish(tf_pcl);
  tf_ros_publisher.publish(tf_ros);
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
     tf_pcl_publisher = nh.advertise<PointCloudRGB> ("tf_pcl", 1);
     tf_ros_publisher = nh.advertise<sensor_msgs::PointCloud2> ("tf_ros", 1);
     orig_ros_publisher = nh.advertise<sensor_msgs::PointCloud2> ("orig_ros", 1);
     // Spin
     ros::spin ();
  }

