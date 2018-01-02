#include <ros/ros.h>
#include <iostream>
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
#include <sensor_msgs/Image.h>
#include <pcl/common/transforms.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
using namespace message_filters;
ros::Publisher tf_pcl_publisher;
ros::Publisher tf_ros_publisher;

tf::TransformListener* lr;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

void  call_back (const sensor_msgs::PointCloud2ConstPtr& input, const sensor_msgs::ImageConstPtr& tag)
{
  int tag_id;
  sensor_msgs::PointCloud2 tf_ros;
  pcl::PointCloud<pcl::PointXYZRGB> tf_pcl;  
  tf::TransformBroadcaster br;
  tf::StampedTransform trans;

  if(tag->step == 0)
  {
    lr->waitForTransform("/viewed_tag_0", input->header.frame_id, ros::Time::now(), ros::Duration(10.0) );
    pcl_ros::transformPointCloud("/viewed_tag_0", *input, tf_ros, *lr); //tf transform to apriltag frame
    std::cout << "0" << std::endl;
  }
  else if(tag->step == 1)
  {
    lr->waitForTransform("/viewed_tag_1", input->header.frame_id, ros::Time::now(), ros::Duration(10.0) );
    pcl_ros::transformPointCloud("/viewed_tag_1", *input, tf_ros, *lr); //tf transform to apriltag frame
    std::cout << "1" << std::endl;
  }

  pcl::fromROSMsg (tf_ros, tf_pcl);//convert from PointCloud2 to pcl_rgb
  //publish to topics
  tf_pcl_publisher.publish(tf_pcl);
  tf_ros_publisher.publish(tf_ros);
  ROS_INFO("Success output");  
}

int main (int argc, char** argv)
{
    // Initialize ROS
    std::cout << "START TO TRANSFORM";
    ros::init (argc, argv, "my_pcl_tutorial");
    ros::NodeHandle nh;

    // Create tf transform listener  
    tf::TransformListener listener(ros::Duration(10));
    lr = &listener;
    //boost::shared_ptr<apriltags_ros::AprilTagDetectionArray> detector_;
    // Create a ROS subscriber for the input point cloud
    message_filters::Subscriber<sensor_msgs::PointCloud2> ros_point(nh,"/camera/depth_registered/points", 10);
    message_filters::Subscriber<sensor_msgs::Image> tags(nh, "/id_output", 10);
    typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    Synchronizer<MySyncPolicy> sync1(MySyncPolicy(10), ros_point, tags); //, camera_info1
    sync1.registerCallback(boost::bind(&call_back,_1,_2));  //
    // Create a ROS publisher for the output point cloud
    tf_pcl_publisher = nh.advertise<PointCloudRGB> ("tf_pcl", 1);
    tf_ros_publisher = nh.advertise<sensor_msgs::PointCloud2> ("tf_ros", 1);
    
    // Spin
    ros::spin ();
}

