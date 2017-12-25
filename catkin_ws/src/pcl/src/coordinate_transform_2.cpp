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
ros::Publisher pub_tf;
ros::Publisher pointcloudXYZ;
ros::Publisher point_tf;
ros::Publisher point_orig;
ros::Publisher pointcloud2_publisher;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;
sensor_msgs::PointCloud2 tf_ros;
sensor_msgs::PointCloud tf_ros1;
sensor_msgs::PointCloud2 tf_ros2;
sensor_msgs::PointCloud hold;
sensor_msgs::PointCloud2 output1;
sensor_msgs::PointCloud2 pcl_to_ros_pointcloud2;
pcl::PointCloud<pcl::PointXYZ> pcl_orig;
pcl::PointCloud<pcl::PointXYZRGB> tt;

void  cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  tf::TransformListener lr;
  tf::TransformBroadcaster br;
  tf::StampedTransform trans;
	pcl::fromROSMsg (*input, pcl_orig);//convert from PointCloud2 to pcl
  pcl::fromROSMsg (*input, tt);
  sensor_msgs::convertPointCloud2ToPointCloud(*input, hold);
  output1 = *input;
  //tf::Stamped<tf::Transform> transform;
  //transform.setOrigin( tf::Vector3(0.0, 2.0, 0.0) );
  //transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
  //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), output1.header.frame_id, "aa"));
  try{
      lr.waitForTransform("viewed_tag_1", output1.header.frame_id, ros::Time(0), ros::Duration(10.0) );
      //lr.lookupTransform("viewed_tag_1", output1.header.frame_id, ros::Time(0), trans);
      //pcl_ros::transformPointCloud("viewed_tag_1", trans, output1, tf_ros2);
      pcl_ros::transformPointCloud("aa", *input, tf_ros2, lr);
    }
    catch( tf::TransformException ex)
    {
      ROS_ERROR("transfrom exception : %s",ex.what());
  }
  
  //std::cout << "tros:  " <<  output1.header.frame_id << std::endl;
  //sensor_msgs::convertPointCloudToPointCloud2(tf_ros1, tf_ros);

  pcl::PointCloud<pcl::PointXYZRGB> tf_pcl;
  pcl::fromROSMsg (tf_ros2, tf_pcl);//convert from PointCloud2 to pcl_rgb

  pcl::toROSMsg(pcl_orig, pcl_to_ros_pointcloud2);//convert back to PointCloud2
  //publish to topics
  //std::cout << "eeee";
  pub_tf.publish (tf_ros2);
  pointcloudXYZ.publish(pcl_orig);
  point_tf.publish(tf_pcl);
  point_orig.publish(tt);
  pointcloud2_publisher.publish(pcl_to_ros_pointcloud2);
  ROS_INFO("Success output");//cout   
}


int   main (int argc, char** argv)
{
     // Initialize ROS
     ros::init (argc, argv, "my_pcl_tutorial");
     ros::NodeHandle nh;  
     // Create a ROS subscriber for the input point cloud
     ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth_registered/points", 1, cloud_cb);
     // Create a ROS publisher for the output point cloud
     pub_tf = nh.advertise<sensor_msgs::PointCloud2> ("tf_ros", 1); 
     pointcloudXYZ = nh.advertise<PointCloudXYZ> ("ros_pointcloudxyz", 1);
     point_tf = nh.advertise<PointCloudRGB> ("tf_pcl", 1);
     point_orig = nh.advertise<PointCloudRGB> ("orig_tf", 1);
     pointcloud2_publisher = nh.advertise<sensor_msgs::PointCloud2> ("pcltoros_pointcloud2", 1);
     // Spin
     ros::spin ();
  }

