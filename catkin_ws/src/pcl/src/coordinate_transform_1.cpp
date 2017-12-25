#include <ros/ros.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
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

void  cloud_cb (const sensor_msgs::PointCloudXYZRGB::ConstPtr& input, const geometry_msgs::PoseArray tag)
{
  pcl::fromROSMsg (*input, pcl_orig);//convert from PointCloud2 to pcl
  pcl::fromROSMsg (*input, tt);
  //std::cout << hold.header.frame_id << endl << endl;
  sensor_msgs::convertPointCloud2ToPointCloud(*input, hold);
  //string a = hold.header.frame_id;
  //std::cout << "hold:  " << hold.header.frame_id << std::endl;
  output1 = *input;

  tf::TransformListener lr;
  tf::TransformBroadcaster br;
  tf::StampedTransform trans;
  tf::Transform transform;
  if (tag.poses[0].position.x != 0)
  {
    for(int i = 0; i < sizeof(tag); i++)
    {
      std::cout << tag.poses[0].position.x;
      try{
      //lr.transformPointCloud("viewed_tag_1", hold , tf_ros1);
        
        std::cout << tag.poses[i].position.x <<"=="<< tag.poses[i].position.y <<"=="<< tag.poses[i].position.z;

        //transform.setOrigin(tf::Vector3(tag.poses[i].position.x, tag.poses[i].position.y, tag.poses[i].position.z) );
        //transform.setRotation( tf::Quaternion(tag.poses[i].orientation.x, tag.poses[i].orientation.y, tag.poses[i].orientation.z, tag.poses[i].orientation.w) );
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), hold.header.frame_id, "tag1"));
        lr.waitForTransform("tag1", hold.header.frame_id, ros::Time(0), ros::Duration(10.0) );
        lr.lookupTransform("tag1", hold.header.frame_id, ros::Time(0), trans);
        pcl_ros::transformPointCloud("ttt", trans, output1, tf_ros2);
      }
      catch( tf::TransformException ex)
      {
          ROS_ERROR("transfrom exception : %s",ex.what());
      }
      
      std::cout << "tros:  " <<  tf_ros1.header.frame_id << std::endl;
      sensor_msgs::convertPointCloudToPointCloud2(tf_ros1, tf_ros);

      pcl::PointCloud<pcl::PointXYZRGB> tf_pcl;
      pcl::fromROSMsg (tf_ros, tf_pcl);//convert from PointCloud2 to pcl_rgb

      pcl::toROSMsg(pcl_orig, pcl_to_ros_pointcloud2);//convert back to PointCloud2
      //publish to topics
      //std::cout << "eeee";
      pub_tf.publish (tf_ros);
      pointcloudXYZ.publish(pcl_orig);
      point_tf.publish(tf_pcl);
      point_orig.publish(tt);
      pointcloud2_publisher.publish(pcl_to_ros_pointcloud2);
      ROS_INFO("Success output");//cout   
    }
  }
}


int   main (int argc, char** argv)
{
     // Initialize ROS
     ros::init (argc, argv, "my_pcl_tutorial");
     ros::NodeHandle nh;  
     // Create a ROS subscriber for the input point cloud
     //=============================
     message_filters::Subscriber<PointCloudXYZRGB> pcl_sub(nh, "/camera/depth_registered/points", 1);
     message_filters::Subscriber<geometry_msgs::PoseArray> rgb_sub(nh, "/sr300/tag_detections_pose", 1);
     typedef sync_policies::ApproximateTime<PointCloudXYZRGB, Image> MySyncPolicy;
     // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
     Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), pcl_sub, rgb_sub);
     //TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::Image > sync(pcl_sub, rgb_sub, 10);
     sync.registerCallback(boost::bind(&func_cb, _1, _2));
     //=============================

     // Create a ROS publisher for the output point cloud
     pub_tf = nh.advertise<sensor_msgs::PointCloud2> ("tf_ros", 1); 
     pointcloudXYZ = nh.advertise<PointCloudXYZ> ("ros_pointcloudxyz", 1);
     point_tf = nh.advertise<PointCloudRGB> ("tf_pcl", 1);
     point_orig = nh.advertise<PointCloudRGB> ("orig_tf", 1);
     pointcloud2_publisher = nh.advertise<sensor_msgs::PointCloud2> ("pcltoros_pointcloud2", 1);
     // Spin
     ros::spin ();
  }

