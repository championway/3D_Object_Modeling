#include <ros/ros.h>
// PCL specific includes
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl_ros/point_cloud.h"
#include <math.h>
ros::Publisher pub;
ros::Publisher pointcloudXYZ;
ros::Publisher point_trans;
ros::Publisher point_orig;
ros::Publisher pointcloud2_publisher;
tf::TransformListener listener;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;
const float theta = -30.0;
const float dx = 0.0;
const float dy = -0.5;
const float dz = 0.0;
const float t_ = theta*M_PI/180.0;

void  cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	 // Create a container for the data.
 	sensor_msgs::PointCloud2 output;
 	sensor_msgs::PointCloud2 pcl_to_ros_pointcloud2;

	// Do data processing here...
	output = *input;
	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::fromROSMsg (*input, cloud);//convert from PointCloud2 to pcl
  pcl::PointCloud<pcl::PointXYZRGB> cloud_rgb;
  pcl::fromROSMsg (*input, cloud_rgb);
	/*
       void fromROSMsg(const sensor_msgs::PointCloud2 &cloud, pcl::PointCloud<T> &pcl_cloud)
       {
            pcl::PCLPointCloud2 pcl_pc2;
            pcl_conversions::toPCL(cloud, pcl_pc2);
            pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud);
       }
       */
  // coordinate transform
  pcl::PointCloud<pcl::PointXYZRGB> tt;
  tt = cloud_rgb;
  /*tt.width = cloud_rgb.width;
  tt.height = cloud_rgb.height;
  tt.is_dense = cloud_rgb.is_dense;
  tt.points.resize(tt.width * tt.height);*/
  for (size_t i = 0; i < cloud_rgb.points.size(); i++){
    /*tt.points[i].x = cloud_rgb.points[i].x + dx*1;
    tt.points[i].y = cos(t_)*cloud_rgb.points[i].y - sin(t_)*cloud_rgb.points[i].z + dy*1;
    tt.points[i].z = sin(t_)*cloud_rgb.points[i].y + cos(t_)*cloud_rgb.points[i].z + dz*1;

    tt.points[i].y = -( cos(t_)*cloud_rgb.points[i].x + sin(t_)*cloud_rgb.points[i].z + dx*1 );
    tt.points[i].z = -( cloud_rgb.points[i].y + dy*1 );
    tt.points[i].x = -sin(t_)*cloud_rgb.points[i].x + cos(t_)*cloud_rgb.points[i].z + dz*1;*/

    tt.points[i].rgb = cloud_rgb.points[i].rgb;
    if (tt.points[i].z < 0){
      tt.points[i].r = 255;
      tt.points[i].g = 0;
      tt.points[i].b = 0;
    }
  }
  // end
  pcl::toROSMsg(cloud, pcl_to_ros_pointcloud2);//convert back to PointCloud2
	//publish to topics
  pub.publish (output);
	pointcloudXYZ.publish(cloud);
  point_orig.publish(cloud_rgb);
  point_trans.publish(tt);
	pointcloud2_publisher.publish(pcl_to_ros_pointcloud2);
	//ROS_INFO("Success output");//cout
}   

/*int* coordinate_tansform(float t, float dx, float dy, float dz, int x, int y, int z){
  t = t*M_PI/180.0;
  float trans_mat[4][4]={{1, 0, 0, dx}, {0, cos(t), -sin(t), dy}, {0, sin(t), cos(t), dz}, {0, 0, 0, 1}};
  int ori_c[4][1]={{x}, {y}, {z}, {1}};
  new int new_c[3][1]={0};
  for(int m = 0; m < 4; m++){
    for(int n = 0; n < 4; n++){
      new_c[m][0] += trans_mat[m][n] * ori_c[n][0];
    }
  }
  return new_c;
}*/

int   main (int argc, char** argv)
{
     // Initialize ROS
     ros::init (argc, argv, "my_pcl_tutorial");
     ros::NodeHandle nh;   
     // Create a ROS subscriber for the input point cloud
     ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth_registered/points", 1, cloud_cb);
     // Create a ROS publisher for the output point cloud
     pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1); 
     pointcloudXYZ = nh.advertise<PointCloudXYZ> ("ros_pointcloudxyz", 1);
     point_orig = nh.advertise<PointCloudRGB> ("ros_orig", 1);
     point_trans = nh.advertise<PointCloudRGB> ("ros_trans", 1);
     pointcloud2_publisher = nh.advertise<sensor_msgs::PointCloud2> ("pcltoros_pointcloud2", 1);
     // Spin
     ros::spin ();
  }

