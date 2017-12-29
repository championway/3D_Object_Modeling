#include <iostream> 
#include <stdio.h>
#include <stdlib.h>
// PCL specific includes

#include <ros/ros.h>
#include<sensor_msgs/PointCloud2.h>
#include<sensor_msgs/Image.h>
#include<sensor_msgs/CameraInfo.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include "pcl_ros/point_cloud.h"

//Synchronize
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

//opencv, cv bridge
#include <cv_bridge/cv_bridge.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <image_transport/image_transport.h>

// cmath map for transformation
#include <string>
#include <cmath>
#include <map>

//Publisher
ros::Publisher model_publisher;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;
using namespace message_filters;
using namespace cv;
using namespace std;
Mat background1;
Mat background2;
int value_threshold=10;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr object1 (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr object2 (new pcl::PointCloud<pcl::PointXYZRGB>);


//     [fx'  0  cx' Tx]
// P = [ 0  fy' cy' Ty]
//    [ 0   0   1   0]
double fx1,fy1,cx1,cy1 ;

//transformation
map<string, float> pose_A, pose_B;

double fx2 ,fy2,cx2, cy2;
void combineTwoPCL(PointCloudRGB::Ptr& , PointCloudRGB::Ptr&);
void pclRGB_transform(PointCloudRGB::Ptr& input, map<string, float> &pose)
{
	float qx = pose["x"];
	float qy = pose["y"];
	float qz = pose["z"];
	float garma = pose["grama"];
	float COS = cos(garma);
	float SIN = sin(garma); 
	float t_x, t_y, t_z;
	for (int i=0; i< input->points.size(); i++)
	{
		t_x = COS * input->points[i].x - SIN * input->points[i].z + qx; 
		t_y = input->points[i].y  + qy;
		t_z = SIN * input->points[i].x + COS * input->points[i].z + qz;
		input->points[i].x = t_x;
		input->points[i].y = t_y;
		input->points[i].z = t_z;
	}
}
void  update_point1 (const sensor_msgs::ImageConstPtr& depth_image, const sensor_msgs::ImageConstPtr& rgb_image, const sensor_msgs::CameraInfoConstPtr& camera_info)//
{ 
	cv_bridge::CvImagePtr cv_ptr,cv_ptr_depth;
	cv_ptr = cv_bridge::toCvCopy(rgb_image, sensor_msgs::image_encodings::BGR8); 
	cv_ptr_depth = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::MONO8); 
	Mat image_with_ground_hsv, background, image_image_minus; 
	Mat hsv_mask = Mat::zeros(cv_ptr->image.rows,cv_ptr->image.cols,CV_8U);
	cvtColor(cv_ptr->image,image_with_ground_hsv,CV_BGR2HSV);
	cvtColor(background1,background,CV_BGR2HSV);

	image_image_minus = image_with_ground_hsv - background;
	for (int row=0;row<hsv_mask.rows;row++){
		for(int column=0;column<hsv_mask.cols;column++){
			if( abs( image_image_minus.at<Vec3b>(row,column)[2])>value_threshold)
				hsv_mask.at<uchar>(row,column)=1;
		}
	}
	int count=0;
	for (int row=0;row<hsv_mask.rows;row++){
		for(int column=0;column<hsv_mask.cols;column++){
			if( hsv_mask.at<uchar>(row,column)>0){
				double inv_fx = 1.0/fx1;
				double inv_fy = 1.0/fy1;
				double xc =cv_ptr_depth->image.at<uchar>(row,column);
				double yc = -(column-cx1) *  cv_ptr_depth->image.at<uchar>(row,column) * inv_fx;
				double zc = -(row-cy1) *  cv_ptr_depth->image.at<uchar>(row,column) * inv_fy;
				object1->points[count].x=xc;
				object1->points[count].y=yc;
				object1->points[count].z=zc;
				object1->points[count].r=cv_ptr->image.at<Vec3b>(row,column)[2];
				object1->points[count].g=cv_ptr->image.at<Vec3b>(row,column)[1];
				object1->points[count].b=cv_ptr->image.at<Vec3b>(row,column)[0];
			}
			count++;	
		}
	}
	object1->points.resize(count);
	pclRGB_transform(object1, pose_A);
}

void  align_model (const sensor_msgs::ImageConstPtr depth_image, const sensor_msgs::ImageConstPtr& rgb_image, const sensor_msgs::CameraInfoConstPtr& camera_info)
{ 
	cv_bridge::CvImagePtr cv_ptr,cv_ptr_depth;
	cv_ptr = cv_bridge::toCvCopy(rgb_image, sensor_msgs::image_encodings::BGR8); 
	cv_ptr_depth = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::MONO8); 
	Mat image_with_ground_hsv, background, image_image_minus; 
	Mat hsv_mask = Mat::zeros(cv_ptr->image.rows,cv_ptr->image.cols,CV_8U);
	cvtColor(cv_ptr->image,image_with_ground_hsv,CV_BGR2HSV);
	cvtColor(background1,background,CV_BGR2HSV);

	image_image_minus = image_with_ground_hsv - background;
	for (int row=0;row<hsv_mask.rows;row++){
		for(int column=0;column<hsv_mask.cols;column++){
			if( abs( image_image_minus.at<Vec3b>(row,column)[2])>value_threshold)
				hsv_mask.at<uchar>(row,column)=1;
		}
	}
	int count=0;
	for (int row=0;row<hsv_mask.rows;row++){
		for(int column=0;column<hsv_mask.cols;column++){
			if( hsv_mask.at<uchar>(row,column)>0){
				double inv_fx = 1.0/fx1;
				double inv_fy = 1.0/fy1;
				double xc =cv_ptr_depth->image.at<uchar>(row,column);
				double yc = -(column-cx1) *  cv_ptr_depth->image.at<uchar>(row,column) * inv_fx;
				double zc = -(row-cy1) *  cv_ptr_depth->image.at<uchar>(row,column) * inv_fy;
				object2->points[count].x=xc;
				object2->points[count].y=yc;
				object2->points[count].z=zc;
				object2->points[count].r=cv_ptr->image.at<Vec3b>(row,column)[2];
				object2->points[count].g=cv_ptr->image.at<Vec3b>(row,column)[1];
				object2->points[count].b=cv_ptr->image.at<Vec3b>(row,column)[0];
			}
			count++;	
		}
	}
	object2->points.resize(count);
	pclRGB_transform(object2, pose_B);
	combineTwoPCL(object1, object2);
	
	
}

void combineTwoPCL(PointCloudRGB::Ptr& object_cloud, PointCloudRGB::Ptr& align_cloud)
{
	PointCloudRGB::Ptr combine_object(new PointCloudRGB);
	sensor_msgs::PointCloud2 combine_pcl_model;
	//object_cloud->header.frame_id = "/camera_link";
	combine_object->header.frame_id = "/camera_link";
	combine_object->points.resize(object_cloud->points.size() + align_cloud->points.size());
	for(int i=0; i<object_cloud->points.size();i++)
		combine_object->points[i] = object_cloud->points[i];
	for(int i=object_cloud->points.size(); i< combine_object->points.size(); i++)
		combine_object->points[i] = align_cloud->points[i];
	
	pcl::toROSMsg(*combine_object, combine_pcl_model);
	printf("Combine pcl object size: %d", combine_object->points.size());
	// publish here
	model_publisher.publish(combine_pcl_model);	
}


int main(int argc, char** argv){

	//Read Background 
	//background1 = imread("/home/michael/sis/baichung/background_camera1.jpg",CV_LOAD_IMAGE_COLOR);
	//background2 = imread("/home/michael/sis/baichung/background_camera2.jpg",CV_LOAD_IMAGE_COLOR);
	background1 = imread("/home/chses910372/3D_Object_Modeling/catkin_ws/src/pcl/image/background1.jpg",CV_LOAD_IMAGE_COLOR);
	background2 = imread("/home/chses910372/3D_Object_Modeling/catkin_ws/src/pcl/image/background2.jpg",CV_LOAD_IMAGE_COLOR);

	ros::init(argc, argv, "combine_pcl");
	ros::NodeHandle nh; 
	sensor_msgs::CameraInfoConstPtr camerainfo1 = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/abc/depth/camera_info", ros::Duration(10));

	//     [fx'  0  cx' Tx]
	// P = [ 0  fy' cy' Ty]
	//    [ 0   0   1   0]
	fx1 = camerainfo1->P[0];
	fy1 = camerainfo1->P[5];
	cx1 = camerainfo1->P[2];
	cy1 = camerainfo1->P[6];

	sensor_msgs::CameraInfoConstPtr camerainfo2 = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/def/depth/camera_info", ros::Duration(10));
	fx2 = camerainfo2->P[0];
	fy2 = camerainfo2->P[5];
	cx2 = camerainfo2->P[2];
	cy2 = camerainfo2->P[6];
	nh.getParam("/pcl/camera_abc", pose_A);
	nh.getParam("/pcl/camera_def", pose_B);
	nh.getParam("/pcl/value_threshold", value_threshold);
	model_publisher = nh.advertise<sensor_msgs::PointCloud2>("/combine_model", 10);
	/////////////////////////////Declare camera 1 subscriber/////////////////////////////
    message_filters::Subscriber<sensor_msgs::Image> depth_image_camera1(nh,"/abc/depth/image", 10);
    message_filters::Subscriber<sensor_msgs::Image> rgb_image_camera1(nh, "/abc/rgb/image_color", 10);
    message_filters::Subscriber<sensor_msgs::CameraInfo> camera_info1(nh, "/abc/depth/camera_info", 10);
    typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> MySyncPolicy1;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    Synchronizer<MySyncPolicy1> sync1(MySyncPolicy1(10), depth_image_camera1, rgb_image_camera1,camera_info1); //, camera_info1
    sync1.registerCallback(boost::bind(&update_point1,_1,_2,_3));  //,_3
    
    /////////////////////////////Declare camera 2 subscriber/////////////////////////////
    message_filters::Subscriber<sensor_msgs::Image> depth_image_camera2(nh,"/def/depth/image", 10);
    message_filters::Subscriber<sensor_msgs::Image> rgb_image_camera2(nh, "/def/rgb/image_color", 10);
    message_filters::Subscriber<sensor_msgs::CameraInfo> camera_info2(nh, "/def/depth/camera_info", 10);
    typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> MySyncPolicy2;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    Synchronizer<MySyncPolicy2> sync2(MySyncPolicy2(10), depth_image_camera2, rgb_image_camera2,camera_info2);
    sync2.registerCallback(boost::bind(&align_model, _1, _2,_3));
	

	ros::spin();

}

