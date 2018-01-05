#include <isam/isam.h>
#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
using namespace std;
using namespace isam;
using namespace Eigen;
ros::Publisher tf_ros_publisher;

void toEulerAngle(const geometry_msgs::Quaternion q, double& roll, double& pitch, double& yaw)
{
  // roll (x-axis rotation)
  double sinr = +2.0 * (q.w * q.x + q.y * q.z);
  double cosr = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
  roll = atan2(sinr, cosr);

  // pitch (y-axis rotation)
  double sinp = +2.0 * (q.w * q.y - q.z * q.x);
  if (fabs(sinp) >= 1)
    pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
    pitch = asin(sinp);

  // yaw (z-axis rotation)
  double siny = +2.0 * (q.w * q.z + q.x * q.y);
  double cosy = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);  
  yaw = atan2(siny, cosy);
}

void tag_cb(const apriltags_ros::AprilTagDetectionArray::ConstPtr& input)
{
  int tag_length = input->detections.size();
  if ( tag_length > 1)
  {
    //std::cout << "Start iSam" << std::endl;
    //=======Delcare======= 
    Slam slam;  // instance of the main class that manages and optimizes the pose graph
    vector<Pose3d_Node*> pose_nodes;  // locally remember poses
    Pose3d tag0, tag1;
    Noise noise3 = Information(100. * eye(7));
    Noise noise2 = Information(100. * eye(7));

    for(int i = 0; i < tag_length; i++)
    {
      double r, p, y;
      toEulerAngle(input->detections[i].pose.pose.orientation, r, p, y);
      if (input->detections[i].id == 0)
        {tag0.set(input->detections[i].pose.pose.position.x, input->detections[i].pose.pose.position.y, input->detections[i].pose.pose.position.z, r, p, y);
        std::cout<< input->detections[i].pose.pose.position.x << ", " << input->detections[i].pose.pose.position.y << ", " << input->detections[i].pose.pose.position.z << ", " << r << ", " << p << ", " <<  y<< std::endl;}
      if (input->detections[i].id == 1)
        {tag1.set(input->detections[i].pose.pose.position.x, input->detections[i].pose.pose.position.y, input->detections[i].pose.pose.position.z, r, p, y);}
      //std::cout<< input->detections[i].pose.pose.position.x << std::endl << input->detections[i].pose.pose.position.y << std::endl << input->detections[i].pose.pose.position.z << std::endl << r << std::endl << p << std::endl <<  y<< std::endl;
    }
    
    //=================================================
    Pose3d_Node* tag0_node = new Pose3d_Node();  // create a first pose (a node)
    slam.add_node(tag0_node);  // add it to the graph
    pose_nodes.push_back(tag0_node);  // also remember it locally
    Pose3d_Node* tag1_node = new Pose3d_Node();
    slam.add_node(tag1_node);
    pose_nodes.push_back(tag1_node);
    Pose3d_Node* camera_node = new Pose3d_Node();
    slam.add_node(camera_node);
    pose_nodes.push_back(camera_node);
    //=================================================
    Pose3d origin(0, 0, 0, 0, 0, 0);
    Pose3d_Factor* prior = new Pose3d_Factor(pose_nodes[2], origin, noise3);
    slam.add_factor(prior);  // add it to the graph
    //=================================================
    Pose3d_Pose3d_Factor* camera_tag0 = new Pose3d_Pose3d_Factor(pose_nodes[2], pose_nodes[0], tag0, noise2);
    slam.add_factor(camera_tag0);
    //=================================================
    Pose3d_Pose3d_Factor* camera_tag1 = new Pose3d_Pose3d_Factor(pose_nodes[2], pose_nodes[1], tag1, noise2);
    slam.add_factor(camera_tag1);
    //=================================================
    Pose3d fixed(-0.166, 0.248, 0, 0, 0, 0);
    Pose3d_Pose3d_Factor* fixex_factor = new Pose3d_Pose3d_Factor(pose_nodes[0], pose_nodes[1], fixed, noise2);
    slam.add_factor(fixex_factor);
    //=================================================

    // optimize the graph
    //Properties prop = slam.properties();
    //prop.method = DOG_LEG;
    //slam.set_properties(prop);
    slam.batch_optimization();

    // accessing the current estimate of a specific pose
    cout << pose_nodes[0]->value() << endl << endl << endl;
    tf::Transform transform;
    tf::Quaternion q;
    tf::TransformBroadcaster br;
    cout << "1"<<endl;
    transform.setOrigin(tf::Vector3(0,2,1));
    cout << "2"<<endl;
    q.setRPY(0, 0, 1);
    cout << "3"<<endl;
    transform.setRotation(q);
    cout << "4"<<endl;
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/isam_tf", "/camera_rgb_optical_frame"));
    // printing the complete graph
    //cout << endl << "Full graph:" << endl;
    //slam.write(cout);
    
    //publish to topics
    //tf_ros_publisher.publish(tf_pcl);
    ROS_INFO("Success output");
  }
  
}

int main(int argc, char** argv) {
  // Initialize ROS
  std::cout << "START ISAM";
  ros::init (argc, argv, "apriltags_isam");
  ros::NodeHandle nh;
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<apriltags_ros::AprilTagDetectionArray> ("/sr300/tag_detections", 1, tag_cb);
  // Create a ROS publisher for the output point cloud
  tf_ros_publisher = nh.advertise<apriltags_ros::AprilTagDetectionArray> ("tf_pcl", 1);
  // Spin
  ros::spin ();
  
}