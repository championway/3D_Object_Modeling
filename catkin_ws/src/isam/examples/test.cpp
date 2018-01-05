#include <isam/isam.h>
#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
using namespace std;
using namespace isam;
using namespace Eigen;
ros::Publisher tf_ros_publisher;

int main() {
  // instance of the main class that manages and optimizes the pose graph
  Slam slam;

  // locally remember poses
  vector<Pose3d_Node*> pose_nodes;

  Noise noise3 = Information(100. * eye(7));
  Noise noise2 = Information(100. * eye(2));
  Pose3d tag0(0.209,0.124,0.871,3.025,-0.471,-1.237);
  Pose3d tag1(-0.0762,0.1974,0.8218,2.90989,-0.419,-1.24);
  // create a first pose (a node)
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
    Pose3d_Factor* prior = new Pose3d_Factor(pose_nodes[0], origin, noise3);
    slam.add_factor(prior);  // add it to the graph
    //=================================================
    Pose3d fixed(-0.166, 0.248, 0.14, 0, 0, 0);
    Pose3d_Pose3d_Factor* fixex_factor = new Pose3d_Pose3d_Factor(pose_nodes[0], pose_nodes[1], fixed, noise2);
    slam.add_factor(fixex_factor);
    //=================================================
    Pose3d_Pose3d_Factor* camera_tag0 = new Pose3d_Pose3d_Factor(pose_nodes[0], pose_nodes[2], tag0, noise2);
    slam.add_factor(camera_tag0);
    //=================================================
    Pose3d_Pose3d_Factor* camera_tag1 = new Pose3d_Pose3d_Factor(pose_nodes[1], pose_nodes[2], tag1, noise2);
    slam.add_factor(camera_tag1);
    //=================================================

    // optimize the graph
    Properties prop = slam.properties();
    prop.method = DOG_LEG;
    slam.set_properties(prop);
    cout << endl << "Here" << endl;
    slam.batch_optimization();
  // printing the complete graph
  cout << endl << "Full graph:" << endl;
  slam.write(cout);

  return 0;
}