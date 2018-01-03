#include <isam/isam.h>

using namespace std;
using namespace isam;
using namespace Eigen;

int main() {
  // instance of the main class that manages and optimizes the pose graph
  Slam slam;

  // locally remember poses
  vector<Pose3d_Node*> pose_nodes;

  Noise noise3 = Information(100. * eye(3));
  Noise noise2 = Information(100. * eye(2));
  //=================================================
  Pose3d_Node* tag0_node = new Pose3d_Node();  // create a first pose (a node)
  slam.add_node(tag0_node);  // add it to the graph
  pose_nodes.push_back(tag0_node);  // also remember it locally

  // create a prior measurement (a factor)
  Pose3d tag0(0., 0., 0., 0., 0., 0.);
  Pose3d_Factor* tag0_factor = new Pose3d_Factor(pose_nodes[0], tag0, noise3);
  slam.add_factor(tag0_factor);  // add it to the graph
  //=================================================
  Pose3d_Node* tag1_node = new Pose3d_Node();
  slam.add_node(tag1_node);
  pose_nodes.push_back(tag1_node);

  // connect to previous with odometry measurement
  Pose3d tag1(0., 1., 0., 0., 0., 0.); // x,y,theta
  Pose3d_Pose3d_Factor* tag1_factor = new Pose3d_Pose3d_Factor(pose_nodes[0], pose_nodes[1], tag1, noise2);
  slam.add_factor(tag1_factor);
  //=================================================
  Pose3d_Node* camera_node = new Pose3d_Node();
  slam.add_node(camera_node);
  pose_nodes.push_back(camera_node);

  // connect to previous with odometry measurement
  Pose3d camera(0., 2., 0., 0., 0., 0.); // x,y,theta
  Pose3d_Pose3d_Factor* camera_factor = new Pose3d_Pose3d_Factor(pose_nodes[1], pose_nodes[2], camera, noise2);
  slam.add_factor(camera_factor);
  //=================================================

  // optimize the graph
  slam.batch_optimization();

  // accessing the current estimate of a specific pose
  cout << "Camera pose: " << pose_nodes[2]->value() << endl;

  // printing the complete graph
  cout << endl << "Full graph:" << endl;
  slam.write(cout);

  return 0;
}
