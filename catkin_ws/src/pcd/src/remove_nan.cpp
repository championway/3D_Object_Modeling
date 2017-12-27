#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/buffer.h>
// PCL includes
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;

int main (int argc, char** argv)
{
  PointCloudXYZRGB::Ptr cloud_in (new PointCloudXYZRGB);
  PointCloudXYZRGB::Ptr cloud_out(new PointCloudXYZRGB);

  // Load .pcd file from argv[1]
  int ret = pcl::io::loadPCDFile ("1514290554413349.pcd", *cloud_in);
  if (ret < 0) {
      PCL_ERROR("Couldn't read file %s\n", argv[1]);
    return -1;
  }
  std::cout << "size of input: " << cloud_in->points.size() << std::endl;

  // Remove NaN point
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, indices);
  // Save .pcd file
  int vec = pcl::io::savePCDFile("removeNaN.pcd", *cloud_in);

  std::cout << "size of output: " << cloud_in->points.size() << std::endl;  
  return 0;
}