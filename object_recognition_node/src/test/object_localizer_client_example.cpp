/**
 * @file object_localizer_client_example.cpp
 * @brief Example demonstrating the usage of the LocalizeObjects service.
 * @author Venkatraman Narayanan
 * Carnegie Mellon University, 2016
 */

#include <eigen_conversions/eigen_msg.h>
#include <object_recognition_node/object_localizer_service.h>
#include <perception_utils/pcl_typedefs.h>
#include <ros/package.h>
#include <ros/ros.h>

#include <pcl/io/pcd_io.h>

using std::vector;
using std::string;
using namespace sbpl_perception;

#define MIN_X -1.5
#define MAX_X 1.5
#define MIN_Y -1.5
#define MAX_Y 1.5

int main(int argc, char **argv) {
  ros::init(argc, argv, "object_localizer_client_node");
  // The camera pose and preprocessed point cloud, both in world frame.
  Eigen::Isometry3d camera_pose;
  camera_pose.matrix() <<
                       -0.0462167, -0.089684, -0.994897, 0.085115,
                        0.0351477, -0.995491, 0.0881048, 0.0635849,
                       -0.998313, -0.0308964,  0.0491605,   0.834256,
                                0,           0,          0,         1;

  const string demo_pcd_file = ros::package::getPath("sbpl_perception") +
                               "/demo/2017test/apc2017.pcd";
  // Objects for storing the point clouds.
  pcl::PointCloud<PointT>::Ptr cloud_in(new PointCloud);

  // Read the input PCD file from disk.
  if (pcl::io::loadPCDFile<PointT>(demo_pcd_file.c_str(),
                                   *cloud_in) != 0) {
    std::cerr << "Could not find demo PCD file!" << endl;
    return -1;
  }

  ros::NodeHandle nh;
  ros::ServiceClient client =
    nh.serviceClient<object_recognition_node::LocalizeObjects>("object_localizer_service");
  object_recognition_node::LocalizeObjects srv;
  auto &req = srv.request;
  req.x_min = MIN_X;
  req.x_max = MAX_X;
  req.y_min = MIN_Y;
  req.y_max = MAX_Y;
  req.support_surface_height = 0.0;
  req.object_ids = vector<string>({"expo_dry_erase_board_eraser"});
  //req.object_ids = vector<string>({"poland_spring_water"});
  //req.object_ids = vector<string>({"duct_tape"});
  tf::matrixEigenToMsg(camera_pose.matrix(), req.camera_pose);
  pcl::toROSMsg(*cloud_in, req.input_organized_cloud);

  if (client.call(srv)) {
    ROS_INFO("Episode Statistics\n");

    for (size_t ii = 0; ii < srv.response.stats_field_names.size(); ++ii) {
      ROS_INFO("%s: %f", srv.response.stats_field_names[ii].c_str(), srv.response.stats[ii]);
    }

    ROS_INFO("Model to scene object transforms:");

    for (size_t ii = 0; ii < req.object_ids.size(); ++ii) {

      Eigen::Matrix4d pose(srv.response.object_transforms[ii].data.data());
      Eigen::Affine3d object_transform;
      // Transpose to convert column-major raw data initialization to row-major.
      object_transform.matrix() = pose.transpose();

      ROS_INFO_STREAM("Object: " << req.object_ids[ii] << std::endl << object_transform.matrix() << std::endl << std::endl);
    }
  } else {
    ROS_ERROR("Failed to call the object localizer service");
    return 1;
  }

  return 0;
}

