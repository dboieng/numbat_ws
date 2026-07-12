#include "utils.h"

#include <string>

#include <eigen3/Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace Eigen;

Eigen::Affine3d matrix4fToAffine3d(const Eigen::Matrix4f& in)
{
  return Eigen::Affine3d(in.cast<double>());
}

geometry_msgs::msg::TransformStamped eigAndFramesToTrans(
  const Eigen::Affine3d& transform,
  const std::string& parent_frame_id,
  const std::string& child_frame_id,
  const rclcpp::Time& stamp)
{
  geometry_msgs::msg::TransformStamped transform_stamped =
    tf2::eigenToTransform(transform);

  transform_stamped.header.stamp = stamp;
  transform_stamped.header.frame_id = parent_frame_id;
  transform_stamped.child_frame_id = child_frame_id;

  return transform_stamped;
}

geometry_msgs::msg::TransformStamped createTransform(
  const std::string& parent_frame_id,
  const std::string& child_frame_id,
  double x,
  double y,
  double z,
  double roll,
  double pitch,
  double yaw,
  const rclcpp::Time& stamp)
{
  geometry_msgs::msg::TransformStamped tr_stamped;

  tr_stamped.header.stamp = stamp;
  tr_stamped.header.frame_id = parent_frame_id;
  tr_stamped.child_frame_id = child_frame_id;

  tr_stamped.transform.translation.x = x;
  tr_stamped.transform.translation.y = y;
  tr_stamped.transform.translation.z = z;

  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  q.normalize();

  tr_stamped.transform.rotation.x = q.x();
  tr_stamped.transform.rotation.y = q.y();
  tr_stamped.transform.rotation.z = q.z();
  tr_stamped.transform.rotation.w = q.w();

  return tr_stamped;
}