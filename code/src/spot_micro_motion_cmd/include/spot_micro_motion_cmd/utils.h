#pragma once

#include <string>

#include <eigen3/Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

// Utility functions for Spot Micro motion command node.

// Convert an Eigen Matrix4f to an Eigen Affine3d.
Eigen::Affine3d matrix4fToAffine3d(
  const Eigen::Matrix4f& in);

// Create a ROS 2 TransformStamped from an Eigen Affine3d.
// The timestamp is passed in by the caller, usually this->now().
geometry_msgs::msg::TransformStamped eigAndFramesToTrans(
  const Eigen::Affine3d& transform,
  const std::string& parent_frame_id,
  const std::string& child_frame_id,
  const rclcpp::Time& stamp);

// Create a ROS 2 TransformStamped from translation, rotation,
// parent frame, child frame, and timestamp.
geometry_msgs::msg::TransformStamped createTransform(
  const std::string& parent_frame_id,
  const std::string& child_frame_id,
  double x,
  double y,
  double z,
  double roll,
  double pitch,
  double yaw,
  const rclcpp::Time& stamp);