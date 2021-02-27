#ifndef ros_interaction_hpp
#define ros_interaction_hpp
#include <Eigen/Geometry>
#include <string>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

bool obtain_transform(const std::string &, const std::string &,
                      const tf2_ros::Buffer &, Eigen::Affine3d &);
void broadcast(const Eigen::Affine3d &transform, const std::string &,
               const std::string);
#endif
