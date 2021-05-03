#pragma once
#include <object_pose/positions.h>
#include <ros/ros.h>
#include <string>
#include <tf2_ros/static_transform_broadcaster.h>
#include <utils/utils.hpp>

namespace GraspingFunction {
utils::DOF
centroidGraspPose(const geometry_msgs::TransformStamped &ros_transform,
                  const std::string &name, double yaw);
utils::DOF
generateGraspPose(const geometry_msgs::TransformStamped &ros_transform,
                  const std::string &name, double yaw);
} // namespace GraspingFunction

class GraspPoseBroadcaster {
  public:
    GraspPoseBroadcaster(ros::NodeHandle &n_, const std::string &publication,
                         const std::string &type);
    void callback(const object_pose::positions &msg);

  private:
    static std::string shortName(const std::string &, const std::string &);
    ros::Publisher pub_;
    tf2_ros::StaticTransformBroadcaster br;
    utils::DOF (*grasping_func)(const geometry_msgs::TransformStamped &,
                                const std::string &, double);
};
