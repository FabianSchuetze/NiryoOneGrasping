#ifndef GPDInteraction_hpp
#define GPDInteraction_hpp
#include <Eigen/Geometry>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <gpd_ros/GraspConfigList.h>
#include <object_pose/positions.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <vector>

namespace gpd {
struct Hand {
    double x, y, z, pitch, yaw;
};
geometry_msgs::TransformStamped rosTransform(const Eigen::Isometry3d &,
                                             const std::string &, std::string);
std::string shortName(const std::string &, std::string);
geometry_msgs::Pose generateGraspPose(const Hand &);
class GPDInteraction {
  public:
    GPDInteraction(ros::NodeHandle &, const std::string &);
    void callback_grasp_pose(const gpd_ros::GraspConfigList &);
    void callback_object_pose(const object_pose::positions &);

  private:
    ros::Publisher pub_;
    ros::Publisher publish_pointcloud_;
    std::vector<Eigen::Isometry3d> possible_transforms;
    tf2_ros::StaticTransformBroadcaster br;
    bool grasp_pose_received;
    static Eigen::Isometry3d generateTransformation(const Hand &);
    static Eigen::Isometry3d
    convertHandToTransform(const gpd_ros::GraspConfig &);
    void publishPointCloud(const std::string &name);
    int filterPossibleTransforms(const Eigen::Isometry3d &);
    Eigen::Isometry3d generateHand(const Eigen::Isometry3d &, int);
    static double correctPitch(double);
};
} // namespace gpd
#endif
