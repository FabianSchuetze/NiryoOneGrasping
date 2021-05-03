#include <utils/utils.hpp>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <object_pose/positions.h>
#include <open3d/Open3D.h>
#include <ros/ros.h>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <filesystem>

using param = std::pair<std::string, std::string>;
constexpr std::size_t QUEUE(10);
constexpr double BEND_ARM(1.5);
constexpr double HAND_LENGTH(0.08);

utils::DOF 
centroidGraspPose(const geometry_msgs::TransformStamped &ros_transform,
                  const std::string &name, double yaw) {
    Eigen::Isometry3d transform = tf2::transformToEigen(ros_transform);
    ROS_WARN_STREAM("The incoming transform is:\n " << transform.matrix());
    auto [r, p, yw] = utils::RPY(transform);
    ROS_WARN_STREAM("The r,p, y: " << r << ", " << p << ", " << yw);
    double x = transform(0,3);
    double y = transform(1,3);
    double z = transform(2,3) + HAND_LENGTH;
    utils::DOF dof(x, y, z, 0, BEND_ARM, yaw);
    return dof;
}

std::string shortName(const std::string &input_name,
                      std::string extension) { // NOLINT
    const std::string fn = std::filesystem::path(input_name).filename();
    std::string delimiter = "_";
    std::string token = fn.substr(0, fn.find(delimiter));
    ROS_WARN_STREAM("Incoming: " << input_name << ", " << "token: " << token);
    return token + extension;
}

class SubscribeAndPublish {
  public:
    SubscribeAndPublish(ros::NodeHandle &n_, const std::string &publication) {
        // ROS_WARN_STREAM("The publication is" << publication);
        pub_ = n_.advertise<geometry_msgs::PoseArray>(publication, 1, true);
    }
    //TODO Differnt type
    void callback(const object_pose::positions &msg) {
        const auto now = ros::Time::now();
        std::vector<geometry_msgs::TransformStamped> transforms;
        geometry_msgs::PoseArray poses;
        poses.header.frame_id = msg.poses.header.frame_id;
        poses.header.stamp = now;
        for (std::size_t i = 0; i < msg.objects.size(); ++i) {
            geometry_msgs::Pose pose = msg.poses.poses[i];
            const std::string &name = msg.objects[i];
            auto [roll, pitch, yaw] = utils::RPY(pose.orientation);
            //TODO: The positive z axis also differs fro the other program!
            utils::DOF dof(pose.position.x, pose.position.y, pose.position.z, 0, 0, yaw);
            auto transformStamped = dof.transformStamped();
            //TODO: Only diff to other grasp pose is this func -> HARMOIZE!!!
            utils::DOF dof_out = centroidGraspPose(transformStamped, name, yaw);
            geometry_msgs::TransformStamped tf_out = dof_out.transformStamped();
            geometry_msgs::Pose grasp_pose = dof_out.pose();
            transformStamped.header.stamp = now;
            transformStamped.header.frame_id = msg.poses.header.frame_id;
            tf_out.header.frame_id = msg.poses.header.frame_id;
            std::string sho = shortName(name, "_in");
            std::string sho_out = shortName(name, "_grasp");
            transformStamped.child_frame_id = sho;
            tf_out.child_frame_id = sho_out;
            ROS_WARN_STREAM("Grasp pose for "
                            << transformStamped.child_frame_id
                            << " is\n: x, y, z, yaw: " << grasp_pose.position.x
                            << ", " << grasp_pose.position.y << ", "
                            << grasp_pose.position.z << ", " << yaw);
            // ROS_WARN_STREAM(
            //"The child_frame_id is: " << transformStamped.child_frame_id);
            poses.poses.push_back(grasp_pose);
            transforms.push_back(transformStamped);
            transforms.push_back(tf_out);
        }
        pub_.publish(poses);
        br.sendTransform(transforms);
    }

  private:
    ros::Publisher pub_;
    tf2_ros::StaticTransformBroadcaster br;

}; // End of class SubscribeAndPublisL

int main(int argc, char **argv) {
    ros::init(argc, argv, "centroid_broadcaster");
    ros::NodeHandle nh;
    // TODO: Provide new launch file
    param incoming_poses("grasp_pose_broadcaster/centroids", "");
    param outgoing_poses("grasp_pose_broadcaster/outgoing_poses", "");
    utils::readParameters(nh, incoming_poses, outgoing_poses);
    SubscribeAndPublish interaction(nh, outgoing_poses.second);
    ros::Subscriber sub =
        nh.subscribe(incoming_poses.second, QUEUE,
                     &SubscribeAndPublish::callback, &interaction);
    ros::spin();
    return 0;
};
