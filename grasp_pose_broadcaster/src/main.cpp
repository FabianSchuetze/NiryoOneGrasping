#include "grasp_pose_broadcaster.hpp"
//#include <filesystem>
//#include <geometry_msgs/PoseArray.h>
//#include <geometry_msgs/TransformStamped.h>
//#include <object_pose/positions.h>
//#include <open3d/Open3D.h>
//#include <ros/ros.h>
//#include <string>
//#include <tf2/LinearMath/Quaternion.h>
//#include <tf2_eigen/tf2_eigen.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include <tf2_ros/static_transform_broadcaster.h>
#include <utils/utils.hpp>

using param = std::pair<std::string, std::string>;
constexpr std::size_t QUEUE(10);
//constexpr double BEND_ARM(1.5);
//constexpr double HAND_LENGTH(0.08);

//utils::DOF
//generateGraspPose(const geometry_msgs::TransformStamped &ros_transform,
                  //const std::string &name, double yaw) {
    //open3d::geometry::TriangleMesh mesh;
    //if (!open3d::io::ReadTriangleMesh(name, mesh)) {
        //ROS_ERROR_STREAM("Could not read mesh file " << name);
        //throw std::runtime_error("Couldd not read mesh file");
    //}
    //Eigen::Isometry3d transform = tf2::transformToEigen(ros_transform);
    //ROS_WARN_STREAM("The incoming transform is:\n " << transform.matrix());
    //auto [r, y, p] = utils::RPY(transform);
    //ROS_WARN_STREAM("The r,p, y: " << r << ", " << p << ", " << y);
    //mesh.Transform(transform.matrix());
    //const Eigen::Vector3d center = mesh.GetCenter();
    //const Eigen::Vector3d bound = mesh.GetMaxBound();
    //utils::DOF dof(center(0), center(1), bound(2) + HAND_LENGTH, 0, BEND_ARM,
                   //yaw);
    //return dof;
//}

//std::string shortName(const std::string &input_name,
                      //const std::string& extension) { // NOLINT
    //const std::string fn = std::filesystem::path(input_name).filename();
    //std::string delimiter = "_";
    //std::string token = fn.substr(0, fn.find(delimiter));
    //return token + extension;
//}

//class SubscribeAndPublish {
  //public:
    //SubscribeAndPublish(ros::NodeHandle &n_, const std::string &publication) {
        //// ROS_WARN_STREAM("The publication is" << publication);
        //pub_ = n_.advertise<geometry_msgs::PoseArray>(publication, 1, true);
    //}
    //void callback(const object_pose::positions &msg) {
        //const auto now = ros::Time::now();
        //std::vector<geometry_msgs::TransformStamped> transforms;
        //geometry_msgs::PoseArray poses;
        //poses.header.frame_id = msg.poses.header.frame_id;
        //poses.header.stamp = now;
        //for (std::size_t i = 0; i < msg.objects.size(); ++i) {
            //geometry_msgs::Pose pose = msg.poses.poses[i];
            //const std::string &name = msg.objects[i];
            //auto [roll, pitch, yaw] = utils::RPY(pose.orientation);
            //utils::DOF dof(pose.position.x, pose.position.y, 0, 0, 0, yaw);
            //auto transformStamped = dof.transformStamped();
            //// auto grasp_pose = generateGraspPose(transformStamped, name, yaw);
            //utils::DOF dof_out = generateGraspPose(transformStamped, name, yaw);
            //geometry_msgs::TransformStamped tf_out = dof_out.transformStamped();
            //geometry_msgs::Pose grasp_pose = dof_out.pose();
            //transformStamped.header.stamp = now;
            //transformStamped.header.frame_id = msg.poses.header.frame_id;
            //tf_out.header.frame_id = msg.poses.header.frame_id;
            //std::string sho = shortName(name, "_in");
            //std::string sho_out = shortName(name, "_grasp");
            //transformStamped.child_frame_id = sho;
            //tf_out.child_frame_id = sho_out;
            //ROS_WARN_STREAM("Grasp pose for "
                            //<< transformStamped.child_frame_id
                            //<< " is\n: x, y, z, yaw: " << grasp_pose.position.x
                            //<< ", " << grasp_pose.position.y << ", "
                            //<< grasp_pose.position.z << ", " << yaw);
            //// ROS_WARN_STREAM(
            ////"The child_frame_id is: " << transformStamped.child_frame_id);
            //poses.poses.push_back(grasp_pose);
            //transforms.push_back(transformStamped);
            //transforms.push_back(tf_out);
        //}
        //pub_.publish(poses);
        //br.sendTransform(transforms);
    //}

  //private:
    //ros::Publisher pub_;
    //tf2_ros::StaticTransformBroadcaster br;

//}; // End of class SubscribeAndPublisL

int main(int argc, char **argv) {
    ros::init(argc, argv, "grasp_broadcaster");
    ros::NodeHandle nh;
    param incoming_poses("grasp_pose_broadcaster/estimated_poses", "");
    param outgoing_poses("grasp_pose_broadcaster/outgoing_poses", "");
    //param type("grasp_pose_broadcaster/outgoing_poses", "");
    utils::readParameters(nh, incoming_poses, outgoing_poses);
    GraspPoseBroadcaster interaction(nh, outgoing_poses.second, incoming_poses.second);
    ros::Subscriber sub =
        nh.subscribe(incoming_poses.second, QUEUE,
                     &GraspPoseBroadcaster::callback, &interaction);
    ros::spin();
    sub.shutdown();
    return 0;
};
