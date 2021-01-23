#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <ros/ros.h>

#include <Eigen/Geometry>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
//#include <moveit_msgs/DisplayTrajectory.h>

//#include <moveit_msgs/AttachedCollisionObject.h>
//#include <moveit_msgs/CollisionObject.h>

//#include <moveit_visual_tools/moveit_visual_tools.h>

cv::FileStorage read_file(std::string& path) {
    cv::FileStorage fs;
    fs.open(path, cv::FileStorage::READ);
    return fs;
}

Eigen::Affine3d read_transformation(const cv::FileStorage& fs,
                                    const std::string& pos) {
    cv::Mat transformation;
    fs[pos] >> transformation;
    Eigen::Matrix4d tmp;
    cv::cv2eigen(transformation, tmp);
    Eigen::Affine3d trans;
    trans.matrix() = tmp;
    return trans;
}

geometry_msgs::Pose convert_to_ros(const Eigen::Affine3d& transform) {
    geometry_msgs::Pose target_pose;
    Eigen::Quaterniond quat(transform.rotation());
    target_pose.position.x = transform.translation().x();
    target_pose.position.y = transform.translation().y();
    target_pose.position.z = transform.translation().z();
    target_pose.orientation.w = quat.w();
    target_pose.orientation.x = quat.x();
    target_pose.orientation.y = quat.y();
    target_pose.orientation.z = quat.z();
    return target_pose;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "generate_images");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    std::string file;
    if ((node_handle.getParam("/DisplayImage/transformations_file", file))) {
        std::cout << "the file locatio is: " << file.c_str() << std::endl;
        ROS_DEBUG("the file locatio is: %s", file.c_str());
    } else {
        ROS_DEBUG("that did not work: %s", file.c_str());
        ros::shutdown();
        return 1;
    }
    spinner.start();
    cv::FileStorage nodes = read_file(file);
    if (!nodes.isOpened()) {
        std::cerr << "Failed to open " << file << std::endl;
    }
    int n_iter = (int)nodes["frameCount"];
    static const std::string PLANNING_GROUP = "panda_arm";

    // The :move_group_interface:`MoveGroupInterface` class can be easily
    // setup using just the name of the planning group you would like to control
    // and plan for.
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    // We will use the :planning_scene_interface:`PlanningSceneInterface`
    // class to add and remove collision objects in our "virtual world" scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Raw pointers are frequently used to refer to the planning group for
    // improved performance.
    const robot_state::JointModelGroup* joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // Visualization
    // ^^^^^^^^^^^^^ No visualization so far
    //
    //
    // We can print the name of the reference frame for this robot.
    ROS_INFO_NAMED("tutorial", "Planning frame: %s",
                   move_group.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    ROS_INFO_NAMED("tutorial", "End effector link: %s",
                   move_group.getEndEffectorLink().c_str());

    // We can get a list of all the groups in the robot: not available in
    // kinect
    // ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    // std::copy(move_group.getJointModelGroupNames().begin(),
    // move_group.getJointModelGroupNames().end(),
    // std::ostream_iterator<std::string>(std::cout, ", "));

    // std::vector<geometry_msgs::Pose> target_poses(
    //{target_pose1, target_pose2, target_pose3});
    // for (const auto& pose : target_poses) {
    //}

    // ros::shutdown();
    // return 0;
    for (int i = 0; i < n_iter; ++i) {
        std::string pos("T1_" + std::to_string(i));
        Eigen::Affine3d trans = read_transformation(nodes, pos);
        geometry_msgs::Pose pose = convert_to_ros(trans);
        move_group.setPoseTarget(pose);
        move_group.move();
    }
    ros::shutdown();
    return 0;
}
