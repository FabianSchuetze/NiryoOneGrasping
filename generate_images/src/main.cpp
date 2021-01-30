#include <eigen_conversions/eigen_msg.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

#include <Eigen/Geometry>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>

//cv::FileStorage read_file(std::string& path) {
    //cv::FileStorage fs;
    //fs.open(path, cv::FileStorage::READ);
    //return fs;
//}

//Eigen::Affine3d read_transformation(const cv::FileStorage& fs,
                                             //const std::string& pos) {
    //cv::Mat transformation;
    //fs[pos] >> transformation;
    //Eigen::Matrix4d tmp;
    //cv::cv2eigen(transformation, tmp);
    //Eigen::Affine3d trans(tmp);
    //return trans;
//}

geometry_msgs::Pose convert_to_ros(const Eigen::Affine3d& transform) {
    geometry_msgs::Pose target_pose;
    Eigen::Quaterniond quat(transform.rotation());
    float x = target_pose.position.x = transform.translation().x();
    float y = target_pose.position.y = transform.translation().y();
    float z = target_pose.position.z = transform.translation().z();
    std::cout << "Planner: The  position is: (x, y, z): " << x << ", " << y
              << ", " << z << std::endl;
    float w = target_pose.orientation.w = quat.w();
    x = target_pose.orientation.x = quat.x();
    y = target_pose.orientation.y = quat.y();
    z = target_pose.orientation.z = quat.z();
    std::cout << "planner: quternion : (w, x, y, z): " << w << ", " << x << ", "
              << y << ", " << z << std::endl;
    return target_pose;
}

void print_output(const geometry_msgs::TransformStamped& transformStamped) {
    geometry_msgs::Pose pose;
    geometry_msgs::Transform transform(transformStamped.transform);
    float x = transform.translation.x;
    float y = transform.translation.y;
    float z = transform.translation.z;
    std::cout << "Listener: The  position is: (x, y, z): " << x << ", " << y
              << ", " << z << std::endl;
    float w = transform.rotation.w;
    x = transform.rotation.x;
    y = transform.rotation.y;
    z = transform.rotation.z;
    std::cout << "Listener: quternion : (w, x, y, z): " << w << ", " << x
              << ", " << y << ", " << z << std::endl;
    std::cout << "\n";
}

bool get_transform(const tf2_ros::Buffer& buffer,
                   geometry_msgs::TransformStamped& trans) {
    bool success(false);
    try {
        trans = buffer.lookupTransform("hand_link", "tool_link", ros::Time(0));
	success = true;
    } catch (tf2::TransformException& ex) {
        ROS_ERROR("%s", ex.what());
    }
    return success;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "generate_images");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    std::string file;
    if ((node_handle.getParam("/DisplayImage/transformations_file", file))) {
        std::cout << "the file location is: " << file.c_str() << std::endl;
        ROS_DEBUG("the file location is: %s", file.c_str());
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
    const int n_iter = (int)nodes["frameCount"];
    const std::string PLANNING_GROUP = "arm";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    ROS_INFO_NAMED("tutorial", "Planning frame: %s",
                   move_group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("tutorial", "End effector link: %s",
                   move_group.getEndEffectorLink().c_str());
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener listener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;
    for (int i = 0; i < n_iter; ++i) {
        bool obtained_transform = get_transform(tfBuffer, transformStamped);
	if (!obtained_transform) {
	    continue;
	}
        std::string pos("T1_" + std::to_string(i));
        Eigen::Affine3d T_base_finger = read_transformation(nodes, pos);
        Eigen::Affine3d T_finger_hand = tf2::transformToEigen(transformStamped);
        Eigen::Affine3d T_base_hand = T_base_finger * T_finger_hand;
        geometry_msgs::Pose pose = convert_to_ros(T_base_hand);
        move_group.setPoseTarget(pose);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group.plan(my_plan) ==
                        moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success) {
            move_group.move();
        }
        geometry_msgs::TransformStamped location;
        try {
            location =
                tfBuffer.lookupTransform("world", "tool_link", ros::Time(0));
        } catch (tf2::TransformException& ex) {
            ROS_ERROR("%s", ex.what());
        }

        print_output(location);
    }
    ros::shutdown();
    return 0;
}
