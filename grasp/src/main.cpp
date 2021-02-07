#include <gpd_ros/GraspConfigList.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <ros/ros.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

geometry_msgs::Pose convert_message(const gpd_ros::GraspConfigList& msg) {
    gpd_ros::GraspConfig first = msg.grasps[0];
    geometry_msgs::Pose pose;
    pose.position = first.position;
    std::cout << "received the message" << std::endl;
    return pose;
}
void move_arm(const gpd_ros::GraspConfigList& msg) {
    std::cout << "inside the move arm function";
    static const std::string PLANNING_GROUP = "arm";
    static moveit::planning_interface::MoveGroupInterface move_group(
        PLANNING_GROUP);
    geometry_msgs::Pose pose = convert_message(msg);
    move_group.setPoseTarget(pose);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success) {
        move_group.move();
    } else {
        std::cout << "not working" << std::endl;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "grasp");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    std::cout << "inside" << std::endl;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener listener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;
    ros::Subscriber sub =
        node_handle.subscribe("detect_grasps/clustered_grasps", 1000, move_arm);
    ros::spin();
    return 0;
}
