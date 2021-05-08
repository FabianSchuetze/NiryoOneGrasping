#include "picking.hpp"
#include <Eigen/Geometry>
#include <chrono>
#include <eigen_conversions/eigen_msg.h>
#include <niryo_one_msgs/RobotMoveAction.h>
#include <niryo_one_msgs/SetInt.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <thread>
#include <utils/utils.hpp>

static constexpr int MAX_ATTPEMTS(10);
static constexpr float MAX_DURATION(10.0);
static constexpr int TOOL_ID(13);
static constexpr int MAX_SPEED(1000);
static constexpr int CMD_TYPE(6);
static constexpr float HALF_ANGLE(1.5);
static constexpr double SAFTY_X(0.13);
static constexpr double SAFTY_Z(0.13);
static constexpr double MAX_ROLL(1e-4);

Picking::Picking() : robot("/niryo_one/commander/robot_action/", true) { ; };

void Picking::connectToRobot(ros::NodeHandle &node) {
    establish_connection(robot, "robot");
    setGripper(node);
}

// Picking::EEFPosition Picking::Position(double x, double y, double z,
// double pitch, bool open_gripper) {
// NiryoPose _pose;
//_pose.first.x = x;
//_pose.first.y = y;
//_pose.first.z = z;
// EEFPosition pose1 = pose(_pose, open_gripper);
// pose1.pose.second.pitch = pitch;
// return pose1;
//}

Picking::EEFPosition Picking::Position(double x, double y, double z,
                                       double roll, double pitch, double yaw,
                                       bool open_gripper) {
    NiryoPose _pose;
    _pose.first.x = x;
    _pose.first.y = y;
    _pose.first.z = z;
    EEFPosition pose1 = pose(_pose, open_gripper);
    pose1.pose.second.roll = roll;
    pose1.pose.second.pitch = pitch;
    pose1.pose.second.yaw = yaw;
    return pose1;
}

void Picking::sendTransform(const Eigen::Isometry3d &frame,
                            std::string frame_id) {
    auto ros_frame = tf2::eigenToTransform(frame);
    ros_frame.header.frame_id = "base_link";
    ros_frame.child_frame_id = std::move(frame_id);
    br.sendTransform(ros_frame);
}

Picking::EEFPosition
Picking::PreGrasp_orientate(const Eigen::Isometry3d &incoming_pose) {
    Eigen::Isometry3d move(Eigen::Matrix4d::Identity(4, 4));
    constexpr double APPROACH_X(-0.24), APPROACH_Z(0.2);
    move.matrix()(0, 3) = APPROACH_X;
    move.matrix()(2, 3) = APPROACH_Z;
    Eigen::Isometry3d approach = incoming_pose * move;
    NiryoPose p = convert(approach, "PreGrasp_orientate");
    sendTransform(approach, "PreGrasp_orientate");
    // p.first.z = 0.25; // SAFTY FIRST
    EEFPosition pose1 = pose(p, true);
    return pose1;
}

Picking::EEFPosition
Picking::PreGrasp_descend(const Eigen::Isometry3d &incoming_pose) {
    Eigen::Isometry3d move(Eigen::Matrix4d::Identity(4, 4));
    constexpr double APPROACH_X(-0.12), APPROACH_Z(0.04);
    move.matrix()(0, 3) = APPROACH_X;
    move.matrix()(2, 3) = APPROACH_Z;
    Eigen::Isometry3d approach = incoming_pose * move;
    NiryoPose p = convert(approach, "PreGrasp_descend");
    sendTransform(approach, "PreGrasp_descend");
    // TODO: Check if works without safty checks
    // p.first.z = 0.10; // SAFTY FIRST
    EEFPosition pose1 = pose(p, true);
    return pose1;
}

Picking::EEFPosition
Picking::computeGrasp(const Eigen::Isometry3d &incoming_pose) {
    Eigen::Isometry3d move(Eigen::Matrix4d::Identity(4, 4));
    constexpr double APPROACH_X(-0.07);
    move.matrix()(0, 3) = APPROACH_X;
    Eigen::Isometry3d approach = incoming_pose * move;
    NiryoPose p = convert(approach, "Grasp");
    sendTransform(approach, "Grasp");
    // p.first.z = 0.10; // SAFTY FIRST
    EEFPosition pose1 = pose(p, true);
    return pose1;
}

Picking::EEFPosition Picking::Close(const Eigen::Isometry3d &incoming_pose) {
    Eigen::Isometry3d move(Eigen::Matrix4d::Identity(4, 4));
    constexpr double APPROACH_X(-0.07);
    move.matrix()(0, 3) = APPROACH_X;
    Eigen::Isometry3d approach = incoming_pose * move;
    NiryoPose p = convert(approach, "Close");
    sendTransform(approach, "Close");
    // p.first.z = 0.10; // SAFTY FIRST
    EEFPosition pose1 = pose(p, false);
    return pose1;
}

Picking::EEFPosition
Picking::PostGrasp(const Eigen::Isometry3d &incoming_pose) {
    Eigen::Isometry3d postGrasp = incoming_pose;
    constexpr double APPROACH_Z(0.25);
    postGrasp.matrix()(2, 3) = APPROACH_Z;
    NiryoPose p = convert(postGrasp, "PostGrasp");
    // p.first.z = 0.25;
    sendTransform(postGrasp, "PostGrasp");
    EEFPosition pose1 = pose(p, false);
    return pose1;
}

Picking::EEFPosition Picking::pose(const NiryoPose &p, bool open) {
    EEFPosition eef;
    eef.pose = p;
    eef.open = open;
    return eef;
}

void Picking::setGripper(ros::NodeHandle &node) {
    ROS_INFO("Setting gripper");
    ros::ServiceClient changeToolClient_;
    changeToolClient_ =
        node.serviceClient<niryo_one_msgs::SetInt>("/niryo_one/change_tool/");
    niryo_one_msgs::SetInt srv;
    srv.request.value = TOOL_ID;
    while (!changeToolClient_.call(srv)) {
        ROS_WARN("Could not set the tool type. Trying again in one second");
        ros::Duration(1.0).sleep();
    }
    ROS_INFO("Success");
}

void Picking::establish_connection(const NiryoClient &ac,
                                   const std::string &what) {
    ROS_INFO_STREAM("Connecting to " << what);
    size_t attempts(0);
    while (!ac.waitForServer(ros::Duration(MAX_DURATION))) {
        ROS_WARN_STREAM("  Error connecting to " << what << ". Trying again");
        ++attempts;
        if (attempts > MAX_ATTPEMTS) {
            throw std::runtime_error("Could not achieve connection");
        }
    }
    ROS_INFO_STREAM("Connection to " << what << " established");
}

bool Picking::GripperAperture(bool open) {
    niryo_one_msgs::ToolCommand tcmd;
    if (open) {
        tcmd.cmd_type = 1;
    } else {
        tcmd.cmd_type = 2;
    }
    tcmd.gripper_open_speed = MAX_SPEED;
    tcmd.tool_id = TOOL_ID;
    niryo_one_msgs::RobotMoveActionGoal action;
    action.goal.cmd.cmd_type = CMD_TYPE;
    action.goal.cmd.tool_cmd = tcmd;
    robot.sendGoal(action.goal);
    bool success = robot.waitForResult(ros::Duration(MAX_DURATION));
    return success;
}

bool Picking::MoveEEF(const NiryoPose &pose) {
    niryo_one_msgs::RobotMoveCommand cmd;
    cmd.cmd_type = 2;
    cmd.position = pose.first;
    cmd.rpy = pose.second;
    niryo_one_msgs::RobotMoveActionGoal action;
    action.goal.cmd = cmd;
    ROS_INFO("Sending command:");
    if (cmd.position.z < SAFTY_Z) {
        ROS_ERROR_STREAM("Received z value "
                         << cmd.position.z << ", "
                         << "smaller than safty margins. Abort!");
        throw std::runtime_error("");
    }
    ROS_INFO("position: %.2f, %.2f, %.2f", cmd.position.x, cmd.position.y,
             cmd.position.z);
    ROS_INFO("rpy (r,p,y):  %.2f, %.2f, %2f", cmd.rpy.roll, cmd.rpy.pitch,
             cmd.rpy.yaw);
    robot.sendGoal(action.goal);
    bool success = robot.waitForResult(ros::Duration(MAX_DURATION));
    return success;
}

void Picking::moveToPosition(const EEFPosition &eef) {
    const bool movement = Picking::MoveEEF(eef.pose);
    if (!movement) {
        ROS_WARN("Could not move the arm");
    }
    const bool aperture = Picking::GripperAperture(eef.open);
    if (!aperture) {
        ROS_WARN("Could not open the gripper");
    }
}

void Picking::moveArm(const geometry_msgs::Pose &pose) {
    Eigen::Isometry3d grasp_pose(Eigen::Matrix4d::Identity(4, 4));
    tf::poseMsgToEigen(pose, grasp_pose);
    ROS_WARN_STREAM("The grap frame is:\n" << grasp_pose.matrix());
    std::chrono::seconds sec(1);
    std::vector<Picking::EEFPosition> movements;
    auto [roll, pitch, yaw] = utils::RPY(grasp_pose);
    if (std::abs(pitch) < 1.0) {
        movements.push_back(Position(0.15, 0, 0.3, 0, 0, 0, true));  // NOLINT
        movements.push_back(Position(0.15, 0, 0.15, 0, 0, 0, true)); // NOLINT
    }
    movements.push_back(PreGrasp_orientate(grasp_pose));
    movements.push_back(PreGrasp_descend(grasp_pose));
    movements.push_back(computeGrasp(grasp_pose));
    movements.push_back(Close(grasp_pose));
    movements.push_back(PostGrasp(grasp_pose));
    movements.push_back(
        Position(0.1, -0.2, 0.25, 0, HALF_ANGLE, 0, false)); // NOLINT
    movements.push_back(
        Position(0.1, -0.2, 0.25, 0, HALF_ANGLE, 0, true)); // NOLINT
    movements.push_back(
        Position(0.3, 0, 0.35, 0, HALF_ANGLE, 0, false)); // NOLINT
    for (const auto &movement : movements) {
        moveToPosition(movement);
        std::this_thread::sleep_for(sec);
    }
    std::this_thread::sleep_for(sec);
}

Picking::NiryoPose Picking::convert(const Eigen::Isometry3d &frame,
                                    const std::string &name) {
    auto [roll, pitch, yaw] = utils::RPY(frame);
    NiryoPose pose;
    pose.first.x = frame(0, 3);
    pose.first.y = frame(1, 3);
    pose.first.z = frame(2, 3);
    pose.second.roll = roll;
    pose.second.pitch = pitch;
    pose.second.yaw = yaw;
    if (pose.first.x < SAFTY_X) {
        ROS_ERROR_STREAM("Recived x value of " << pose.first.x
                                               << "set to 0.13 for safty");
        pose.first.x = SAFTY_X;
    }
    if (pose.first.z < SAFTY_Z) {
        ROS_ERROR_STREAM("Recived x value of " << pose.first.z
                                               << "set to 0.1 for safty");
        pose.first.z = SAFTY_Z;
    }
    ROS_WARN_STREAM("Compute " << name << " pose:\n"
                               << pose.first.x << ", " << pose.first.y << ", "
                               << pose.first.z << ", " << pose.second.roll
                               << ", " << pose.second.pitch << ", "
                               << pose.second.yaw);
    return pose;
}

std::vector<geometry_msgs::Pose>
Picking::check(const geometry_msgs::PoseArray &poses) {
    std::vector<geometry_msgs::Pose> grasp_poses;
    for (const auto &pose : poses.poses) {
        auto [roll, pitch, yaw] = utils::RPY(pose.orientation);
        if (std::abs(roll) > MAX_ROLL) {
            ROS_ERROR_STREAM("Cannot accept roll values, was: " << roll);
            throw std::runtime_error("Cannot accept roll parameters");
        }
        grasp_poses.push_back(pose);
        ROS_WARN_STREAM("The graping pose is x,y,z"
                        << pose.position.x << ", " << pose.position.y << ", "
                        << pose.position.z << ", "
                        << "and roll, pitch, yaw" << roll << ", " << pitch
                        << ", " << yaw);
    }
    return grasp_poses;
}

void Picking::moveJoints(const std::vector<double> &position) {
    niryo_one_msgs::RobotMoveCommand cmd;
    cmd.cmd_type = 1;
    cmd.joints = position;
    niryo_one_msgs::RobotMoveActionGoal action;
    action.goal.cmd = cmd;
    robot.sendGoal(action.goal);
    if (!robot.waitForResult(ros::Duration(MAX_DURATION))) {
        ROS_WARN("Joints cloud not be moved");
    }
}

void Picking::callback(const geometry_msgs::PoseArray &poses) {
    ROS_WARN_STREAM("Recived sequence: " << poses.header.seq);
    std::vector<geometry_msgs::Pose> niryo_poses = check(poses);
    for (const auto &pose : niryo_poses) {
        moveArm(pose);
    }
}
