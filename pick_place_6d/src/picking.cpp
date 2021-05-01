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

Picking::Picking() : robot("/niryo_one/commander/robot_action/", true) { ; };

void Picking::connectToRobot(ros::NodeHandle &node) {
    establish_connection(robot, "robot"); // wait for the action server to start
    setGripper(node);
}

//// namespace Picking {
Picking::EndEffectorPosition Picking::FinalPositions(double x, double y,
                                                     double z,
                                                     bool open_gripper,
                                                     std::string) {
    NiryoPose _pose;
    _pose.first.x = x;
    _pose.first.y = y;
    _pose.first.z = z;
    EndEffectorPosition pose1 = pose(_pose, open_gripper);
    pose1.pose.second.pitch = HALF_ANGLE;
    return pose1;
}

void Picking::sendTransform(const Eigen::Isometry3d &frame,
                            std::string frame_id) {
    // geometry_msgs::Transform ros_frame;
    auto ros_frame = tf2::eigenToTransform(frame);
    // ros_frame.header.stamp = ros::time::now();
    ros_frame.header.frame_id = "base_link";
    ros_frame.child_frame_id = frame_id;
    br.sendTransform(ros_frame);
}
// Picking::EndEffectorPosition Picking::Rest() {
// NiryoPose _pose;
// geometry_msgs::Point p = point(0.3, 0, 0.35);
//_pose.first = p;
// EndEffectorPosition pose1 = pose(_pose, false);
// pose1.pose.second.pitch = HALF_ANGLE;
// return pose1;
//}
// Picking::EndEffectorPosition Picking::PreFinal() {
// NiryoPose _pose;
// geometry_msgs::Point p = point(0.1, -0.2, 0.25);
//_pose.first = p;
// EndEffectorPosition pose1 = pose(_pose, false);
// pose1.pose.second.pitch = HALF_ANGLE;
// return pose1;
//}
// Picking::EndEffectorPosition Picking::Final() {
// NiryoPose _pose;
// geometry_msgs::Point p = point(0.1, -0.2, 0.25);
//_pose.first = p;
// EndEffectorPosition pose1 = pose(_pose, true);
// pose1.pose.second.pitch = HALF_ANGLE;
// return pose1;
//}

Picking::EndEffectorPosition
Picking::PreGrasp_orientate(const Eigen::Isometry3d &incoming_pose) {
    Eigen::Isometry3d move(Eigen::Matrix4d::Identity(4, 4));
    move.matrix()(0, 3) = -0.14;
    move.matrix()(2, 3) = 0.2;
    Eigen::Isometry3d approach = incoming_pose * move;
    NiryoPose p = convertToNiryo(approach, "PreGrasp_orientate");
    sendTransform(approach, "PreGrasp_orientate");
    // p.first.x = approach.matrix()(0, 3);
    // p.first.y = approach.matrix()(1, 3);
    // p.first.z = approach.matrix()(2, 3);
    // if (p.first.x < 0.11) {
    // ROS_ERROR_STREAM("Recived x value of " << p.first.x
    //<< "set to 0.1 for safty");
    // p.first.x = 0.13;
    //}
    // ROS_WARN_STREAM("Orientate Pose: " << p.first.x << ", " << p.first.y
    // <<
    // ", "
    //<< p.first.z << ", " << p.second.roll
    //<< ", " << p.second.pitch << ", "
    //<< p.second.yaw);
    p.first.z = 0.25; // SAFTY FIRST
    EndEffectorPosition pose1 = pose(p, true);
    return pose1;
}

Picking::EndEffectorPosition
Picking::PreGrasp_descend(const Eigen::Isometry3d &incoming_pose) {
    Eigen::Isometry3d move(Eigen::Matrix4d::Identity(4, 4));
    move.matrix()(0, 3) = -0.12;
    move.matrix()(2, 3) = 0.04;
    Eigen::Isometry3d approach = incoming_pose * move;
    NiryoPose p = convertToNiryo(approach, "PreGrasp_descend");
    sendTransform(approach, "PreGrasp_descend");
    // ROS_WARN_STREAM("Pre Grasp Desecend: Incoming pose\n"
    //<< incoming_pose.matrix() << "\n move\n"
    //<< move.matrix());
    // p.first.x = approach.matrix()(0, 3);
    // if (p.first.x < 0.11) {
    // ROS_ERROR_STREAM("Recived x value of " << p.first.x
    //<< "set to 0.1 for safty");
    // p.first.x = 0.13;
    //}
    // p.first.y = approach.matrix()(1, 3);
    // p.first.z = approach.matrix()(2, 3);
    // p.first.z = approach.matrix()(2, 3);
    // if (p.first.z < 0.03) {
    // ROS_ERROR_STREAM("Recived z value of " << p.first.z
    //<< "set to 0.04 for safty");
    // p.first.z = 0.035;
    //}
    // ROS_WARN_STREAM("Descend Pose: " << p.first.x << ", " << p.first.y << ",
    // "
    //<< p.first.z << ", " << p.second.roll
    //<< ", " << p.second.pitch << ", "
    //<< p.second.yaw);
    p.first.z = 0.15; // SAFTY FIRST
    EndEffectorPosition pose1 = pose(p, true);
    return pose1;
}

Picking::EndEffectorPosition
Picking::computeGrasp(const Eigen::Isometry3d &incoming_pose) {
    Eigen::Isometry3d move(Eigen::Matrix4d::Identity(4, 4));
    move.matrix()(0, 3) = -0.07;
    // TODO: No final offset possible at the end, part of the grasp suggestion
    // move.matrix()(2, 3) = 0.03;
    Eigen::Isometry3d approach = incoming_pose * move;
    NiryoPose p = convertToNiryo(approach, "Grasp");
    sendTransform(approach, "Grasp");
    // p.first.x = approach.matrix()(0, 3);
    // if (p.first.x < 0.11) {
    // ROS_ERROR_STREAM("Recived x value of " << p.first.x
    //<< "set to 0.1 for safty");
    // p.first.x = 0.13;
    //}
    // p.first.y = approach.matrix()(1, 3);
    // p.first.z = approach.matrix()(2, 3);
    // if (p.first.z < 0.03) {
    // ROS_ERROR_STREAM("Recived z value of " << p.first.z
    //<< "set to 0.04 for safty");
    // p.first.z = 0.035;
    //}
    // ROS_WARN_STREAM("Compute Grasp Pose: "
    //<< p.first.x << ", " << p.first.y << ", " << p.first.z
    //<< ", " << p.second.roll << ", " << p.second.pitch << ", "
    //<< p.second.yaw);
    p.first.z = 0.15; // SAFTY FIRST
    EndEffectorPosition pose1 = pose(p, true);
    return pose1;
}
Picking::EndEffectorPosition
Picking::Close(const Eigen::Isometry3d &incoming_pose) {
    Eigen::Isometry3d move(Eigen::Matrix4d::Identity(4, 4));
    move.matrix()(0, 3) = -0.07;
    // TODO: No final offset possible at the end, part of the grasp suggestion
    // move.matrix()(2, 3) = 0.03;
    Eigen::Isometry3d approach = incoming_pose * move;
    NiryoPose p = convertToNiryo(approach, "Close");
    sendTransform(approach, "Close");
    // p.first.x = approach.matrix()(0, 3);
    // if (p.first.x < 0.11) {
    // ROS_ERROR_STREAM("Recived x value of " << p.first.x
    //<< "set to 0.1 for safty");
    // p.first.x = 0.11;
    //}
    // p.first.y = approach.matrix()(1, 3);
    // p.first.z = approach.matrix()(2, 3);
    ////}
    // ROS_WARN_STREAM("Close Pose: " << p.first.x << ", " << p.first.y << ", "
    //<< p.first.z << ", " << p.second.roll << ", "
    //<< p.second.pitch << ", " << p.second.yaw);
    p.first.z = 0.15; // SAFTY FIRST
    EndEffectorPosition pose1 = pose(p, false);
    return pose1;
}

Picking::EndEffectorPosition
Picking::PostGrasp(const Eigen::Isometry3d &incoming_pose) {
    Eigen::Isometry3d postGrasp = incoming_pose;
    postGrasp.matrix()(2, 3) = 0.25;
    NiryoPose p = convertToNiryo(postGrasp, "PostGrasp");
    p.first.z = 0.25;
    sendTransform(postGrasp, "PostGrasp");
    EndEffectorPosition pose1 = pose(p, false);
    return pose1;
}

geometry_msgs::Point Picking::point(float x, float y, float z) {
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

Picking::EndEffectorPosition Picking::pose(const NiryoPose &p, bool open) {
    EndEffectorPosition eef;
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

template <typename T>
void Picking::establish_connection(const T &ac, const std::string &what) {
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

template void
Picking::establish_connection<Picking::NiryoClient>(const NiryoClient &,
                                                    const std::string &);

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
    ROS_INFO("position: %.2f, %.2f, %.2f", cmd.position.x, cmd.position.y,
             cmd.position.z);
    ROS_INFO("rpy (r,p,y):  %.2f, %.2f, %2f", cmd.rpy.roll, cmd.rpy.pitch,
             cmd.rpy.yaw);
    robot.sendGoal(action.goal);
    bool success = robot.waitForResult(ros::Duration(MAX_DURATION));
    return success;
}

void Picking::moveToPosition(const EndEffectorPosition &eef) {
    const bool movement = Picking::MoveEEF(eef.pose);
    if (!movement) {
        ROS_WARN("Could not move the arm");
    }
    const bool aperture = Picking::GripperAperture(eef.open);
    if (!aperture) {
        ROS_WARN("Could not open the gripper");
    }
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

void Picking::moveArm(const geometry_msgs::Pose &pose) {
    // geometry_msgs::Quaternion quat_msg =
    // tf::createQuaternionMsgFromRollPitchYaw(
    // pose.second.roll, pose.second.pitch, pose.second.yaw);
    // Eigen::Quaternion<double> quat;
    // Eigen::Vector3d linear;
    // tf::quaternionMsgToEigen(pose.orientation, quat);
    // tf::pointMsgToEigen(pose.position, linear);
    Eigen::Isometry3d grasp_pose{};
    grasp_pose.matrix() = Eigen::Matrix4d::Identity(4, 4);
    // grasp_pose.matrix() = Eigen::Matrix4d::Identity(4, 4);
    // grasp_pose.linear() = quat.toRotationMatrix();
    // grasp_pose.translation() = linear;
    tf::poseMsgToEigen(pose, grasp_pose);
    ROS_WARN_STREAM("The grap frame is:\n" << grasp_pose.matrix());
    std::chrono::seconds sec(1);
    const auto pre_grasp_orientate = PreGrasp_orientate(grasp_pose);
    const auto pre_grasp_descend = PreGrasp_descend(grasp_pose);
    const auto grasp = computeGrasp(grasp_pose);
    const auto close = Close(grasp_pose);
    const auto post_grasp = PostGrasp(grasp_pose);
    // geometry_msgs::Point p = point(0.1, -0.2, 0.25);
    const auto pre_final = FinalPositions(0.1, -0.2, 0.25, false, "PreFinal");
    // geometry_msgs::Point p = point(0.1, -0.2, 0.25);
    const auto open = FinalPositions(0.1, -0.2, 0.25, true, "Final");
    // geometry_msgs::Point p = point(0.3, 0, 0.35);
    const auto rest_position = FinalPositions(0.3, 0, 0.35, false, "Rest");
    std::vector<Picking::EndEffectorPosition> movements = {
        pre_grasp_orientate, pre_grasp_descend, grasp, close,
        post_grasp,          pre_final,         open,  rest_position};
    for (const auto &movement : movements) {
        moveToPosition(movement);
        std::this_thread::sleep_for(sec);
    }
    std::this_thread::sleep_for(sec);
}

// std::tuple<double, double, double>
// convertQuaternionToRPY(const geometry_msgs::Quaternion &quat) {
// tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
// tf::Matrix3x3 rotation(q);
// double roll(0.0), pitch(0.0), yaw(0.0);
// rotation.getRPY(roll, pitch, yaw);
// return {roll, pitch, yaw};
//}

Picking::NiryoPose Picking::convertToNiryo(const Eigen::Isometry3d &frame,
                                           std::string name) {
    auto [roll, pitch, yaw] = utils::RPY(frame);
    NiryoPose pose;
    pose.first.x = frame(0, 3);
    pose.first.y = frame(1, 3);
    pose.first.z = frame(2, 3);
    pose.second.roll = roll;
    pose.second.pitch = pitch;
    pose.second.yaw = yaw;
    if (pose.first.x < 0.11) {
        ROS_ERROR_STREAM("Recived x value of " << pose.first.x
                                               << "set to 0.1 for safty");
        pose.first.x = 0.13;
    }
    if (pose.first.z < 0.03) {
        ROS_ERROR_STREAM("Recived x value of " << pose.first.z
                                               << "set to 0.1 for safty");
        pose.first.z = 0.03;
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
        // NiryoPose niryo_pose;
        auto [roll, pitch, yaw] = utils::RPY(pose.orientation);
        // auto [roll, pitch, yaw] = convertQuaternionToRPY(pose.orientation);
        if (std::abs(roll) > 1e-4) {
            ROS_ERROR_STREAM("Cannot accept roll values, was: " << roll);
            throw std::runtime_error("Cannot accept roll parameters");
        }
        // niryo_pose.second.roll = 0;
        // niryo_pose.second.pitch = pitch;
        // niryo_pose.second.yaw = yaw;
        // niryo_pose.first = pose.position;
        grasp_poses.push_back(pose);
        ROS_WARN_STREAM("The graping pose is x,y,z"
                        << pose.position.x << ", " << pose.position.y << ", "
                        << pose.position.z << ", "
                        << "and roll, pitch, yaw" << roll << ", " << pitch
                        << ", " << yaw);
    }
    return grasp_poses;
}

void Picking::callback(const geometry_msgs::PoseArray &poses) {
    ROS_WARN_STREAM("Inside the callback");
    std::vector<geometry_msgs::Pose> niryo_poses = check(poses);
    for (const auto &pose : niryo_poses) {
        moveArm(pose);
    }
}
