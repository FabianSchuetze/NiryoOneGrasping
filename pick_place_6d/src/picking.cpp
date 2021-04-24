#include "picking.hpp"
#include <chrono>
#include <niryo_one_msgs/RobotMoveAction.h>
#include <niryo_one_msgs/SetInt.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_datatypes.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <Eigen/Geometry>
#include <thread>

static constexpr int MAX_ATTPEMTS(10);
static constexpr float MAX_DURATION(10.0);
static constexpr int TOOL_ID(13);
static constexpr int MAX_SPEED(200);
static constexpr int CMD_TYPE(6);
static constexpr float HALF_ANGLE(1.5);
static constexpr float FIFITY_DEGREE(0.87);


Picking::Picking() : robot("/niryo_one/commander/robot_action/", true) { ; };

void Picking::connectToRobot(ros::NodeHandle &node) {
    establish_connection(robot, "robot"); // wait for the action server to start
    setGripper(node);
}

//// namespace Picking {
Picking::EndEffectorPosition Picking::Rest() {
    NiryoPose _pose;
    geometry_msgs::Point p = point(0.3, 0, 0.35);
    _pose.first = p;
    EndEffectorPosition pose1 = pose(_pose, false);
    pose1.pose.second.pitch = HALF_ANGLE;
    return pose1;
}
Picking::EndEffectorPosition Picking::PreFinal() {
    NiryoPose _pose;
    geometry_msgs::Point p = point(0.1, -0.2, 0.25);
    _pose.first = p;
    EndEffectorPosition pose1 = pose(_pose, false);
    pose1.pose.second.pitch = HALF_ANGLE;
    return pose1;
}
Picking::EndEffectorPosition Picking::Final() {
    NiryoPose _pose;
    geometry_msgs::Point p = point(0.1, -0.2, 0.25);
    _pose.first = p;
    EndEffectorPosition pose1 = pose(_pose, true);
    pose1.pose.second.pitch = HALF_ANGLE;
    return pose1;
}

Picking::EndEffectorPosition Picking::computePreGrasp_orientate(NiryoPose p,
        const Eigen::Isometry3d& incoming_pose) {
    if (p.second.pitch == 0) {
        Eigen::Isometry3d move;
        move.matrix()(0,3) = -0.1;
        move.matrix()(2,3) = 0.2;
        Eigen::Isometry3d approach = incoming_pose * move;
        p.first.x = approach.matrix()(0,3);
        p.first.y = approach.matrix()(1,3);
        p.first.z = approach.matrix()(2,3);
        if (p.first.x < 0.11) {
            ROS_ERROR_STREAM("Recived x value of " << p.first.x << 
                    "set to 0.1 for safty");
            p.first.x = 0.11;
        }
    } else {
        p.first.z = 0.20;
    }
    ROS_WARN_STREAM("Orientate Pose: "  << p.first.x << ", " << p.first.y <<
            ", " << p.first.z << ", " << p.second.roll << ", " << 
            p.second.pitch << ", " << p.second.yaw);
    //p.first.z = 0.25; // SAFTY FIRST
    EndEffectorPosition pose1 = pose(p, true);
    return pose1;
}

Picking::EndEffectorPosition Picking::computePreGrasp_descend(NiryoPose p,
        const Eigen::Isometry3d& incoming_pose) {
    if (p.second.pitch > 1.49) {
        p.first.z += 0.15;
    } else if (p.second.pitch == 0.0) {
        Eigen::Isometry3d move;
        move.matrix()(0,3) = -0.07;
        move.matrix()(2,3) = 0.10;
        Eigen::Isometry3d approach = incoming_pose * move;
        p.first.x = approach.matrix()(0,3);
        if (p.first.x < 0.11) {
            ROS_ERROR_STREAM("Recived x value of " << p.first.x << 
                    "set to 0.1 for safty");
            p.first.x = 0.11;
        }
        p.first.y = approach.matrix()(1,3);
        p.first.z = approach.matrix()(2,3);
    }
    ROS_WARN_STREAM("Descend Pose: " << p.first.x << ", " << p.first.y <<
            ", " << p.first.z << ", " << p.second.roll << ", " << 
            p.second.pitch << ", " << p.second.yaw);
    //p.first.z = 0.15; // SAFTY FIRST
    EndEffectorPosition pose1 = pose(p, true);
    return pose1;
}

Picking::EndEffectorPosition Picking::computeGrasp(NiryoPose p,
        const Eigen::Isometry3d& incoming_pose) {
    if (p.second.pitch > 1.49) {
        p.first.z += 0.085;
    } else if (p.second.pitch == 0.0) {
        Eigen::Isometry3d move;
        move.matrix()(0,3) = -0.04;
        move.matrix()(2,3) = 0.04;
        Eigen::Isometry3d approach = incoming_pose * move;
        p.first.x = approach.matrix()(0,3);
        if (p.first.x < 0.11) {
            ROS_ERROR_STREAM("Recived x value of " << p.first.x << 
                    "set to 0.1 for safty");
            p.first.x = 0.11;
        }
        p.first.y = approach.matrix()(1,3);
        p.first.z = approach.matrix()(2,3);
    }
    ROS_WARN_STREAM("Compute Grasp Pose: " << p.first.x << ", " << p.first.y <<
            ", " << p.first.z << ", " << p.second.roll << ", " << 
            p.second.pitch << ", " << p.second.yaw);
    //p.first.z = 0.15; // SAFTY FIRST
    EndEffectorPosition pose1 = pose(p, true);
    return pose1;
}
Picking::EndEffectorPosition Picking::Close(NiryoPose p,
        const Eigen::Isometry3d& incoming_pose) {
    if (p.second.pitch > 1.49) {
        p.first.z += 0.085;
    } else if (p.second.pitch == 0.0) {
        Eigen::Isometry3d move;
        move.matrix()(0,3) = -0.04;
        move.matrix()(2,3) = 0.04;
        Eigen::Isometry3d approach = incoming_pose * move;
        p.first.x = approach.matrix()(0,3);
        if (p.first.x < 0.11) {
            ROS_ERROR_STREAM("Recived x value of " << p.first.x << 
                    "set to 0.1 for safty");
            p.first.x = 0.11;
        }
        p.first.y = approach.matrix()(1,3);
        p.first.z = approach.matrix()(2,3);
    }
    ROS_WARN_STREAM("Close Pose: " << p.first.x << ", " << p.first.y <<
            ", " << p.first.z << ", " << p.second.roll << ", " << 
            p.second.pitch << ", " << p.second.yaw);
    //p.first.z = 0.15; // SAFTY FIRST
    EndEffectorPosition pose1 = pose(p, false);
    return pose1;
}

Picking::EndEffectorPosition Picking::PostGrasp(NiryoPose p,
        const Eigen::Isometry3d& incoming_pose) {
    p.first.z = 0.25;
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

void Picking::moveArm(const NiryoPose &pose) {
    //tf2::Quaternion quat;
    //quat.setRPY( pose.second.roll, pose.second.pitch, pose.second.yaw);
    geometry_msgs::Quaternion quat_msg = tf::createQuaternionMsgFromRollPitchYaw(
            pose.second.roll, pose.second.pitch, pose.second.yaw);
    Eigen::Quaternion<double> quat;
    Eigen::Vector3d linear;
    tf::quaternionMsgToEigen(quat_msg, quat);
    tf::pointMsgToEigen(pose.first, linear);
    Eigen::Isometry3d grasp_pose{};
    grasp_pose.matrix() = Eigen::Matrix4d::Identity(4,4);
    grasp_pose.linear() = quat.toRotationMatrix();
    grasp_pose.translation() = linear;
    ROS_WARN_STREAM("The grap frame is:\n" << grasp_pose.matrix());
    std::chrono::seconds sec(1);
    const Picking::EndEffectorPosition pre_grasp_orientate =
        Picking::computePreGrasp_orientate(pose, grasp_pose);
    const Picking::EndEffectorPosition pre_grasp_descend =
        Picking::computePreGrasp_descend(pose, grasp_pose);
    const EndEffectorPosition grasp = computeGrasp(pose, grasp_pose);
    const EndEffectorPosition close = Close(pose, grasp_pose);
    const EndEffectorPosition post_grasp = PostGrasp(pose, grasp_pose);
    const EndEffectorPosition pre_final = PreFinal();
    const EndEffectorPosition open = Final();
    const EndEffectorPosition rest_position = Rest();
    std::vector<Picking::EndEffectorPosition> movements = {
        pre_grasp_orientate, pre_grasp_descend, grasp, close,
        post_grasp,          pre_final,         open,  rest_position};
    for (const auto &movement : movements) {
        moveToPosition(movement);
        std::this_thread::sleep_for(sec);
    }
    std::this_thread::sleep_for(sec);
}

std::tuple<double, double, double>
convertQuaternionToRPY(const geometry_msgs::Quaternion &quat) {
    tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf::Matrix3x3 rotation(q);
    double roll(0.0), pitch(0.0), yaw(0.0);
    rotation.getRPY(roll, pitch, yaw);
    return {roll, pitch, yaw};
}

std::vector<Picking::NiryoPose>
Picking::convertToNiryo(const geometry_msgs::PoseArray &poses) {
    std::vector<NiryoPose> niryo_poses;
    for (const auto &pose : poses.poses) {
        NiryoPose niryo_pose;
        ROS_WARN_STREAM("The incoming quaternion is: " <<
                pose.orientation.x  << ", " <<
                pose.orientation.y << ", " <<
                pose.orientation.z << ", " <<
                pose.orientation.w);
        auto [roll, pitch, yaw] = convertQuaternionToRPY(pose.orientation);
        ROS_WARN_STREAM("The incoming pitch and yaw is: " << pitch << ", " << yaw);
        niryo_pose.second.roll = 0;
        niryo_pose.second.pitch = pitch;
        niryo_pose.second.yaw = yaw;
        niryo_pose.first = pose.position;
        //niryo_pose.first.y = pose.position.y;
        //niryo_pose.first.z = pose.position.z;
        niryo_poses.push_back(niryo_pose);
        ROS_WARN_STREAM("The graping pose is x,y,z"
                        << niryo_pose.first.x << ", " << niryo_pose.first.y
                        << ", " << niryo_pose.first.z << ", "
                        << "and roll, pitch, yaw" << niryo_pose.second.roll
                        << ", " << niryo_pose.second.pitch << ", "
                        << niryo_pose.second.yaw);
    }
    return niryo_poses;
}

void Picking::callback(const geometry_msgs::PoseArray &poses) {
    ROS_WARN_STREAM("Inside the callback");
    // if poses > exceeds the previous pose
    std::vector<NiryoPose> niryo_poses = convertToNiryo(poses);
    for (const auto &pose : niryo_poses) {
        moveArm(pose);
    }
}
