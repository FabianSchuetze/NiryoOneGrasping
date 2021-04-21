#include "picking.hpp"
#include <chrono>
#include <niryo_one_msgs/RobotMoveAction.h>
#include <niryo_one_msgs/SetInt.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <thread>

static constexpr int MAX_ATTPEMTS(10);
static constexpr float MAX_DURATION(10.0);
static constexpr int TOOL_ID(13);
static constexpr int MAX_SPEED(200);
static constexpr int CMD_TYPE(6);

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
    EndEffectorPosition pose1 = pose(_pose, true);
    pose1.pose.second.pitch = 1.5;
    return pose1;
}
Picking::EndEffectorPosition Picking::PreFinal() {
    NiryoPose _pose;
    geometry_msgs::Point p = point(0.1, -0.2, 0.25);
    _pose.first = p;
    EndEffectorPosition pose1 = pose(_pose, false);
    pose1.pose.second.pitch = 1.5;
    return pose1;
}
Picking::EndEffectorPosition Picking::Final() {
    NiryoPose _pose;
    geometry_msgs::Point p = point(0.1, -0.2, 0.25);
    _pose.first = p;
    EndEffectorPosition pose1 = pose(_pose, true);
    pose1.pose.second.pitch = 1.5;
    return pose1;
}

Picking::EndEffectorPosition Picking::computePreGrasp_orientate(NiryoPose p) {
    p.first.z = 0.25;
    if (p.second.pitch == 0) {
        p.first.x -= 0.08; //descend and approach
    }
    EndEffectorPosition pose1 = pose(p, true);
    return pose1;
}

Picking::EndEffectorPosition Picking::computePreGrasp_descend(NiryoPose p) {
    if (p.second.pitch == 1.5) {
        p.first.z += 0.15;
    } else if (p.second.pitch == 0.0) {
        p.first.z += 0.15;
        p.first.x -= 0.03;
    }
    EndEffectorPosition pose1 = pose(p, true);
    return pose1;
}

Picking::EndEffectorPosition Picking::computeGrasp(NiryoPose p) {
    if (p.second.pitch == 1.5) {
        p.first.z += 0.085;
    } else if (p.second.pitch == 0.0) {
        p.first.z = 0.03;
        p.first.x -= 0.03;
    }
    EndEffectorPosition pose1 = pose(p, true);
    return pose1;
}
Picking::EndEffectorPosition Picking::Close(NiryoPose p) {
    if (p.second.pitch == 1.5) {
        p.first.z += 0.085;
    } else if (p.second.pitch == 0.0) {
        p.first.z = 0.03;
        p.first.x -= 0.03;
    }
    EndEffectorPosition pose1 = pose(p, false);
    return pose1;
}

Picking::EndEffectorPosition Picking::PostGrasp(NiryoPose p) {
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
    std::chrono::seconds sec(1);
    const Picking::EndEffectorPosition pre_grasp_orientate =
        Picking::computePreGrasp_orientate(pose);
    const Picking::EndEffectorPosition pre_grasp_descend =
        Picking::computePreGrasp_descend(pose);
    const EndEffectorPosition grasp = computeGrasp(pose);
    const EndEffectorPosition close = Close(pose);
    const EndEffectorPosition post_grasp = PostGrasp(pose);
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
        auto [roll, pitch, yaw] = convertQuaternionToRPY(pose.orientation);
        niryo_pose.second.roll = 0;
        if (0.5 < pitch <= 1.5) {
            niryo_pose.second.pitch = 1.5;
        } else if (-0.5 < pitch < 0.5) {
            niryo_pose.second.pitch = 0;
        }
        //niryo_pose.second.pitch = 1.5; // always bend the hand
        niryo_pose.second.yaw = yaw;
        niryo_pose.first.x = pose.position.x;
        niryo_pose.first.y = pose.position.y;
        niryo_pose.first.z = pose.position.z;
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
