#include "picking.hpp"
#include <niryo_one_msgs/RobotMoveAction.h>
#include <niryo_one_msgs/SetInt.h>

static constexpr int MAX_ATTPEMTS = 10;
static constexpr float MAX_DURATION = 10.0;
constexpr static int TOOL_ID(13);

Picking::Picking()
    : robot("/niryo_one/commander/robot_action/", true),
      target("poseaction", true), n("~") {
    ;
};

void Picking::connectToRobot() {
    establish_connection(robot, "robot"); // wait for the action server to start
    setGripper();
}

void Picking::connectToPositionServer() {
    establish_connection(target, "target");
}
// namespace Picking {
Picking::EndEffectorPosition Picking::Rest() {
    geometry_msgs::Point p = point(0.3, 0, 0.35);
    EndEffectorPosition pose1 = pose(p, true);
    return pose1;
}
// geometry_msgs::Point p;
// p.x = 0.3,
// p.y = 0;
// p.z = 0.35;
// niryo_one_msgs::RPY rot;
// rot.roll = 0;
// rot.pitch = 1.5;
// rot.yaw = 0;
// NiryoPose pose1(p, rot);
// EndEffectorPosition eef;
// eef.pose = pose1;
// eef.open = true;
// return eef;
//}
Picking::EndEffectorPosition Picking::PreFinal() {
    geometry_msgs::Point p = point(0.1, -0.2, 0.2);
    EndEffectorPosition pose1 = pose(p, false);
    return pose1;
}
// geometry_msgs::Point p;
// p.x = 0.1;
// p.y = -0.2;
// p.z = 0.2;
// niryo_one_msgs::RPY rot;
// rot.roll = 0;
// rot.pitch = 1.5;
// rot.yaw = 0;
// NiryoPose pose1(p, rot);
// EndEffectorPosition eef;
// eef.pose = pose1;
// eef.open = false;
// return eef;
//}
Picking::EndEffectorPosition Picking::Final() {
    geometry_msgs::Point p = point(0.1, -0.2, 0.2);
    EndEffectorPosition pose1 = pose(p, true);
    return pose1;
}
// p.x = 0.1;
// p.y = -0.2;
// p.z = 0.2;
// niryo_one_msgs::RPY rot;
// rot.roll = 0;
// rot.pitch = 1.5;
// rot.yaw = 0;
// NiryoPose pose1(p, rot);
// EndEffectorPosition eef;
// eef.pose = pose1;
// eef.open = true;
// return eef;
//}
Picking::EndEffectorPosition
Picking::computePreGrasp(const std::vector<double> &goal) {
    geometry_msgs::Point p = point(goal[0], goal[1], goal[2] + 0.15);
    // p.z = goal[2] + 0.15;
    if (p.z < 0.135) {
        throw std::runtime_error("Z values cannot be lower than 0.135");
    }
    EndEffectorPosition pose1 = pose(p, true);
    return pose1;
}

// rot.roll = 0;
// rot.pitch = 1.5;
// rot.yaw = 0;
// NiryoPose pose1(p, rot);
// EndEffectorPosition eef;
// eef.pose = pose1;
// eef.open = true;
// return eef;
//}

Picking::EndEffectorPosition Picking::computeGrasp(const std::vector<double> &goal) {
    geometry_msgs::Point p = point(goal[0], goal[1], goal[2]);
    if (p.z < 0.135) {
        p.z = 0.135;
        ROS_WARN_STREAM("Set the height value to 0.135");
        // throw std::runtime_error("Z values cannot be lower than 0.135");
    }
    EndEffectorPosition pose1 = pose(p, true);
    return pose1;
}
    //niryo_one_msgs::RPY rot;
    //rot.roll = 0;
    //rot.pitch = 1.5;
    //rot.yaw = 0;
    //NiryoPose pose1(p, rot);
    //EndEffectorPosition eef;
    //eef.pose = pose1;
    //eef.open = true;
    //return eef;
//}
Picking::EndEffectorPosition Picking::Close(const std::vector<double> &goal) {
    geometry_msgs::Point p = point(goal[0], goal[1], goal[2]);
    if (p.z < 0.135) {
        p.z = 0.135;
        ROS_WARN_STREAM("Set the height value to 0.135");
        // throw std::runtime_error("Z values cannot be lower than 0.135");
    }
    EndEffectorPosition pose1 = pose(p, false);
    return pose1;
}
    //geometry_msgs::Point p;
    //p.x = goal[0];
    //p.y = goal[1];
    //p.z = goal[2];
    //if (p.z < 0.135) {
        //p.z = 0.135;
        //ROS_WARN_STREAM("Set the height value to 0.135");
        //// throw std::runtime_error("Z values cannot be lower than 0.135");
    //}
    //niryo_one_msgs::RPY rot;
    //rot.roll = 0;
    //rot.pitch = 1.5;
    //rot.yaw = 0;
    //NiryoPose pose1(p, rot);
    //EndEffectorPosition eef;
    //eef.pose = pose1;
    //eef.open = false;
    //return eef;
//}
niryo_one_msgs::RPY Picking::rotation() {
    niryo_one_msgs::RPY rot;
    rot.roll = 0;
    rot.pitch = 1.5;
    rot.yaw = 0;
    return rot;
}
geometry_msgs::Point Picking::point(float x, float y, float z) {
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

Picking::EndEffectorPosition Picking::pose(const geometry_msgs::Point &p,
                                           bool open) {
    niryo_one_msgs::RPY rot = rotation();
    NiryoPose pose1(p, rot);
    EndEffectorPosition eef;
    eef.pose = pose1;
    eef.open = open;
    return eef;
}

void Picking::setGripper() {
    ROS_INFO("Setting gripper");
    ros::ServiceClient changeToolClient_;
    changeToolClient_ =
        n.serviceClient<niryo_one_msgs::SetInt>("/niryo_one/change_tool/");
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
template void
Picking::establish_connection<Picking::PoseClient>(const PoseClient &,
                                                   const std::string &);

std::vector<double> Picking::obtainPose() {
    pose_detection::BroadcastPoseGoal goal;
    target.sendGoal(goal);
    geometry_msgs::TransformStamped result;
    bool finished = target.waitForResult(ros::Duration(MAX_DURATION));
    if (finished) {
        result = target.getResult()->pose;
        actionlib::SimpleClientGoalState state = target.getState();
        ROS_INFO_STREAM("Action finished: " << state.toString());
    } else {
        ROS_INFO_STREAM("Could not obtain pose before timeout.");
        throw std::runtime_error("Could not obtain pose before timeout");
    }
    const auto &translation = result.transform.translation;
    // (TODO) Add proper rotation;
    std::vector<double> pose = {
        translation.x, translation.y, translation.z, 0, 0, 0};
    return pose;
}

bool Picking::GripperAperture(bool open) {
    niryo_one_msgs::ToolCommand tcmd;
    if (open) {
        tcmd.cmd_type = 1;
    } else {
        tcmd.cmd_type = 2;
    }
    tcmd.gripper_open_speed = 200;
    tcmd.tool_id = 13;
    niryo_one_msgs::RobotMoveActionGoal action;
    action.goal.cmd.cmd_type = 6;
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
