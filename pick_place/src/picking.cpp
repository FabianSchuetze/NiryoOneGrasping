#include "picking.hpp"
#include <niryo_one_msgs/RobotMoveAction.h>
#include <niryo_one_msgs/SetInt.h>

static constexpr int MAX_ATTPEMTS = 10;
static constexpr float MAX_DURATION = 10.0;

namespace Picking {
EndEffectorPosition computePreGrasp(const std::vector<double> &goal) {
    geometry_msgs::Point p;
    p.x = goal[0];
    p.y = goal[1];
    p.z = goal[2] + 0.15;
    if (p.z < 0.135) {
        throw std::runtime_error("Z values cannot be lower than 0.135");
    }
    niryo_one_msgs::RPY rot;
    rot.roll = 0;
    rot.pitch = 1.5;
    rot.yaw = 0;
    NiryoPose pose1(p, rot);
    EndEffectorPosition eef;
    eef.pose = pose1;
    eef.open = false;
    return eef;
}

EndEffectorPosition computeGrasp(const std::vector<double> &goal) {
    geometry_msgs::Point p;
    p.x = goal[0];
    p.y = goal[1];
    p.z = goal[2] + 0.135;
    if (p.z < 0.135) {
        throw std::runtime_error("Z values cannot be lower than 0.135");
    }
    niryo_one_msgs::RPY rot;
    rot.roll = 0;
    rot.pitch = 1.5;
    rot.yaw = 0;
    NiryoPose pose1(p, rot);
    EndEffectorPosition eef;
    eef.pose = pose1;
    eef.open = true;
    return eef;
}
void setGripper(ros::NodeHandle &node, int toolID) {
    ROS_INFO("Setting gripper");
    ros::ServiceClient changeToolClient_;
    changeToolClient_ =
        node.serviceClient<niryo_one_msgs::SetInt>("/niryo_one/change_tool/");
    niryo_one_msgs::SetInt srv;
    srv.request.value = toolID;
    while (!changeToolClient_.call(srv)) {
        ROS_WARN("Could not set the tool type. Trying again in one second");
        ros::Duration(1.0).sleep();
    }
    ROS_INFO("Success");
}

template <typename T>
void establish_connection(const T &ac, const std::string &what) {
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

template void establish_connection<NiryoClient>(const NiryoClient &,
                                                const std::string &);
template void establish_connection<PoseClient>(const PoseClient &,
                                               const std::string &);

std::vector<double> obtainPose(PoseClient &ac) {
    pose_detection::BroadcastPoseGoal goal;
    ac.sendGoal(goal);
    geometry_msgs::TransformStamped result;
    bool finished = ac.waitForResult(ros::Duration(MAX_DURATION));
    if (finished) {
        result = ac.getResult()->pose;
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
    } else {
        ROS_INFO("Action did not finish before the time out.");
    }
    const auto &translation = result.transform.translation;
    // (TODO) Add proper rotation;
    std::vector<double> pose = {
        translation.x, translation.y, translation.z, 0, 0, 0};
    return pose;
}
bool GripperAperture(NiryoClient &ac, bool open) {
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
    ac.sendGoal(action.goal);
    bool success = ac.waitForResult(ros::Duration(5.0));
    return success;
}

bool MoveEEF(NiryoClient& ac, const NiryoPose& pose) {
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
    ac.sendGoal(action.goal);
    bool success = ac.waitForResult(ros::Duration(5.0));
    return success;
}

void positionGoal(NiryoClient &ac, const EndEffectorPosition &eef) {
    bool movement = MoveEEF(ac, eef.pose);
    if (!movement) {
        ROS_WARN("Could not move the arm");
    }
    bool aperture = GripperAperture(ac, eef.open);
    if (!aperture) {
        ROS_WARN("Could not open the gripper");
    }
}

} // namespace Picking
