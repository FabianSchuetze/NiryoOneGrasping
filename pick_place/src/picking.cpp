#include "picking.hpp"

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

void establish_connection(NiryoClient& ac) {
    ROS_INFO("Connecting to robot  ========================");
    size_t attempts(0);
    while (!ac.waitForServer(ros::Duration(3.0))) {
        ROS_WARN("  Error connecting to Robot. Trying again");
        ++attempts;
        if (attempts > 10)
            throw std::runtime_error("Could not achieve connection");
    }
    ROS_INFO("  Robot Connection established");
}

} // namespace Picking
