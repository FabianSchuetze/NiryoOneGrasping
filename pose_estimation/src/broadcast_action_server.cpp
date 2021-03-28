#include "broadcast_action_server.hpp"
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>

BroadcastPoseAction::BroadcastPoseAction(const std::string &name,
                                         ros::NodeHandle *nodehandle)
    : nh_(*nodehandle),
      as_(
          nh_, name, [this](auto &&x) { executeCB(x); }, false) {
    initializeService();
};

void BroadcastPoseAction::initializeService() { as_.start(); }

void BroadcastPoseAction::executeCB(
    const pose_detection::BroadcastPoseGoalConstPtr &goal) {
    ros::Rate r(1);
    bool success = false;
    feedback_.finished = false;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;
    while (true) {
        if (as_.isPreemptRequested() || !ros::ok()) {
            ROS_INFO_STREAM("Preempted Server ");
            as_.setPreempted();
            success = false;
            break;
        }
        try {
            transformStamped =
                tfBuffer.lookupTransform("base_link", "object", ros::Time(0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            as_.publishFeedback(feedback_);
            r.sleep();
            continue;
        }
        success = true;
        break;
    }
    if (success) {
        result_.pose = transformStamped;
        ROS_INFO_STREAM("Succeeded");
        as_.setSucceeded(result_);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "poseaction");
    ros::NodeHandle nh;
    BroadcastPoseAction poseaction("poseaction", &nh); // NOLINT
    ros::spin();
    return 0;
}
