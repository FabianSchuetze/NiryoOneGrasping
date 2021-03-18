#include "broadcast_action_server.hpp"
#include <tf2/LinearMath/Quaternion.h>
static constexpr uint QUEUE = 10;
static constexpr char listen[] = "/grasp_position"; // NOLINT

BroadcastPoseAction::BroadcastPoseAction(const std::string &name,
                                         ros::NodeHandle *nodehandle)
    : nh_(*nodehandle),
      as_(nh_, name, boost::bind(&BroadcastPoseAction::executeCB, this, _1),
          false) {
    initializeSubscriber();
    initializeService();
};

void BroadcastPoseAction::initializeService() { as_.start(); }

void BroadcastPoseAction::initializeSubscriber() {
    subscriber_ = nh_.subscribe("grasp_position", 1,
                                &BroadcastPoseAction::receiveInfo, this);
}

void BroadcastPoseAction::executeCB(
    const pose_detection::BroadcastPoseGoalConstPtr &goal) {
    ros::Rate r(1);
    bool success = false;
    feedback_.finished = false;
    geometry_msgs::TransformStamped tmp;
    while (true) {
        if (as_.isPreemptRequested() || !ros::ok()) {
            ROS_INFO_STREAM("Preempted Server ");
            as_.setPreempted();
            success = false;
            break;
        }
        {
            std::lock_guard<std::mutex> guard(g_pages_mutex);
            // (TODO)Need to check if lock can be acquired and wait otherwise to
            // publish feedback!
            tmp = pose;
            tmp.transform.translation.x = 10;
            // (TODO) Check the most recent time different and set success based
            // on this!
            success = true;
        }
        as_.publishFeedback(feedback_);
        r.sleep();
        break;
    }

    if (success) {
        result_.pose = tmp;
        ROS_INFO_STREAM("Succeeded");
        // set the action state to succeeded
        as_.setSucceeded(result_);
    }
}

void BroadcastPoseAction::receiveInfo(const geometry_msgs::Point &msg) {
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    std::lock_guard<std::mutex> guard(g_pages_mutex);
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "camera_depth_optical_frame";
    pose.child_frame_id = "object";
    pose.transform.translation.x = msg.x;
    pose.transform.translation.y = msg.y;
    pose.transform.translation.z = msg.z;
    pose.transform.rotation.x = q.x();
    pose.transform.rotation.y = q.y();
    pose.transform.rotation.z = q.z();
    pose.transform.rotation.w = q.w();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "poseaction");
    ros::NodeHandle nh;
    BroadcastPoseAction poseaction("poseaction", &nh);
    ros::spin();
    return 0;
}
