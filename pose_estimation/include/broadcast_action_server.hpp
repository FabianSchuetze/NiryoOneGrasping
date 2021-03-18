#ifndef broadcast_action_server_hpp
#define broadcast_action_server_hpp
#include <actionlib/server/simple_action_server.h>
#include <pose_detection/BroadcastPoseAction.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <mutex>

class BroadcastPoseAction {
  public:
    explicit BroadcastPoseAction(const std::string &name, ros::NodeHandle*);
    void executeCB(const pose_detection::BroadcastPoseGoalConstPtr &);
    void receiveInfo(const geometry_msgs::Point &);

  private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<pose_detection::BroadcastPoseAction>
        as_; // NodeHandle instance must be created before this line. Otherwise
             // strange error occurs.
    // create messages that are used to published feedback/result
    pose_detection::BroadcastPoseFeedback feedback_;
    pose_detection::BroadcastPoseResult result_;
    geometry_msgs::TransformStamped pose;
    ros::Subscriber subscriber_;
    std::mutex g_pages_mutex;
    void initializeService();
    void initializeSubscriber();
};
#endif
