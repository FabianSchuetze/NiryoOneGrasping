#include "pose_estimation.hpp"
#include <ros/ros.h>
#include <string>
//using namespace ObjectPose;
static constexpr std::size_t QUEUE(10);
int main(int argc, char **argv) {
    ros::init(argc, argv, "cluster");
    ros::NodeHandle nh;
    ObjectPose::PoseEstimation pose_estimation("XX");
    ros::Rate rate(1);
    std::string _topic;
    nh.getParam("/cluster/topic", _topic);
    if (_topic.empty()) {
        ROS_WARN_STREAM("Rosparam /cluster/topic not defined");
        return 1;
    }
    ROS_WARN_STREAM("The name for the topic is " << _topic);
    ros::Subscriber sub =
        nh.subscribe(_topic, QUEUE, &ObjectPose::PoseEstimation::callback, &pose_estimation);
    // get the segmented pointcloud and do the clusterin yourself
    // load the shaped the iterate over them
    //
}
