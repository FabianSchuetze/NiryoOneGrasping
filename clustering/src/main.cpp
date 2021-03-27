#include "scene.hpp"
#include <ros/ros.h>
static constexpr std::size_t QUEUE(10);
static constexpr std::size_t RATE(1);
int main(int argc, char **argv) {
    ros::init(argc, argv, "cluster");
    ros::NodeHandle nh;
    Scene scene;
    ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", QUEUE,
                                       &Scene::callback, &scene);
    // ros::Publisher pub =
    // nh.advertise<geometry_msgs::Point>("grasp_position", 1);
    ros::Rate rate(RATE);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    while (ros::ok()) {
        rate.sleep();
        ros::spinOnce();
        bool success = scene.pointCloud(cloud);
        if (!success)
            continue;
        ROS_INFO_STREAM("Got the data");
    }
    sub.shutdown();
    return 0;
}
