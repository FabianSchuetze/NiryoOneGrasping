#include <pcl/io/pcd_io.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>

#include <sstream>

#include "picking.hpp"
#include "ros/ros.h"
#include "scene.hpp"
#include <filesystem>
#include <fstream>

static constexpr std::size_t QUEUE(10);
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
static constexpr std::size_t N_JOINTS(6);

std::vector<std::vector<double>> readJointPositions() {
    std::filesystem::path fn("/home/fabian/Documents/work/transforms/src/"
                             "pick_place/data/positions.txt");
    std::ifstream myFile(fn);
    std::string line;
    std::vector<std::vector<double>> positions;
    while (std::getline(myFile, line)) {
        std::stringstream ss(line);
        std::vector<double> tmp;
        double val(0.0);
        while (ss >> val) {
            tmp.push_back(val);
        }
        if (!(tmp.size() == N_JOINTS)) {
            ROS_WARN_STREAM("Did not load 6 joint elments");
            throw std::runtime_error("Did not load 6 joint elements");
        }
        positions.push_back(std::move(tmp));
    }
    return positions;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "move_joints");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(3);
    spinner.start();
    Picking picker;
    picker.connectToRobot();
    picker.connectToPositionServer();
    std::vector<std::vector<double>> joint_positions;
    try {
        joint_positions = readJointPositions();
    } catch (std::runtime_error &e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }
    ros::Rate rate(1);
    Clustering::Scene scene;
    ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", QUEUE,
                                       &Scene::callback, &scene);
    PointCloud::Ptr cloud(new PointCloud);
    pcl::PCDWriter writer;
    for (const auto &joint_position : joint_positions) {
        // while (ros::ok()) {
        // const auto t1 = std::chrono::high_resolution_clock::now();
        rate.sleep();
        ros::spinOnce();
        if (!scene.pointCloud(cloud)) {
            continue;
        }
        std::cout << "received value:\n";
        for (const auto x : joint_position) {
            std::cout << x << ", ";
        }
        picker.moveJoints(joint_position);
    }
    return 0;
}
