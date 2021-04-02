#include <pcl/io/pcd_io.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <sstream>

#include "picking.hpp"
#include "ros/ros.h"
#include "scene.hpp"
#include <filesystem>
#include <fstream>
#include <thread>

namespace fs = std::filesystem;

static constexpr std::size_t QUEUE(10);
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
static constexpr std::size_t N_JOINTS(6);

std::string return_current_time_and_date() {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d-%H-%M");
    return ss.str();
}

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

void save_transform(const tf::StampedTransform& transform, const fs::path& path,
        int iter) {
    fs::path name;
    Eigen::Affine3d T_curr;
    tf::transformTFToEigen(transform, T_curr);
    //auto T_curr = tf2::transformToEigen(transform);
    if (iter < 10) {
        name = "00" + std::to_string(iter) + ".txt";
    } else if (iter < 100) {
        name = "0" + std::to_string(iter) + ".txt";
    } else {
        name = std::to_string(iter) + ".txt";
    }
    const fs::path fn = path / name;
    std::cout << "location: " << fn << std::endl;
    std::ofstream pose_file;
    pose_file << std::scientific;
    pose_file.open(fn);
    if (!pose_file.is_open()) {
        throw std::ios_base::failure("Could not open pose file");
    }
    pose_file << T_curr.matrix() << "\n";
}

void save_img(const cv::Mat &img, const fs::path &path, int iter) {
    fs::path name;
    if (iter < 10) {
        name = "00" + std::to_string(iter) + ".png";
    } else if (iter < 100) {
        name = "0" + std::to_string(iter) + ".png";
    } else {
        name = std::to_string(iter) + ".png";
    }
    const fs::path fn = path / name;
    std::cout << "location: " << fn << std::endl;
    bool success = cv::imwrite(fn, img);
    if (!success) {
        throw std::runtime_error("Could not save img");
    }
}

struct Paths {
    fs::path color_pth;
    fs::path depth_pth;
    fs::path transform_pth;
};

Paths open_folder() {
    std::string current_date = return_current_time_and_date();
    std::filesystem::path second_root(current_date);
    auto root("/home/fabian/Documents/work/transforms/src/pick_place/data");
    Paths paths;
    paths.depth_pth = root / second_root / fs::path("depth_pth");
    paths.color_pth = root / second_root / fs::path("color_pth");
    paths.transform_pth = root / second_root / fs::path("transform_pth");
    fs::create_directories(paths.color_pth);
    fs::create_directories(paths.depth_pth);
    fs::create_directories(paths.transform_pth);
    return paths;
}

bool depth_and_color(PoseEstimation::Scene &scene, cv::Mat &depth,
                     cv::Mat &img) {
    img = scene.img();
    if (img.empty()) {
        ROS_WARN_STREAM("NO data arrived");
        return false;
    }
    depth = scene.depth();
    if (depth.empty()) {
        ROS_WARN_STREAM("NO data arrived");
        return false;
    }
    return true;
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
    // ros::Rate rate(1);
    PoseEstimation::Scene scene;
    ros::Subscriber sub =
        nh.subscribe("/camera/depth_registered/points", QUEUE,
                     &PoseEstimation::Scene::callback, &scene);
    Paths paths = open_folder();
    PointCloud::Ptr cloud(new PointCloud);
    int i(0);
    cv::Mat depth, img;
    tf::TransformListener listener;
    tf::StampedTransform transform;
    for (const auto &joint_position : joint_positions) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::cout << "received value:\n";
        for (const auto x : joint_position) {
            std::cout << x << ", ";
        }
        picker.moveJoints(joint_position);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        try {
            listener.lookupTransform("/base_link", "/turtle1", ros::Time(0),
                                     transform);
        } catch (tf::TransformException& ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
        depth_and_color(scene, depth, img);
        save_img(depth, paths.depth_pth, i);
        save_img(img, paths.color_pth, i);
        save_transform(transform, paths.transform_pth, i);
        ++i;
    }
    sub.shutdown();
    return 0;
}
