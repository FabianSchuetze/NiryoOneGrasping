#include <std_msgs/String.h>
#include <sstream>
#include "move_joints_server.hpp"
#include "picking.hpp"
#include "ros/ros.h"
#include <filesystem>
#include <fstream>
#include <thread>

namespace fs = std::filesystem;
static constexpr std::size_t N_JOINTS(6);

void MoveJointsAction::readJointPositions(const std::string &loc) {
    std::filesystem::path fn(loc);
    std::ifstream myFile(fn);
    if (!myFile.is_open()) {
        std::stringstream ss;
        ss << "Clould not open file " << fn << std::endl;
        throw std::runtime_error(ss.str());
    }
    std::string line;
    while (std::getline(myFile, line)) {
        std::stringstream ss(line);
        std::vector<double> tmp;
        std::string val;
        const char *separator = ",";
        while (std::getline(ss, val, *separator)) {
            tmp.push_back(std::stod(val));
        }
        std::cout << "end line, vector size: " << tmp.size() << std::endl;
        if (tmp.size() == 6) {
            trajectory.push_back(std::move(tmp));
        }
    }
}

MoveJointsAction::MoveJointsAction()
    : as(nh, "pick_place/move_joints", [this](auto&& x) { callback(x); } , false) {
    as.start();
    ROS_WARN_STREAM("The current path is " << fs::current_path());
    std::pair<std::string, std::string> location("move_joints/joint_position_file", "");
    readParameters(location);
    if (!fs::exists(location.second)) {
        ROS_WARN_STREAM("The path " << location.second << "does not exists");
        throw std::runtime_error("The path does not exists");
    }
    picker.connectToRobot();
    readJointPositions(location.second);
    ROS_WARN_STREAM("ActionSerice waiting for input");
}

void MoveJointsAction::callback(const pick_place::MoveJointsGoalConstPtr &) {
    std::cout << "beginning with iteration" << std::endl;
    std::size_t step(0);
    std::size_t sz(trajectory.size());
    while (!as.isPreemptRequested() and ros::ok() and step < sz) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        const std::vector<double> &joint_position = trajectory[step];
        std::cout << "received value:\n";
        for (const auto x : joint_position) {
            std::cout << x << ", ";
        }
        std::cout << "Moving joints " << std::endl;
        picker.moveJoints(joint_position);
        //std::this_thread::sleep_for(std::chrono::seconds(0.2));
        feedback.fraction = static_cast<float>(step) / sz;
        as.publishFeedback(feedback);
        ++step;
    }
    if (step == sz) {
        result.result = true;
        as.setSucceeded(result);
    } else {
        ROS_WARN_STREAM("Did not finish the program");
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "move_joints");
    MoveJointsAction move_joints{};
    ros::spin();
    return 0;
}
