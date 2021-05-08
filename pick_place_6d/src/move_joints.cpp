#include "move_joints_server.hpp"
#include "ros/ros.h"
#include <filesystem>
#include <fstream>
#include <sstream>
#include <std_msgs/String.h>
#include <thread>
#include <utils/utils.hpp>
using param = std::pair<std::string, std::string>;

namespace fs = std::filesystem;
static constexpr std::size_t N_JOINTS(6);
static constexpr std::size_t SLEEP_TIME(50);

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
        if (tmp.size() == N_JOINTS) {
            trajectory.push_back(std::move(tmp));
        }
    }
}

MoveJointsAction::MoveJointsAction(const std::string &location,
                                   ros::NodeHandle &nh)
    : as(
          nh, "pick_place/move_joints", [this](auto &&x) { cb(x); }, false) {
    as.start();
    if (!fs::exists(location)) {
        ROS_WARN_STREAM("The path " << location << "does not exists");
        throw std::runtime_error("The path does not exists");
    }
    picker.connectToRobot(nh);
    readJointPositions(location);
    ROS_WARN_STREAM("ActionSerice waiting for input");
}

void MoveJointsAction::cb(const new_pick_place::MoveJointsGoalConstPtr & x) {
    std::cout << "beginning with iteration" << std::endl;
    std::size_t step(0);
    std::size_t sz(trajectory.size());
    while (!as.isPreemptRequested() and ros::ok() and step < sz) {
        std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_TIME));
        const std::vector<double> &joint_position = trajectory[step];
        ROS_INFO_STREAM("Recived value: " << joint_position[0] << ", " <<
                joint_position[1] << ", " << joint_position[2] << ", " <<
                joint_position[3] << ", " << joint_position[4] << ", " <<
                joint_position[5]);
        picker.moveJoints(joint_position);
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
    ros::NodeHandle nh;
    param location("move_joints/joint_position_file", "");
    utils::readParameters(nh, location);
    MoveJointsAction move_joints(location.second, nh);
    ros::spin();
    return 0;
}
