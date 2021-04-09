#include "picking.hpp"
#include <actionlib/server/simple_action_server.h>
#include <pick_place/MoveJointsAction.h>
#include <filesystem>
#include <ros/ros.h>
#include <vector>
class MoveJointsAction {
  public:
    explicit MoveJointsAction();
    void callback(const pick_place::MoveJointsGoalConstPtr &);

  private:
    void readJointPositions(const std::string &loc);
    std::vector<std::vector<double>> trajectory;
    Picking picker;
    ros::NodeHandle nh;
    actionlib::SimpleActionServer<pick_place::MoveJointsAction> as;
    pick_place::MoveJointsFeedback feedback;
    pick_place::MoveJointsResult result;
    template <typename... T> void readParameters(T &... args) {
        auto read_parameters = [&](auto &t) {
            nh.getParam(t.first, t.second);
            if (t.second.empty()) {
                ROS_WARN_STREAM("Rosparam " << t.first << " not identified");
                throw std::runtime_error("Could not read all parameters");
            }
            ROS_WARN_STREAM("The parameters for " << t.first << " is "
                                                  << t.second);
        };
        (..., read_parameters(args));
    }
};
