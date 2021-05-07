#include "picking.hpp"
#include <actionlib/server/simple_action_server.h>
#include <pick_place/MoveJointsAction.h>
#include <filesystem>
#include <ros/ros.h>
#include <vector>
class MoveJointsAction {
  public:
    explicit MoveJointsAction(const std::string&, ros::NodeHandle&);
    void cb(const pick_place::MoveJointsGoalConstPtr &);

  private:
    void readJointPositions(const std::string &loc);
    std::vector<std::vector<double>> trajectory;
    Picking picker;
    actionlib::SimpleActionServer<pick_place::MoveJointsAction> as;
    pick_place::MoveJointsFeedback feedback;
    pick_place::MoveJointsResult result;
};
