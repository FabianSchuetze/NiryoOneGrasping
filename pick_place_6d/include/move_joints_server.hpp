#include "picking.hpp"
#include <actionlib/server/simple_action_server.h>
#include <new_pick_place/MoveJointsAction.h>
#include <filesystem>
#include <ros/ros.h>
#include <vector>
class MoveJointsAction {
  public:
    explicit MoveJointsAction(const std::string&, ros::NodeHandle&);
    void cb(const new_pick_place::MoveJointsGoalConstPtr &);

  private:
    void readJointPositions(const std::string &loc);
    std::vector<std::vector<double>> trajectory;
    Picking picker;
    actionlib::SimpleActionServer<new_pick_place::MoveJointsAction> as;
    new_pick_place::MoveJointsFeedback feedback;
    new_pick_place::MoveJointsResult result;
};
