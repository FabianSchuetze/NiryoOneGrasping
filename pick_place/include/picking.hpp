#ifndef picking_hpp
#define picking_hpp
#include <actionlib/client/action_client.h>
#include <actionlib/client/simple_action_client.h>
#include <niryo_one_msgs/RobotMoveAction.h>
#include <niryo_one_msgs/SetInt.h>
#include <pose_detection/BroadcastPoseAction.h>
#include <vector>
namespace Picking {

typedef actionlib::SimpleActionClient<niryo_one_msgs::RobotMoveAction>
    NiryoClient;
typedef actionlib::SimpleActionClient<pose_detection::BroadcastPoseAction>
    PoseClient;
using NiryoPose = std::pair<geometry_msgs::Point, niryo_one_msgs::RPY>;
struct EndEffectorPosition {
    NiryoPose pose;
    bool open;
};

EndEffectorPosition computePreGrasp(const std::vector<double> &);
EndEffectorPosition computeGrasp(const std::vector<double> &);
void setGripper(ros::NodeHandle&, int);
template <typename T>
void establish_connection(const T&, const std::string&);
std::vector<double> obtainPose(PoseClient&);
void positionGoal(NiryoClient &, const EndEffectorPosition &);
}; // namespace Picking
#endif
