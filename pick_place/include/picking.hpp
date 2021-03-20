#ifndef picking_hpp
#define picking_hpp
#include <actionlib/client/action_client.h>
#include <actionlib/client/simple_action_client.h>
#include <niryo_one_msgs/RobotMoveAction.h>
#include <niryo_one_msgs/SetInt.h>
#include <pose_detection/BroadcastPoseAction.h>
#include <vector>

class Picking {
  public:
    typedef actionlib::SimpleActionClient<niryo_one_msgs::RobotMoveAction>
        NiryoClient;
    typedef actionlib::SimpleActionClient<pose_detection::BroadcastPoseAction>
        PoseClient;
    using NiryoPose = std::pair<geometry_msgs::Point, niryo_one_msgs::RPY>;
    struct EndEffectorPosition {
        NiryoPose pose;
        bool open;
    };
    Picking();
    void connectToRobot();
    void connectToPositionServer();
    static EndEffectorPosition computePreGrasp(const std::vector<double> &);
    static EndEffectorPosition computeGrasp(const std::vector<double> &);
    static EndEffectorPosition Close(const std::vector<double> &);
    static EndEffectorPosition PreFinal();
    static EndEffectorPosition Final();
    static EndEffectorPosition Rest();
    void setGripper();
    std::vector<double> obtainPose();
    void moveToPosition(const EndEffectorPosition &);

  private:
    NiryoClient robot;
    PoseClient target;
    ros::NodeHandle n;
    template <typename T>
    void establish_connection(const T &, const std::string &);
    static niryo_one_msgs::RPY rotation();
    static geometry_msgs::Point point(float, float, float);
    static EndEffectorPosition pose(const geometry_msgs::Point &, bool);
    bool GripperAperture(bool);
    bool MoveEEF(const NiryoPose &);
};
#endif
