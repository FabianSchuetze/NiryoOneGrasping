#ifndef picking_hpp
#define picking_hpp
#include <actionlib/client/action_client.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseArray.h>
#include <niryo_one_msgs/RobotMoveAction.h>
#include <niryo_one_msgs/SetInt.h>
#include <vector>

class Picking {
  public:
    using NiryoClient =
        actionlib::SimpleActionClient<niryo_one_msgs::RobotMoveAction>;
    using NiryoPose = std::pair<geometry_msgs::Point, niryo_one_msgs::RPY>;
    struct EndEffectorPosition {
        NiryoPose pose;
        bool open;
    };
    Picking();
    void connectToRobot(ros::NodeHandle&);
    static EndEffectorPosition computePreGrasp(NiryoPose);
    static EndEffectorPosition computeGrasp(NiryoPose);
    static EndEffectorPosition Close(NiryoPose);
    static EndEffectorPosition PostGrasp(NiryoPose);
    static EndEffectorPosition PreFinal();
    static EndEffectorPosition Final();
    static EndEffectorPosition Rest();
    void setGripper(ros::NodeHandle&);
    void moveToPosition(const EndEffectorPosition &);
    void moveJoints(const std::vector<double> &);
    void callback(geometry_msgs::PoseArray);
    void moveArm(const NiryoPose&);

  private:
    NiryoClient robot;
    //ros::NodeHandle node;
    template <typename T>
    void establish_connection(const T &, const std::string &);
    static niryo_one_msgs::RPY rotation();
    static geometry_msgs::Point point(float, float, float);
    static EndEffectorPosition pose(const NiryoPose &, bool);
    bool GripperAperture(bool);
    bool MoveEEF(const NiryoPose &);
    std::tuple<double, double, double>
    converQuaternionToRPY(const geometry_msgs::Quaternion &);
    std::vector<Picking::NiryoPose>
    convertToNiryo(const geometry_msgs::PoseArray &);
};
#endif
