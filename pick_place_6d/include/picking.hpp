#ifndef picking_hpp
#define picking_hpp
#include <Eigen/Geometry>
#include <actionlib/client/action_client.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseArray.h>
#include <niryo_one_msgs/RobotMoveAction.h>
#include <niryo_one_msgs/SetInt.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <vector>

class Picking {
//<<<<<<< HEAD
public:
  using NiryoClient =
      actionlib::SimpleActionClient<niryo_one_msgs::RobotMoveAction>;
  using NiryoPose = std::pair<geometry_msgs::Point, niryo_one_msgs::RPY>;
  struct EndEffectorPosition {
    NiryoPose pose;
    bool open;
  };
  Picking();
  void connectToRobot(ros::NodeHandle &);
  EndEffectorPosition PreGrasp_orientate(const Eigen::Isometry3d &);
  EndEffectorPosition PreGrasp_descend(const Eigen::Isometry3d &);
  EndEffectorPosition computeGrasp(const Eigen::Isometry3d &);
  EndEffectorPosition Close(const Eigen::Isometry3d &);
  EndEffectorPosition PostGrasp(const Eigen::Isometry3d &);
  EndEffectorPosition FinalPositions(double x, double y, double z, bool,
                                     std::string);
  EndEffectorPosition StartingPose1();
  EndEffectorPosition StartingPose();
  void setGripper(ros::NodeHandle &);
  void moveToPosition(const EndEffectorPosition &);
   void moveJoints(const std::vector<double> &);
  void callback(const geometry_msgs::PoseArray &);
  void moveArm(const geometry_msgs::Pose &);
//=======
  //public:
    //using NiryoClient =
        //actionlib::SimpleActionClient<niryo_one_msgs::RobotMoveAction>;
    //using NiryoPose = std::pair<geometry_msgs::Point, niryo_one_msgs::RPY>;
    //struct EndEffectorPosition {
        //NiryoPose pose;
        //bool open;
    //};
    //Picking();
    //void connectToRobot(ros::NodeHandle &);
    //EndEffectorPosition PreGrasp_orientate(const Eigen::Isometry3d &);
    //EndEffectorPosition PreGrasp_descend(const Eigen::Isometry3d &);
    //EndEffectorPosition computeGrasp(const Eigen::Isometry3d &);
    //EndEffectorPosition Close(const Eigen::Isometry3d &);
    //EndEffectorPosition PostGrasp(const Eigen::Isometry3d &);
    //EndEffectorPosition FinalPositions(double x, double y, double z, bool, std::string);
    //void setGripper(ros::NodeHandle &);
    //void moveToPosition(const EndEffectorPosition &);
    //void moveJoints(const std::vector<double> &);
    //void callback(const geometry_msgs::PoseArray &);
    //void moveArm(const geometry_msgs::Pose &);
//>>>>>>> 49c0d8e35be1725cd819dd3e11767ca2d6bc7dfa

private:
  tf2_ros::StaticTransformBroadcaster br;
  NiryoClient robot;
  template <typename T>
  void establish_connection(const T &, const std::string &);
  static EndEffectorPosition pose(const NiryoPose &, bool);
  bool GripperAperture(bool);
  bool MoveEEF(const NiryoPose &);
  std::vector<geometry_msgs::Pose> check(const geometry_msgs::PoseArray &);
  static NiryoPose convertToNiryo(const Eigen::Isometry3d &,
                                  const std::string &);
  void sendTransform(const Eigen::Isometry3d &, std::string);
};
#endif
