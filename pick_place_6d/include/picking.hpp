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
  public:
    using NiryoClient =
        actionlib::SimpleActionClient<niryo_one_msgs::RobotMoveAction>;
    using NiryoPose = std::pair<geometry_msgs::Point, niryo_one_msgs::RPY>;
    struct EEFPosition {
        NiryoPose pose;
        bool open;
    };
    Picking();
    void connectToRobot(ros::NodeHandle &);
    EEFPosition PreGrasp_orientate(const Eigen::Isometry3d &);
    EEFPosition PreGrasp_descend(const Eigen::Isometry3d &);
    EEFPosition computeGrasp(const Eigen::Isometry3d &);
    EEFPosition Close(const Eigen::Isometry3d &);
    EEFPosition PostGrasp(const Eigen::Isometry3d &);
    static EEFPosition Position(double x, double y, double z, double roll,
                                double pitch, double yaw, bool);
    static void setGripper(ros::NodeHandle &);
    void moveToPosition(const EEFPosition &);
    void moveJoints(const std::vector<double> &);
    void callback(const geometry_msgs::PoseArray &);
    void moveArm(const geometry_msgs::Pose &);

  private:
    tf2_ros::StaticTransformBroadcaster br;
    NiryoClient robot;
    static void establish_connection(const NiryoClient &, const std::string &);
    static EEFPosition pose(const NiryoPose &, bool);
    bool GripperAperture(bool);
    bool MoveEEF(const NiryoPose &);
    static std::vector<geometry_msgs::Pose>
    check(const geometry_msgs::PoseArray &);
    static NiryoPose convert(const Eigen::Isometry3d &, const std::string &);
    void sendTransform(const Eigen::Isometry3d &, std::string);
};
#endif
