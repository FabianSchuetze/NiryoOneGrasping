#include "GPDInteraction.hpp"
using param = std::pair<std::string, std::string>;
constexpr std::size_t QUEUE(10);

template <typename... T>
void readParameters(const ros::NodeHandle &nh, T &... args) {
    auto read_parameters = [&](auto &t) {
        nh.getParam(t.first, t.second);
        if (t.second.empty()) {
            ROS_ERROR_STREAM("Rosparam " << t.first << " not identified");
            throw std::runtime_error("Could not read all parameters");
        }
        ROS_WARN_STREAM("The parameters for " << t.first << " is " << t.second);
    };
    (..., read_parameters(args));
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "grasp_broadcaster");
    ros::AsyncSpinner spinner(3);
    spinner.start();
    ros::NodeHandle nh;
    param incoming_poses("gpd_interaction/estimated_poses", "");
    param outgoing_poses("gpd_interaction/outgoing_poses", "");
    readParameters(nh, incoming_poses, outgoing_poses);
    gpd::GPDInteraction interaction(nh, outgoing_poses.second);
    ros::Subscriber sub =
        nh.subscribe(incoming_poses.second, QUEUE,
                     &gpd::GPDInteraction::callback_object_pose, &interaction);
    ros::Subscriber sub2 =
        nh.subscribe("/detect_grasps/clustered_grasps", QUEUE,
                     &gpd::GPDInteraction::callback_grasp_pose, &interaction);
    ros::waitForShutdown();
    return 0;
};
