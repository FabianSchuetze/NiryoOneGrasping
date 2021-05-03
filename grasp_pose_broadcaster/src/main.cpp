#include "grasp_pose_broadcaster.hpp"
#include <utils/utils.hpp>

using param = std::pair<std::string, std::string>;
constexpr std::size_t QUEUE(10);

int main(int argc, char **argv) {
    ros::init(argc, argv, "grasp_broadcaster");
    ros::NodeHandle nh;
    param incoming_poses("grasp_pose_broadcaster/estimated_poses", "");
    param outgoing_poses("grasp_pose_broadcaster/outgoing_poses", "");
    utils::readParameters(nh, incoming_poses, outgoing_poses);
    GraspPoseBroadcaster interaction(nh, outgoing_poses.second, incoming_poses.second);
    ros::Subscriber sub =
        nh.subscribe(incoming_poses.second, QUEUE,
                     &GraspPoseBroadcaster::callback, &interaction);
    ros::spin();
    sub.shutdown();
    return 0;
};
