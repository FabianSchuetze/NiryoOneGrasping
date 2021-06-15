#include <Eigen/Geometry>
#include <filesystem>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <object_pose/positions.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/features2d.hpp>
#include <pcl_ros/transforms.h>
#include <utils/utils.hpp>
#include <vector>

#include "match.hpp"
#include "scene.hpp"
#include "target.hpp"

using namespace PoseEstimation;
using param = std::pair<std::string, std::string>;

static constexpr uint QUEUE = 50;
static constexpr uint RATE = 10;
static constexpr float RATIO = 0.7;
static constexpr uint MIN_RANSAC = 5;
static constexpr uint MIN_MATCHES = 10;
static constexpr float RANSAC_THRESHOLD = 0.009;

// TODO: Make this part of the utility
// std::string shortName(const std::string &input_name,
// const std::string &extension) {
// const std::string fn = std::filesystem::path(input_name).filename();
// std::string delimiter = "_";
// std::string token = fn.substr(0, fn.find(delimiter));
//// ROS_WARN_STREAM("Incoming: " << input_name << ", "
////<< "token: " << token);
// return token + extension;
//}

std::vector<Target> readTargets(const std::filesystem::path &path) {
    std::vector<Target> targets;
    std::string line, img;
    ROS_INFO_STREAM("Trying to open file " << path);
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Files can't be opened");
    }
    while (std::getline(file, line)) {
        std::filesystem::path model_description(line);
        std::getline(file, img);
        std::filesystem::path img_path(img);
        targets.emplace_back(model_description, img_path);
    }
    return targets;
    // ROS_WARN_STREAM("The file is: " << line);
    // o3d::geometry::TriangleMesh mesh;
    // bool success = o3d::io::ReadTriangleMesh(line, mesh);
    // if (!success) {
    // throw std::runtime_error("Could not read mesh file");
    //}
    // meshes.emplace_back(std::move(line), std::move(mesh));
    //}
}
// std::vector<Target> read_targets() {
// std::filesystem::path model_description(
//"/home/fabian/Documents/work/transforms/src/pose_estimation/data/"
//"teebox_features_NEW2.yml");
// std::filesystem::path img_path(
//"/home/fabian/Documents/work/realsense/data/2021-03-06-17-01/"
//"color_imgs/125.png");
// std::vector<Target> targets;
// targets.emplace_back(model_description, img_path);
// std::filesystem::path model_description2(
//"/home/fabian/Documents/work/transforms/src/pose_estimation/data/"
//"ibuprofen.yml");
// std::filesystem::path img_path2(
//"/home/fabian/Documents/work/realsense/data/2021-03-22-15-08/"
//"color_imgs/250.png");
// targets.emplace_back(model_description2, img_path2);
// return targets;
//}

bool obtainedFeatures(Scene &scene, const cv::Ptr<cv::SIFT> &sift) {
    if (scene.img().empty()) {
        ROS_WARN_STREAM("NO data arrived");
        return false;
    }
    scene.estimateFeatures(sift);
    return true;
}

bool graspPose(const Target &t, const Scene &scene,
               const std::vector<cv::DMatch> &matches,
               geometry_msgs::Pose &pose) {
    Match::drawMatches(t.img(), t.kps(), scene.img(), scene.kps(), matches);
    const auto [est_ref_points, est_scene_points] =
        Match::corresponding3dPoints(matches, t.points3d(), scene.points3d());
    cv::Mat inliers, transform;
    cv::estimateAffine3D(est_ref_points, est_scene_points, transform, inliers,
                         RANSAC_THRESHOLD);
    if (static_cast<uint>(cv::sum(inliers)[0]) <= MIN_RANSAC) {
        ROS_WARN_STREAM("Could not find the minimum number of matches\n");
        return false;
    }
    cv::Mat grasp_point = Match::averagePosition(est_scene_points, inliers);
    Eigen::Matrix<double, 3, 4> eigen_mat;
    cv::cv2eigen(transform, eigen_mat);
    Eigen::Isometry3d eigen_trans(Eigen::Matrix4d::Identity(4, 4));
    eigen_trans.linear() = eigen_mat.block<3, 3>(0, 0);
    eigen_trans.translation() = eigen_mat.block<3, 1>(0, 3);
    auto [roll, pitch, yaw] = utils::RPY(eigen_trans);
    pose.position.x = grasp_point.at<float>(0, 0);
    pose.position.y = grasp_point.at<float>(0, 1);
    pose.position.z = grasp_point.at<float>(0, 2);
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    ROS_WARN_STREAM("Detected Pose: " << pose.position.x << ", "
                                      << pose.position.y << ", "
                                      << pose.position.z << ", " << yaw;);
    return true;
}

std::vector<std::pair<std::vector<cv::DMatch>, Target>>
matchImages(const std::vector<Target> &targets, const Scene &scene,
            const Match &matcher) {
    std::vector<std::pair<std::vector<cv::DMatch>, Target>> matches;
    for (const Target &t : targets) {
        const auto match =
            matcher.matchDescriptors(t.descriptors(), scene.descriptors());
        if (match.size() > MIN_MATCHES) {
            matches.push_back({match, t});
        }
    }
    return matches;
}

void broadcastFrames(
    const std::vector<std::pair<std::string, geometry_msgs::Pose>> &named_poses,
    std::size_t &iteration, ros::Publisher &pub) {
    object_pose::positions positions;
    std_msgs::Header header;
    header.frame_id = "base_link";
    header.seq = iteration;
    header.stamp = ros::Time::now();
    positions.poses.header = header;
    for (const auto &[name, pose] : named_poses) {
        positions.poses.poses.push_back(pose);
        positions.objects.push_back(utils::shortName(name, ""));
    }
    pub.publish(positions);
    ++iteration;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "publish_pose");
    ros::NodeHandle nh;
    param mesh("visual_pose/input_files", "");
    param estimated_poses("visual_pose/estimated_poses", "");
    utils::readParameters(nh, mesh, estimated_poses);
    std::vector<Target> targets = readTargets(mesh.second);
    Scene scene;
    ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", QUEUE,
                                       &Scene::callback, &scene);
    ros::Publisher pub =
        nh.advertise<object_pose::positions>(estimated_poses.second, 1);
    ros::Rate rate(RATE);
    Match matcher(RATIO);
    const auto sift = cv::SIFT::create(0, 3, 0.04, 10, 1.6, CV_8U);
    std::size_t iteration(0);
    while (ros::ok()) {
        rate.sleep();
        ros::spinOnce();
        if (!obtainedFeatures(scene, sift)) {
            continue;
        }
        const auto matches = matchImages(targets, scene, matcher);
        if (matches.empty()) {
            ROS_DEBUG_STREAM("Did not find a match with any target");
            continue;
        }
        std::vector<std::pair<std::string, geometry_msgs::Pose>> named_poses;
        for (auto [match, t] : matches) {
            geometry_msgs::Pose pose;
            if (graspPose(t, scene, match, pose)) {
                named_poses.emplace_back(t.name(), pose);
            } else {
                ROS_DEBUG_STREAM("Cannot define grasp location");
            }
        }
        if (!named_poses.empty()) {
            broadcastFrames(named_poses, iteration, pub);
        }
    }
    sub.shutdown();
    return 0;
}
