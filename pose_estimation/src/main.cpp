#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <opencv2/features2d.hpp>
#include <vector>

#include "match.hpp"
#include "scene.hpp"
#include "target.hpp"

using namespace PoseEstimation;

static constexpr uint QUEUE = 50;
static constexpr uint RATE = 10;
static constexpr float RATIO = 0.7;
static constexpr uint MIN_RANSAC = 5;
static constexpr uint MIN_MATCHES = 8;
static constexpr float RANSAC_THRESHOLD = 0.009;

std::vector<Target> read_targets() {
    std::filesystem::path model_description(
        "/home/fabian/Documents/work/transforms/src/pose_estimation/data/"
        "teebox_features.yml");
    std::filesystem::path img_path(
        "/home/fabian/Documents/work/realsense/data/2021-03-06-17-01/"
        "color_imgs/125.png");
    std::vector<Target> targets;
    targets.emplace_back(model_description, img_path);
    std::filesystem::path model_description2(
        "/home/fabian/Documents/work/transforms/src/pose_estimation/data/"
        "ibuprofen.yml");
    std::filesystem::path img_path2(
        "/home/fabian/Documents/work/realsense/data/2021-03-22-15-08/"
        "color_imgs/250.png");
    targets.emplace_back(model_description2, img_path2);
    return targets;
}

bool obtainedFeatures(Scene &scene, const cv::Ptr<cv::SIFT> &sift) {
    if (scene.img().empty()) {
        ROS_WARN_STREAM("NO data arrived");
        return false;
    }
    scene.estimateFeatures(sift);
    return true;
}

bool graspPoint(const Target &t, const Scene &scene,
                const std::vector<cv::DMatch> &matches,
                geometry_msgs::Point &point) {
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
    point.x = grasp_point.at<float>(0, 0);
    point.y = grasp_point.at<float>(0, 1);
    point.z = grasp_point.at<float>(0, 2);
    return true;
}

std::vector<std::pair<std::vector<cv::DMatch>, Target>>
matchImages(const std::vector<Target> &targets, const Scene &scene,
            const Match &matcher) {
    std::vector<std::pair<std::vector<cv::DMatch>, Target>> matches;
    for (const Target &t : targets) {
        const auto match =
            matcher.matchDescriptors(t.descriptors(), scene.descriptors());
        if (match.size() > 10) {
            matches.push_back({match, t});
        }
    }
    return matches;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "publish_pose");
    ros::NodeHandle nh;
    std::vector<Target> targets = read_targets();
    Scene scene;
    ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", QUEUE,
                                       &Scene::callback, &scene);
    ros::Publisher pub =
        nh.advertise<geometry_msgs::Point>("grasp_position", 1);
    ros::Rate rate(RATE);
    Match matcher(RATIO);
    const auto sift = cv::SIFT::create(0, 3, 0.04, 10, 1.6, CV_8U);
    while (ros::ok()) {
        rate.sleep();
        ros::spinOnce();
        if (!obtainedFeatures(scene, sift)) {
            continue;
        }
        const auto matches = matchImages(targets, scene, matcher);
        if (matches.empty()) {
            ROS_WARN_STREAM("Did not find a match with any target");
            continue;
        }
        for (size_t i = 0; i < matches.size(); ++i) {
            const auto [match, t ] = matches[i];
            geometry_msgs::Point point;
            if (graspPoint(t, scene, match, point)) {
                pub.publish(point);
            } else {
                ROS_WARN_STREAM("Cannot define grasp location");
            }
        }
    }
    sub.shutdown();
    return 0;
}
