#include <opencv2/features2d.hpp>
#include <vector>

#include "match.hpp"
#include "scene.hpp"
#include "target.hpp"

static constexpr uint QUEUE = 50;
static constexpr uint RATE = 10;
static constexpr float RATIO = 0.7;
static constexpr uint MIN_RANSAC = 5;
static constexpr uint MIN_MATCHES = 10;
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
    return targets;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "publish_pose");
    ros::NodeHandle nh;
    std::vector<Target> targets = read_targets();
    Scene scene;
    // std::cout << "inside the main function" << std::endl;
    ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", QUEUE,
                                       &Scene::callback, &scene);
    // std::filesystem::path first_arg =
    //"/home/fabian/Documents/work/realsense/data/2021-03-12-18-50/"
    //"color_imgs/200.png";
    // std::filesystem::path second_arg =
    //"/home/fabian/Documents/work/realsense/data/2021-03-12-18-50/"
    //"depth_imgs/200.png";
    // scene.deserialize(first_arg, second_arg);
    ros::Rate rate(RATE);
    Match matcher(RATIO);
    const auto sift = cv::SIFT::create(0, 3, 0.04, 10, 1.6, CV_8U);
    while (ros::ok()) {
        rate.sleep();
        ros::spinOnce();
        if (scene.img().empty()) {
            ROS_WARN_STREAM("NO data arrived");
            continue;
        }
        scene.estimateFeatures(sift);
        std::vector<cv::DMatch> matches = matcher.matchDescriptors(
            targets[0].descriptors(), scene.descriptors());
        if (matches.size() < MIN_MATCHES) {
            ROS_WARN_STREAM("Found less than 10 matches\n");
            continue;
        }
        Match::drawMatches(targets[0].img(), targets[0].kps(), scene.img(),
                           scene.kps(), matches);
        const auto [est_ref_points, est_scene_points] =
            Match::corresponding3dPoints(matches, targets[0].points3d(),
                                         scene.points3d());
        cv::Mat inliers, transform;
        cv::estimateAffine3D(est_ref_points, est_scene_points, transform,
                             inliers, RANSAC_THRESHOLD);
        if (static_cast<uint>(cv::sum(inliers)[0]) <= MIN_RANSAC) {
            ROS_WARN_STREAM("Could not find the minimum number of matches\n");
            continue;
        }
        cv::Mat test = Match::averagePosition(est_scene_points, inliers);
    }
    sub.shutdown();
    return 0;
}
