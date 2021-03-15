#include <opencv2/features2d.hpp>
#include <vector>

#include "match.hpp"
#include "scene.hpp"
#include "target.hpp"

constexpr uint QUEUE = 5;
constexpr uint RATE = 10;
constexpr float RATIO = 0.7;

std::vector<Target> read_targets() {
    std::filesystem::path model_description(
        "/home/fabian/Documents/work/transforms/src/pose_estimation/data/"
        "teebox_features.yml");
    std::filesystem::path img_path(
        "/home/fabian/Documents/work/realsense/data/2021-03-04-17-39/"
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
    std::cout << "inside the main function" << std::endl;
    // ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points",
    // QUEUE, &Scene::callback, &scene);
    std::filesystem::path first_arg =
        "/home/fabian/Documents/work/realsense/data/2021-03-12-18-50/"
        "color_imgs/200.png";
    std::filesystem::path second_arg =
        "/home/fabian/Documents/work/realsense/data/2021-03-12-18-50/"
        "depth_imgs/200.png";
    scene.deserialize(first_arg, second_arg);
    ros::Rate rate(RATE);
    Match matcher(RATIO);
    while (ros::ok()) {
        std::cout << "Inside the ros function\n";
        // ros::spin();
        const auto sift = cv::SIFT::create(100, 3, 0.04, 10, 1.6, CV_8U);
        scene.estimateFeatures(sift);
        std::vector<cv::DMatch> matches = matcher.matchDescriptors(
            targets[0].descriptors(), scene.descriptors());
        Match::drawMatches(targets[0].img(), targets[0].kps(), scene.img(),
                           scene.kps(), matches);
        const auto [est_scene_points, est_target_points] =
            Match::corresponding3dPoints(matches, targets[0].points3d(),
                    scene.points3d());
        cv::Mat inliers, not_used;
        cv::estimateAffine3D(est_target_points, est_scene_points, not_used,
                             inliers);
        std::cout << inliers << std::endl;
        rate.sleep();
    }
    // sub.shutdown();
    //}
    // ros::spin();
    return 0;
}
