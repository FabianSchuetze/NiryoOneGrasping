#include <opencv2/features2d.hpp>
#include <vector>

#include "match.hpp"
#include "scene.hpp"
#include "target.hpp"

constexpr uint QUEUE = 5;
constexpr uint RATE = 10;

std::vector<Target> read_targets() {
    std::filesystem::path model_description(
        "/home/fabian/Documents/work/transforms/src/pose_estimation/data/"
        "teebox_features.yml");
    std::filesystem::path img_path(
        "/home/fabian/Documents/work/realsense/data/2021-03-04-17-39/"
        "color_imgs/125.png");
    // Target targets(model_description, img_path);
    std::vector<Target> targets;
    targets.emplace_back(Target(model_description, img_path));
    return targets;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "publish_pose");
    ros::NodeHandle nh;
    std::vector<Target> targets = read_targets();
    Scene scene;
    std::cout << "inside the main function" << std::endl;
    ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", QUEUE,
                                       &Scene::callback, &scene);
    ros::Rate rate(RATE);
    Match matcher(0.75);
    while (ros::ok()) {
        ros::spin();
        auto sift = cv::SIFT::create(100, 3, 0.04, 10, 1.6, CV_8U);
        scene.estimateFeatures(sift);
        std::vector<cv::DMatch> matches = matcher.matchDescriptors(
            targets[0].descriptors(), scene.descriptors());
        Match::drawMaches(scene.img(), scene.kps(), targets[0].img(),
                          targets[0].kps(), matches);
        const auto [est_scene_points, est_target_points] =
            matcher.corresponding3dPoints(matches, scene.points3d(),
                                          targets[0].points3d());
        cv::Mat inliers;
        cv::estimateAffine3D(est_target_points, est_scene_points, cv::Mat(),
                             inliers);

        // cv::Ptr<cv_SIFT> sift = cv::SIFT::create(100);
        // cv::SIFT
        // ros::spin();
        rate.sleep();
    }
    sub.shutdown();
    //}
    // ros::spin();
    return 0;
    // std::vector<Targets> targets;
    // Scene current;
    // matches = match(targets, current);
}
