#include <opencv2/features2d.hpp>
#include <vector>

#include "match.hpp"
#include "scene.hpp"
#include "target.hpp"

static constexpr uint QUEUE = 5;
static constexpr uint RATE = 10;
static constexpr float RATIO = 0.7;
static constexpr uint MIN_MATCHES = 5;
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
        // ros::spin();
        const auto sift = cv::SIFT::create(0, 3, 0.04, 10, 1.6, CV_8U);
        //const auto sift = cv::SIFT::create();
        scene.estimateFeatures(sift);
        std::vector<cv::DMatch> matches = matcher.matchDescriptors(
            targets[0].descriptors(), scene.descriptors());
        Match::drawMatches(targets[0].img(), targets[0].kps(), scene.img(),
                           scene.kps(), matches);
        const auto [est_ref_points, est_scene_points] =
            Match::corresponding3dPoints(matches, targets[0].points3d(),
                    scene.points3d());
        std::cout << "Reference 3d Points\n" << est_ref_points <<std::endl;
        std::cout << "Scene 3d Points\n" << est_scene_points <<std::endl;
        cv::Mat inliers, transform;
        cv::estimateAffine3D(est_ref_points, est_scene_points, transform,
                             inliers, RANSAC_THRESHOLD);
        if (static_cast<uint>(cv::sum(inliers)[0]) > MIN_MATCHES) {
            std::cout << inliers << std::endl;
            std::cout <<  transform << std::endl;
        } else{ 
            std::cerr << "Could not find the minimum number of matches\n";
        }
        rate.sleep();
    }
    // sub.shutdown();
    //}
    // ros::spin();
    return 0;
}
