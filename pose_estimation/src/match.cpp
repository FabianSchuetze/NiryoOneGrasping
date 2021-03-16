#include "match.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>

Match::matches Match::RatioTest(std::vector<matches> &&match_vec) const {
    matches good_matches;
    for (const matches &match : match_vec) {
        const cv::DMatch &m(match[0]), n(match[1]);
        if (m.distance < ratio * n.distance) {
            good_matches.push_back(m);
        }
    }
    return good_matches;
}

Match::matches Match::matchDescriptors(const cv::Mat &target_descriptors,
                                       const cv::Mat &scene_descriptors) {
    std::vector<std::vector<cv::DMatch>> knn_matches;
    matcher.knnMatch(scene_descriptors, target_descriptors, knn_matches, 2);
    return RatioTest(std::move(knn_matches));
}

inline void copy(size_t source_idx, size_t target_idx, const cv::Mat &source,
                 cv::Mat &target) {
    source.row(source_idx).copyTo(target.row(target_idx));
}

std::pair<cv::Mat, cv::Mat>
Match::corresponding3dPoints(const matches &match_vec,
                             const cv::Mat &ref_pop_3d_points,
                             const cv::Mat &scene_pop_3d_points) {
    cv::Mat scene_3d_points = cv::Mat(match_vec.size(), 3, CV_32FC1);
    cv::Mat ref_3d_points = cv::Mat(match_vec.size(), 3, CV_32FC1);
    size_t idx(0);
    for (const cv::DMatch &match : match_vec) {
        copy(match.trainIdx, idx, ref_pop_3d_points, ref_3d_points);
        copy(match.queryIdx, idx, scene_pop_3d_points, scene_3d_points);
        ++idx;
    }
    return {ref_3d_points, scene_3d_points};
}

void Match::drawMatches(const cv::Mat &target_img,
                        const std::vector<cv::KeyPoint> &kps_target,
                        const cv::Mat &scene_img,
                        const std::vector<cv::KeyPoint> &kps_scene,
                        const std::vector<cv::DMatch> &match_vec) {
    if ((scene_img.rows != target_img.rows) ||
        (scene_img.cols != target_img.cols)) {
        std::cerr << "Images don't have the same size, target: "
                  << target_img.rows << ", " << target_img.cols
                  << " vs, scene: " << scene_img.rows << ", " << scene_img.cols
                  << std::endl;
        throw std::runtime_error("");
    }
    cv::Mat img_matches;
    cv::drawMatches(scene_img, kps_scene, target_img, kps_target, match_vec,
                    img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                    std::vector<char>(),
                    cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    // Not GOOD: compiler has to reaorder stack
    cv::imshow("Good Matches", img_matches);
    cv::waitKey();
}

cv::Mat Match::averagePosition(const cv::Mat &population,
                                const cv::Mat &inliers) {
    float rows = static_cast<float>(cv::sum(inliers)[0]);
    cv::Mat matches = cv::Mat(1, 3, CV_32FC1, 0.0);
    for (int i = 0; i < inliers.rows; ++i) {
        if (inliers.at<uint8_t>(i) > 0) {
            matches.row(0) += population.row(i) / rows;
        }
    }
    return matches;
}
