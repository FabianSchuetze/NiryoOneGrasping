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
    matcher.knnMatch(target_descriptors, scene_descriptors, knn_matches, 2);
    return RatioTest(std::move(knn_matches));
}

inline void copy(size_t source_idx, size_t target_idx, const cv::Mat &source,
                 cv::Mat &target) {
    target.row(target_idx) = source.row(source_idx);
}

std::pair<cv::Mat, cv::Mat>
Match::corresponding3dPoints(const matches &match_vec,
                             const cv::Mat &ref_pop_3d_points,
                             const cv::Mat &scene_pop_3d_points) {
    cv::Mat scene_3d_points = cv::Mat(match_vec.size(), 3, CV_32FC1);
    cv::Mat ref_3d_points = cv::Mat(match_vec.size(), 3, CV_32FC1);
    size_t idx(0);
    for (const cv::DMatch &match : match_vec) {
        copy(match.trainIdx, idx, scene_pop_3d_points, scene_3d_points);
        copy(match.queryIdx, idx, ref_pop_3d_points, ref_3d_points);
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
    cv::drawMatches(target_img, kps_target, scene_img, kps_scene, match_vec,
                    img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                    std::vector<char>(),
                    cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    cv::imshow("Good Matches", img_matches);
    cv::waitKey();
}
