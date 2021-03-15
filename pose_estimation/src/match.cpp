#include "match.hpp"
#include "opencv2/highgui.hpp"

Match::matches Match::RatioTest(std::vector<matches> &&match_vec) {
    matches good_matches;
    for (matches &match : match_vec) {
        cv::DMatch &m(match[0]), n(match[1]);
        if (m.distance < ratio * n.distance) {
            good_matches.push_back(std::move(m));
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
                             const cv::Mat &scene_pop_3d_points,
                             const cv::Mat &ref_pop_3d_points) {
    cv::Mat scene_3d_points = cv::Mat(match_vec.size(), 3, CV_32FC1);
    cv::Mat ref_3d_points = cv::Mat(match_vec.size(), 3, CV_32FC1);
    size_t idx(0);
    for (const cv::DMatch &match : match_vec) {
        copy(idx, match.queryIdx, scene_pop_3d_points, scene_3d_points);
        copy(idx, match.trainIdx, ref_pop_3d_points, ref_3d_points);
        ++idx;
    }
    return {scene_3d_points, ref_3d_points};
}

void Match::drawMaches(const cv::Mat &scene_img,
                         const std::vector<cv::KeyPoint> &kps_scene,
                         const cv::Mat &target_img,
                         const std::vector<cv::KeyPoint> &kps_target,
                         const std::vector<cv::DMatch>& match_vec) {
    cv::Mat img_matches;
    cv::drawMatches(scene_img, kps_scene, target_img, kps_target, match_vec,
                    img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                    std::vector<char>(),
                    cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    //-- Show detected matches
    cv::imshow("Good Matches", img_matches);
    cv::waitKey();
}
