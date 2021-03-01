// C++
#include <iostream>
#include <memory>
// OpenCV
#include <kalman.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/utils/filesystem.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
// PnP Tutorial


#include "Mesh.h"
#include "Model.h"
#include "ModelRegistration.h"
#include "PnPProblem.h"
#include "RobustMatcher.h"
#include "Utils.h"
#include "Stream.hpp"
#include "VideoStream.hpp"
#include "RSStream.hpp"
#include "ros_interaction.hpp"
#include "detection_parse_parameters.hpp"
#include <opencv2/core/eigen.hpp>

namespace fs = std::filesystem;
void help();

const std::tuple<CameraParameters, DetectionParameters, KalmanFilterParameters>
parse_input(int argc, char **argv) {
    std::string root(
        "/home/fabian/Documents/work/transforms/src/real_time_pose_estimation/"
        "Data/");
    fs::path parameter_location(root + "parameters.yml");
    fs::path camera_location(root + "camera_parameters.yml");
    fs::path kalman_location(root + "kalman_parameters.yml");
    const DetectionParameters paras =
        readDetectionParameters(parameter_location);
    displayParameters(paras);
    const CameraParameters camera = readCameraParameters(camera_location);
    displayCamera(camera);
    const auto kalman = readKalmanFilterParameters(kalman_location);
    displayKalman(kalman);
    return {camera, paras, kalman};
}

void match_model_and_frame(const DetectionParameters &paras,
                           RobustMatcher &rmatcher,
                           const cv::Mat &descriptors_model,
                           const std::vector<cv::KeyPoint> &keypoints_model,
                           const cv::Mat &frame,
                           std::vector<cv::KeyPoint> &keypoints_scene,
                           std::vector<cv::DMatch> &good_matches) {
    if (paras.fast_match) {
        rmatcher.fastRobustMatch(frame, good_matches, keypoints_scene,
                                 descriptors_model, keypoints_model);
    } else {
        rmatcher.robustMatch(frame, good_matches, keypoints_scene,
                             descriptors_model, keypoints_model);
    }

    cv::Mat frame_matching = rmatcher.getImageMatching();
    if (!frame_matching.empty()) {
        cv::imshow("Keypoints matching", frame_matching);
    }
}

void matchedPoints(const std::vector<cv::DMatch> &matches, const Model &model,
                   const std::vector<cv::KeyPoint> &keypoints_scene,
                   std::vector<cv::Point2f> &matched_points2d,
                   std::vector<cv::Point3f> &matched_points3d) {
    const static std::vector<cv::Point3f> points3d = model.get_points3d();
    for (const cv::DMatch &match : matches) {
        cv::Point3f point3d_model = points3d[match.trainIdx];  // 3D point
        cv::Point2f point2d_scene = keypoints_scene[match.queryIdx].pt;
        matched_points3d.push_back(point3d_model);  // add 3D point
        matched_points2d.push_back(point2d_scene);  // add 2D point
    }
}

void drawInliers(const cv::Mat &inliers_idx, const cv::Mat &frame,
                 const std::vector<cv::Point2f> &points2d) {
    const static cv::Scalar blue(255, 0, 0);
    std::vector<cv::Point2f> inliers;
    // -- Step 4: Catch the inliers keypoints to draw
    for (int inliers_index = 0; inliers_index < inliers_idx.rows;
         ++inliers_index) {
        int n = inliers_idx.at<int>(inliers_index);  // i-inlier
        cv::Point2f point2d = points2d[n];           // i-inlier point 2D
        inliers.push_back(point2d);                  // add i-inlier to list
    }
    draw2DPoints(frame, inliers, blue);
}

void Pose(cv::Mat &pose, const PnPProblem &pnp_detection, Eigen::Affine3d &T) {
    cv::Mat translation = pnp_detection.get_t_matrix();
    cv::Mat rotation = pnp_detection.get_R_matrix();
    Eigen::Matrix3d tmp_rotation;
    Eigen::MatrixXd tmp_translation;
    cv::cv2eigen(rotation, tmp_rotation);
    cv::cv2eigen(translation, tmp_translation);
    T.linear() = tmp_rotation;
    T.translation() = tmp_translation;
    convertToPose(pose, translation, rotation);
}

void drawPose(bool good_measurement, PnPProblem& pnp_detection,
              PnPProblem &pnp_detection_est, const cv::Mat &frame_vis,
              const DetectionParameters &paras) {
    static Mesh mesh;                       // instantiate Mesh object
    mesh.load(paras.ply_read_path);  // load an object mesh
    const float l = 5;
    const static cv::Scalar yellow(0, 255, 255);
    const static cv::Scalar green(0, 255, 0);
    std::vector<cv::Point2f> pose_points2d;
    if (!good_measurement || paras.displayFilteredPose) {
        drawObjectMesh(frame_vis, &mesh, &pnp_detection_est,
                       yellow);  // draw estimated pose

        pose_points2d.push_back(pnp_detection_est.backproject3DPoint(
            cv::Point3f(0, 0, 0)));  // axis center
        pose_points2d.push_back(pnp_detection_est.backproject3DPoint(
            cv::Point3f(l, 0, 0)));  // axis x
        pose_points2d.push_back(pnp_detection_est.backproject3DPoint(
            cv::Point3f(0, l, 0)));  // axis y
        pose_points2d.push_back(pnp_detection_est.backproject3DPoint(
            cv::Point3f(0, 0, l)));                      // axis z
        draw3DCoordinateAxes(frame_vis, pose_points2d);  // draw axes
    } else {
        drawObjectMesh(frame_vis, &mesh, &pnp_detection,
                       green);  // draw current pose

        pose_points2d.push_back(pnp_detection.backproject3DPoint(
            cv::Point3f(0, 0, 0)));  // axis center
        pose_points2d.push_back(
            pnp_detection.backproject3DPoint(cv::Point3f(l, 0, 0)));  // axis x
        pose_points2d.push_back(
            pnp_detection.backproject3DPoint(cv::Point3f(0, l, 0)));  // axis y
        pose_points2d.push_back(
            pnp_detection.backproject3DPoint(cv::Point3f(0, 0, l)));  // axis z
        draw3DCoordinateAxes(frame_vis, pose_points2d);  // draw axes
    }
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "broadcast_location");
    ros::NodeHandle node_handle;
    const std::string from("world"), to("camera_link"), object("object");
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener listener(tfBuffer);
    Eigen::Affine3d T_base_camera, T_camera_object;
    ros::Rate rate(50);
    const auto [camera, paras, kalman] = parse_input(argc, argv);
    //// Some basic colors
    const cv::Scalar red(0, 0, 255);
    const cv::Scalar green(0, 255, 0);
    const cv::Scalar yellow(0, 255, 255);

    cv::Mat frameSave;
    int frameCount = 0;

    PnPProblem pnp_detection(camera), pnp_detection_est(camera);

    Model model;                      // instantiate Model object
    model.load(paras.yml_read_path);  // load a 3D textured object model


    RobustMatcher rmatcher;  // instantiate RobustMatcher

    cv::Ptr<cv::FeatureDetector> detector, descriptor;
    createFeatures(paras.featureName, paras.numKeyPoints, detector, descriptor);
    rmatcher.setFeatureDetector(detector);        // set feature detector
    rmatcher.setDescriptorExtractor(descriptor);  // set descriptor extractor
    rmatcher.setDescriptorMatcher(
        createMatcher(paras.featureName, paras.useFLANN));  // set matcher
    rmatcher.setRatio(paras.ratioTest);  // set ratio test parameter
    if (!model.get_trainingImagePath().empty()) {
        cv::Mat trainingImg = cv::imread(model.get_trainingImagePath());
        rmatcher.setTrainingImage(trainingImg);
    }

    cv::KalmanFilter KF;  // instantiate Kalman Filter
    initKalmanFilter(KF, kalman.nStates, kalman.nMeasurements, kalman.nInputs,
                     kalman.dt);  // init function
    cv::Mat pose(6, 1, CV_64FC1);
    pose.setTo(cv::Scalar(0));

    // Get the MODEL INFO
    const cv::Mat descriptors_model = model.get_descriptors();
    const std::vector<cv::KeyPoint> keypoints_model = model.get_keypoints();

    // Create & Open Window
    cv::namedWindow("REAL TIME DEMO", cv::WINDOW_KEEPRATIO);

    if (!paras.saveDirectory.empty()) {
        if (!cv::utils::fs::exists(paras.saveDirectory)) {
            std::cout << "Create directory: " << paras.saveDirectory
                      << std::endl;
            cv::utils::fs::createDirectories(paras.saveDirectory);
        }
    }
    std::unique_ptr<Stream> stream;
    if (paras.stream == "Video") {
        stream = std::make_unique<VideoStream>(paras);
    } else if (paras.stream == "RS") {
        stream = std::make_unique<RSStream>(paras);
    }
    // cv::VideoCapture cap = openVideo(paras);

    // Measure elapsed time
    cv::TickMeter tm;

    cv::Mat frame, frame_vis, frame_matching;
    while (stream->read(frame) && node_handle.ok() &&
           (char)cv::waitKey(30) != 27)  // run til ESC
    {
        tm.reset();
        tm.start();
        frame_vis = frame.clone();  // refresh visualisation frame

        // -- Step 1: Robust matching between model descriptors and scene
        std::vector<cv::DMatch> good_matches;  // obtain 3D points of model
        // DMatch: Class for matching keypoint descriptors
        std::vector<cv::KeyPoint> keypoints_scene;  // 2D points of the scene
        match_model_and_frame(paras, rmatcher, descriptors_model,
                              keypoints_model, frame, keypoints_scene,
                              good_matches);
        // -- Step 2: Find out the 2D/3D correspondences
        std::vector<cv::Point3f> points3d;  // coordinates found in the scene
        std::vector<cv::Point2f> points2d;  // coordinates found in the scene
        matchedPoints(good_matches, model, keypoints_scene, points2d, points3d);
        draw2DPoints(frame_vis, points2d, red);
        if (good_matches.size() < 4) {
            continue;  // minimally have 4 set of points
        }
        cv::Mat inliers_idx;
        // -- Step 3: Estimate the pose using RANSAC approach
        pnp_detection.estimatePoseRANSAC(
            points3d, points2d, paras.pnpMethod, inliers_idx,
            paras.iterationsCount, paras.reprojectionError, paras.confidence);
        drawInliers(inliers_idx, frame, points2d);

        // -- Step 5: Kalman Filter

        // GOOD MEASUREMENT
        bool good_measurement = false;
        if (inliers_idx.rows >= paras.minInliersKalman) {
            // Pose(pose, pnp_detection, T_camera_object);
            good_measurement = true;
            // Get the measured translation
        }
        // update the Kalman filter with good measurements, otherwise with
        // previous valid measurements
        cv::Mat translation(3, 1, CV_64FC1);
        cv::Mat rotation(3, 3, CV_64FC1);
        updateKalmanFilter(KF, pose, translation, rotation);

        // -- Step 6: Set estimated projection matrix
        pnp_detection_est.set_P_matrix(rotation, translation);
        Pose(pose, pnp_detection_est, T_camera_object);

        // -- Step X: Draw pose and coordinate frame

        // FRAME RATE
        // see how much time has elapsed
        drawPose(good_measurement, pnp_detection, pnp_detection_est, frame_vis, paras);
        tm.stop();

        // calculate current FPS
        double fps = 1.0 / tm.getTimeSec();

        drawFPS(frame_vis, fps, yellow);  // frame ratio
        double detection_ratio =
            ((double)inliers_idx.rows / (double)good_matches.size()) * 100;
        drawConfidence(frame_vis, detection_ratio, yellow);

        // -- Step X: Draw some debugging text
        // Draw some debug text
        int inliers_int = inliers_idx.rows;
        int outliers_int = (int)good_matches.size() - inliers_int;
        std::string inliers_str = IntToString(inliers_int);
        std::string outliers_str = IntToString(outliers_int);
        std::string n = IntToString((int)good_matches.size());
        std::string text = "Found " + inliers_str + " of " + n + " matches";
        std::string text2 =
            "Inliers: " + inliers_str + " - Outliers: " + outliers_str;

        drawText(frame_vis, text, green);
        drawText2(frame_vis, text2, red);

        cv::imshow("REAL TIME DEMO", frame_vis);

        if (!paras.saveDirectory.empty()) {
            const int widthSave =
                !frame_matching.empty() ? frame_matching.cols : frame_vis.cols;
            const int heightSave = !frame_matching.empty()
                                       ? frame_matching.rows + frame_vis.rows
                                       : frame_vis.rows;
            frameSave = cv::Mat::zeros(heightSave, widthSave, CV_8UC3);
            if (!frame_matching.empty()) {
                int startX = (int)((widthSave - frame_vis.cols) / 2.0);
                cv::Mat roi = frameSave(
                    cv::Rect(startX, 0, frame_vis.cols, frame_vis.rows));
                frame_vis.copyTo(roi);

                roi = frameSave(cv::Rect(0, frame_vis.rows, frame_matching.cols,
                                         frame_matching.rows));
                frame_matching.copyTo(roi);
            } else {
                frame_vis.copyTo(frameSave);
            }

            std::string saveFilename = cv::format(
                std::string(paras.saveDirectory + "/image_%04d.png").c_str(),
                frameCount);
            cv::imwrite(saveFilename, frameSave);
            frameCount++;
        }
        obtain_transform(from, to, tfBuffer, T_base_camera);
        T_base_camera = T_base_camera * T_camera_object;
        broadcast(T_base_camera, from, object);
        rate.sleep();
    }
    // Close and Destroy Window
    cv::destroyWindow("REAL TIME DEMO");
    std::cout << "GOODBYE ..." << std::endl;
}

/**********************************************************************************************************/
void help() {
    std::cout
        << "------------------------------------------------------------------"
           "--------"
        << std::endl
        << "This program shows how to detect an object given its 3D textured "
           "model. You can choose to "
        << "use a recorded video or the webcam." << std::endl
        << "Usage:" << std::endl
        << "./cpp-tutorial-pnp_detection -help" << std::endl
        << "Keys:" << std::endl
        << "'esc' - to quit." << std::endl
        << "------------------------------------------------------------------"
           "--------"
        << std::endl
        << std::endl;
}

/**********************************************************************************************************/
