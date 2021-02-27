// C++
#include <iostream>
// OpenCV
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
#include "detection_parse_parameters.hpp"

/**  GLOBAL VARIABLES  **/

namespace fs = std::filesystem;
/**  Functions headers  **/
void help();
void initKalmanFilter(cv::KalmanFilter &KF, int nStates, int nMeasurements,
                      int nInputs, double dt);
void predictKalmanFilter(cv::KalmanFilter &KF, cv::Mat &translation_predicted,
                         cv::Mat &rotation_predicted);
void updateKalmanFilter(cv::KalmanFilter &KF, cv::Mat &measurements,
                        cv::Mat &translation_estimated,
                        cv::Mat &rotation_estimated);
void fillMeasurements(cv::Mat &measurements,
                      const cv::Mat &translation_measured,
                      const cv::Mat &rotation_measured);

/**  Main program  **/
const std::pair<CameraParameters, DetectionParameters> parse_input(
    int argc, char **argv) {
    std::string root(
        "/home/fabian/Documents/work/transforms/src/real_time_pose_estimation/"
        "Data/");
    fs::path parameter_location(root + "parameters.yml");
    fs::path camera_location(root + "camera_parameters.yml");
    const DetectionParameters paras =
        readDetectionParameters(parameter_location);
    displayParameters(paras);
    const CameraParameters camera = readCameraParameters(camera_location);
    displayCamera(camera);
    return {camera, paras};
}

cv::VideoCapture openVideo(const DetectionParameters &paras) {
    cv::VideoCapture cap;             // instantiate VideoCapture
    cap.open(paras.video_read_path);  // open a recorded video
    if (!cap.isOpened())              // check if we succeeded
    {
        std::cout << "Could not open the camera device" << std::endl;
        throw std::runtime_error("");
    }
    return cap;
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

int main(int argc, char *argv[]) {
    const auto [camera, paras] = parse_input(argc, argv);
    //// Some basic colors
    cv::Scalar red(0, 0, 255);
    cv::Scalar green(0, 255, 0);
    cv::Scalar blue(255, 0, 0);
    cv::Scalar yellow(0, 255, 255);

    cv::Mat frameSave;
    int frameCount = 0;

    PnPProblem pnp_detection(camera), pnp_detection_est(camera);

    Model model;                      // instantiate Model object
    model.load(paras.yml_read_path);  // load a 3D textured object model

    Mesh mesh;                       // instantiate Mesh object
    mesh.load(paras.ply_read_path);  // load an object mesh

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

    cv::KalmanFilter KF;    // instantiate Kalman Filter
    int nStates = 18;       // the number of states
    int nMeasurements = 6;  // the number of measured states
    int nInputs = 0;        // the number of control actions
    double dt = 0.125;      // time between measurements (1/FPS)

    initKalmanFilter(KF, nStates, nMeasurements, nInputs, dt);  // init function
    cv::Mat measurements(nMeasurements, 1, CV_64FC1);
    measurements.setTo(cv::Scalar(0));

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
    cv::VideoCapture cap = openVideo(paras);

    // Measure elapsed time
    cv::TickMeter tm;

    cv::Mat frame, frame_vis, frame_matching;
    while (cap.read(frame) && (char)cv::waitKey(30) != 27)  // run til ESC
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
        std::vector<cv::Point3f>
            points3d_model;  // container for the model 3D
                             // coordinates found in the scene
        std::vector<cv::Point2f>
            points2d_scene;  // container for the model 2D
                             // coordinates found in the scene
        matchedPoints(good_matches, model, keypoints_scene, points2d_scene,
                      points3d_model);

        // Draw outliers
        draw2DPoints(frame_vis, points2d_scene, red);

        cv::Mat inliers_idx;
        std::vector<cv::Point2f> list_points2d_inliers;

        // Instantiate estimated translation and rotation
        bool good_measurement = false;

        if (good_matches.size() >= 4)  // OpenCV requires solvePnPRANSAC to
                                       // minimally have 4 set of points
        {
            // -- Step 3: Estimate the pose using RANSAC approach
            pnp_detection.estimatePoseRANSAC(
                points3d_model, points2d_scene, paras.pnpMethod, inliers_idx,
                paras.iterationsCount, paras.reprojectionError,
                paras.confidence);

            // -- Step 4: Catch the inliers keypoints to draw
            for (int inliers_index = 0; inliers_index < inliers_idx.rows;
                 ++inliers_index) {
                int n = inliers_idx.at<int>(inliers_index);  // i-inlier
                cv::Point2f point2d = points2d_scene[n];  // i-inlier point 2D
                list_points2d_inliers.push_back(
                    point2d);  // add i-inlier to list
            }

            // Draw inliers points 2D
            draw2DPoints(frame_vis, list_points2d_inliers, blue);

            // -- Step 5: Kalman Filter

            // GOOD MEASUREMENT
            if (inliers_idx.rows >= paras.minInliersKalman) {
                // Get the measured translation
                cv::Mat translation_measured = pnp_detection.get_t_matrix();

                // Get the measured rotation
                cv::Mat rotation_measured = pnp_detection.get_R_matrix();

                // fill the measurements vector
                fillMeasurements(measurements, translation_measured,
                                 rotation_measured);
                good_measurement = true;
            }

            // update the Kalman filter with good measurements, otherwise with
            // previous valid measurements
            cv::Mat translation_estimated(3, 1, CV_64FC1);
            cv::Mat rotation_estimated(3, 3, CV_64FC1);
            updateKalmanFilter(KF, measurements, translation_estimated,
                               rotation_estimated);

            // -- Step 6: Set estimated projection matrix
            pnp_detection_est.set_P_matrix(rotation_estimated,
                                           translation_estimated);
        }

        // -- Step X: Draw pose and coordinate frame
        float l = 5;
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
            pose_points2d.push_back(pnp_detection.backproject3DPoint(
                cv::Point3f(l, 0, 0)));  // axis x
            pose_points2d.push_back(pnp_detection.backproject3DPoint(
                cv::Point3f(0, l, 0)));  // axis y
            pose_points2d.push_back(pnp_detection.backproject3DPoint(
                cv::Point3f(0, 0, l)));                      // axis z
            draw3DCoordinateAxes(frame_vis, pose_points2d);  // draw axes
        }

        // FRAME RATE
        // see how much time has elapsed
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
void initKalmanFilter(cv::KalmanFilter &KF, int nStates, int nMeasurements,
                      int nInputs, double dt) {
    KF.init(nStates, nMeasurements, nInputs, CV_64F);  // init Kalman Filter

    setIdentity(KF.processNoiseCov,
                cv::Scalar::all(1e-5));  // set process noise
    setIdentity(KF.measurementNoiseCov,
                cv::Scalar::all(1e-2));                // set measurement noise
    setIdentity(KF.errorCovPost, cv::Scalar::all(1));  // error covariance

    /** DYNAMIC MODEL **/

    //  [1 0 0 dt  0  0 dt2   0   0 0 0 0  0  0  0   0   0   0]
    //  [0 1 0  0 dt  0   0 dt2   0 0 0 0  0  0  0   0   0   0]
    //  [0 0 1  0  0 dt   0   0 dt2 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  1  0  0  dt   0   0 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  1  0   0  dt   0 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  0  1   0   0  dt 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  0  0   1   0   0 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  0  0   0   1   0 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  0  0   0   0   1 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  0  0   0   0   0 1 0 0 dt  0  0 dt2   0   0]
    //  [0 0 0  0  0  0   0   0   0 0 1 0  0 dt  0   0 dt2   0]
    //  [0 0 0  0  0  0   0   0   0 0 0 1  0  0 dt   0   0 dt2]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  1  0  0  dt   0   0]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  0  1  0   0  dt   0]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  1   0   0  dt]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   1   0   0]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   1   0]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   0   1]

    // position
    KF.transitionMatrix.at<double>(0, 3) = dt;
    KF.transitionMatrix.at<double>(1, 4) = dt;
    KF.transitionMatrix.at<double>(2, 5) = dt;
    KF.transitionMatrix.at<double>(3, 6) = dt;
    KF.transitionMatrix.at<double>(4, 7) = dt;
    KF.transitionMatrix.at<double>(5, 8) = dt;
    KF.transitionMatrix.at<double>(0, 6) = 0.5 * pow(dt, 2);
    KF.transitionMatrix.at<double>(1, 7) = 0.5 * pow(dt, 2);
    KF.transitionMatrix.at<double>(2, 8) = 0.5 * pow(dt, 2);

    // orientation
    KF.transitionMatrix.at<double>(9, 12) = dt;
    KF.transitionMatrix.at<double>(10, 13) = dt;
    KF.transitionMatrix.at<double>(11, 14) = dt;
    KF.transitionMatrix.at<double>(12, 15) = dt;
    KF.transitionMatrix.at<double>(13, 16) = dt;
    KF.transitionMatrix.at<double>(14, 17) = dt;
    KF.transitionMatrix.at<double>(9, 15) = 0.5 * pow(dt, 2);
    KF.transitionMatrix.at<double>(10, 16) = 0.5 * pow(dt, 2);
    KF.transitionMatrix.at<double>(11, 17) = 0.5 * pow(dt, 2);

    /** MEASUREMENT MODEL **/

    //  [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
    //  [0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
    //  [0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
    //  [0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0]
    //  [0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0]
    //  [0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0]

    KF.measurementMatrix.at<double>(0, 0) = 1;   // x
    KF.measurementMatrix.at<double>(1, 1) = 1;   // y
    KF.measurementMatrix.at<double>(2, 2) = 1;   // z
    KF.measurementMatrix.at<double>(3, 9) = 1;   // roll
    KF.measurementMatrix.at<double>(4, 10) = 1;  // pitch
    KF.measurementMatrix.at<double>(5, 11) = 1;  // yaw
}

/**********************************************************************************************************/
void updateKalmanFilter(cv::KalmanFilter &KF, cv::Mat &measurement,
                        cv::Mat &translation_estimated,
                        cv::Mat &rotation_estimated) {
    // First predict, to update the internal statePre variable
    // Mat prediction = KF.predict();

    // The "correct" phase that is going to use the predicted value and our
    // measurement
    cv::Mat estimated = KF.correct(measurement);

    // Estimated translation
    translation_estimated.at<double>(0) = estimated.at<double>(0);
    translation_estimated.at<double>(1) = estimated.at<double>(1);
    translation_estimated.at<double>(2) = estimated.at<double>(2);

    // Estimated euler angles
    cv::Mat eulers_estimated(3, 1, CV_64F);
    eulers_estimated.at<double>(0) = estimated.at<double>(9);
    eulers_estimated.at<double>(1) = estimated.at<double>(10);
    eulers_estimated.at<double>(2) = estimated.at<double>(11);

    // Convert estimated quaternion to rotation matrix
    rotation_estimated = euler2rot(eulers_estimated);
}

/**********************************************************************************************************/
void fillMeasurements(cv::Mat &measurements,
                      const cv::Mat &translation_measured,
                      const cv::Mat &rotation_measured) {
    // Convert rotation matrix to euler angles
    cv::Mat measured_eulers(3, 1, CV_64F);
    measured_eulers = rot2euler(rotation_measured);

    // Set measurement to predict
    measurements.at<double>(0) = translation_measured.at<double>(0);  // x
    measurements.at<double>(1) = translation_measured.at<double>(1);  // y
    measurements.at<double>(2) = translation_measured.at<double>(2);  // z
    measurements.at<double>(3) = measured_eulers.at<double>(0);       // roll
    measurements.at<double>(4) = measured_eulers.at<double>(1);       // pitch
    measurements.at<double>(5) = measured_eulers.at<double>(2);       // yaw
}
