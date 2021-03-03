#include <iostream>
#include <memory>
// OpenCV
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/utils/filesystem.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
// PnP Tutorial

#include "textured_pnp.hpp"
// clang-format off
#include <opencv2/core/eigen.hpp>
// clang-format on
void help();
namespace fs = std::filesystem;

void convertToPose(cv::Mat &measurements, const cv::Mat &translation_measured,
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

const std::tuple<CameraParameters, DetectionParameters> parse_input(
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

void Pose(cv::Mat &pose, const PnPProblem &pnp, Eigen::Affine3d &T) {
    cv::Mat translation = pnp.get_t_matrix();
    cv::Mat rotation = pnp.get_R_matrix();
    Eigen::Matrix3d tmp_rotation;
    Eigen::MatrixXd tmp_translation;
    cv::cv2eigen(rotation, tmp_rotation);
    cv::cv2eigen(translation, tmp_translation);
    T.linear() = tmp_rotation;
    T.translation() = tmp_translation;
    convertToPose(pose, translation, rotation);
}

void drawPose(bool good_measurement, PnPProblem &pnp_detection,
              const cv::Mat &frame_vis,
              const DetectionParameters &paras) {
    static Mesh mesh;                // instantiate Mesh object
    mesh.load(paras.ply_read_path);  // load an object mesh
    const float l = 5;
    const static cv::Scalar yellow(0, 255, 255);
    const static cv::Scalar green(0, 255, 0);
    std::vector<cv::Point2f> pose_points2d;
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
    //}
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "broadcast_location");
    ros::NodeHandle node_handle;
    const std::string from("world"), to("camera_link"), object("object");
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener listener(tfBuffer);
    Eigen::Affine3d T_base_camera, T_camera_object, tmp_mat;
    ros::Rate rate(30);
    const auto [camera, paras] = parse_input(argc, argv);
    bool set_once = false;
    //// Some basic colors
    const cv::Scalar red(0, 0, 255);
    const cv::Scalar green(0, 255, 0);
    const cv::Scalar yellow(0, 255, 255);

    cv::Mat frameSave;

    PnPProblem pnp_detection(camera);

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

    cv::Mat pose(6, 1, CV_64FC1);
    pose.setTo(cv::Scalar(0));

    // Get the MODEL INFO
    const cv::Mat descriptors_model = model.get_descriptors();
    const std::vector<cv::KeyPoint> keypoints_model = model.get_keypoints();

    // Create & Open Window
    cv::namedWindow("REAL TIME DEMO", cv::WINDOW_KEEPRATIO);

    std::unique_ptr<Stream> stream;
    if (paras.stream == "Video") {
        stream = std::make_unique<VideoStream>(paras);
    } else if (paras.stream == "RS") {
        stream = std::make_unique<RSStream>(paras);
    } else if (paras.stream == "File") {
        stream = std::make_unique<FileStream>(paras);
    } else {
        std::cout << "Could not understand: " << paras.stream << std::endl;
        throw std::runtime_error("");
    }

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
        const cv::Mat t = pnp_detection.get_t_matrix();
        std::cout << "The t matrix is: " << t << std::endl;

        // -- Step 5: Kalman Filter

        // GOOD MEASUREMENT
        bool good_measurement = false;
        std::cout << "Number of inliers: " << inliers_idx.rows  << std::endl;
        if (inliers_idx.rows >= paras.minInliersKalman) {
            cv::Mat tmp_pose(6, 1, CV_64FC1);
            tmp_pose.setTo(cv::Scalar(0));
            Pose(tmp_pose, pnp_detection, tmp_mat);
            T_camera_object.matrix() = tmp_mat.matrix();
            good_measurement = true;
            set_once = true;
            std::cout << "set once" << std::endl;
            // if (!set_once)
            // pose = tmp_pose;
            // set_once = true;

            //} else {
            // pose = tmp_pose * 0.9 + 0.1 * pose;
            // T_camera_object.matrix() = T_camera_object.matrix() * 0.1 +
            // tmp_mat.matrix() * 0.9;
            //}
        }
        if (set_once) {
            bool gotTransform =
                obtain_transform(from, to, tfBuffer, T_base_camera);
                std::cout << "Estimation\n"
                          << T_camera_object.matrix() << std::endl;
            if (gotTransform) {
                std::cout << "Base camera\n"
                          << T_base_camera.matrix() << std::endl;
                T_base_camera = T_base_camera * T_camera_object;
                std::cout << "The matrix is:\n"
                          << T_base_camera.matrix() << std::endl;
                Eigen::Quaterniond quat(T_base_camera.linear());
                std::cout << "The quaternion is: " << quat.coeffs()
                          << std::endl;
                broadcast(T_base_camera, from, object);
            }
        }
        // see how much time has elapsed
        drawPose(good_measurement, pnp_detection, frame_vis,
                 paras);
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
