// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2018 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------

#include "pose_estimation.hpp"
#include <Eigen/Geometry>
#include <open3d/pipelines/registration/Registration.h>

#include <open3d/geometry/KDTreeFlann.h>
#include <open3d/geometry/PointCloud.h>
#include <open3d/pipelines/registration/Feature.h>
#include <open3d/utility/Console.h>
#include <open3d/utility/Helper.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
//#include <utils/utils.hpp>

using namespace open3d;
using namespace open3d::pipelines;
using namespace open3d::pipelines::registration;

static RegistrationResult EvaluateRANSACBasedOnCorrespondence(
    const geometry::PointCloud &source, const geometry::PointCloud &target,
    const CorrespondenceSet &corres, double max_correspondence_distance,
    const Eigen::Matrix4d &transformation) {
    RegistrationResult result(transformation);
    double error2 = 0.0;
    int good = 0;
    double max_dis2 = max_correspondence_distance * max_correspondence_distance;
    for (const auto &c : corres) {
        double dis2 =
            (source.points_[c[0]] - target.points_[c[1]]).squaredNorm();
        if (dis2 < max_dis2) {
            good++;
            error2 += dis2;
            result.correspondence_set_.push_back(c);
        }
    }
    if (good == 0) {
        result.fitness_ = 0.0;
        result.inlier_rmse_ = 0.0;
    } else {
        result.fitness_ = (double)good / (double)corres.size();
        result.inlier_rmse_ = std::sqrt(error2 / (double)good);
    }
    return result;
}

RegistrationResult ModifiedRegistrationRANSACBasedOnCorrespondence(
    const geometry::PointCloud &source, const geometry::PointCloud &target,
    const CorrespondenceSet &corres, double max_correspondence_distance,
    const TransformationEstimation &estimation
    /* = TransformationEstimationPointToPoint(false)*/,
    int ransac_n /* = 3*/,
    const std::vector<std::reference_wrapper<const CorrespondenceChecker>>
        &checkers /* = {}*/,
    const RANSACConvergenceCriteria &criteria
    /* = RANSACConvergenceCriteria()*/) {
    if (ransac_n < 3 || (int)corres.size() < ransac_n ||
        max_correspondence_distance <= 0.0) {
        return RegistrationResult();
    }
    //double min_quality = 0.0;

    RegistrationResult best_result;
    int exit_itr = -1;

#pragma omp parallel
    {
        CorrespondenceSet ransac_corres(ransac_n);
        RegistrationResult best_result_local;
        int exit_itr_local = criteria.max_iteration_;
        double min_quality(0);

#pragma omp for nowait
        for (int itr = 0; itr < criteria.max_iteration_; itr++) {
            if (itr < exit_itr_local) {
                for (int j = 0; j < ransac_n; j++) {
                    ransac_corres[j] = corres[utility::UniformRandInt(
                        0, static_cast<int>(corres.size()) - 1)];
                }

                Eigen::Matrix4d transformation =
                    estimation.ComputeTransformation(source, target,
                                                     ransac_corres);
                transformation(2, 3) = 0; // impose movement on the ground

                // Check transformation: inexpensive
                bool check = true;
                for (const auto &checker : checkers) {
                    if (!checker.get().Check(source, target, ransac_corres,
                                             transformation)) {
                        check = false;
                        break;
                    }
                }
                if (!check) {
                    continue;
                }
                auto [roll, pitch, yaw] = utils::RPY(transformation);
                // double yaw = ObjectPose::calculateYaw(transformation);
                // double roll = ObjectPose::calculateRoll(transformation);
                // double pitch = ObjectPose::calculatePitch(transformation);
                if (std::abs(yaw) > (3.14 / 2.0)) {
                    continue;
                }
                if ((std::abs(roll) > (3.14 / 10.0)) or
                    (std::abs(pitch) > (3.14 / 10.0))) {
                    continue;
                }
                if (std::isnan(pitch) or std::isnan(roll) or std::isnan(yaw)) {
                    continue;
                }
                geometry::PointCloud pcd = source;
                pcd.Transform(transformation);
                auto result = EvaluateRANSACBasedOnCorrespondence(
                    pcd, target, corres, max_correspondence_distance,
                    transformation);
                auto reverse_result = registration::EvaluateRegistration(
                    target, pcd, max_correspondence_distance,
                    result.transformation_.inverse());
                double quality =
                    std::min(result.fitness_, reverse_result.fitness_);

                if ((result.IsBetterRANSACThan(best_result_local)) and
                    (quality > min_quality)) {
                    //ROS_WARN_STREAM("Better result, roll, yaw, pitch "
                                    //<< roll << ", " << yaw << ", " << pitch);
                    best_result_local = result;
                    min_quality = quality;

                    // Update exit condition if necessary
                    double exit_itr_d =
                        std::log(1.0 - criteria.confidence_) /
                        std::log(1.0 - std::pow(result.fitness_, ransac_n));
                    exit_itr_local =
                        exit_itr_d < double(criteria.max_iteration_)
                            ? static_cast<int>(std::ceil(exit_itr_d))
                            : exit_itr_local;
                }
            } // if < exit_itr_local
        }     // for loop
#pragma omp critical
        {
            if (best_result_local.IsBetterRANSACThan(best_result)) {
                best_result = best_result_local;
            }
            if (exit_itr_local > exit_itr) {
                exit_itr = exit_itr_local;
            }
        }
    }
    utility::LogDebug("RANSAC exits at {:d}-th iteration: inlier ratio {:e}, "
                      "RMSE {:e}",
                      exit_itr, best_result.fitness_, best_result.inlier_rmse_);
    return best_result;
}

RegistrationResult
ObjectPose::PoseEstimation::ModifiedRegistrationRANSACBasedOnFeatureMatching(
    const geometry::PointCloud &source, const geometry::PointCloud &target,
    const Feature &source_feature, const Feature &target_feature,
    bool mutual_filter, double max_correspondence_distance,
    const TransformationEstimation &estimation
    /* = TransformationEstimationPointToPoint(false)*/,
    int ransac_n /* = 3*/,
    const std::vector<std::reference_wrapper<const CorrespondenceChecker>>
        &checkers /* = {}*/,
    const RANSACConvergenceCriteria &criteria
    /* = RANSACConvergenceCriteria()*/) {
    if (ransac_n < 3 || max_correspondence_distance <= 0.0) {
        return RegistrationResult();
    }

    int num_src_pts = int(source.points_.size());
    int num_tgt_pts = int(target.points_.size());

    geometry::KDTreeFlann kdtree_target(target_feature);
    pipelines::registration::CorrespondenceSet corres_ij(num_src_pts);

#pragma omp parallel for
    for (int i = 0; i < num_src_pts; i++) {
        std::vector<int> corres_tmp(1);
        std::vector<double> dist_tmp(1);

        kdtree_target.SearchKNN(Eigen::VectorXd(source_feature.data_.col(i)), 1,
                                corres_tmp, dist_tmp);
        int j = corres_tmp[0];
        corres_ij[i] = Eigen::Vector2i(i, j);
    }

    // Do reverse check if mutual_filter is enabled
    if (mutual_filter) {
        geometry::KDTreeFlann kdtree_source(source_feature);
        pipelines::registration::CorrespondenceSet corres_ji(num_tgt_pts);

#pragma omp parallel for
        for (int j = 0; j < num_tgt_pts; ++j) {
            std::vector<int> corres_tmp(1);
            std::vector<double> dist_tmp(1);
            kdtree_source.SearchKNN(
                Eigen::VectorXd(target_feature.data_.col(j)), 1, corres_tmp,
                dist_tmp);
            int i = corres_tmp[0];
            corres_ji[j] = Eigen::Vector2i(i, j);
        }

        pipelines::registration::CorrespondenceSet corres_mutual;
        for (int i = 0; i < num_src_pts; ++i) {
            int j = corres_ij[i](1);
            if (corres_ji[j](0) == i) {
                corres_mutual.emplace_back(i, j);
            }
        }

        // Empirically mutual correspondence set should not be too small
        if (int(corres_mutual.size()) >= ransac_n * 3) {
            utility::LogDebug("{:d} correspondences remain after mutual filter",
                              corres_mutual.size());
            return ModifiedRegistrationRANSACBasedOnCorrespondence(
                source, target, corres_mutual, max_correspondence_distance,
                estimation, ransac_n, checkers, criteria);
        }
        utility::LogDebug(
            "Too few correspondences after mutual filter, fall back to "
            "original correspondences.");
    }

    return ModifiedRegistrationRANSACBasedOnCorrespondence(
        source, target, corres_ij, max_correspondence_distance, estimation,
        ransac_n, checkers, criteria);
}
