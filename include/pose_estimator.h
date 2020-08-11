#pragma once

#include <iostream>
#include <string>
#include "../src/objectdetection.h"
#include "scene.h"

class PoseEstimator
{
private:
    pcl::Correspondences matches;
    pcl::PointXYZ base_point;
    Eigen::Vector3d e_x, e_y, e_z;
    double eigenv_1 = 0.0;
    double eigenv_2 = 0.0;
    double eigenv_3 = 0.0;
    std::vector<Eigen::Matrix3d> rotation_estimates;
    std::vector<std::vector<double>> pose_estimation;

public:
    PoseEstimator()
    {

    }

    std::string get_RANSAC_pose_estimation(Eigen::Matrix4f transformation_matrix) {
        pose_estimation.clear();
        pose_estimation.push_back(ransac_matrix_to_GNR_angles(transformation_matrix));
        pose_estimation.push_back(ransac_matrix_to_translation(transformation_matrix));
        return create_writable_pose_estimation(pose_estimation);
    }

    std::vector<double> ransac_matrix_to_GNR_angles(Eigen::Matrix4f transformation_matrix) {
        vector<double> angles;
        double r31 = transformation_matrix(2, 0);
        double r32 = transformation_matrix(2, 1);
        double r33 = transformation_matrix(2, 2);
        double r11 = transformation_matrix(0, 0);
        double r21 = transformation_matrix(1, 0);

        double beta = atan2(-r31, sqrt(pow(r11, 2) + pow(r21, 2)));
        double alpha = atan2((r21 / cos(beta)), (r11 / cos(beta)));
        double gamma = atan2((r32 / cos(beta)), (r33 / cos(beta)));

        angles.push_back(alpha * 180 / PI);
        angles.push_back(beta * 180 / PI);
        angles.push_back(gamma * 180 / PI);

        std::cout << "alpha: " << (double)alpha * 180 / PI << std::endl;
        std::cout << "beta: " << (double)beta * 180 / PI << std::endl;
        std::cout << "gamma: " << (double)gamma * 180 / PI << std::endl;

        return angles;
    }

    std::vector<double> ransac_matrix_to_translation(Eigen::Matrix4f transformation_matrix) {
        vector<double> translation;
        double x_translation = transformation_matrix(0, 3);
        double y_translation = transformation_matrix(1, 3);
        double z_translation = transformation_matrix(2, 3);

        translation.push_back(x_translation);
        translation.push_back(y_translation);
        translation.push_back(z_translation);

        std::cout << "x_translation: " << (double)x_translation;
        std::cout << "y_translation: " << (double)y_translation;
        std::cout << "z_translation: " << (double)z_translation;

        return translation;
    }


    std::string create_writable_pose_estimation(vector<vector<double>> pose_estimation) {
        std::string data = "";
        for (int i = 0; i < pose_estimation.size(); ++i) {
            for (int j = 0; j < pose_estimation[i].size(); ++j) {
                data += std::to_string(pose_estimation[i][j]) + ",";
            }
            data += "\n";
        }
        return data;
    }

    std::string get_ISS_pose_estimation(Scene& query, Scene& target, pcl::Correspondences corresp) {
        pose_estimation.clear();
        set_matches(corresp);
        for (int i = 0; i < matches.size(); ++i) {
            //Calculate ISS Reference Frame for query point from collection of matches
            Eigen::Matrix3d query_cov, query_frame;
            getScatterMatrix(query, query.keypoint_indices[matches.at(i).index_query], query_cov);
            query_frame = get_ref_frame(query_cov);
            //Calculate ISS Reference Frame for target point from collection of matches
            Eigen::Matrix3d target_cov, target_frame;
            getScatterMatrix(target, target.keypoint_indices[matches.at(i).index_match], target_cov);
            target_frame = get_ref_frame(target_cov);
            //Calculate transformation between two reference frames
            rotation_estimates.push_back(get_rotation(query_frame, target_frame));
            pose_estimation.push_back(transformation_matrix_to_GNR_angles(rotation_estimates[i]));
        }
        return create_writable_pose_estimation(pose_estimation);
    }

    void set_matches(pcl::Correspondences corresp) {
        matches = corresp;
    }

    void getScatterMatrix(Scene& scene, const int& current_index, Eigen::Matrix3d& cov_m)
    {
        const PointXYZ& current_point = scene.cloud->points[current_index];

        double central_point[3];
        memset(central_point, 0, sizeof(double) * 3);

        central_point[0] = current_point.x;
        central_point[1] = current_point.y;
        central_point[2] = current_point.z;

        cov_m = Eigen::Matrix3d::Zero();

        std::vector<int> nn_indices;
        std::vector<float> nn_distances;
        int n_neighbors;

        search_for_neighbors(scene, current_index, nn_indices, nn_distances);

        n_neighbors = static_cast<int> (nn_indices.size());

        if (n_neighbors < 5)
            return;

        double cov[9];
        memset(cov, 0, sizeof(double) * 9);

        for (int n_idx = 0; n_idx < n_neighbors; n_idx++)
        {
            const pcl::PointXYZ& n_point = scene.cloud->points[nn_indices[n_idx]];

            double neigh_point[3];
            memset(neigh_point, 0, sizeof(double) * 3);

            neigh_point[0] = n_point.x;
            neigh_point[1] = n_point.y;
            neigh_point[2] = n_point.z;

            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    cov[i * 3 + j] += (neigh_point[i] - central_point[i]) * (neigh_point[j] - central_point[j]);
        }

        cov_m << cov[0], cov[1], cov[2],
            cov[3], cov[4], cov[5],
            cov[6], cov[7], cov[8];
    }

    void search_for_neighbors(Scene& scene, int index, std::vector<int>& nn_indices, std::vector<float> nn_distances) {
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        tree->setInputCloud(scene.cloud);
        base_point = scene.cloud->at(index);
        tree->radiusSearch(base_point, (7 * scene.resolution), nn_indices, nn_distances);
    }

    Eigen::Matrix3d get_ref_frame(Eigen::Matrix3d cov_m) {
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov_m);
        Eigen::Matrix3d frame;
        e_y << solver.eigenvectors()(3), solver.eigenvectors()(4), solver.eigenvectors()(5);
        e_x << solver.eigenvectors()(6), solver.eigenvectors()(7), solver.eigenvectors()(8);
        e_z = e_x.cross(e_y);
        frame << e_x, e_y, e_z;
        return frame;
    }

    Eigen::Matrix3d get_rotation(Eigen::Matrix3d query, Eigen::Matrix3d target) {
        using namespace Eigen;
        Matrix3d R;
        R = target * query.transpose();
        return R;
    }

    std::vector<double> transformation_matrix_to_GNR_angles(Eigen::Matrix3d transformation_matrix) {
        vector<double> angles;
        double r31 = transformation_matrix(2, 0);
        double r32 = transformation_matrix(2, 1);
        double r33 = transformation_matrix(2, 2);
        double r11 = transformation_matrix(0, 0);
        double r21 = transformation_matrix(1, 0);

        double beta = atan2(-r31, sqrt(pow(r11, 2) + pow(r21, 2)));
        double alpha = atan2((r21 / cos(beta)), (r11 / cos(beta)));
        double gamma = atan2((r32 / cos(beta)), (r33 / cos(beta)));

        angles.push_back(alpha * 180 / PI);
        angles.push_back(beta * 180 / PI);
        angles.push_back(gamma * 180 / PI);

        std::cout << "alpha: " << (double)alpha * 180 / PI << std::endl;
        std::cout << "beta: " << (double)beta * 180 / PI << std::endl;
        std::cout << "gamma: " << (double)gamma * 180 / PI << std::endl;

        return angles;
    }

    std::string get_rotation_estimates() {
        std::string temp = "";
        for (int i = 0; i < rotation_estimates.size(); ++i) {
            for (int j = 0; j < 9; ++j) {
                temp += std::to_string(rotation_estimates[i].data()[j]) + ",";
            }
            temp += "\n";
        }
        return temp;
    }

    void calc_eigenvalues(Eigen::Matrix3d cov_m) {
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov_m);
        const double& e1c = solver.eigenvalues()[2];
        const double& e2c = solver.eigenvalues()[1];
        const double& e3c = solver.eigenvalues()[0];
        eigenv_1 = e1c;
        eigenv_2 = e2c;
        eigenv_3 = e3c;
    }

    std::vector<double> transformation_matrix_to_euler_angles(Eigen::Matrix3d transformation_matrix) {
        vector<double> angles;
        double cosBeta = transformation_matrix(2, 2);
        double negCosAlphaSinBeta = transformation_matrix(2, 1);
        double sinBetaCosGamma = transformation_matrix(1, 2);

        double beta = acos(cosBeta) * 180 / PI;
        double alpha = acos(-(negCosAlphaSinBeta / sin((beta * PI / 180))));
        double gamma = acos(sinBetaCosGamma / sin((beta * PI / 180)));

        angles.push_back(alpha);
        angles.push_back(beta);
        angles.push_back(gamma);

        std::cout << "alpha: " << (double)alpha << std::endl;
        std::cout << "beta: " << (double)beta << std::endl;
        std::cout << "gamma: " << (double)gamma << std::endl;

        return angles;
    }
};
