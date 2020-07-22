#pragma once
#include <pcl/point_cloud.h>
#include "scene.h"

class Registrator {
private:
	Scene* Query;
	Scene* Target;
	pcl::Correspondences InputCorrespondences;
	pcl::Correspondences RANSACCorrespondences;
	Eigen::Matrix4f ransac_transformation;
	Eigen::Matrix4f icp_transformation;
	pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> RansacRejector;
	pcl::PointCloud<pcl::PointXYZ>::Ptr ransac_aligned_cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr ransac_aligned_keypoints;
	pcl::PointCloud<pcl::PointXYZ>::Ptr icp_aligned_cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr icp_aligned_keypoints;
	bool query_aligned = false;
	std::vector<float> euclidean_distances;
	std::string NNDR_euclidean_pair;
	float distance_threshold = 0.0f;
	pcl::Correspondences true_positives;
	std::string footer;

public:

	Registrator(Scene& query_, Scene& target_, pcl::Correspondences correspondences_)
		: Query(&query_), Target(&target_),
		InputCorrespondences(correspondences_),
		ransac_aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>),
		ransac_aligned_keypoints(new pcl::PointCloud<pcl::PointXYZ>),
		icp_aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>),
		icp_aligned_keypoints(new pcl::PointCloud<pcl::PointXYZ>) {
	}
	void set_distance_threshold(float threshold) {
		distance_threshold = threshold;
	}

	void init_RANSAC() {
		RansacRejector.setMaximumIterations(1000);
		//Inlier Threshold Set to 5mm since this is approximately the tof standard deviation
		RansacRejector.setInlierThreshold(0.005);
		RansacRejector.setInputSource(Query->keypoints.makeShared());
		RansacRejector.setInputTarget(Target->keypoints.makeShared());
		RansacRejector.setInputCorrespondences(boost::make_shared<pcl::Correspondences>(InputCorrespondences));
	}

	Eigen::Matrix4f do_ransac() {
		RansacRejector.getCorrespondences(RANSACCorrespondences);
		std::cout << "# Correspondences found after RANSAC: " << RANSACCorrespondences.size() << endl;
		ransac_transformation = RansacRejector.getBestTransformation();
		cout << "RANSAC Transformation Matrix yielding the largest number of inliers.  : \n" << ransac_transformation << endl;
		pcl::transformPointCloud(*Query->cloud, *ransac_aligned_cloud, ransac_transformation);
		pcl::transformPointCloud(Query->keypoints, *ransac_aligned_keypoints, ransac_transformation);
		return ransac_transformation;
	}

	Eigen::Matrix4f do_icp() {
		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		icp.setInputSource(ransac_aligned_cloud);
		icp.setInputTarget(Target->cloud);
		pcl::PointCloud<pcl::PointXYZ> temp;
		icp.align(*icp_aligned_cloud);
		std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
		std::cout << icp.getFinalTransformation() << std::endl;
		icp_transformation << icp.getFinalTransformation();
		pcl::transformPointCloud(*ransac_aligned_keypoints, *icp_aligned_keypoints, icp_transformation);
		query_aligned = true;
		return icp_transformation;
	}

	void calculate_euclidean_distances() {
		std::vector<float> distance;
		pcl::Correspondences Correspondences;
		if (RANSACCorrespondences.size() >= RANSAC_MIN_MATCHES) {
			Correspondences = RANSACCorrespondences;
		}
		else {
			Correspondences = InputCorrespondences;
			std::cout << "RANSAC not executed: euclidean distance can not be calculated accurately -> No reliable TP / FP data available" << endl;
		}
		for (int i = 0; i < Correspondences.size(); ++i) {
			distance.push_back(
				pow((icp_aligned_keypoints->at(Correspondences.at(i).index_query).x - Target->keypoints.at(Correspondences.at(i).index_match).x), 2) +
				pow((icp_aligned_keypoints->at(Correspondences.at(i).index_query).y - Target->keypoints.at(Correspondences.at(i).index_match).y), 2) +
				pow((icp_aligned_keypoints->at(Correspondences.at(i).index_query).z - Target->keypoints.at(Correspondences.at(i).index_match).z), 2)
			);
			distance[i] = sqrt(distance[i]);
		}
		euclidean_distances = distance;
	}

	void concatenate_NNDR_and_euclidean_distance() {
		pcl::Correspondences Correspondences;
		if (RANSACCorrespondences.size() >= RANSAC_MIN_MATCHES) {
			Correspondences = RANSACCorrespondences;
		}
		else {
			Correspondences = InputCorrespondences;
		}
		std::string data = "";
		for (int i = 0; i < Correspondences.size(); ++i)
		{
			data += std::to_string(Correspondences.at(i).distance) + ',' + std::to_string(euclidean_distances[i]);
			data += "\n";
		}
		NNDR_euclidean_pair = data;
	}

	void calculate_true_positives() {
		pcl::Correspondences temp;
		for (int i = 0; i < euclidean_distances.size(); ++i) {
			if (euclidean_distances[i] < distance_threshold) {
				temp.push_back(RANSACCorrespondences.at(i));
			}
			else {
				std::cout << "FP @ " + to_string(RANSACCorrespondences.at(i).index_match) << endl;
			}
		}
		true_positives = temp;
	}

	void print_precision_recall() {
		pcl::Correspondences Correspondences;
		if (RANSACCorrespondences.size() >= RANSAC_MIN_MATCHES) {
			Correspondences = RANSACCorrespondences;
		}
		else {
			Correspondences = InputCorrespondences;
		}
		std::cout << "TP: " << true_positives.size() << ", FP: " << Correspondences.size() - true_positives.size() << endl;
		std::cout << "Precision: " << (float)true_positives.size() / (float)Correspondences.size() << " Recall: " << true_positives.size() / (float)(Query->keypoints.size()) << endl;
	}

	void create_footer() {
		int NOF_keypoints = (Target->keypoints.size() < Query->keypoints.size()) ? Target->keypoints.size() : Query->keypoints.size();
		footer = std::to_string(NOF_keypoints) + "," + std::to_string(distance_threshold) + "\n";
	}

	std::string get_NNDR_euclidean_pair() {
		return NNDR_euclidean_pair;
	}

	std::string get_footer() {
		return footer;
	}

	bool is_query_aligned() {
		return query_aligned;
	}

	int get_number_of_matches() {
		int matches = (RANSACCorrespondences.size() > InputCorrespondences.size()) ? RANSACCorrespondences.size() : InputCorrespondences.size();
		return matches;
	}

	int get_number_of_tp() {
		return true_positives.size();
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr get_icp_aligned_cloud() {
		return icp_aligned_cloud;
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr get_icp_aligned_keypoints() {
		return icp_aligned_keypoints;
	}
};