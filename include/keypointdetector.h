#pragma once
#ifndef KEYPOINTDETECTOR_H
#define KEYPOINTDETECTOR_H

#include <pcl/point_cloud.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/trajkovic_3d.h>
#include "normals.h"
#include "scene.h"
class KeypointDetector
{
private:
	int neighbor_count = 5;
	float threshold = 0.9;

public:
	pcl::PointCloud<pcl::PointXYZ> keypoints;

	void set_neighbor_count(int count) {
		neighbor_count = count;
	}
	void set_threshold(float value) {
		threshold = value;
	}

	bool identical_point(pcl::PointXYZ point_ref, pcl::PointXYZ point_in) {
		return((point_ref.x == point_in.x) && (point_ref.y == point_in.y) && (point_ref.z == point_in.z));
	}

	void calculate_keypoint_indices(Scene& scene) {
		for (int i = 0; i < keypoints.size(); ++i) {
			for (int j = 0; j < scene.cloud->size(); ++j) {
				if (identical_point(keypoints.points.at(i), scene.cloud->points.at(j)))
					scene.keypoint_indices.push_back(j);
			}
		}
	}

	//void  calculateVoxelgridKeypoints(pcl::PointCloud<pcl::PointXYZ> model, pcl::PointCloud<pcl::PointXYZ> scene, float leaf_size_model, float leaf_size_scene)
	//{
	//	// Find Keypoints on the input cloud
	//	pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
	//	voxelgrid.setLeafSize(leaf_size_model, leaf_size_model, leaf_size_model);
	//	voxelgrid.setInputCloud(model.makeShared());
	//	voxelgrid.filter(queryKeypoints_);
	//	voxelgrid.setLeafSize(leaf_size_scene, leaf_size_scene, leaf_size_scene);
	//	voxelgrid.setInputCloud(scene.makeShared());
	//	voxelgrid.filter(targetKeypoints_);
	//}

	void calculateIssKeypoints(Scene& scene) {
		//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
		pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ, pcl::Normal> issDetector;
		issDetector.setNormals(scene.normals.makeShared());
		//issDetector.setSearchMethod(tree);
		issDetector.setSalientRadius(7*scene.resolution);
		//nonMax radius set to 5mm since this is approx stddev of melexis camera
		issDetector.setNonMaxRadius(0.005f);
		issDetector.setThreshold21(threshold);
		issDetector.setThreshold32(threshold);
		issDetector.setMinNeighbors(neighbor_count);
		issDetector.setNumberOfThreads(4);
		issDetector.setInputCloud(scene.cloud);
		issDetector.compute(keypoints);
		calculate_keypoint_indices(scene);
		std::cout << "No. Keypoints: " << keypoints.size() << " of: " << scene.cloud->size() << std::endl;
	}
};
#endif