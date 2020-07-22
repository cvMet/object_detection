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

	void calculateIssKeypoints(Scene scene) {
		//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
		pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ, pcl::Normal> issDetector;
		issDetector.setNormals(scene.normals.makeShared());
		//issDetector.setSearchMethod(tree);
		issDetector.setSalientRadius(7*scene.resolution);
		//nonMax radius set to 5mm since this is approx stddev of melexis camera
		//issDetector.setNonMaxRadius(4 * resolution);
		issDetector.setNonMaxRadius(0.005f);
		issDetector.setThreshold21(threshold);
		issDetector.setThreshold32(threshold);
		issDetector.setMinNeighbors(neighbor_count);
		issDetector.setNumberOfThreads(4);
		issDetector.setInputCloud(scene.cloud);
		issDetector.compute(keypoints);
		std::cout << "No. Keypoints: " << keypoints.size() << " of: " << scene.cloud->size() << std::endl;
	}
};
#endif