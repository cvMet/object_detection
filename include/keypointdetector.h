#pragma once
#ifndef KEYPOINTDETECTOR_H
#define KEYPOINTDETECTOR_H

#include <pcl/point_cloud.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/trajkovic_3d.h>
#include "normals.h"
class KeypointDetector
{
public:
	pcl::PointCloud<pcl::PointXYZ> modelKeypoints_, sceneKeypoints_;
	void  calculateVoxelgridKeypoints(pcl::PointCloud<pcl::PointXYZ> model, pcl::PointCloud<pcl::PointXYZ> scene, float leaf_size_model, float leaf_size_scene)
	{
		// Find Keypoints on the input cloud
		pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
		voxelgrid.setLeafSize(leaf_size_model, leaf_size_model, leaf_size_model);

		voxelgrid.setInputCloud(model.makeShared());
		voxelgrid.filter(modelKeypoints_);

		voxelgrid.setLeafSize(leaf_size_scene, leaf_size_scene, leaf_size_scene);
		voxelgrid.setInputCloud(scene.makeShared());
		voxelgrid.filter(sceneKeypoints_);
	}

	void calculateIssKeypoints(pcl::PointCloud<pcl::PointXYZ>& out, pcl::PointCloud<pcl::PointXYZ> cloud, pcl::PointCloud<pcl::Normal> normals, float resolution, float threshold, int neighbor) {
		//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
		pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ, pcl::Normal> issDetector;
		issDetector.setNormals(normals.makeShared());
		//issDetector.setSearchMethod(tree);
		issDetector.setSalientRadius(6*resolution);
		//nonMax radius set to 5mm since this is approx stddev of melexis camera
		//issDetector.setNonMaxRadius(4 * resolution);
		issDetector.setNonMaxRadius(0.005f);
		issDetector.setThreshold21(threshold);
		issDetector.setThreshold32(threshold);
		issDetector.setMinNeighbors(neighbor);
		issDetector.setNumberOfThreads(4);
		issDetector.setInputCloud(cloud.makeShared());
		issDetector.compute(out);
		std::cout << "No. Keypoints: " << out.size() << " of: " << cloud.size() << std::endl;
	}
};
#endif