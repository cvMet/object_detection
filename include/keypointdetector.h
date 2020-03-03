#pragma once
#ifndef VOX_H
#define VOX_H

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

	void calculateIssKeypoints(pcl::PointCloud<pcl::PointXYZ> model, pcl::PointCloud<pcl::PointXYZ> scene, float model_resolution, float scene_resolution, float threshold) {
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
		pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> issDetector;
		issDetector.setSearchMethod(tree);
		issDetector.setSalientRadius(6 * model_resolution);
		issDetector.setNonMaxRadius(4 * model_resolution);
		issDetector.setThreshold21(threshold);
		issDetector.setThreshold32(threshold);
		issDetector.setMinNeighbors(5);
		issDetector.setNumberOfThreads(4);
		issDetector.setInputCloud(model.makeShared());
		issDetector.compute(modelKeypoints_);

		issDetector.setSalientRadius(6 * scene_resolution);
		issDetector.setNonMaxRadius(4 * scene_resolution);
		issDetector.setInputCloud(scene.makeShared());
		issDetector.compute(sceneKeypoints_);
	}
};
#endif