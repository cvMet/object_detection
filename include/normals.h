#pragma once
#ifndef NORMAL_H
#define NORMAL_H
#include <pcl/point_cloud.h>

class Normals
{
public:
	pcl::PointCloud<pcl::Normal> modelNormals_, sceneNormals_;
	pcl::PointCloud<pcl::PointXYZ> model, scene;

	void calculateNormals(float radiusModel, float radiusScene)
	{
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
		pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
		normalEstimation.setSearchMethod(kdtree);
		// Set coordinates of viewpoint (x,y,z)
		normalEstimation.setViewPoint(0, 0, 0.36);
		//Estimate Model Normals
		normalEstimation.setRadiusSearch(radiusModel);
		normalEstimation.setInputCloud(model.makeShared());
		normalEstimation.compute(modelNormals_);
		//Estimate Scene Normals
		normalEstimation.setRadiusSearch(radiusScene);
		normalEstimation.setInputCloud(scene.makeShared());
		normalEstimation.compute(sceneNormals_);
	}

	void removeNaNNormals() {
		//Remove NAN normals and their correspondences in model and scene
		std::vector<int> index;
		pcl::removeNaNNormalsFromPointCloud(modelNormals_, modelNormals_, index);
		pcl::PointCloud<pcl::PointXYZ> tempCloud;
		for (int i = 0; i < modelNormals_.size(); ++i) {
			tempCloud.push_back(model.at(index[i]));
		}
		model = tempCloud;
		pcl::removeNaNNormalsFromPointCloud(sceneNormals_, sceneNormals_, index);
		tempCloud.clear();
		for (int i = 0; i < sceneNormals_.size(); ++i) {
			tempCloud.push_back(scene.at(index[i]));
		}
		scene = tempCloud;
	}
};
#endif