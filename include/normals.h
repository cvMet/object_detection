#pragma once
#ifndef NORMAL_H
#define NORMAL_H
#include <pcl/point_cloud.h>

class Normals
{
public:
	pcl::PointCloud<pcl::Normal> queryNormals_, targetNormals_;
	pcl::PointCloud<pcl::PointXYZ> query, target;

	void calculateNormals(float radiusModel, float radiusScene)
	{
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
		pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
		normalEstimation.setSearchMethod(kdtree);
		// Set coordinates of viewpoint (x,y,z)
		normalEstimation.setViewPoint(0, 0, 10);
		//Estimate Model Normals
		normalEstimation.setRadiusSearch(radiusModel);
		normalEstimation.setInputCloud(query.makeShared());
		normalEstimation.compute(queryNormals_);
		//Estimate Scene Normals
		normalEstimation.setRadiusSearch(radiusScene);
		normalEstimation.setInputCloud(target.makeShared());
		normalEstimation.compute(targetNormals_);
	}

	void removeNaNNormals() {
		//Remove NAN normals and their correspondences in model and scene
		std::vector<int> index;
		pcl::removeNaNNormalsFromPointCloud(queryNormals_, queryNormals_, index);
		pcl::PointCloud<pcl::PointXYZ> tempCloud;
		for (int i = 0; i < queryNormals_.size(); ++i) {
			tempCloud.push_back(query.at(index[i]));
		}
		query = tempCloud;
		pcl::removeNaNNormalsFromPointCloud(targetNormals_, targetNormals_, index);
		tempCloud.clear();
		for (int i = 0; i < targetNormals_.size(); ++i) {
			tempCloud.push_back(target.at(index[i]));
		}
		target = tempCloud;
	}
};
#endif