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
		normalEstimation.setRadiusSearch(radiusModel);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
		normalEstimation.setSearchMethod(kdtree);

		normalEstimation.setInputCloud(model.makeShared());
		normalEstimation.compute(modelNormals_);
		normalEstimation.setRadiusSearch(radiusScene);
		normalEstimation.setInputCloud(scene.makeShared());
		normalEstimation.compute(sceneNormals_);
	}
};
#endif