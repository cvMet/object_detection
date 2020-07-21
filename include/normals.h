#pragma once
//#ifndef NORMAL_H
//#define NORMAL_H
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class Normals
{
private:
	int scalefactor = 5;


public:
	pcl::PointCloud<pcl::Normal> normals;
	pcl::PointCloud<pcl::Normal> queryNormals_, targetNormals_;
	pcl::PointCloud<pcl::PointXYZ> query, target;

	void set_scalefactor(int factor) {
		scalefactor = factor;
	}
	void calculateNormals(float radius, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
		pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
		normalEstimation.setSearchMethod(kdtree);
		// Set coordinates of viewpoint (x,y,z)
		normalEstimation.setViewPoint(0, 0, 10);
		//Estimate Model Normals
		normalEstimation.setRadiusSearch(scalefactor * radius);
		normalEstimation.setInputCloud(cloud);
 		normalEstimation.compute(normals);
	}

	void removeNaNNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
		//Remove NAN normals and their correspondences in model and scene
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		pcl::removeNaNNormalsFromPointCloud(normals, normals, inliers->indices);
		extract.setInputCloud(cloud);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*cloud);
	}

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
//#endif