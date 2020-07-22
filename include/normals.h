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
};
//#endif