#pragma once

#ifndef CLOUDCREATOR_H
#define CLOUDCREATOR_H

#include "boost/filesystem.hpp"
#include "boost/filesystem/path.hpp"
#include <iostream>           
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/filters/shadowpoints.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/iss_3d.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/median_filter.h>
#include <sstream>
#include "../include/filehandler.h"
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/mls.h>
#include <filesystem>
#include "../include/tree.h"

class CloudCreator
{
public:
	const int rows = 240, columns = 320;
	//distance array values must be in [mm], x_pixel = pixel width [mm], y_pixel = pixel height [mm], f = focal length of camera [mm]
	pcl::PointCloud<pcl::PointXYZ> distance_array_to_cloud(std::vector<std::vector<float>> distance_array, float f, float x_pixel, float y_pixel)
	{
		pcl::PointCloud<pcl::PointXYZ> point_cloud;
		point_cloud.width = columns;
		point_cloud.height = rows;
		//is_dense - specifies if all the data in points is finite (true), or whether it might contain Inf/NaN values (false)
		point_cloud.is_dense = false;
		//points - the data array that contains all points of type PointT.
		point_cloud.points.resize(point_cloud.width * point_cloud.height);
		int ind = 0;
		double normalize = 0;
		for (int i = 0; i < rows; ++i)
		{
			for (int j = 0; j < columns; ++j)
			{
				float XcDx = (j - columns / 2) * x_pixel;
				float YcDy = (i - rows / 2) * y_pixel;
				//make sure the cloud is in m since the csv file is in mm
				normalize = 0.001 / sqrt(pow(XcDx, 2) + pow(YcDy, 2) + pow(f, 2));
				point_cloud.points[ind].x = normalize * distance_array[i][j] * XcDx;
				point_cloud.points[ind].y = normalize * distance_array[i][j] * YcDy;
				point_cloud.points[ind].z = normalize * distance_array[i][j] * f;
				++ind;
			}
		}
		return point_cloud;
	}

	pcl::PointCloud<pcl::PointXYZ> remove_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float resolution) {
		//find all the points within a point cloud that support a plane model and remove them
		//from http://pointclouds.org/documentation/tutorials/planar_segmentation.php
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		// Optional
		seg.setOptimizeCoefficients(false);
		// Mandatory
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(resolution / 20);
		seg.setInputCloud(cloud);
		seg.segment(*inliers, *coefficients);

		pcl::PointCloud<pcl::PointXYZ> output;
		//if no inliers could be found return the input cloud
		if (inliers->indices.size() == 0) {
			output = *cloud;
		}
		else {
			int j = 0;
			for (int i = 0; i < cloud->size(); ++i) {
				if (j < inliers->indices.size())
				{
					if (inliers->indices.at(j) == i) {
						++j;
					}
					else {
						output.push_back(cloud->at(i));
					}
				}
				else {
					continue;
				}
			}
		}
		return output;
	}

	pcl::PointCloud<pcl::PointXYZ> remove_outliers(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
		pcl::PointCloud<pcl::PointXYZ> output;
		// Create the filtering object
		//from: http://pointclouds.org/documentation/tutorials/statistical_outlier.php
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		sor.setInputCloud(cloud);
		sor.setMeanK(15);
		sor.setStddevMulThresh(1.0);
		sor.filter(output);
		return output;
	}

	pcl::PointCloud<pcl::PointXYZ> remove_outliers_mean(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float threshold) {
		//threshold in m, how far away from mean points should be considered
		pcl::PointCloud<pcl::PointXYZ> output;
		float mean = 0;
		for (int i = 0; i < cloud->size(); ++i) {
			mean += cloud->at(i).z;
			//std::cout << cloud->at(i).z << endl;
		}
		mean = mean / cloud->size();
		std::cout << "Mean: " << mean << endl;
		for (int i = 0; i < cloud->size(); ++i) {
			if (cloud->at(i).z < mean - threshold) {
				output.push_back(cloud->at(i));
			}
		}
		return output;
	}

	pcl::PointCloud<pcl::PointXYZ> median_filter_cloud(pcl::PointCloud<pcl::PointXYZ> cloud, int window_size) {
		pcl::MedianFilter<pcl::PointXYZ> median;
		pcl::PointCloud<pcl::PointXYZ> finalCloud;
		median.setInputCloud(cloud.makeShared());
		median.setWindowSize(window_size);
		median.applyFilter(cloud);
		std::vector<int> vec;
		finalCloud = cloud;
		pcl::removeNaNFromPointCloud(finalCloud, finalCloud, vec);
		return finalCloud;
	}

	pcl::PointCloud<pcl::PointXYZ> noise_filter(pcl::PointCloud<pcl::PointXYZ> cloud, float noise_threshold) {
		pcl::PointCloud<pcl::PointXYZ> finalCloud;
		for (int i = 0; i < cloud.size(); ++i) {
			if (cloud.at(i).z > noise_threshold) {
				finalCloud.push_back(cloud.at(i));
			}
		}
		std::vector<int> vec;
		pcl::removeNaNFromPointCloud(finalCloud, finalCloud, vec);
		return finalCloud;
	}

	pcl::PointCloud<pcl::PointXYZ> remove_background(pcl::PointCloud<pcl::PointXYZ> point_cloud, pcl::PointCloud<pcl::PointXYZ> background, float threshold) {
		pcl::PointCloud<pcl::PointXYZ> output_cloud;
		float z;
		PointXYZ point;
		for (int i = 0; i < point_cloud.size(); ++i) {
			z = background.at(i).z - point_cloud.at(i).z;
			if (z > threshold) {
				point.x = point_cloud.at(i).x;
				point.y = point_cloud.at(i).y;
				point.z = z;
				output_cloud.push_back(point);
			}
			else {
				z = 0;
			}
		}
		return output_cloud;
	}

	pcl::PointCloud<pcl::PointXYZ> roi_filter(pcl::PointCloud<pcl::PointXYZ> cloud, string dimension, float lower_limit, float higher_limit) {
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud(cloud.makeShared());
		pass.setFilterFieldName(dimension);
		pass.setFilterLimits(lower_limit, higher_limit);
		pass.filter(cloud);
		return cloud;
	}

	double computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
	{
		double res = 0.0;
		int n_points = 0;
		int nres;
		std::vector<int> indices(2);
		std::vector<float> sqr_distances(2);
		pcl::search::KdTree<pcl::PointXYZ> tree;
		tree.setInputCloud(cloud);

		for (size_t i = 0; i < cloud->size(); ++i)
		{
			if (!std::isfinite((*cloud)[i].x))
			{
				continue;
			}
			//Considering the second neighbor since the first is the point itself.
			nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
			if (nres == 2)
			{
				res += sqrt(sqr_distances[1]);
				++n_points;
			}
		}
		if (n_points != 0)
		{
			res /= n_points;
		}
		std::cout << "Resolution: " << res << endl;
		return res;
	}

	void get_cloud_size(pcl::PointCloud<pcl::PointXYZ> cloud) {
		std::cout << "Point Cloud Size: " << cloud.size() << endl;
	}

	void show_cloud(pcl::PointCloud<pcl::PointXYZ> cloud) {
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
		viewer->setBackgroundColor(0, 0, 0);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color1(cloud.makeShared(), 255, 255, 255);
		viewer->addPointCloud<pcl::PointXYZ>(cloud.makeShared(), single_color1, "sample cloud1");
		viewer->addCoordinateSystem(0.1);
		while (!viewer->wasStopped())
		{
			viewer->spinOnce(100);
			std::this_thread::sleep_for(100ms);
		}
	}

};
#endif