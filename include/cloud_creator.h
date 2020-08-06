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
private:
	const int rows = 240, columns = 320;

public:
	
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

	pcl::PointCloud<pcl::PointXYZ> segment_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool get_inlier, float resol) {
		PointCloud<pcl::PointXYZ>::Ptr	cloud_liers(new PointCloud<pcl::PointXYZ>);
		// Segment the ground
		pcl::ModelCoefficients::Ptr plane(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr 		inliers_plane(new pcl::PointIndices);

		// Make room for a plane equation (ax+by+cz+d=0)
		plane->values.resize(4);

		pcl::SACSegmentation<PointXYZ> seg;				// Create the segmentation object
		seg.setOptimizeCoefficients(false);				// Optional
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setDistanceThreshold(resol);
		seg.setInputCloud(cloud);
		seg.segment(*inliers_plane, *plane);

		if (inliers_plane->indices.size() == 0) {
			PCL_ERROR("Could not estimate a planar model for the given dataset.\n");
		}
		pcl::ExtractIndices<PointXYZ> extract;
		extract.setInputCloud(cloud);
		extract.setIndices(inliers_plane);
		if (get_inlier) {
			extract.setNegative(false);			// Extract the inliers
			extract.filter(*cloud_liers);		// cloud_inliers contains the plane
		}
		else {
			extract.setNegative(true);				// Extract the outliers
			extract.filter(*cloud_liers);		// cloud_outliers contains everything but the plane
		}
		return *cloud_liers;
	}

	pcl::PointCloud<pcl::PointXYZ> remove_outliers(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int k) {
		pcl::PointCloud<pcl::PointXYZ> output;
		// Create the filtering object
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		sor.setInputCloud(cloud);
		sor.setMeanK(k);
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

	pcl::PointCloud<pcl::PointXYZ> median_filter(pcl::PointCloud<pcl::PointXYZ> cloud, int window_size) {
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

	pcl::PointCloud<pcl::PointXYZ> downsample_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float leaf_size) {
		pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
		pcl::VoxelGrid<pcl::PointXYZ> downsampler;
		downsampler.setInputCloud(cloud);
		downsampler.setLeafSize(leaf_size, leaf_size, leaf_size);
		downsampler.filter(filtered_cloud);
		return filtered_cloud;
	}

	float get_median(std::vector<float> values) {
		std::vector<float>::iterator first = values.begin();
		std::vector<float>::iterator last = values.end();
		std::vector<float>::iterator middle = first + (last - first) / 2;
		std::nth_element(first, middle, last);
		return *middle;
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr median_filter_unordered_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,vector<vector<int>> neighbor_indices, vector<vector<float>> neighbor_distances, int window_size) {
			pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointXYZ filtered_point;
			for (int point = 0; point < neighbor_indices.size(); ++point) {
				vector<float> x_values;
				vector<float> y_values;
				vector<float> z_values;
				//Case # neighbors found is less than the number passed in search call 
				if (neighbor_indices[point].size() < window_size) {
					for (int i = 0; i < neighbor_indices[point].size(); ++i) {
						x_values.push_back(cloud->at(neighbor_indices[point][i]).x);
						y_values.push_back(cloud->at(neighbor_indices[point][i]).y);
						z_values.push_back(cloud->at(neighbor_indices[point][i]).z);
					}
					filtered_point.x = get_median(x_values);
					filtered_point.y = get_median(y_values);
					filtered_point.z = get_median(z_values);
				}
				//Case # neighbors found = number passed in search call 
				else {
					for (int i = 0; i < window_size; ++i) {
						x_values.push_back(cloud->at(neighbor_indices[point][i]).x);
						y_values.push_back(cloud->at(neighbor_indices[point][i]).y);
						z_values.push_back(cloud->at(neighbor_indices[point][i]).z);
					}
					filtered_point.x = get_median(x_values);
					filtered_point.y = get_median(y_values);
					filtered_point.z = get_median(z_values);
				}
				filtered->points.push_back(filtered_point);
			}
			return filtered;
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

	pcl::PointCloud<pcl::PointXYZ> remove_background_ordered(pcl::PointCloud<pcl::PointXYZ> point_cloud, pcl::PointCloud<pcl::PointXYZ> background, float threshold) {
		pcl::PointCloud<pcl::PointXYZ> output_cloud(320,240);
		float z;
		for (int i = 0; i < point_cloud.width; ++i) {
			for (int j = 0; j < point_cloud.height; ++j) {
				z = background.at(i,j).z - point_cloud.at(i,j).z;
				if (z > threshold) {
					output_cloud.at(i, j).x = point_cloud.at(i,j).x;
					output_cloud.at(i, j).y = point_cloud.at(i,j).y;
					output_cloud.at(i, j).z = z;
				}
				else {
					z = 0;
				}
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

	std::vector<pcl::PointCloud<PointXYZ>> ordered_euclidean_cluster_extraction(pcl::PointCloud<PointXYZ>::Ptr input_cloud) {
		// Creating the KdTree object for the search method of the extraction
		vector<pcl::PointCloud<PointXYZ>> clusters;
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud(input_cloud);

		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
		ec.setClusterTolerance(0.005);
		ec.setMinClusterSize(500);
		ec.setMaxClusterSize(5000);
		ec.setSearchMethod(tree);
		ec.setInputCloud(input_cloud);
		ec.extract(cluster_indices);

		int j = 0;
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>(320, 240));
			for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++) {
				cloud_cluster->points[*pit] = (input_cloud->points[*pit]);
			}
			std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
			clusters.push_back(*cloud_cluster);
			j++;
		}

		return clusters;
	}

	std::vector<pcl::PointCloud<PointXYZ>> euclidean_cluster_extraction(pcl::PointCloud<PointXYZ>::Ptr input_cloud) {
		// Creating the KdTree object for the search method of the extraction
		vector<pcl::PointCloud<PointXYZ>> clusters;
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud(input_cloud);
		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
		ec.setClusterTolerance(0.005);
		ec.setMinClusterSize(500);
		ec.setMaxClusterSize(5000);
		ec.setSearchMethod(tree);
		ec.setInputCloud(input_cloud);
		ec.extract(cluster_indices);
		int j = 0;
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
			for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++) {
				cloud_cluster->points.push_back(input_cloud->points[*pit]); //*
			}
			cloud_cluster->width = cloud_cluster->points.size();
			cloud_cluster->height = 1;
			cloud_cluster->is_dense = true;
			std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
			clusters.push_back(*cloud_cluster);
			j++;
		}
		return clusters;
	}
	
	float compute_cloud_resolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
	{
		float res = 0.0;
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

	//Can be used as conditional clustering criteria
	bool enforceIntensitySimilarity(const pcl::PointXYZI& point_a, const pcl::PointXYZI& point_b, float squared_distance)
	{
		if (std::abs(point_a.intensity - point_b.intensity) < 0.1f)
			return (true);
		else
			return (false);
	}

	//Can be used as conditional clustering criteria
	bool enforceCurvatureOrIntensitySimilarity(const PointXYZINormal& point_a, const PointXYZINormal& point_b, float squared_distance)
	{
		Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.getNormalVector3fMap(), point_b_normal = point_b.getNormalVector3fMap();
		if (std::abs(point_a.intensity - point_b.intensity) < 5.0f)
			return (true);
		if (std::abs(point_a_normal.dot(point_b_normal)) < 0.05)
			return (true);
		return (false);
	}

	//Can be used as conditional clustering criteria
	bool customRegionGrowing(const PointXYZINormal& point_a, const PointXYZINormal& point_b, float squared_distance)
	{
		Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.getNormalVector3fMap(), point_b_normal = point_b.getNormalVector3fMap();
		if (squared_distance < 10000)
		{
			if (std::abs(point_a.intensity - point_b.intensity) < 8.0f)
				return (true);
			if (std::abs(point_a_normal.dot(point_b_normal)) < 0.06)
				return (true);
		}
		else
		{
			if (std::abs(point_a.intensity - point_b.intensity) < 3.0f)
				return (true);
		}
		return (false);
	}

	void get_cloud_size(pcl::PointCloud<pcl::PointXYZ> cloud) {
		std::cout << "Point Cloud Size: " << cloud.size() << endl;
	}

	void show_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
		viewer->setBackgroundColor(0, 0, 0);
		viewer->initCameraParameters();
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> gold(cloud, 226, 176, 7);
		viewer->addPointCloud<pcl::PointXYZ>(cloud, gold, "sample cloud1");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud1");
		viewer->addCoordinateSystem(0.1);
		while (!viewer->wasStopped())
		{
			viewer->spinOnce(100);
			std::this_thread::sleep_for(100ms);
		}
	}

	void get_neighbors(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, vector<vector<int>>& neighbor_indices, vector<vector<float>>& neighbor_distances) {
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
		tree->setInputCloud(cloud);
		for (int point = 0; point < cloud->points.size(); ++point) {
			vector<int> neighbors;
			vector<float> distances;
			pcl::PointXYZ query_point = cloud->at(point);
			//Radius based search
			tree->radiusSearch(query_point, 5.0f, neighbors, distances);
			neighbor_indices.push_back(neighbors);
			neighbor_distances.push_back(distances);
		}
	}



};
#endif