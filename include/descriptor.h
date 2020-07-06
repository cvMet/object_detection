#pragma once

#ifndef DESCRIPTOR_H
#define DESCRIPTOR_H

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include "bshot_bits.h"
#include "normals.h"
#include "keypointdetector.h"

#define isshot 1 //1 if SHOT Descriptor, 0 if FPFH
//Typedef
typedef pcl::PointXYZ PointType;
typedef pcl::Normal NormalType;


class Descriptor
{
public:
	KeypointDetector KeypointDetector;
	Normals NormalEstimator;

#if isshot
	std::vector<bshot_descriptor> queryDescriptor_;
	std::vector<bshot_descriptor> targetDescriptor_;
#else
	pcl::PointCloud<pcl::FPFHSignature33> modelDescriptor_;
	pcl::PointCloud<pcl::FPFHSignature33> sceneDescriptor_;
#endif

	pcl::PointCloud<pcl::PointXYZ> query_, target_;

	void calculateDescriptor(float support_radius) {
		bshot bshotEstimation;
		bshotEstimation.cloud1_normals = NormalEstimator.queryNormals_;
		bshotEstimation.cloud1_keypoints = KeypointDetector.queryKeypoints_;
		bshotEstimation.cloud1 = query_;
		
		bshotEstimation.calculate_SHOT(support_radius);
		bshotEstimation.compute_single_bshot();

		queryDescriptor_ = bshotEstimation.cloud1_bshot;
	}
	
	void calculateDescriptor(float modelSupportradius, float sceneSupportradius) {
#if isshot

		bshot bshotEstimation;
		bshotEstimation.cloud1_normals = NormalEstimator.queryNormals_;
		bshotEstimation.cloud2_normals = NormalEstimator.targetNormals_;
		bshotEstimation.cloud1_keypoints = KeypointDetector.queryKeypoints_;
		bshotEstimation.cloud2_keypoints = KeypointDetector.targetKeypoints_;
		bshotEstimation.cloud1 = query_;
		bshotEstimation.cloud2 = target_;

		bshotEstimation.calculate_SHOT(modelSupportradius, sceneSupportradius);
		bshotEstimation.compute_bshot();

		queryDescriptor_ = bshotEstimation.cloud1_bshot;
		targetDescriptor_ = bshotEstimation.cloud2_bshot;
#else 
		pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfhEstimation;

		pcl::search::KdTree<PointXYZ>::Ptr tree(new pcl::search::KdTree<PointXYZ>);
		fpfhEstimation.setSearchMethod(tree);
		//Calculate Model descriptors
		fpfhEstimation.setRadiusSearch(modelSupportradius);
		fpfhEstimation.setInputCloud(KeypointDetector.modelKeypoints_.makeShared());
		fpfhEstimation.setInputNormals(NormalEstimator.modelNormals_.makeShared());
		fpfhEstimation.setSearchSurface(model_.makeShared());
		fpfhEstimation.compute(modelDescriptor_);
		//Calculate scene descriptors
		fpfhEstimation.setRadiusSearch(sceneSupportradius);
		fpfhEstimation.setInputCloud(KeypointDetector.sceneKeypoints_.makeShared());
		fpfhEstimation.setInputNormals(NormalEstimator.sceneNormals_.makeShared());
		fpfhEstimation.setSearchSurface(scene_.makeShared());
		fpfhEstimation.compute(sceneDescriptor_);
#endif
	}

};
#endif