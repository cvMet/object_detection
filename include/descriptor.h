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
	KeypointDetector keypointDetect;
	Normals normal;
	pcl::PointCloud<pcl::PointXYZ> model_, scene_;
#if isshot
	std::vector<bshot_descriptor> modelDescriptor_;
	std::vector<bshot_descriptor> sceneDescriptor_;
#else
	pcl::PointCloud<pcl::FPFHSignature33> modelDescriptor_;
	pcl::PointCloud<pcl::FPFHSignature33> sceneDescriptor_;
#endif
	void calculateDescriptor(float modelSupportradius, float sceneSupportradius) {
#if isshot

		bshot bshotEstimation;
		bshotEstimation.cloud1_normals = normal.modelNormals_;
		bshotEstimation.cloud2_normals = normal.sceneNormals_;
		bshotEstimation.cloud1_keypoints = keypointDetect.modelKeypoints_;
		bshotEstimation.cloud2_keypoints = keypointDetect.sceneKeypoints_;
		bshotEstimation.cloud1 = model_;
		bshotEstimation.cloud2 = scene_;

		bshotEstimation.calculate_SHOT(modelSupportradius,sceneSupportradius);
		bshotEstimation.compute_bshot();

		modelDescriptor_ = bshotEstimation.cloud1_bshot;
		sceneDescriptor_ = bshotEstimation.cloud2_bshot;
#else 
		pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfhEstimation;
		
		pcl::search::KdTree<PointXYZ>::Ptr tree(new pcl::search::KdTree<PointXYZ>);
		fpfhEstimation.setSearchMethod(tree);
		//Calculate Model descriptors
		fpfhEstimation.setRadiusSearch(modelSupportradius);
		fpfhEstimation.setInputCloud(keypointDetect.modelKeypoints_.makeShared());
		fpfhEstimation.setInputNormals(normal.modelNormals_.makeShared());
		fpfhEstimation.setSearchSurface(model_.makeShared());
		fpfhEstimation.compute(modelDescriptor_);
		//Calculate scene descriptors
		fpfhEstimation.setRadiusSearch(sceneSupportradius);
		fpfhEstimation.setInputCloud(keypointDetect.sceneKeypoints_.makeShared());
		fpfhEstimation.setInputNormals(normal.sceneNormals_.makeShared());
		fpfhEstimation.setSearchSurface(scene_.makeShared());
		fpfhEstimation.compute(sceneDescriptor_);
#endif
	}
	
};
#endif