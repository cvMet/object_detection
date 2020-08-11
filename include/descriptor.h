#pragma once

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
#include "scene.h"
#include "keypointdetector.h"

//#define isshot 1 //1 if SHOT Descriptor, 0 if FPFH
//Typedef
typedef pcl::PointXYZ PointType;
typedef pcl::Normal NormalType;


class Descriptor
{
private:
	int support_radius = 30;
	struct {
		std::vector<bshot_descriptor> bshot_descriptors;
		pcl::PointCloud<pcl::FPFHSignature33> fpfh_descriptors;
	}descriptor_collection;

public:
	void set_support_radius(int value) {
		support_radius = value;
	}
	std::vector<bshot_descriptor> descriptors;
	pcl::PointCloud<pcl::FPFHSignature33> fpfh_descriptors;



	void calculateDescriptor(Scene scene) {
		//#if isshot
		bshot bshotEstimation;
		bshotEstimation.normals = scene.normals;
		bshotEstimation.keypoints = scene.keypoints;
		bshotEstimation.cloud = scene.cloud;

		bshotEstimation.calculate_SHOT(float(support_radius * scene.resolution));
		bshotEstimation.compute_bshot();

		descriptors = bshotEstimation.bshot;
		descriptor_collection.bshot_descriptors = descriptors;
	}
	void calculate_fpfh_descriptor(Scene scene){
		pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfhEstimation;
		pcl::search::KdTree<PointXYZ>::Ptr tree(new pcl::search::KdTree<PointXYZ>);
		fpfhEstimation.setSearchMethod(tree);
		//Calculate Model descriptors
		fpfhEstimation.setRadiusSearch(20 * scene.resolution);
		fpfhEstimation.setInputCloud(scene.keypoints.makeShared());
		fpfhEstimation.setInputNormals(scene.normals.makeShared());
		fpfhEstimation.setSearchSurface(scene.cloud);
		fpfhEstimation.compute(fpfh_descriptors);
		descriptor_collection.fpfh_descriptors = fpfh_descriptors;

	}
};