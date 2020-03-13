/*
Parts of this code were taken from:
https://github.com/saimanoj18/iros_bshot
@INPROCEEDINGS{bshot_iros2015,
author={S. M. Prakhya and Bingbing Liu and Weisi Lin},
booktitle={Intelligent Robots and Systems (IROS), 2015 IEEE/RSJ International Conference on},
title={B-SHOT: A binary feature descriptor for fast and efficient keypoint matching on 3D point clouds},
year={2015},
pages={1929-1934},
doi={10.1109/IROS.2015.7353630},
month={Sept},}

additional parts were taken from PCL Tutorials or written by Joël Carlen, Student at the Lucerne University of Applied Sciences and Arts
*/

//Includes
#include <iostream>
#include <pcl/filters/median_filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/keypoints/sift_keypoint.h>
#include "../include/keypointdetector.h"
#include "../include/matching.h"
#include "../include/filehandler.h"
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/registration/icp.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/shadowpoints.h>
#include <random>

#ifndef isshot
#define isshot 1	//For SHOT-Descriptor use 1 for FPFH 0
#endif

//Namespaces
using namespace std;
using namespace pcl;

//Model is to be matched in scene
string model_filename = "../../../../Aufnahmen/Datensets/Melexis_Punktwolken/001_Bottlewhite1model.ply";
string scene_filename = "../../../../Aufnahmen/Datensets/Melexis_Punktwolken/003_Bottlewhite3model.ply";
string filename = model_filename.substr(52, (model_filename.length() - 56));
string pr_filename = "../../../../PR/" + filename + ".csv";
clock_t start, end_time;
pcl::Correspondences corr;

bool running = false;

//Descriptor Parameters
float shotRadius_ = 35;
float fpfhRadius_ = 20;
const bool gt_generation(false);

#if isshot
const float supportRadius_ = shotRadius_;
#else
const float supportRadius_ = fpfhRadius_;
#endif

Eigen::Matrix4f get_ransac_transformation_matrix(pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> &RansacRejector) {
	Eigen::Matrix4f mat = RansacRejector.getBestTransformation();
	cout << "RANSAC Transformation Matrix yielding the largest number of inliers.  : \n" << mat << endl;
	// int ransac_corr = corr.size();
	return mat;
}

Eigen::Matrix4f icp(pcl::PointCloud<PointType> model, pcl::PointCloud<PointType> scene) {
	Eigen::Matrix4f mat;
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(model.makeShared());
	icp.setInputTarget(scene.makeShared());
	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);
	std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;
	mat << icp.getFinalTransformation();
	return mat;
}

pcl::PointCloud<PointType> load_3dmodel(string filename, string fileformat) {
	pcl::PointCloud<PointType> temp_cloud;
	if (fileformat == "ply") { //if File is a pointcloud in ply format
		std::cout << "Filename read" << endl;
		if (pcl::io::loadPLYFile(filename, temp_cloud) == -1)
		{
			std::cout << "Error loading model cloud." << std::endl;
			return (temp_cloud);
		}
	}
	else if (fileformat == "pcd") { //if File is a pointcloud in pcd format
		std::cout << "Filename read" << endl;
		if (pcl::io::loadPCDFile(filename, temp_cloud) == -1)
		{
			std::cout << "Error loading model cloud." << std::endl;
			return (temp_cloud);
		}
	}
	else if (fileformat == "png") { //if depth image is stored as a png file (T-LESS dataset)
		char* file;
		FileHandler filehandler;
		file = &filename[0];
		temp_cloud = filehandler.getCloudFromPNG(file);
	}
	else {
		std::cout << "Unknown model file type. Check if there are any dots in the files path." << endl;
		return temp_cloud;
	}
	return temp_cloud;
}

pcl::PointCloud<PointType> addGaussianNoise(pcl::PointCloud<PointType> pointCloud, float standardDeviation) {
	//from: https://github.com/PointCloudLibrary/pcl/blob/master/tools/add_gaussian_noise.cpp
	std::random_device rd;
	std::mt19937 rng(rd());
	std::normal_distribution<float> nd(0.0f, standardDeviation);

	for (std::size_t point_i = 0; point_i < pointCloud.size(); ++point_i)
	{
		pointCloud.points[point_i].x = pointCloud.points[point_i].x + nd(rng);
		pointCloud.points[point_i].y = pointCloud.points[point_i].y + nd(rng);
		pointCloud.points[point_i].z = pointCloud.points[point_i].z + nd(rng);
	}
	return pointCloud;
}

pcl::Correspondences get_true_positives(float &distance_threshold, std::vector<float> &euclidean_distances){
	pcl::Correspondences temp;
	for (int i = 0; i < corr.size(); ++i) {
		if (euclidean_distances[i] < distance_threshold) {
			temp.push_back(corr.at(i));
		}
		else {
			std::cout << "FP @ " + to_string(corr.at(i).index_match) << endl;
		}
	}
	return temp;
}

std::vector<float> calculate_euclidean_distance(const pcl::PointCloud<pcl::PointXYZ> model_keypoints, const pcl::PointCloud<pcl::PointXYZ> scene_keypoints) {
	std::vector<float> distance;
	for (int i = 0; i < corr.size(); ++i) {
		distance.push_back((model_keypoints.at(corr.at(i).index_query).x - scene_keypoints.at(corr.at(i).index_match).x) *
			(model_keypoints.at(corr.at(i).index_query).x - scene_keypoints.at(corr.at(i).index_match).x) +
			(model_keypoints.at(corr.at(i).index_query).y - scene_keypoints.at(corr.at(i).index_match).y) *
			(model_keypoints.at(corr.at(i).index_query).y - scene_keypoints.at(corr.at(i).index_match).y) +
			(model_keypoints.at(corr.at(i).index_query).z - scene_keypoints.at(corr.at(i).index_match).z) *
			(model_keypoints.at(corr.at(i).index_query).z - scene_keypoints.at(corr.at(i).index_match).z));
		distance[i] = sqrt(distance[i]);
	}
	return distance;
}

string concatenate_results(KeypointDetector& Detector, std::vector<float>& euclidean_distance, const float& distance_threshold, const float& c_threshold) {
	std::string data = "";
	if (Detector.sceneKeypoints_.size() < Detector.modelKeypoints_.size()) {
		data = std::to_string(Detector.sceneKeypoints_.size()) + "," + std::to_string(distance_threshold) + "," + std::to_string(c_threshold) + "\n";
	}
	else {
		data = std::to_string(Detector.modelKeypoints_.size()) + "," + std::to_string(distance_threshold) + "," + std::to_string(c_threshold) + "\n";
	}
	for (int i = 0; i < corr.size(); ++i)
	{
		data += std::to_string(corr.at(i).distance) + ',' + std::to_string(euclidean_distance[i]);
		data += "\n";
	}
	return data;
}

double compute_cloud_resolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
//returns the mean distance from a point to its nearest neighbour
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
	std::cout << "Cloud resolution: " << res << endl;
	return res;
}

void ransac_rejection(pcl::Correspondences corresp, float resolution, pcl::PointCloud<pcl::PointXYZ> modelKeypoints, pcl::PointCloud<pcl::PointXYZ> sceneKeypoints, pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> &RansacRejector) {
	pcl::CorrespondencesConstPtr correspond = boost::make_shared<pcl::Correspondences>(corresp);
	double sac_threshold = 30.0 * resolution;
	RansacRejector.setInputSource(modelKeypoints.makeShared());
	RansacRejector.setInputTarget(sceneKeypoints.makeShared());
	RansacRejector.setInlierThreshold(sac_threshold);
	RansacRejector.setInputCorrespondences(correspond);
	RansacRejector.getCorrespondences(corr);
	std::cout << "# Correspondences found after RANSAC: " << corr.size() << endl;

}

void time_meas() {
	double cpuTimeUsed;
	if (!running) {
		running = true;
		start = clock();
	}
	else {
		end_time = clock();
		running = false;
		cpuTimeUsed = ((double)(end_time - start)) / CLOCKS_PER_SEC;
		std::cout << "Time taken: " << (double)cpuTimeUsed << std::endl;
	}
}

void time_meas(string action) {
	double cpuTimeUsed;
	if (!running){
		running = true;
		start = clock();
	}
	else {
		end_time = clock();
		running = false;
		cpuTimeUsed = ((double)(end_time - start)) / CLOCKS_PER_SEC;
		std::cout << "Time taken for: " + action + " " << (double)cpuTimeUsed << std::endl;
	}
}

void print_results(pcl::Correspondences& true_positives, KeypointDetector& detector) {
	std::cout << "TP: " << true_positives.size() << ", FP: " << corr.size() - true_positives.size() << endl;
	std::cout << "Precision: " << (float)true_positives.size() / (float)corr.size() << " Recall: " << true_positives.size() / (float)(detector.modelKeypoints_.size()) << endl;
};

int main(int argc, char* argv[])
{
	FileHandler filehandler;
	pcl::PointCloud<PointType> model;
	pcl::PointCloud<PointType> scene;
	float modelResolution, sceneResolution;
	pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> RansacRejector;

	//Get fileformat of the model -> last three letters of filename
	std::string model_fileformat = model_filename.std::string::substr(model_filename.length() - 3);
	std::string scene_fileformat = scene_filename.std::string::substr(scene_filename.length() - 3);

	model = load_3dmodel(model_filename, model_fileformat);
	scene = load_3dmodel(scene_filename, scene_fileformat);
	modelResolution = static_cast<float> (compute_cloud_resolution(model.makeShared()));
	sceneResolution = static_cast<float> (compute_cloud_resolution(scene.makeShared()));

	//calculate NormalEstimatorals
	time_meas();
	Normals NormalEstimator;
	NormalEstimator.model = model;
	NormalEstimator.scene = scene;
	NormalEstimator.calculateNormals(7.0f * modelResolution, 7.0f * sceneResolution);
	NormalEstimator.removeNaNNormals();
	model = NormalEstimator.model;
	scene = NormalEstimator.scene;
	time_meas("normal estimation");

	//Detect keypoints
	time_meas();
	KeypointDetector KeypointDetector;
	KeypointDetector.calculateIssKeypoints(model, scene, NormalEstimator.modelNormals_, modelResolution, sceneResolution, 0.7f);
	time_meas("detecting keypoints");

	//Calculate descriptor for each keypoint
	time_meas();
	Descriptor Describer;
	Describer.normal = NormalEstimator;
	Describer.keypointDetect = KeypointDetector;
	Describer.model_ = model;
	Describer.scene_ = scene;
	Describer.calculateDescriptor(supportRadius_ * modelResolution, supportRadius_ * sceneResolution);
	time_meas("calculating descriptor");
	
	float c_threshold = 0.95f;

	for (int j = 1; j < 6; ++j) {
		c_threshold += 0.01f;
		if (c_threshold > 1) {
			c_threshold = 1;
		}
		else {
			c_threshold = c_threshold;
		}
		//Matching
		time_meas();
		Matching Matcher;
		Matcher.desc = Describer;
		Matcher.calculateCorrespondences(c_threshold);
		time_meas("matching");
		if (Matcher.corresp.size() == 0) { continue; }
		else {
#if 1
			// RANSAC based Correspondence Rejection with ICP
			ransac_rejection(Matcher.corresp, modelResolution, KeypointDetector.modelKeypoints_, KeypointDetector.sceneKeypoints_, RansacRejector);
			Eigen::Matrix4f transformation_matrix = get_ransac_transformation_matrix(RansacRejector);
			pcl::transformPointCloud(KeypointDetector.modelKeypoints_, KeypointDetector.modelKeypoints_, transformation_matrix);
			pcl::transformPointCloud(model, model, transformation_matrix);
			// Iterative closest Point ICP
			transformation_matrix = icp(model, scene);
			pcl::transformPointCloud(KeypointDetector.modelKeypoints_, KeypointDetector.modelKeypoints_, transformation_matrix);
			pcl::transformPointCloud(model, model, transformation_matrix);
#else
			pcl::Correspondences corr = match.corresp;
#endif

			//Enable if the evaluation according to Guo et al. should be done
#if 1
			//Calculate euclidean distance of a model keypoint to its matched scene keypoint: sqrt(delta_x^2 + delta_y^2 + delta_z^2)
			std::vector<float> euclidean_distance;
			euclidean_distance = calculate_euclidean_distance(KeypointDetector.modelKeypoints_, KeypointDetector.sceneKeypoints_);

			//A match is considered TP if the euclidean distance of a model keypoint to its matched scene keypoint is less than half the supportradius
			pcl::Correspondences true_positives;
			float distance_threshold = shotRadius_ * modelResolution / 2;
			true_positives = get_true_positives(distance_threshold, euclidean_distance);

			//Store the NNDR and the euclidean distance for the evaluation according to Guo et al.
			std::string results = concatenate_results(KeypointDetector, euclidean_distance, distance_threshold, c_threshold);
			filehandler.writeToFile(results, pr_filename);
			print_results(true_positives, KeypointDetector);
#endif
		}
	}

//Enable if the visualization module should be used
#if 0
	//
	// VISUALIZATION MODULE
	//
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);


	//viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters();

	//Move model so that it is separated from the scene to see correspondences
	Eigen::Matrix4f t;
	t << 1, 0, 0, modelResolution * 200,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;

	pcl::transformPointCloud(KeypointDetector.modelKeypoints_, KeypointDetector.modelKeypoints_, t);
	pcl::transformPointCloud(model, model, t);
	//Add model keypoints to visualizer
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color1(KeypointDetector.modelKeypoints_.makeShared(), 200, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(KeypointDetector.modelKeypoints_.makeShared(), single_color1, "sample cloud1");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "sample cloud1");
	//add scene keypoints to visualizer
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(KeypointDetector.sceneKeypoints_.makeShared(), 0, 0, 150);
	viewer->addPointCloud<pcl::PointXYZ>(KeypointDetector.sceneKeypoints_.makeShared(), single_color2, "sample cloud2");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "sample cloud2");
	//add model points to visualizer
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color3(model.makeShared(), 155, 155, 155);
	viewer->addPointCloud<pcl::PointXYZ>(model.makeShared(), single_color3, "sample cloud3");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud3");
	//add scene points to visualizer
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color4(scene.makeShared(), 120, 120, 120);
	viewer->addPointCloud<pcl::PointXYZ>(scene.makeShared(), single_color4, "sample cloud4");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud4");
	//add lines between the correspondences
	viewer->addCorrespondences<pcl::PointXYZ>(KeypointDetector.modelKeypoints_.makeShared(), KeypointDetector.sceneKeypoints_.makeShared(), corr/*goodCorr/*corresp*/, "correspondences");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		std::this_thread::sleep_for(100ms);
	}
#endif
	return 0;
}