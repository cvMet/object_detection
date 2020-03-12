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
string modelFilename_ = "../../../../Aufnahmen/Datensets/Melexis_Punktwolken/001_Bottlewhite1model.ply";
string sceneFilename_ = "../../../../Aufnahmen/Datensets/Melexis_Punktwolken/003_Bottlewhite3model.ply";
string filename = modelFilename_.substr(52, (modelFilename_.length() - 56));
string pr_filename = "../../../../PR/" + filename + ".csv";
clock_t start, end_time;

bool running = false;

//Descriptor Parameters
const float c_threshold = 1.0f;
float shotRadius_ = 35;
float fpfhRadius_ = 20;
const bool gt_generation(false);

#if isshot
const float supportRadius_ = shotRadius_;
#else
const float supportRadius_ = fpfhRadius_;
#endif

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

double computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
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

//pcl::Correspondences ransac_rejection(pcl::Correspondences corresp, float resolution, pcl::PointCloud<pcl::PointXYZ> modelKeypoints, pcl::PointCloud<pcl::PointXYZ> sceneKeypoints) {
//	pcl::CorrespondencesConstPtr correspond = boost::make_shared<pcl::Correspondences>(corresp);
//	double sac_threshold = 30.0 * resolution;
//	pcl::Correspondences corr;
//	pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> Ransac_Rejector;
//	Ransac_Rejector.setInputSource(modelKeypoints.makeShared());
//	Ransac_Rejector.setInputTarget(sceneKeypoints.makeShared());
//	Ransac_Rejector.setInlierThreshold(sac_threshold);
//	Ransac_Rejector.setInputCorrespondences(correspond);
//	Ransac_Rejector.getCorrespondences(corr);
//	std::cout << "Correspondences found after(RANSAC): " << corr.size() << endl;
//	return corr;
//}

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

int main(int argc, char* argv[])
{
	FileHandler filehandler;
	Eigen::Matrix4f matrix;
	pcl::PointCloud<PointType> model;
	pcl::PointCloud<PointType> scene;
	float modelResolution, sceneResolution;

	//Get fileformat of the model -> last three letters of filename
	std::string model_fileformat = modelFilename_.std::string::substr(modelFilename_.length() - 3);
	std::string scene_fileformat = sceneFilename_.std::string::substr(sceneFilename_.length() - 3);

	model = load_3dmodel(modelFilename_, model_fileformat);
	scene = load_3dmodel(sceneFilename_, scene_fileformat);
	//calculate model and scene resolution
	modelResolution = static_cast<float> (computeCloudResolution(model.makeShared()));
	sceneResolution = static_cast<float> (computeCloudResolution(scene.makeShared()));

	//calculate normals
	time_meas();
	Normals norm;
	norm.model = model;
	norm.scene = scene;
	norm.calculateNormals(7.0f * modelResolution, 7.0f * sceneResolution);
	norm.removeNaNNormals();
	model = norm.model;
	scene = norm.scene;
	time_meas("normal estimation");

	//Detect keypoints
	time_meas();
	KeypointDetector keypointDetect;
	keypointDetect.calculateIssKeypoints(model, scene, norm.modelNormals_, modelResolution, sceneResolution, 0.7f);
	time_meas("detecting keypoints");

	//Calculate descriptor for each keypoint
	time_meas();
	Descriptor des;
	des.normal = norm;
	des.keypointDetect = keypointDetect;
	des.model_ = model;
	des.scene_ = scene;
	des.calculateDescriptor(supportRadius_ * modelResolution, supportRadius_ * sceneResolution);
	time_meas("calculating descriptor");

	//Matching
	time_meas();
	Matching match;
	match.desc = des;
	match.calculateCorrespondences(c_threshold);
	time_meas("matching");

	// RANSAC based Correspondence Rejection with ICP
#if 1
	//corr = ransac_rejection(match.corresp);

	pcl::CorrespondencesConstPtr correspond = boost::make_shared<pcl::Correspondences>(match.corresp);
	pcl::Correspondences corr;
	double sac_threshold = 30.0 * modelResolution;
	pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> Ransac_Rejector;
	Ransac_Rejector.setInputSource(keypointDetect.modelKeypoints_.makeShared());
	Ransac_Rejector.setInputTarget(keypointDetect.sceneKeypoints_.makeShared());
	Ransac_Rejector.setInlierThreshold(sac_threshold);
	Ransac_Rejector.setInputCorrespondences(correspond);
	Ransac_Rejector.getCorrespondences(corr);
	std::cout << "Correspondences found after(RANSAC): " << corr.size() << endl;


	Eigen::Matrix4f mat = Ransac_Rejector.getBestTransformation();
	cout << "Ransac Transformation Matrix yielding the largest number of inliers.  : \n" << mat << endl;
	int ransac_corr = corr.size();
	corr = match.corresp;		//comment out if RANSAC should be used
	pcl::transformPointCloud(keypointDetect.modelKeypoints_, keypointDetect.modelKeypoints_, mat);
	pcl::transformPointCloud(model, model, mat);

	// Iterative closest Point ICP
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(model.makeShared());
	icp.setInputTarget(scene.makeShared());
	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);
	std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;
	matrix << icp.getFinalTransformation();
	pcl::transformPointCloud(keypointDetect.modelKeypoints_, keypointDetect.modelKeypoints_, matrix);
	pcl::transformPointCloud(model, model, matrix);
	std::string data = "";

	//Enable if the evaluation according to Buch et al. should be done
#if 0
	string filename = "Buch_tless_dataset_shot.csv";
	std::ifstream file(filename);
	if (!file.is_open()) {
		std::cout << "File: " << " could not be read" << std::endl;
	}
	std::string line = "";
	// Iterate through each line and split the content using delimeter
	std::vector<std::vector<float>> pointcloud(nRows, std::vector<float>(nCols, 0));
	int indRow = 0;
	data = "";
	while (getline(file, line)) {
		data += line + "\n";
	}
	//Add a new entry and save the file
	//first column are the number of matches before, second after RANSAC
	//third column specifies if the object is positive (1) or negative (0) needs to be changed manually
	data += to_string(corr.size()) + "," + to_string(ransac_corr) + ",1" + "\n";
	filehandler.writeToFile(data, filename);
#endif

#else
	pcl::Correspondences corr = match.corresp;
#endif

	//Enable if the evaluation according to Guo et al. should be done
#if 1
	//Calculate euclidean distance of a model keypoint to its matched scene keypoint: sqrt(delta_x^2 + delta_y^2 + delta_z^2)
	std::vector<float> distance;
	for (int i = 0; i < corr.size(); ++i) {
		distance.push_back((keypointDetect.modelKeypoints_.at(corr.at(i).index_query).x - keypointDetect.sceneKeypoints_.at(corr.at(i).index_match).x) *
			(keypointDetect.modelKeypoints_.at(corr.at(i).index_query).x - keypointDetect.sceneKeypoints_.at(corr.at(i).index_match).x) + 
			(keypointDetect.modelKeypoints_.at(corr.at(i).index_query).y - keypointDetect.sceneKeypoints_.at(corr.at(i).index_match).y) * 
			(keypointDetect.modelKeypoints_.at(corr.at(i).index_query).y - keypointDetect.sceneKeypoints_.at(corr.at(i).index_match).y) + 
			(keypointDetect.modelKeypoints_.at(corr.at(i).index_query).z - keypointDetect.sceneKeypoints_.at(corr.at(i).index_match).z) * 
			(keypointDetect.modelKeypoints_.at(corr.at(i).index_query).z - keypointDetect.sceneKeypoints_.at(corr.at(i).index_match).z));
		distance[i] = sqrt(distance[i]);
	}
	//Get number of TP's
	//A match is considered TP if the euclidean distance of a model keypoint to its
	//matched scene keypoint is less than half the supportradius
	pcl::Correspondences tpCorr;
	float dist_thresh = shotRadius_ * modelResolution / 2;
	for (int i = 0; i < corr.size(); ++i) {
		if (distance[i] < dist_thresh) {
			tpCorr.push_back(corr.at(i));
		}
	}
	//Store the NNDR and the euclidean distance for the evaluation according to Guo et al.
	//The evaluation is done in MATLAB
	if (keypointDetect.sceneKeypoints_.size() < keypointDetect.modelKeypoints_.size()) {
		data = std::to_string(keypointDetect.sceneKeypoints_.size()) + "," + std::to_string(dist_thresh) + "\n";
	}
	else {
		data = std::to_string(keypointDetect.modelKeypoints_.size()) + "," + std::to_string(dist_thresh) + "\n";
	}
	pcl::PointCloud<pcl::PointXYZ> keypoints;
	for (int i = 0; i < corr.size(); ++i)
	{
		data += std::to_string(corr.at(i).distance) + ',' + std::to_string(distance[i]);
		data += "\n";
		keypoints.push_back(keypointDetect.sceneKeypoints_.at(corr.at(i).index_match));
	}
	//Write to File
	if (isshot) {
		filehandler.writeToFile(data, pr_filename);
	}
	else {
		filehandler.writeToFile(data, pr_filename);
	}
	//Calculate Precision and Recall and print them
	std::cout << "TP: " << tpCorr.size() << ", FP: " << corr.size() - tpCorr.size() << endl;
	std::cout << "Precision: " << (float)tpCorr.size() / (float)corr.size() << " Recall: " << tpCorr.size() / (float)(keypointDetect.modelKeypoints_.size()) << endl;
#endif
//Enable if the visualization module should be used
#if 1
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

	pcl::transformPointCloud(keypointDetect.modelKeypoints_, keypointDetect.modelKeypoints_, t);
	pcl::transformPointCloud(model, model, t);
	//Add model keypoints to visualizer
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color1(keypointDetect.modelKeypoints_.makeShared(), 200, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(keypointDetect.modelKeypoints_.makeShared(), single_color1, "sample cloud1");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "sample cloud1");
	//add scene keypoints to visualizer
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(keypointDetect.sceneKeypoints_.makeShared(), 0, 0, 150);
	viewer->addPointCloud<pcl::PointXYZ>(keypointDetect.sceneKeypoints_.makeShared(), single_color2, "sample cloud2");
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
	viewer->addCorrespondences<pcl::PointXYZ>(keypointDetect.modelKeypoints_.makeShared(), keypointDetect.sceneKeypoints_.makeShared(), corr/*goodCorr/*corresp*/, "correspondences");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		std::this_thread::sleep_for(100ms);
	}
#endif
	return 0;
}