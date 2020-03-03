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
string modelFilename_;
string sceneFilename_;
//Descriptor Parameters
const float c_threshold = 1.0f;
float shotRadius_ = 35;
float fpfhRadius_ = 20;

#if isshot
const float supportRadius_ = shotRadius_;
#else
const float supportRadius_ = fpfhRadius_;
#endif
const bool gt_generation(false);

//returns the mean distance from a point to its nearest neighbour
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
	return res;
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

int main(int argc, char* argv[])
{
	pcl::PointCloud<PointType> model;
	pcl::PointCloud<PointType> scene;
	float modelResolution, sceneResolution;
	int nRows = 400, nCols = 400;
	modelFilename_ = "../Aufnahmen/Datensets/TLESS/Kinect/Scene/PointClouds/07/Modified/t_less_model07_median_0000_modified.ply";// T-LESS Pointcloud
	sceneFilename_ = "../Aufnahmen/Datensets/TLess/Kinect/Scene/PointClouds/07/Modified/t_less_model07_median_0001_modified.ply";
	std::cout << "Enter scene filename: ";
	getline(cin, sceneFilename_);
	//
	//Read input files and make a Point Cloud
	//
	FileHandler filehandler;
	//Get fileformat of the model
	std::string fileformat = modelFilename_.std::string::substr(modelFilename_.length() - 3); //get the fileformat -> at end of filename
	//Read the model
	if (fileformat == "ply") { //if File is a pointcloud in ply format
		std::cout << "Filename read" << endl;
		if (pcl::io::loadPLYFile(modelFilename_, model) == -1)
		{
			std::cout << "Error loading model cloud." << std::endl;
			return (-1);
		}
	}
	else if (fileformat == "pcd") { //if File is a pointcloud in pcd format
		std::cout << "Filename read" << endl;
		if (pcl::io::loadPCDFile(modelFilename_, model) == -1)
		{
			std::cout << "Error loading model cloud." << std::endl;
			return (-1);
		}
	}
	else if (fileformat == "csv") { //if depth image is stored in a csv file (epc660)
		std::string bkgrFilename;
		std::cout << "Enter Background filename: ";
		getline(cin, bkgrFilename);
		std::vector<std::vector<float>> cloud = filehandler.readCsv(modelFilename_);
		std::vector<std::vector<float>> background = filehandler.readCsv(bkgrFilename);
		if (cloud.size() < 240 || background.size() < 240) {
			return -1; //file was not read correctly
		}
		pcl::PointCloud<pcl::PointXYZ> pointCloud = filehandler.getCloud(cloud, 16.0f, 0.02f, 0.02f, 0.05f);
		pcl::PointCloud<pcl::PointXYZ> finalCloud;
		pcl::PointCloud<pcl::PointXYZ> bkgrCloud = filehandler.getCloud(background, 16.0f, 0.02f, 0.02f, 0.05f);
		float z;
		PointXYZ point;
		//subtract background from image
		for (int i = 0; i < pointCloud.size(); ++i) {
			z = bkgrCloud.at(i).z - pointCloud.at(i).z;
			if (z > 0.002) {//do not add points below 2mm (they belong to the background)
				point.x = pointCloud.at(i).x;
				point.y = pointCloud.at(i).y;
				point.z = z;
				finalCloud.push_back(point);
			}
		}
		model = finalCloud;
	}
	else if (fileformat == "png") { //if depth image is stored as a png file (T-LESS dataset)
		char *file;
		file = &modelFilename_[0];
		model = filehandler.getCloudFromPNG(file);
	}
	else {
		std::cout << "Unknown model file type. Check if there are any dots in the files path." << endl;
		return -1;
	}

	//Get fileformat of the scene
	fileformat = sceneFilename_.std::string::substr(sceneFilename_.length() - 3); //get the fileformat -> at end of filename
	//Read the scene
	if (fileformat == "ply") { //if File is a pointcloud in ply format
		std::cout << "Filename read" << endl;
		if (pcl::io::loadPLYFile(sceneFilename_, scene) == -1)
		{
			std::cout << "Error loading model cloud." << std::endl;
			return (-1);
		}
	}
	else if (fileformat == "pcd") { //if File is a pointcloud in pcd format
		std::cout << "Filename read" << endl;
		if (pcl::io::loadPCDFile(sceneFilename_, scene) == -1)
		{
			std::cout << "Error loading model cloud." << std::endl;
			return (-1);
		}
	}
	else if (fileformat == "csv") { //if depth image(s) is(are) stored in a csv file (epc660)
		std::string bkgrFilename;
		std::cout << "Enter Background filename: ";
		getline(cin, bkgrFilename);
		std::vector<std::vector<float>> cloud = filehandler.readCsv(sceneFilename_);
		std::vector<std::vector<float>> background = filehandler.readCsv(bkgrFilename);
		if (cloud.size() < 240 || background.size() < 240) {
			return 2; //file was not read correctly
		}
		pcl::PointCloud<pcl::PointXYZ> pointCloud = filehandler.getCloud(cloud, 16.0f, 0.02f, 0.02f, 0.05f);
		pcl::PointCloud<pcl::PointXYZ> finalCloud;
		pcl::PointCloud<pcl::PointXYZ> bkgrCloud = filehandler.getCloud(background, 16.0f, 0.02f, 0.02f, 0.05f);
		float z;
		PointXYZ point;
		//subtract background from image
		for (int i = 0; i < pointCloud.size(); ++i) {
			z = bkgrCloud.at(i).z - pointCloud.at(i).z;
			if (z > 0.002) {
				point.x = pointCloud.at(i).x;
				point.y = pointCloud.at(i).y;
				point.z = z;
				finalCloud.push_back(point);
			}
		}
		scene = finalCloud;
	}
	else if (fileformat == "png") { //if depth image is stored as a png file (T-LESS dataset)
		char* file;
		file = &sceneFilename_[0];
		scene = filehandler.getCloudFromPNG(file);
			}
	else {
		std::cout << "Unknown scene file type. Check if there are any dots in the files path." << endl;
		return -1;
	}

	Eigen::Matrix4f matrix;
	//calculate model and scene resolution
	modelResolution = static_cast<float> (computeCloudResolution(model.makeShared()));
	sceneResolution = static_cast<float> (computeCloudResolution(scene.makeShared()));
	std::cout << "Model resolution: " << modelResolution << endl;
	std::cout << "Scene resolution: " << sceneResolution << endl;

	//
	//Add Gaussian Noise to evaluate the best object
	//
	//scene = addGaussianNoise(scene,sceneResolution);


	clock_t start, end;
	double cpuTimeUsed;
	start = clock();
	//
	//calculate normals
	//
	Normals norm;
	norm.model = model;
	norm.scene = scene;
	norm.calculateNormals(5.0f * modelResolution, 5.0f * sceneResolution);

	//Remove NAN normals and their correspondences in model and scene
	std::vector<int> index;
	pcl::removeNaNNormalsFromPointCloud(norm.modelNormals_, norm.modelNormals_, index);
	pcl::PointCloud<pcl::PointXYZ> tempCloud;
	for (int i = 0; i < norm.modelNormals_.size(); ++i) {
		tempCloud.push_back(model.at(index[i]));
	}
	model = tempCloud;
	pcl::removeNaNNormalsFromPointCloud(norm.sceneNormals_, norm.sceneNormals_, index);
	tempCloud.clear();
	for (int i = 0; i < norm.sceneNormals_.size(); ++i) {
		tempCloud.push_back(scene.at(index[i]));
	}
	scene = tempCloud;

	end = clock();
	cpuTimeUsed = ((double)(end - start)) / CLOCKS_PER_SEC;
	std::cout << "Time taken for normal estimation: " << (double)cpuTimeUsed << std::endl;

	clock_t start1, end1;
	double cpuTimeUsed1;
	start1 = clock();
	//
	//Detect keypoints
	//
	KeypointDetector keypointDetect;
	keypointDetect.calculateIssKeypoints(model, scene, modelResolution, sceneResolution, 0.975f);
	//keypointDetect.calculateVoxelgridKeypoints(model, scene, 11 * modelResolution, 11 * sceneResolution);
	end1 = clock();
	cpuTimeUsed1 = ((double)(end1 - start1)) / CLOCKS_PER_SEC;
	std::cout << "Time taken for Keypoint Detection : " << (double)cpuTimeUsed1 << std::endl;

	std::cout << "No. Model Keypoints: " << keypointDetect.modelKeypoints_.size() << " of: " << model.size() << endl;
	std::cout << "No. Scene Keypoints: " << keypointDetect.sceneKeypoints_.size() << " of: " << scene.size() << endl;


	clock_t start2, end2;
	double cpuTimeUsed2;
	start2 = clock();
	//
	//Calculate descriptor for each keypoint
	//
	Descriptor des;
	des.normal = norm;
	des.keypointDetect = keypointDetect;
	des.model_ = model;
	des.scene_ = scene;


	des.calculateDescriptor(supportRadius_ * modelResolution, supportRadius_ * sceneResolution);

	end2 = clock();
	cpuTimeUsed2 = ((double)(end2 - start2)) / CLOCKS_PER_SEC;
	std::cout << "Time taken for Descriptor : " << (double)cpuTimeUsed2 << std::endl;


	clock_t start3, end3;
	double cpu_time_used3;
	start3 = clock();
	//
	//Matching
	//
	Matching match;
	match.desc = des;
	match.calculateCorrespondences(c_threshold);
	end3 = clock();
	cpu_time_used3 = ((double)(end3 - start3)) / CLOCKS_PER_SEC;
	std::cout << "Time taken for Matching : " << (double)cpu_time_used3 << std::endl;

	std::cout << "Correspondences found: " << match.corresp.size() << endl;
	//
	// RANSAC based Correspondence Rejection with ICP
	//
#if 1
	pcl::CorrespondencesConstPtr correspond = boost::make_shared< pcl::Correspondences >(match.corresp);
	pcl::Correspondences corr;
	pcl::registration::CorrespondenceRejectorSampleConsensus< pcl::PointXYZ > Ransac_based_Rejection;
	Ransac_based_Rejection.setInputSource(keypointDetect.modelKeypoints_.makeShared());
	Ransac_based_Rejection.setInputTarget(keypointDetect.sceneKeypoints_.makeShared());
	double sac_threshold = 30.0 * modelResolution;
	Ransac_based_Rejection.setInlierThreshold(sac_threshold);
	Ransac_based_Rejection.setInputCorrespondences(correspond);
	Ransac_based_Rejection.getCorrespondences(corr);

	std::cout << "Correspondences found after(RANSAC): " << corr.size() << endl;
	Eigen::Matrix4f mat = Ransac_based_Rejection.getBestTransformation();
	cout << "Mat : \n" << mat << endl;
	int ransac_corr = corr.size();
	corr = match.corresp;		//comment out if RANSAC should be used
	pcl::transformPointCloud(keypointDetect.modelKeypoints_, keypointDetect.modelKeypoints_, mat);
	pcl::transformPointCloud(model, model, mat);

	//
	// Iterative closest Point ICP
	//
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(model.makeShared());
	icp.setInputTarget(scene.makeShared());
	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);
	std::cout << "has converged:" << icp.hasConverged() << " score: " <<
		icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;
	matrix << icp.getFinalTransformation();
	pcl::transformPointCloud(keypointDetect.modelKeypoints_, keypointDetect.modelKeypoints_, matrix);
	pcl::transformPointCloud(model, model, matrix);


	std::string data = "";
//Enable if the evaluation according to Buch et al. should be done
#if 0
	string filename = "../../../../BAT/Tests/PR_Kurven/TLess/Kinect/Applikationsspezifisch/07/rotation/tless_dataset_shot.csv";
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
#if 0

	//Calculate euclidean distance of a model keypoint to its matched scene keypoint
	std::vector<float> distance;
	for (int i = 0; i < corr.size(); ++i) {
		distance.push_back((keypointDetect.modelKeypoints_.at(corr.at(i).index_query).x - keypointDetect.sceneKeypoints_.at(corr.at(i).index_match).x) * (keypointDetect.modelKeypoints_.at(corr.at(i).index_query).x - keypointDetect.sceneKeypoints_.at(corr.at(i).index_match).x) + (keypointDetect.modelKeypoints_.at(corr.at(i).index_query).y - keypointDetect.sceneKeypoints_.at(corr.at(i).index_match).y) * (keypointDetect.modelKeypoints_.at(corr.at(i).index_query).y - keypointDetect.sceneKeypoints_.at(corr.at(i).index_match).y) + (keypointDetect.modelKeypoints_.at(corr.at(i).index_query).z - keypointDetect.sceneKeypoints_.at(corr.at(i).index_match).z) * (keypointDetect.modelKeypoints_.at(corr.at(i).index_query).z - keypointDetect.sceneKeypoints_.at(corr.at(i).index_match).z));
		distance[i] = sqrt(distance[i])
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

	//
	//Store the NNDR and the euclidean distance for the evaluation according to Guo et al.
	//The evaluation is done in MATLAB
	//
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

	if (isshot) {
		filehandler.writeToFile(data, "tless_dataset_shot.csv");
	}
	else {
		filehandler.writeToFile(data, "tless_dataset_fpfh.csv");
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
	viewer->setBackgroundColor(255, 255, 255);


	//viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters();

	//Move model so that it is separated from the scene to see correspondences
	Eigen::Matrix4f t;
	t << 1, 0, 0, modelResolution * 400,
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
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
#endif
	return 0;
}