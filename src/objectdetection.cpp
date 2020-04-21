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
#include "../include/cloud_creator.h"
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/registration/icp.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/shadowpoints.h>
#include <random>

#ifndef isshot
#define isshot 1	//For SHOT-Descriptor use 1 for FPFH 0
#endif

//Namespaces
namespace fs = boost::filesystem;
using namespace std;
using namespace pcl;

//Model is to be matched in scene

clock_t start, end_time;
pcl::Correspondences corr;
bool running = false;
int accumulated_keypoints = 0;
const int rows = 240, columns = 320;

//Descriptor Parameters
float shotRadius_ = 35;
float fpfhRadius_ = 20;

#if isshot
const float supportRadius_ = shotRadius_;
string descriptor = "B_SHOT";
#else
const float supportRadius_ = fpfhRadius_;
string descriptor = "FPFH";
#endif

Eigen::Matrix4f get_ransac_transformation_matrix(pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> &RansacRejector) {
	cout << "# Iterations: " << RansacRejector.getMaximumIterations() << endl;
	Eigen::Matrix4f mat = RansacRejector.getBestTransformation();
	cout << "RANSAC Transformation Matrix yielding the largest number of inliers.  : \n" << mat << endl;
	// int ransac_corr = corr.size();
	return mat;
}

Eigen::Matrix4f icp(pcl::PointCloud<PointType> model, pcl::PointCloud<PointType> scene) {
	Eigen::Matrix4f mat, guess;
	guess <<	-1, 0, 0, 0,
				0, 1, 0, 0,
				0, 0, -1, 0,
				0, 0, 0, 1;
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(model.makeShared());
	icp.setInputTarget(scene.makeShared());
	pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
	icp.align(aligned_cloud);
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
		distance.push_back(
			pow((model_keypoints.at(corr.at(i).index_query).x - scene_keypoints.at(corr.at(i).index_match).x), 2) +
			pow((model_keypoints.at(corr.at(i).index_query).y - scene_keypoints.at(corr.at(i).index_match).y), 2) +
			pow((model_keypoints.at(corr.at(i).index_query).z - scene_keypoints.at(corr.at(i).index_match).z), 2)
		);
		distance[i] = sqrt(distance[i]);
	}
	return distance;
}

vector<tuple<string, float>> assemble_stats(vector<tuple<string, float>> processing_times, pcl::PointCloud<PointType> model, pcl::PointCloud<PointType> scene, float modelResolution, float sceneResolution, KeypointDetector& Detector) {
	vector<tuple<string, float>> stats;

	stats.push_back(make_tuple("suport_radius", float(supportRadius_)));
	stats.push_back(make_tuple("points_model_cloud", float(model.points.size())));
	stats.push_back(make_tuple("points_scene_cloud", float(scene.points.size())));
	stats.push_back(make_tuple("model_resolution", float(modelResolution)));
	stats.push_back(make_tuple("scene_resolution", float(sceneResolution)));
	stats.push_back(make_tuple("model_keypoints", float(Detector.modelKeypoints_.size())));
	stats.push_back(make_tuple("scene_keypoints", float(Detector.sceneKeypoints_.size())));

	for (int i = 0; i < processing_times.size(); ++i) {
		stats.push_back(processing_times[i]);
	}
	return stats;
}

string concatenate_results(KeypointDetector& Detector, std::vector<float>& euclidean_distance, const float& distance_threshold) {
	std::string data = "";
	if (Detector.sceneKeypoints_.size() < Detector.modelKeypoints_.size()) {
		data = std::to_string(Detector.sceneKeypoints_.size()) + "," + std::to_string(distance_threshold) +  "\n";
	}
	else {
		data = std::to_string(Detector.modelKeypoints_.size()) + "," + std::to_string(distance_threshold) +  "\n";
	}
	for (int i = 0; i < corr.size(); ++i)
	{
		data += std::to_string(corr.at(i).distance) + ',' + std::to_string(euclidean_distance[i]);
		data += "\n";
	}
	return data;
}

string create_writable_stats(vector<tuple<string, float>> stats) {
	std::string data = "";
	for (int i = 0; i < stats.size(); ++i) {
		data += get<0>(stats[i]) + ',' + std::to_string(get<1>(stats[i]));
		data += "\n";
	}
	return data;
}

string concatenate_distances(std::vector<float>& euclidean_distance) {
	std::string data = "";
	for (int i = 0; i < corr.size(); ++i)
	{
		data += std::to_string(corr.at(i).distance) + ',' + std::to_string(euclidean_distance[i]);
		data += "\n";
	}
	return data;
}

string get_fileformat(string filename) {
	//Get fileformat of the model -> last three letters of filename
	return filename.std::string::substr(filename.length() - 3);
}

string get_identifier(string filename, int name_pos, int extension_pos){
	return filename.substr(name_pos, (extension_pos - name_pos));
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

tuple<string, float> time_meas(string action) {
	double cpuTimeUsed;
	if (!running){
		running = true;
		start = clock();
	}
	else {
		end_time = clock();
		running = false;
		cpuTimeUsed = ((double)end_time - (double)start) / CLOCKS_PER_SEC;
		std::cout << "Time taken for: " + action + " " << (double)cpuTimeUsed << std::endl;
	}
	return make_tuple(action, cpuTimeUsed);
}

void print_results(pcl::Correspondences& true_positives, KeypointDetector& detector) {
	std::cout << "TP: " << true_positives.size() << ", FP: " << corr.size() - true_positives.size() << endl;
	std::cout << "Precision: " << (float)true_positives.size() / (float)corr.size() << " Recall: " << true_positives.size() / (float)(detector.modelKeypoints_.size()) << endl;
};

void accumulate_keypoints(KeypointDetector& Detector) {
	if (Detector.sceneKeypoints_.size() < Detector.modelKeypoints_.size()) {
		accumulated_keypoints += Detector.sceneKeypoints_.size();
	}
	else {
		accumulated_keypoints += Detector.modelKeypoints_.size();
	}
}

void get_all_file_names(const fs::path& root, const string& ext, vector<fs::path>& ret)
{
	if (!fs::exists(root) || !fs::is_directory(root)) return;

	fs::recursive_directory_iterator it(root);
	fs::recursive_directory_iterator endit;

	while (it != endit)
	{
		if (fs::is_regular_file(*it) && it->path().extension() == ext) ret.push_back(it->path().filename());
		++it;

	}
}


int main(int argc, char* argv[])
{
	pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> RansacRejector;
	FileHandler FileHandler;
	CloudCreator CloudCreator;

	//Clouds used during cloud generation
	pcl::PointCloud<pcl::PointXYZ> model_cloud;
	pcl::PointCloud<pcl::PointXYZ> background_cloud;
	pcl::PointCloud<pcl::PointXYZ> filtered_model;
	pcl::PointCloud<pcl::PointXYZ> filtered_background;
	pcl::PointCloud<pcl::PointXYZ> final_cloud;
	pcl::PointCloud<pcl::PointXYZ> roi_cloud;

	//Clouds used during objectdetection
	pcl::PointCloud<PointType> model;
	pcl::PointCloud<PointType> scene;

	vector<fs::path> model_depth_names;
	vector<fs::path> background_depth_names;
	vector<fs::path> cloud_names;

	vector<float> keypointdetector_threshold = {0.7f };
	vector<int> keypointdetector_nof_neighbors = {3 };

	float modelResolution, sceneResolution;

	//Paths used during cloudgen
	string model_depth_directory = "../../../../datasets/conveyor/raw_depth/gipfeli/0_tran/model";
	string background_depth_directory = "../../../../datasets/conveyor/raw_depth/gipfeli/0_tran/background";
	string depth_extension = ".txt";

	//cloudgen & objectdetection
	string cloud_directory = "../../../../clouds/conveyor/g/3d_filtered/0_tran";

	//path used during objectdetection
	string pr_root = "../../../../PR/Buch";
	string stats_root = "../../../../stats";
	string object = "Gipfeli";
	string dataset = "conveyor";
	string preprocessor_mode = "3d_filtered";
	string transformation = "0_tran";

	get_all_file_names(model_depth_directory, depth_extension, model_depth_names);
	get_all_file_names(background_depth_directory, depth_extension, background_depth_names);

	//Define if clouds need to be generated first
#if 1

	std::string bkgr_depth_filename = background_depth_directory + "/" + background_depth_names[0].string();
	std::vector<std::vector<float>> background_distance_array = FileHandler.melexis_txt_to_distance_array(bkgr_depth_filename, rows, columns);
	background_cloud = CloudCreator.distance_array_to_cloud(background_distance_array, 6.0f, 0.015f, 0.015f);
	filtered_background = CloudCreator.median_filter_cloud(background_cloud, 10);
	//CloudCreator.show_cloud(filtered_background);

	for (int i = 0; i < model_depth_names.size(); ++i) {
		vector<tuple<string, float>> creation_stats;
		std::string model_depth_filename = model_depth_directory + "/" + model_depth_names[i].string();

		time_meas();
		std::vector<std::vector<float>> model_distance_array = FileHandler.melexis_txt_to_distance_array(model_depth_filename, rows, columns);
		creation_stats.push_back(time_meas("time_textToDistance"));
		time_meas();
		model_cloud = CloudCreator.distance_array_to_cloud(model_distance_array, 6.0f, 0.015f, 0.015f);
		creation_stats.push_back(time_meas("time_distanceToCloud"));
		time_meas();
		filtered_model = CloudCreator.median_filter_cloud(model_cloud, 10);
		//	CloudCreator.show_cloud(filtered_model);
		creation_stats.push_back(time_meas("time_medianfilter"));
		time_meas();
		final_cloud = CloudCreator.remove_background(filtered_model, filtered_background, 0.004f);

		creation_stats.push_back(time_meas("time_backgroundRemoval"));
		roi_cloud = CloudCreator.roi_filter(final_cloud, "x", -0.2f, 0.125f);

		string filename = cloud_directory + "/" + background_depth_names[i].string();
		string substr = filename.substr(0, (filename.length() - 4)) + ".ply";
		string statistics = create_writable_stats(creation_stats);
		string stats_filename = filename.substr(0, (filename.length() - 4)) + "_stats.csv";
		pcl::io::savePLYFileASCII(substr, roi_cloud);
		FileHandler.writeToFile(statistics, stats_filename);
	}
#endif

//enable for automated detection process
#if 0
	get_all_file_names(cloud_directory, ".ply", cloud_names);
	//string model_filename = cloud_directory + "/" + cloud_names[0].string();
	string model_filename = "../../../../clouds/conveyor/g/3d_filtered/0_tran/Gipfeli_2.ply";

	for (int neighbor = 0; neighbor < keypointdetector_nof_neighbors.size(); ++neighbor) {
		for (int threshold = 0; threshold < keypointdetector_threshold.size(); ++threshold) {
			for (int scene_number = 0; scene_number < cloud_names.size(); ++scene_number) {
				vector<tuple<string, float>> processing_times;
				vector<tuple<string, float>> stats;
				Normals NormalEstimator;
				KeypointDetector KeypointDetector;
				Descriptor Describer;
				Matching Matcher;

				string scene_filename = cloud_directory + "/" + cloud_names[scene_number].string();
				int name_pos = scene_filename.find("Gipfeli", 0);
				int extension_pos = scene_filename.find(".ply", 0);
				string model_identifier = get_identifier(model_filename, name_pos, extension_pos);
				string scene_identifier = get_identifier(scene_filename, name_pos, extension_pos);
				string pr_filename = pr_root		+ "/" + dataset + "/" + object + "/" + preprocessor_mode + "/" + descriptor
													+ "/iss" + std::to_string(keypointdetector_nof_neighbors[neighbor])
													+ "_th" + std::to_string(keypointdetector_threshold[threshold]).substr(0,3)
													+ "/" + transformation
													+ "/"	+ model_identifier + "_to_" + scene_identifier + ".csv";
				string stats_filename = stats_root	+ "/" + dataset + "/" + object + "/" + preprocessor_mode + "/" + descriptor
													+ "/iss" + std::to_string(keypointdetector_nof_neighbors[neighbor]) 
													+ "_th"	+ std::to_string(keypointdetector_threshold[threshold]).substr(0, 3)
													+ "/" + transformation
													+ "/" + model_identifier + "_to_" + scene_identifier + "_stats.csv";

				std::string model_fileformat = get_fileformat(model_filename);
				std::string scene_fileformat = get_fileformat(scene_filename);

				model = load_3dmodel(model_filename, model_fileformat);
				scene = load_3dmodel(scene_filename, scene_fileformat);
				modelResolution = static_cast<float> (compute_cloud_resolution(model.makeShared()));
				sceneResolution = static_cast<float> (compute_cloud_resolution(scene.makeShared()));

				//Estimate Normals
				time_meas();
				NormalEstimator.model = model;
				NormalEstimator.scene = scene;
				NormalEstimator.calculateNormals(7.0f * modelResolution, 7.0f * sceneResolution);
				NormalEstimator.removeNaNNormals();
				model = NormalEstimator.model;
				scene = NormalEstimator.scene;
				processing_times.push_back(time_meas("normal estimation"));

				//Detect keypoints
				time_meas();
				KeypointDetector.calculateIssKeypoints(model, scene, NormalEstimator.modelNormals_, modelResolution, sceneResolution,
					keypointdetector_threshold[threshold] ,keypointdetector_nof_neighbors[neighbor]);
				processing_times.push_back(time_meas("detecting keypoints"));

				//Calculate descriptor for each keypoint
				time_meas();
				Describer.NormalEstimator = NormalEstimator;
				Describer.KeypointDetector = KeypointDetector;
				Describer.model_ = model;
				Describer.scene_ = scene;
				Describer.calculateDescriptor(supportRadius_ * modelResolution, supportRadius_ * sceneResolution);
				processing_times.push_back(time_meas("calculating descriptor"));

				//Matching
				time_meas();
				float c_threshold = 1.0f;
				Matcher.desc = Describer;
				Matcher.calculateCorrespondences(c_threshold);
				processing_times.push_back(time_meas("matching"));

				// RANSAC based Correspondence Rejection with ICP, default iterations = 1000, default threshold = 0.05
				time_meas();
				RansacRejector.setMaximumIterations(1000);
				ransac_rejection(Matcher.corresp, modelResolution, KeypointDetector.modelKeypoints_, KeypointDetector.sceneKeypoints_, RansacRejector);
				Eigen::Matrix4f transformation_matrix = get_ransac_transformation_matrix(RansacRejector);
				pcl::transformPointCloud(KeypointDetector.modelKeypoints_, KeypointDetector.modelKeypoints_, transformation_matrix);
				pcl::transformPointCloud(model, model, transformation_matrix);
				processing_times.push_back(time_meas("Ransac Rejection"));

				// Iterative closest Point ICP
				time_meas();
				transformation_matrix = icp(model, scene);
				pcl::transformPointCloud(KeypointDetector.modelKeypoints_, KeypointDetector.modelKeypoints_, transformation_matrix);
				pcl::transformPointCloud(model, model, transformation_matrix);
				processing_times.push_back(time_meas("ICP"));

				//Calculate euclidean distance of a model keypoint to its matched scene keypoint: sqrt(delta_x^2 + delta_y^2 + delta_z^2)
				std::vector<float> euclidean_distance = calculate_euclidean_distance(KeypointDetector.modelKeypoints_, KeypointDetector.sceneKeypoints_);
				float distance_threshold = shotRadius_ * modelResolution / 2;
				pcl::Correspondences true_positives = get_true_positives(distance_threshold, euclidean_distance);
				print_results(true_positives, KeypointDetector);

				//Enable if evaluation according to Guo et al.
#if 0
			//A match is considered TP if the euclidean distance of a model keypoint to its matched scene keypoint is less than half the supportradius
				pcl::Correspondences true_positives = get_true_positives(distance_threshold, euclidean_distance);

				//Store the NNDR and the euclidean distance for the evaluation according to Guo et al.
				pr_filename = "../../../../PR/Guo/" + model_name + "_to_" + scene_name + ".csv";
				std::string results = concatenate_results(KeypointDetector, euclidean_distance, distance_threshold);
				filehandler.writeToFile(results, pr_filename);
				print_results(true_positives, KeypointDetector);
#endif
				//Enable if evaluation according to Buch et al.
#if 1
				accumulate_keypoints(KeypointDetector);
				std::string results = concatenate_distances(euclidean_distance);
				stats = assemble_stats(processing_times, model, scene, modelResolution, sceneResolution, KeypointDetector);
				std::string statistics = create_writable_stats(stats);
				FileHandler.writeToFile(results, pr_filename);
				FileHandler.writeToFile(statistics, stats_filename);

#endif
				std::string NOF_keypoints = std::to_string(accumulated_keypoints) + "," + std::to_string(distance_threshold) + "\n";
				FileHandler.writeToFile(NOF_keypoints, pr_filename);
				NOF_keypoints = "";
				accumulated_keypoints = 0;
				corr.clear();
			}
		}
	}
#endif

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