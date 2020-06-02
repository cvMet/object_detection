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
float shotRadius_ = 30;
float fpfhRadius_ = 20;

#if isshot
const float supportRadius_ = shotRadius_;
string descriptor = "B_SHOT";
#else
const float supportRadius_ = fpfhRadius_;
string descriptor = "FPFH";
#endif
typedef pcl::PointXYZINormal PointTypeFull;
typedef pcl::PointXYZI PointTypeIO;

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

std::vector<pcl::PointCloud<PointXYZ>> euclidean_cluster_extraction_ws(pcl::PointCloud<PointXYZ> input_cloud, string filename) {
	// Creating the KdTree object for the search method of the extraction
	vector<pcl::PointCloud<PointXYZ>> clusters;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(input_cloud.makeShared());

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(0.005);
	ec.setMinClusterSize(500);
	ec.setMaxClusterSize(5000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(input_cloud.makeShared());
	ec.extract(cluster_indices);

	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++) {
			cloud_cluster->points.push_back(input_cloud.points[*pit]); //*
		}
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
		clusters.push_back(*cloud_cluster);
		string substr = filename.substr(0, (filename.length() - 4)) + "cluster_" + to_string(j) + ".ply";
		pcl::io::savePLYFileASCII(substr, *cloud_cluster);
		j++;
	}

	return clusters;
}

std::vector<pcl::PointCloud<PointXYZ>> euclidean_cluster_extraction(pcl::PointCloud<PointXYZ> input_cloud) {
	// Creating the KdTree object for the search method of the extraction
	vector<pcl::PointCloud<PointXYZ>> clusters;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(input_cloud.makeShared());

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(0.005);
	ec.setMinClusterSize(500);
	ec.setMaxClusterSize(5000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(input_cloud.makeShared());
	ec.extract(cluster_indices);

	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++) {
			cloud_cluster->points.push_back(input_cloud.points[*pit]); //*
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

string create_writable_angles(vector<tuple<string, vector<double>>> angles) {
	std::string data = "";
	for (int i = 0; i < angles.size(); ++i) {
		data += get<0>(angles[i])+",";
			for(int j = 0; j < get<1>(angles[i]).size(); ++j) {
				data += std::to_string(get<1>(angles[i])[j])+',';
		};
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

void ransac_rejection(pcl::Correspondences corresp, pcl::PointCloud<pcl::PointXYZ> modelKeypoints, pcl::PointCloud<pcl::PointXYZ> sceneKeypoints, pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> &RansacRejector) {
	pcl::CorrespondencesConstPtr correspond = boost::make_shared<pcl::Correspondences>(corresp);
	RansacRejector.setInputSource(modelKeypoints.makeShared());
	RansacRejector.setInputTarget(sceneKeypoints.makeShared());
	//Inlier Threshold Set to 5mm since this is approximately the tof standard deviation
	RansacRejector.setInlierThreshold(0.005);
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

vector<double> get_angles(Eigen::Matrix4f transformation_matrix) {
	vector<double> angles;
	double neg_sinTheta = transformation_matrix(2, 0);
	double sinPhicosTheta = transformation_matrix(2, 1);
	double cosThetacosPsi = transformation_matrix(0, 0);
	
	double theta = -asin(neg_sinTheta)* 180 / 3.14159265;
	double phi = asin(sinPhicosTheta / cos((theta* 3.14159265/180))) * 180 / 3.14159265;
	double psi = acos(cosThetacosPsi / cos((theta * 3.14159265 / 180))) * 180 / 3.14159265;

	angles.push_back(theta);
	angles.push_back(phi);
	angles.push_back(psi);
	std::cout << "theta: " << (double)theta << std::endl;
	std::cout << "phi: " << (double)phi << std::endl;
	std::cout << "psi: " << (double)psi << std::endl;
	return angles;

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

bool enforceIntensitySimilarity(const pcl::PointXYZI& point_a, const pcl::PointXYZI& point_b, float squared_distance)
{
	if (std::abs(point_a.intensity - point_b.intensity) < 0.1f)
		return (true);
	else
		return (false);
}

bool enforceCurvatureOrIntensitySimilarity(const PointTypeFull& point_a, const PointTypeFull& point_b, float squared_distance)
{
	Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.getNormalVector3fMap(), point_b_normal = point_b.getNormalVector3fMap();
	if (std::abs(point_a.intensity - point_b.intensity) < 5.0f)
		return (true);
	if (std::abs(point_a_normal.dot(point_b_normal)) < 0.05)
		return (true);
	return (false);
}

bool customRegionGrowing(const PointTypeFull& point_a, const PointTypeFull& point_b, float squared_distance)
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

int nthOccurrence(const std::string& str, const std::string& findMe, int nth)
{
	size_t  pos = 0;
	int     cnt = 0;

	while (cnt != nth)
	{
		pos += 1;
		pos = str.find(findMe, pos);
		if (pos == std::string::npos)
			return -1;
		cnt++;
	}
	return pos;
}

int value_between_seperator(string& str, string seperator, int pos) {
	int temp;
	//Case 1 digit
	if (str.substr(pos + 2, 1).compare(seperator) == 0) {
		temp = std::stoi(str.substr(pos + 1, 1));
	}
	// Case 2 digits
	else if (str.substr(pos + 3, 1).compare(seperator) == 0) {
		temp = std::stoi(str.substr(pos + 1, 2));
	}
	//Case 3 digits
	else {
		temp = std::stoi(str.substr(pos + 1, 3));
	}
	return temp;
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
	pcl::PointCloud<pcl::PointXYZ> temp_cloud;

	//Clouds used during objectdetection
	pcl::PointCloud<PointType> model;
	pcl::PointCloud<PointType> scene;

	vector<fs::path> model_depth_names;
	vector<fs::path> background_depth_names;
	vector<fs::path> cloud_names;

	vector<float> keypointdetector_threshold = {0.7f};
	vector<int> keypointdetector_nof_neighbors = {3, 5 };

	float modelResolution, sceneResolution;

	//Paths used during cloudgen
	string model_depth_directory = "../../../../datasets/23_04_20/raw_depth/muttern_schraeg/0_tran/model";
	string background_depth_directory = "../../../../datasets/23_04_20/raw_depth/muttern_schraeg/0_tran/background";
	string depth_extension = ".txt";
	string cloud_directory = "../../../../clouds/23_04_20/ms/3d_filtered/experimental";

	//path used during objectdetection
	string query_directory = "../../../../clouds/23_04_20/ms/3d_filtered/experimental/query_models";
	string scene_directory = "../../../../clouds/23_04_20/ms/3d_filtered/experimental/SOR_50";
	vector<fs::path> query_names;
	vector<fs::path> scene_names;

	string pr_root = "../../../../PR/Buch";
	string stats_root = "../../../../stats";
	string object = "muttern_schraeg";
	string dataset = "23_04_20";
	string preprocessor_mode = "3d_filtered";
	string transformation = "clustered";

	vector<tuple<string, vector<double>>> angles;

	get_all_file_names(model_depth_directory, depth_extension, model_depth_names);
	get_all_file_names(background_depth_directory, depth_extension, background_depth_names);

	//Enable if clouds need to be generated first
#if 0

	std::string bkgr_depth_filename = background_depth_directory + "/" + background_depth_names[0].string();
	std::vector<std::vector<float>> background_distance_array = FileHandler.melexis_txt_to_distance_array(bkgr_depth_filename, rows, columns);
	background_cloud = CloudCreator.distance_array_to_cloud(background_distance_array, 6.0f, 0.015f, 0.015f);
	filtered_background = CloudCreator.median_filter_cloud(background_cloud, 4);
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
		filtered_model = CloudCreator.median_filter_cloud(model_cloud, 4);
		creation_stats.push_back(time_meas("time_medianfilter"));
		time_meas();
		final_cloud = CloudCreator.remove_background(filtered_model, filtered_background, 0.005f);

		creation_stats.push_back(time_meas("time_backgroundRemoval"));
		roi_cloud = CloudCreator.roi_filter(final_cloud, "x", -0.2f, 0.1f);
		temp_cloud = CloudCreator.remove_outliers(roi_cloud.makeShared(),15);
		string filename = cloud_directory + "/" + background_depth_names[i].string();
		string substr = filename.substr(0, (filename.length() - 4)) + ".ply";
		string statistics = create_writable_stats(creation_stats);
		string stats_filename = filename.substr(0, (filename.length() - 4)) + "_stats.csv";
		//pcl::io::savePLYFileASCII(substr, temp_cloud);
	

		//CloudCreator.show_cloud(roi_cloud);
		//CloudCreator.show_cloud(temp_cloud);
		
		//pcl::io::savePLYFileASCII(substr, temp_cloud);
		//FileHandler.writeToFile(statistics, stats_filename);
	}
#endif

	//Enable if several clouds should get merged into one cloud
#if 1
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> transformed_clouds;
	std::vector<tuple<Eigen::Matrix4f, Eigen::Matrix4f>> transformation_matrices;
	pcl::PointCloud<pcl::PointXYZ>::Ptr origin_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	//Paths used during cloudgen
	model_depth_directory = "../../../../datasets/20_03_20/raw_depth/WhiteBottle/0_tran/model";
	background_depth_directory = "../../../../datasets/20_03_20/raw_depth/WhiteBottle/0_tran/background";
	depth_extension = ".txt";
	cloud_directory = "../../../../clouds/mastercloud/unfiltered";

	model_depth_names.clear();
	background_depth_names.clear();
	get_all_file_names(model_depth_directory, depth_extension, model_depth_names);
	get_all_file_names(background_depth_directory, depth_extension, background_depth_names);

	std::string bkgr_depth_filename = background_depth_directory + "/" + background_depth_names[0].string();
	std::vector<std::vector<float>> background_distance_array = FileHandler.melexis_txt_to_distance_array(bkgr_depth_filename, rows, columns);
	background_cloud = CloudCreator.distance_array_to_cloud(background_distance_array, 6.0f, 0.015f, 0.015f);

	for (int i = 0; i < model_depth_names.size(); ++i) {
		std::string model_depth_filename = model_depth_directory + "/" + model_depth_names[i].string();
		std::vector<std::vector<float>> model_distance_array = FileHandler.melexis_txt_to_distance_array(model_depth_filename, rows, columns);
		model_cloud = CloudCreator.distance_array_to_cloud(model_distance_array, 6.0f, 0.015f, 0.015f);
		final_cloud = CloudCreator.remove_background(model_cloud, background_cloud, 0.005f);
		roi_cloud = CloudCreator.roi_filter(final_cloud, "x", -0.2f, 0.1f);
		temp_cloud = CloudCreator.remove_outliers(roi_cloud.makeShared(), 15);
		string filename = cloud_directory + "/" + background_depth_names[i].string();
		string substr = filename.substr(0, (filename.length() - 4)) + ".ply";
		pcl::io::savePLYFileASCII(substr, temp_cloud);
	}


	//Load origin cloud
	string origin_cloud_path = "../../../../clouds/20_03_20/WB/0_Tran/WhiteBottle_0_0_0_0.ply";
	cloud_directory = "../../../../clouds/mastercloud";
	get_all_file_names(cloud_directory, ".ply", cloud_names);
	if (pcl::io::loadPLYFile(origin_cloud_path, *origin_cloud) == -1)
	{
		std::cout << "Error loading initial master cloud." << std::endl;
	}
	CloudCreator.remove_outliers(origin_cloud, 15);
	*origin_cloud = euclidean_cluster_extraction(*origin_cloud)[0];
	float origin_res = static_cast<float> (compute_cloud_resolution(origin_cloud));
	//Load all permutation clouds into vector
	for (int cloud = 0; cloud < cloud_names.size(); ++cloud) {
		std::vector<pcl::PointCloud<PointXYZ>> clusters;
		pcl::PointCloud<pcl::PointXYZ>::Ptr permutation_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		string cloud_filename = cloud_directory + "/" + cloud_names[cloud].string();
		if (pcl::io::loadPLYFile(cloud_filename, *permutation_cloud) == -1)
		{
			std::cout << "Error loading permutation cloud." << std::endl;
		}
		CloudCreator.remove_outliers(permutation_cloud, 15);
		//Euclidean Clustering
		clusters.push_back(euclidean_cluster_extraction(*permutation_cloud)[0]);
		clouds.push_back(clusters[0].makeShared());
	}

	for (int cloud = 0; cloud < clouds.size(); ++cloud) {
		Normals NormalEstimator;
		KeypointDetector KeypointDetector;
		Descriptor Describer;
		Matching Matcher;
		pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>);

		float permutation_res = static_cast<float> (compute_cloud_resolution(clouds[cloud]));
		//Estimate Normals
		NormalEstimator.model = *origin_cloud;
		NormalEstimator.scene = *clouds[cloud];
		NormalEstimator.calculateNormals(5 * origin_res, 5 * permutation_res);
		NormalEstimator.removeNaNNormals();
		*origin_cloud = NormalEstimator.model;
		*clouds[cloud] = NormalEstimator.scene;

		//Detect keypoints
		time_meas();
		KeypointDetector.calculateIssKeypoints(KeypointDetector.modelKeypoints_, *origin_cloud, NormalEstimator.modelNormals_, origin_res, 0.9f, 5);
		KeypointDetector.calculateIssKeypoints(KeypointDetector.sceneKeypoints_, *clouds[cloud], NormalEstimator.sceneNormals_, permutation_res, 0.9f, 5);

		//Calculate descriptor for each keypoint
		time_meas();
		Describer.NormalEstimator = NormalEstimator;
		Describer.KeypointDetector = KeypointDetector;
		Describer.model_ = *origin_cloud;
		Describer.scene_ = *clouds[cloud];
		Describer.calculateDescriptor(30 * origin_res, 30 * permutation_res);

		//Matching
		float c_threshold = 1.0f;
		Matcher.desc = Describer;
		Matcher.calculateCorrespondences(c_threshold);

		// RANSAC based Correspondence Rejection with ICP, default iterations = 1000, default threshold = 0.05
		RansacRejector.setMaximumIterations(1000);
		ransac_rejection(Matcher.corresp, KeypointDetector.modelKeypoints_, KeypointDetector.sceneKeypoints_, RansacRejector);
		Eigen::Matrix4f ransac_transformation = get_ransac_transformation_matrix(RansacRejector);
		pcl::transformPointCloud(KeypointDetector.modelKeypoints_, KeypointDetector.modelKeypoints_, ransac_transformation);
		pcl::transformPointCloud(*origin_cloud, *temp_cloud, ransac_transformation);

		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		icp.setMaximumIterations(30);
		icp.setRANSACOutlierRejectionThreshold(0.005);
		icp.setTransformationEpsilon(1e-9);
		icp.setInputSource(temp_cloud);
		icp.setInputTarget(clouds[cloud]);
		icp.align(*aligned_cloud);
		Eigen::Matrix4f icp_transformation;
		icp_transformation = icp.getFinalTransformation();
		std::cout << icp_transformation << std::endl;

		//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
		//viewer->setBackgroundColor(0, 0, 0);
		//viewer->initCameraParameters();
		////Add origin points to visualizer
		//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue(origin_cloud, 0, 0, 200);
		//viewer->addPointCloud<pcl::PointXYZ>(origin_cloud, blue, "sample cloud1");
		//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud1");
		////add unaligned points to visualizer
		//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(clouds[cloud], 200, 0, 0);
		//viewer->addPointCloud<pcl::PointXYZ>(clouds[cloud], red, "sample cloud2");
		//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud2");
		////add points after ransac to visualizer
		//viewer->addPointCloud<pcl::PointXYZ>(temp_cloud, "sample cloud3");
		//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud3");
		//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "sample cloud3");
		//while (!viewer->wasStopped())
		//{
		//	viewer->spinOnce(100);
		//	std::this_thread::sleep_for(100ms);
		//}

		//Add inverse of transformation matrices to corresponding vector to enable target-to-source transformation
		transformation_matrices.push_back(make_tuple(ransac_transformation.inverse(), icp_transformation.inverse()));

	}
	for (int i = 0; i < clouds.size(); ++i) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
		//viewer->setBackgroundColor(0, 0, 0);
		//viewer->initCameraParameters();
		////Add origin points to visualizer
		//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue(origin_cloud, 0, 0, 200);
		//viewer->addPointCloud<pcl::PointXYZ>(origin_cloud, blue, "sample cloud1");
		//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud1");
		////add unaligned points to visualizer
		//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(clouds[i], 200, 0, 0);
		//viewer->addPointCloud<pcl::PointXYZ>(clouds[i], red, "sample cloud2");
		//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud2");
		//transform unaligned points and add to visualizer
		Eigen::Matrix4f rt = get<0>(transformation_matrices[i]);
		Eigen::Matrix4f it = get<1>(transformation_matrices[i]);
		pcl::transformPointCloud(*clouds[i], *transformed_cloud, it);
		pcl::transformPointCloud(*transformed_cloud, *transformed_cloud, rt);
		transformed_clouds.push_back(transformed_cloud);
		/*viewer->addPointCloud<pcl::PointXYZ>(transformed_cloud, "sample cloud3");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud3");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "sample cloud3");
		while (!viewer->wasStopped())
		{
			viewer->spinOnce(100);
			std::this_thread::sleep_for(100ms);
		}*/
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr master_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	*master_cloud = *origin_cloud;
	for (int i = 0; i < transformed_clouds.size(); ++i) {
		*master_cloud += *transformed_clouds[i];
	}
	pcl::io::savePLYFileASCII("../../../../clouds/mastercloud/mastercloud.ply", *master_cloud);
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->initCameraParameters();
	//Add origin points to visualizer
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> gold(master_cloud, 226, 176, 7);
	viewer->addPointCloud<pcl::PointXYZ>(master_cloud, gold, "sample cloud1");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud1");
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		std::this_thread::sleep_for(100ms);
	}
	*master_cloud = CloudCreator.remove_outliers(master_cloud, 50);
	pcl::io::savePLYFileASCII("../../../../clouds/mastercloud/mastercloud_SOR.ply", *master_cloud);
#endif
	//Enable for automated detection process
#if 0
	get_all_file_names(query_directory, ".ply", query_names);
	get_all_file_names(scene_directory, ".ply", scene_names);

	for (int query_number = 0; query_number < query_names.size(); ++query_number)
	{
		string model_filename = query_directory + "/" + query_names[query_number].string();
		for (int neighbor = 0; neighbor < keypointdetector_nof_neighbors.size(); ++neighbor)
		{
			for (int threshold = 0; threshold < keypointdetector_threshold.size(); ++threshold)
			{
				for (int scene_number = 0; scene_number < scene_names.size(); ++scene_number)
				{

					vector<tuple<string, float>> processing_times, stats;
					Normals NormalEstimator;
					KeypointDetector KeypointDetector;
					Descriptor Describer;
					Matching Matcher;

					model_filename = "../../../../clouds/23_04_20/ms/3d_filtered/experimental/query_models/M26_0_0_0_0_SOR_50.ply";
					string scene_filename = scene_directory + "/" + scene_names[scene_number].string();

					int name_pos_model = model_filename.find("M26", 0);
					int ext_pos_model = model_filename.find(".ply", 0);
					int name_pos = scene_filename.find("M26", 0);
					int extension_pos = scene_filename.find(".ply", 0);
					string model_identifier = get_identifier(model_filename, name_pos_model, ext_pos_model);
					string scene_identifier = get_identifier(scene_filename, name_pos, extension_pos);

					string pr_filename = pr_root + "/" + dataset + "/" + object + "/" + preprocessor_mode + "/" + descriptor
						+ "/iss" + std::to_string(keypointdetector_nof_neighbors[neighbor])
						+"_th" + std::to_string(keypointdetector_threshold[threshold]).substr(0, 3)
						+ "/" + transformation
						+ "/" + model_identifier + "_to_" + scene_identifier + ".csv";
					string stats_filename = stats_root + "/" + dataset + "/" + object + "/" + preprocessor_mode + "/" + descriptor
						+ "/iss" + std::to_string(keypointdetector_nof_neighbors[neighbor])
						+ "_th" + std::to_string(keypointdetector_threshold[threshold]).substr(0, 3)
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
					NormalEstimator.calculateNormals(5 * modelResolution, 5 * sceneResolution);
					NormalEstimator.removeNaNNormals();
					model = NormalEstimator.model;
					scene = NormalEstimator.scene;
					processing_times.push_back(time_meas("normal estimation"));
					//Detect keypoints
					time_meas();
					KeypointDetector.calculateIssKeypoints(KeypointDetector.modelKeypoints_, model, NormalEstimator.modelNormals_, modelResolution,
						keypointdetector_threshold[threshold], keypointdetector_nof_neighbors[neighbor]);
					KeypointDetector.calculateIssKeypoints(KeypointDetector.sceneKeypoints_, scene, NormalEstimator.sceneNormals_, sceneResolution,
						keypointdetector_threshold[threshold], keypointdetector_nof_neighbors[neighbor]);
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
					ransac_rejection(Matcher.corresp, KeypointDetector.modelKeypoints_, KeypointDetector.sceneKeypoints_, RansacRejector);
					Eigen::Matrix4f transformation_matrix = get_ransac_transformation_matrix(RansacRejector);
					pcl::transformPointCloud(KeypointDetector.modelKeypoints_, KeypointDetector.modelKeypoints_, transformation_matrix);
					pcl::transformPointCloud(model, model, transformation_matrix);
					processing_times.push_back(time_meas("Ransac Rejection"));

					// Iterative closest Point ICP
					time_meas();
					Eigen::Matrix4f icp_transformation_matrix = icp(model, scene);
					pcl::transformPointCloud(model, model, icp_transformation_matrix);
					pcl::transformPointCloud(KeypointDetector.modelKeypoints_, KeypointDetector.modelKeypoints_, icp_transformation_matrix);
					processing_times.push_back(time_meas("ICP"));

					vector<double> temp = get_angles(icp_transformation_matrix);
					angles.push_back(make_tuple(model_identifier + "_to_" + scene_identifier,temp));
					//get_angles(icp_transformation_matrix);

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
	//#if 1
	//				accumulate_keypoints(KeypointDetector);
	//				std::string results = concatenate_distances(euclidean_distance);
	//				stats = assemble_stats(processing_times, model, scene, modelResolution, sceneResolution, KeypointDetector);
	//				std::string statistics = create_writable_stats(stats);
	//				FileHandler.writeToFile(results, pr_filename);
	//				//FileHandler.writeToFile(statistics, stats_filename);
	//
	//#endif
	//				std::string NOF_keypoints = std::to_string(accumulated_keypoints) + "," + std::to_string(distance_threshold) + "\n";
	//				FileHandler.writeToFile(NOF_keypoints, pr_filename);
	//				NOF_keypoints = "";
	//				accumulated_keypoints = 0;
	//				//corr.clear();

#endif

	//Enable if the visualization module should be used
#if 0

					Eigen::Matrix3f	temp, Rx, Ry, Rz;
					Eigen::Matrix4f T;
					Eigen::Vector4f vec;
					vec << 0, 0, 0, 1;

					float psi = 0;
					float theta = 0;
					float phi =0.262;

					Rz << cos(psi), -sin(psi), 0,
						sin(psi), cos(psi), 0,
						0, 0, 1;
					Ry << cos(theta), 0, sin(theta),
						0, 1, 0,
						-sin(theta), 0, cos(theta);
					Rx << 1, 0, 0,
						0, cos(phi), -sin(phi),
						0, sin(phi), cos(phi);

					temp = Rz * Ry * Rx;
					for (int i = 0; i < temp.rows(); ++i) {
						for (int j = 0; j < temp.cols(); ++j) {
							T(i, j) = temp(i, j);
						}
					}
					for (int i = 0; i < temp.rows(); ++i) {
						T(i, 3) = 0;
						T(3, i) = 0;
					}
					T(3, 3) = 1;
					std::cout << "temp: " << temp << endl;
					std::cout << "T: " << T << endl;

					pcl::transformPointCloud(model, model, T);

					boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
					viewer->setBackgroundColor(0, 0, 0);
					//viewer->addCoordinateSystem (0.1);
					viewer->initCameraParameters();

					//Move model so that it is separated from the scene to see correspondences
					Eigen::Matrix4f t;
					t << 1, 0, 0, modelResolution * 100,
						0, 1, 0, 0,
						0, 0, 1, 0,
						0, 0, 0, 1;

					pcl::transformPointCloud(KeypointDetector.modelKeypoints_, KeypointDetector.modelKeypoints_, t);
					pcl::transformPointCloud(model, model, t);

					//Add model keypoints to visualizer
					pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(KeypointDetector.modelKeypoints_.makeShared(), 200, 0, 0);
					viewer->addPointCloud<pcl::PointXYZ>(KeypointDetector.modelKeypoints_.makeShared(), red, "sample cloud1");
					viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "sample cloud1");
					
					//add scene keypoints to visualizer
					pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue(KeypointDetector.sceneKeypoints_.makeShared(), 0, 0, 150);
					viewer->addPointCloud<pcl::PointXYZ>(KeypointDetector.sceneKeypoints_.makeShared(), blue, "sample cloud2");
					viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "sample cloud2");
					//add lines between the correspondences
					viewer->addCorrespondences<pcl::PointXYZ>(KeypointDetector.modelKeypoints_.makeShared(), KeypointDetector.sceneKeypoints_.makeShared(), corr, "correspondences");

					//add model points to visualizer
					viewer->addPointCloud<pcl::PointXYZ>(model.makeShared(), "sample cloud3");
					viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud3");
					viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "sample cloud3");

					//add scene points to visualizer
					//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> white_scene(scene.makeShared(), 155, 155, 155);
					viewer->addPointCloud<pcl::PointXYZ>(scene.makeShared(), "sample cloud4");
					viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud4");
					viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "sample cloud4");

					////add normals
					//viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(model.makeShared(), NormalEstimator.modelNormals_.makeShared(), 15, 0.005, "model_normals");
					//viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(scene.makeShared(), NormalEstimator.sceneNormals_.makeShared(), 15, 0.005, "scene_normals");
					//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "model_normals");
					//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "scene_normals");

					//// fixed camera params for normal pictures
					//viewer->setCameraPosition(-0.0100684, -0.00191237, 0.0384372, -0.0153327, -0.0521215, 0.00973529, 0.12715, -0.502269, 0.855312);
					//viewer->setCameraFieldOfView(0.523598775);
					//viewer->setCameraClipDistances(0.000134497, 0.134497);
					//viewer->setPosition(0, 0);
					//viewer->setSize(1024, 756);

					while (!viewer->wasStopped())
					{
						viewer->spinOnce(100);
						std::this_thread::sleep_for(100ms);
					}

					corr.clear();

				}
				//string angle_stats = create_writable_angles(angles);
				//FileHandler.writeToFile(angle_stats, "../../../../stats/angles/angles.csv");
			}
		}
	}
#endif

	return 0;
}