//Disclaimer
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
#include "../include/menu/base_menu.h"
#include "../include/menu/main_menu.h"
#include "../include/menu/cloudcreation_menu.h"
#include "../include/menu/detection_menu.h"
#include "../include/menu/filter_menu.h"
#include "../include/menu/merge_menu.h"
#include "objectdetection.h"

//Namespaces
namespace fs = boost::filesystem;
using namespace std;
using namespace pcl;

//Typedefs
typedef pcl::PointXYZINormal PointTypeFull;
typedef pcl::PointXYZI PointTypeIO;

#ifndef isshot
#define isshot 1	//For SHOT-Descriptor use 1 for FPFH 0
#endif
/*
______
| ___ \
| |_/ /__ _  _ __  __ _  _ __ ___   ___
|  __// _` || '__|/ _` || '_ ` _ \ / __|
| |  | (_| || |  | (_| || | | | | |\__ \
\_|   \__,_||_|   \__,_||_| |_| |_||___/
Params used during cloudgen & objectdetection
*/
vector<tuple<string, bool>> execution_params;
vector<tuple<string, bool>> filter;
vector<float> keypointdetector_threshold = { 0.7f };
vector<int> keypointdetector_nof_neighbors = { 5 };

string object = "buerli";
string dataset = "threshold_eval";
string preprocessor_mode = "3d_filtered";

vector<tuple<string, vector<double>>> angles;
float queryResolution, targetResolution;

class CloudCreationMenu;
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

Eigen::Matrix4f get_ransac_transformation_matrix(pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ>& RansacRejector) {
	cout << "# Iterations: " << RansacRejector.getMaximumIterations() << endl;
	Eigen::Matrix4f mat = RansacRejector.getBestTransformation();
	cout << "RANSAC Transformation Matrix yielding the largest number of inliers.  : \n" << mat << endl;
	// int ransac_corr = corr.size();
	return mat;
}

Eigen::Matrix4f icp(pcl::PointCloud<PointType> query, pcl::PointCloud<PointType> target) {
	Eigen::Matrix4f mat, guess;
	guess << -1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, -1, 0,
		0, 0, 0, 1;
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(query.makeShared());
	icp.setInputTarget(target.makeShared());
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
			std::cout << "Error loading query cloud." << std::endl;
			return (temp_cloud);
		}
	}
	else {
		std::cout << "Unknown query file type. Check if there are any dots in the files path." << endl;
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

pcl::Correspondences get_true_positives(float& distance_threshold, std::vector<float>& euclidean_distances) {
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

std::vector<pcl::PointCloud<PointXYZ>> euclidean_cluster_extraction_save_ordered_output(pcl::PointCloud<PointXYZ> input_cloud, string filename) {
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

std::vector<pcl::PointCloud<PointXYZ>> euclidean_cluster_extraction_ordered_output(pcl::PointCloud<PointXYZ> input_cloud) {
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
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>(320, 240));
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++) {
			cloud_cluster->points[*pit] = (input_cloud.points[*pit]);
		}
		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
		clusters.push_back(*cloud_cluster);
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

std::vector<float> calculate_euclidean_distance(const pcl::PointCloud<pcl::PointXYZ> query_keypoints, const pcl::PointCloud<pcl::PointXYZ> target_keypoints) {
	std::vector<float> distance;
	for (int i = 0; i < corr.size(); ++i) {
		distance.push_back(
			pow((query_keypoints.at(corr.at(i).index_query).x - target_keypoints.at(corr.at(i).index_match).x), 2) +
			pow((query_keypoints.at(corr.at(i).index_query).y - target_keypoints.at(corr.at(i).index_match).y), 2) +
			pow((query_keypoints.at(corr.at(i).index_query).z - target_keypoints.at(corr.at(i).index_match).z), 2)
		);
		distance[i] = sqrt(distance[i]);
	}
	return distance;
}

vector<tuple<string, float>> assemble_stats(vector<tuple<string, float>> processing_times, pcl::PointCloud<PointType> query, pcl::PointCloud<PointType> target, float queryResolution, float targetResolution, KeypointDetector& Detector) {
	vector<tuple<string, float>> stats;

	stats.push_back(make_tuple("suport_radius", float(supportRadius_)));
	stats.push_back(make_tuple("points_query_cloud", float(query.points.size())));
	stats.push_back(make_tuple("points_target_cloud", float(target.points.size())));
	stats.push_back(make_tuple("query_resolution", float(queryResolution)));
	stats.push_back(make_tuple("target_resolution", float(targetResolution)));
	stats.push_back(make_tuple("query_keypoints", float(Detector.queryKeypoints_.size())));
	stats.push_back(make_tuple("target_keypoints", float(Detector.targetKeypoints_.size())));

	for (int i = 0; i < processing_times.size(); ++i) {
		stats.push_back(processing_times[i]);
	}
	return stats;
}

string concatenate_results(KeypointDetector& Detector, std::vector<float>& euclidean_distance, const float& distance_threshold) {
	std::string data = "";
	if (Detector.targetKeypoints_.size() < Detector.queryKeypoints_.size()) {
		data = std::to_string(Detector.targetKeypoints_.size()) + "," + std::to_string(distance_threshold) + "\n";
	}
	else {
		data = std::to_string(Detector.queryKeypoints_.size()) + "," + std::to_string(distance_threshold) + "\n";
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
		data += get<0>(angles[i]) + ",";
		for (int j = 0; j < get<1>(angles[i]).size(); ++j) {
			data += std::to_string(get<1>(angles[i])[j]) + ',';
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
	//Get fileformat of the query -> last three letters of filename
	return filename.std::string::substr(filename.length() - 3);
}

string get_identifier(string filename, int name_pos, int extension_pos) {
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

void ransac_rejection(pcl::Correspondences corresp, pcl::PointCloud<pcl::PointXYZ> queryKeypoints, pcl::PointCloud<pcl::PointXYZ> targetKeypoints, pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ>& RansacRejector) {
	pcl::CorrespondencesConstPtr correspond = boost::make_shared<pcl::Correspondences>(corresp);
	RansacRejector.setInputSource(queryKeypoints.makeShared());
	RansacRejector.setInputTarget(targetKeypoints.makeShared());
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
	if (!running) {
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
	std::cout << "Precision: " << (float)true_positives.size() / (float)corr.size() << " Recall: " << true_positives.size() / (float)(detector.queryKeypoints_.size()) << endl;
};

void accumulate_keypoints(KeypointDetector& Detector) {
	if (Detector.targetKeypoints_.size() < Detector.queryKeypoints_.size()) {
		accumulated_keypoints += Detector.targetKeypoints_.size();
	}
	else {
		accumulated_keypoints += Detector.queryKeypoints_.size();
	}
}

vector<double> get_angles(Eigen::Matrix4f transformation_matrix) {
	vector<double> angles;
	double neg_sinTheta = transformation_matrix(2, 0);
	double sinPhicosTheta = transformation_matrix(2, 1);
	double cosThetacosPsi = transformation_matrix(0, 0);

	double theta = -asin(neg_sinTheta) * 180 / 3.14159265;
	double phi = asin(sinPhicosTheta / cos((theta * 3.14159265 / 180))) * 180 / 3.14159265;
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

bool get_path() {
	string path;
	cin >> path;
	if (!boost::filesystem::exists(path))
	{
		std::cout << "Path entered was not valid!" << std::endl;
		return false;
	}
	return true;
}

void set_dataset(string dataset_name) {
	dataset = dataset_name;
}

void set_object(string object_name) {
	object = object_name;
}

void set_execution_param(string id) {
	for (int i = 0; i < execution_params.size(); ++i) {
		if (id.compare(get<0>(execution_params[i])) == 0) {
			get<1>(execution_params[i]) = true;
		}
	}
}

bool toggle_filter(string id) {
	for (int i = 0; i < filter.size(); ++i) {
		if (id.compare(get<0>(filter[i])) == 0) {
			get<1>(filter[i]) = !(get<1>(filter[i]));
			return !(get<1>(filter[i]));
		}
	}
	return false;
}

string get_input() {
	std::cout << "enter value: " << std::endl;
	string input;
	cin >> input;
	return input;
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

float get_median(std::vector<float> values) {
	std::vector<float>::iterator first = values.begin();
	std::vector<float>::iterator last = values.end();
	std::vector<float>::iterator middle = first + (last - first) / 2;
	std::nth_element(first, middle, last);
	return *middle;
}



int main(int argc, char* argv[])
{
	pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> RansacRejector;
	FileHandler FileHandler;
	CloudCreator CloudCreator;

	execution_params.push_back(make_tuple("cloudcreation",false));
	execution_params.push_back(make_tuple("merging", false));
	execution_params.push_back(make_tuple("detection", false));

	filter.push_back(make_tuple("median", false));
	filter.push_back(make_tuple("roi", false));
	filter.push_back(make_tuple("sor", false));

	BaseMenu* CurrentMenu = new MainMenu;
	bool quit = false;
	bool execute = false;
	while (!quit && !execute)
	{
		CurrentMenu->clearScreen();
		CurrentMenu->printText();

		char choice = 'z';
		cin >> choice;

		BaseMenu* NewMenuPointer = CurrentMenu->getNextMenu(toupper(choice), quit, execute);

		if (NewMenuPointer)
		{
			//Delete previous menue if new window is no subwindow of it (prevent memory leaks)
			if (!NewMenuPointer->child) {
				delete CurrentMenu;
			}
			CurrentMenu = NewMenuPointer;
		}
	}


	/*
	Paths used during cloudgen
	*/
	string depth_directory = "../../../../datasets/" + dataset + "/raw_depth/" + object;
	string depth_extension = ".txt";
	string cloud_directory = "../../../../clouds/" + dataset + "/" + object;
	vector<fs::path> depth_names;
	vector<fs::path> cloud_names;
	//Paths used during objectdetection
	string query_directory = "../../../../clouds/" + dataset + "/" + object + "/" + preprocessor_mode;
	string target_directory = "../../../../clouds/" + dataset + "/" + object + "/" + preprocessor_mode;
	string pr_root = "../../../../PR/Buch";
	string stats_root = "../../../../stats";
	vector<fs::path> query_names;
	vector<fs::path> target_names;
	
	/*
	 Clouds used during cloud generation
	*/
	pcl::PointCloud<pcl::PointXYZ> query_cloud;
	pcl::PointCloud<pcl::PointXYZ> background_cloud;
	pcl::PointCloud<pcl::PointXYZ> filtered_query;
	pcl::PointCloud<pcl::PointXYZ> filtered_background;
	pcl::PointCloud<pcl::PointXYZ> final_cloud;
	pcl::PointCloud<pcl::PointXYZ> roi_cloud;
	pcl::PointCloud<pcl::PointXYZ> temp_cloud;
	//Clouds used during objectdetection
	pcl::PointCloud<PointType> query;
	pcl::PointCloud<PointType> target;


	//Unordered filtering playground
#if 0
	pcl::PointCloud<pcl::PointXYZ>::Ptr unordered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPLYFile("../../../../clouds/mastercloud/mastercloud.ply", *unordered_cloud) == -1)
	{
		std::cout << "Error loading initial master cloud." << std::endl;
	}
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	tree->setInputCloud(unordered_cloud);
	vector<vector<int>> neighbor_indices;
	vector<vector<float>> neighbor_distances;
	for (int point = 0; point < unordered_cloud->points.size(); ++point) {
		vector<int> neighbors;
		vector<float> distances;
		pcl::PointXYZ query_point = unordered_cloud->at(point);
		//Radius based search
#if 0
		tree->radiusSearch(query_point, 0.005f, neighbors, distances);
#endif
		//NN based search
#if 1
		tree->nearestKSearch(query_point, 10, neighbors, distances);
#endif
		neighbor_indices.push_back(neighbors);
		neighbor_distances.push_back(distances);
	}
	pcl::PointXYZ filtered_point;
	vector<int> max_points_to_process = { 3,5,10,100 };

	//MEAN FILTERING
#if 0
	for (int point = 0; point < neighbor_indices.size(); ++point) {
		if (neighbor_indices[point].size() < max_points_to_process) {
			for (int i = 0; i < neighbor_indices[point].size(); ++i) {
				filtered_point.x += unordered_cloud->at(neighbor_indices[point][i]).x;
				filtered_point.y += unordered_cloud->at(neighbor_indices[point][i]).y;
				filtered_point.z += unordered_cloud->at(neighbor_indices[point][i]).z;
			}
			filtered_point.x = filtered_point.x / neighbor_indices[point].size();
			filtered_point.y = filtered_point.y / neighbor_indices[point].size();
			filtered_point.z = filtered_point.z / neighbor_indices[point].size();
		}
		else {
			for (int i = 0; i < max_points_to_process; ++i) {
				filtered_point.x += unordered_cloud->at(neighbor_indices[point][i]).x;
				filtered_point.y += unordered_cloud->at(neighbor_indices[point][i]).y;
				filtered_point.z += unordered_cloud->at(neighbor_indices[point][i]).z;
			}
			filtered_point.x = filtered_point.x / max_points_to_process;
			filtered_point.y = filtered_point.y / max_points_to_process;
			filtered_point.z = filtered_point.z / max_points_to_process;
		}
		filtered->points.push_back(filtered_point);
	}
#endif

	//MEDIAN FILTERING
#if 1
	for (int max = 0; max < max_points_to_process.size(); ++max) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
		for (int point = 0; point < neighbor_indices.size(); ++point) {
			vector<float> x_values;
			vector<float> y_values;
			vector<float> z_values;
			//Case # neighbors found is less than the number passed in search call 
			if (neighbor_indices[point].size() < max_points_to_process[max]) {
				for (int i = 0; i < neighbor_indices[point].size(); ++i) {
					x_values.push_back(unordered_cloud->at(neighbor_indices[point][i]).x);
					y_values.push_back(unordered_cloud->at(neighbor_indices[point][i]).y);
					z_values.push_back(unordered_cloud->at(neighbor_indices[point][i]).z);
				}
				filtered_point.x = get_median(x_values);
				filtered_point.y = get_median(y_values);
				filtered_point.z = get_median(z_values);
			}
			//Case # neighbors found = number passed in search call 
			else {
				for (int i = 0; i < max_points_to_process[max]; ++i) {
					x_values.push_back(unordered_cloud->at(neighbor_indices[point][i]).x);
					y_values.push_back(unordered_cloud->at(neighbor_indices[point][i]).y);
					z_values.push_back(unordered_cloud->at(neighbor_indices[point][i]).z);
				}
				filtered_point.x = get_median(x_values);
				filtered_point.y = get_median(y_values);
				filtered_point.z = get_median(z_values);
			}
			filtered->points.push_back(filtered_point);
		}
		string filename = "../../../../clouds/mastercloud/mastercloud_median_NN" + to_string(max_points_to_process[max]) + ".ply";
		pcl::io::savePLYFileASCII(filename, *filtered);
	}
#endif
#endif

	//Element 0 of exec params = CLOUDCREATION
	if (std::get<1>(execution_params[0])) {
		get_all_file_names(depth_directory, depth_extension, depth_names);
		std::string bkgr_depth_filename = depth_directory + "/" + depth_names[0].string();
		std::vector<std::vector<float>> background_distance_array = FileHandler.melexis_txt_to_distance_array(bkgr_depth_filename, rows, columns);
		background_cloud = CloudCreator.distance_array_to_cloud(background_distance_array, 6.0f, 0.015f, 0.015f);
		CloudCreator.show_cloud(background_cloud);
		if (std::get<1>(filter[0])) {
			background_cloud = CloudCreator.median_filter_cloud(background_cloud, 5);
		}
		CloudCreator.show_cloud(background_cloud);
		//Pop first element (background without query)
		depth_names.erase(depth_names.begin());
		for (int i = 0; i < depth_names.size(); ++i) {
			vector<tuple<string, float>> creation_stats;
			std::string query_depth_filename = depth_directory + "/" + depth_names[i].string();
			time_meas();
			std::vector<std::vector<float>> query_distance_array = FileHandler.melexis_txt_to_distance_array(query_depth_filename, rows, columns);
			creation_stats.push_back(time_meas("time_textToDistance"));
			time_meas();
			query_cloud = CloudCreator.distance_array_to_cloud(query_distance_array, 6.0f, 0.015f, 0.015f);
			creation_stats.push_back(time_meas("time_distanceToCloud"));
			//If median filtering enabled
			if (std::get<1>(filter[0])) {
				time_meas();
				query_cloud = CloudCreator.median_filter_cloud(query_cloud, 5);
				creation_stats.push_back(time_meas("time_medianfilter"));
			}
			time_meas();
			final_cloud = CloudCreator.remove_background(query_cloud, background_cloud, 0.005f);
			creation_stats.push_back(time_meas("time_backgroundRemoval"));
			//temp_cloud = CloudCreator.remove_outliers(final_cloud.makeShared(),10);
			string filename = cloud_directory + "/" + depth_names[i].string();
			string substr = filename.substr(0, (filename.length() - 4)) + ".ply";
			//string statistics = create_writable_stats(creation_stats);
			//string stats_filename = filename.substr(0, (filename.length() - 4)) + "_stats.csv";
			pcl::io::savePLYFileASCII(substr, final_cloud);
			//FileHandler.writeToFile(statistics, stats_filename);
		}
	}

	//Element 1 of exec params = CLOUD MERGING
	if (std::get<1>(execution_params[1])) {
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> transformed_clouds;
		std::vector<tuple<Eigen::Matrix4f, Eigen::Matrix4f>> transformation_matrices;
		pcl::PointCloud<pcl::PointXYZ>::Ptr origin_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
		//Enable if you want to create unfiltered clouds that stay organized
#if 0
	//Paths used during cloudgen
		query_depth_directory = "../../../../datasets/20_03_20/raw_depth/WhiteBottle/0_tran/query";
		background_depth_directory = "../../../../datasets/20_03_20/raw_depth/WhiteBottle/0_tran/background";
		depth_extension = ".txt";
		cloud_directory = "../../../../clouds/mastercloud/unfiltered";

		query_depth_names.clear();
		background_depth_names.clear();
		get_all_file_names(query_depth_directory, depth_extension, query_depth_names);
		get_all_file_names(background_depth_directory, depth_extension, background_depth_names);

		std::string bkgr_depth_filename = background_depth_directory + "/" + background_depth_names[0].string();
		std::vector<std::vector<float>> background_distance_array = FileHandler.melexis_txt_to_distance_array(bkgr_depth_filename, rows, columns);
		background_cloud = CloudCreator.distance_array_to_cloud(background_distance_array, 6.0f, 0.015f, 0.015f);

		for (int i = 0; i < query_depth_names.size(); ++i) {
			std::string query_depth_filename = query_depth_directory + "/" + query_depth_names[i].string();
			std::vector<std::vector<float>> query_distance_array = FileHandler.melexis_txt_to_distance_array(query_depth_filename, rows, columns);
			query_cloud = CloudCreator.distance_array_to_cloud(query_distance_array, 6.0f, 0.015f, 0.015f);
			final_cloud = CloudCreator.remove_background(query_cloud, background_cloud, 0.0055f);
			CloudCreator.show_cloud(query_cloud);
			CloudCreator.show_cloud(background_cloud);
			CloudCreator.show_cloud(final_cloud);
			string filename = cloud_directory + "/" + background_depth_names[i].string();
			string substr = filename.substr(0, (filename.length() - 4)) + ".ply";
			pcl::io::savePLYFileASCII(substr, final_cloud);
		}
#endif

		//Load origin cloud
		string origin_cloud_path = "../../../../clouds/20_03_20/WB/0_tran/unfiltered/WhiteBottle_0_0_0_0.ply";
		cloud_directory = "../../../../clouds/mastercloud";
		get_all_file_names(cloud_directory, ".ply", cloud_names);
		if (pcl::io::loadPLYFile(origin_cloud_path, *origin_cloud) == -1)
		{
			std::cout << "Error loading initial master cloud." << std::endl;
		}
		CloudCreator.remove_outliers(origin_cloud, 15);
		*origin_cloud = euclidean_cluster_extraction(*origin_cloud)[0];
		pcl::io::savePLYFileASCII("../../../../clouds/mastercloud/origincluster.ply", *origin_cloud);
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
			//clouds.push_back(permutation_cloud);
			//CloudCreator.remove_outliers(permutation_cloud, 15);
			//Euclidean Clustering
			clusters.push_back(euclidean_cluster_extraction(*permutation_cloud)[0]);

			clouds.push_back(clusters[0].makeShared());
			pcl::io::savePLYFileASCII("../../../../clouds/mastercloud/cluster_permutationCloud" + to_string(cloud) + ".ply", *clouds[cloud]);
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
			NormalEstimator.query = *origin_cloud;
			NormalEstimator.target = *clouds[cloud];
			NormalEstimator.calculateNormals(5 * origin_res, 5 * permutation_res);
			NormalEstimator.removeNaNNormals();
			*origin_cloud = NormalEstimator.query;
			*clouds[cloud] = NormalEstimator.target;

			//Detect keypoints
			time_meas();
			KeypointDetector.calculateIssKeypoints(KeypointDetector.queryKeypoints_, *origin_cloud, NormalEstimator.queryNormals_, origin_res, 0.9f, 5);
			KeypointDetector.calculateIssKeypoints(KeypointDetector.targetKeypoints_, *clouds[cloud], NormalEstimator.targetNormals_, permutation_res, 0.9f, 5);

			//Calculate descriptor for each keypoint
			time_meas();
			Describer.NormalEstimator = NormalEstimator;
			Describer.KeypointDetector = KeypointDetector;
			Describer.query_ = *origin_cloud;
			Describer.target_ = *clouds[cloud];
			Describer.calculateDescriptor(30 * origin_res, 30 * permutation_res);

			//Matching
			float c_threshold = 1.0f;
			Matcher.desc = Describer;
			Matcher.calculateCorrespondences(c_threshold);

			// RANSAC based Correspondence Rejection with ICP, default iterations = 1000, default threshold = 0.05
			RansacRejector.setMaximumIterations(1000);
			ransac_rejection(Matcher.corresp, KeypointDetector.queryKeypoints_, KeypointDetector.targetKeypoints_, RansacRejector);
			Eigen::Matrix4f ransac_transformation = get_ransac_transformation_matrix(RansacRejector);
			pcl::transformPointCloud(KeypointDetector.queryKeypoints_, KeypointDetector.queryKeypoints_, ransac_transformation);
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
			////add icp aligned points
			//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> icpc(aligned_cloud, 226, 176, 7);
			//viewer->addPointCloud<pcl::PointXYZ>(aligned_cloud, icpc, "icp cloud");
			//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "icp cloud");
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

			boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
			viewer->setBackgroundColor(0, 0, 0);
			viewer->initCameraParameters();
			//Add origin points to visualizer
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue(origin_cloud, 0, 0, 200);
			viewer->addPointCloud<pcl::PointXYZ>(origin_cloud, blue, "sample cloud1");
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud1");
			//add unaligned points to visualizer
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(clouds[i], 200, 0, 0);
			viewer->addPointCloud<pcl::PointXYZ>(clouds[i], red, "sample cloud2");
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud2");

			//transform unaligned points and add to visualizer
			Eigen::Matrix4f rt = get<0>(transformation_matrices[i]);
			Eigen::Matrix4f it = get<1>(transformation_matrices[i]);
			pcl::transformPointCloud(*clouds[i], *transformed_cloud, it);
			pcl::transformPointCloud(*transformed_cloud, *transformed_cloud, rt);
			transformed_clouds.push_back(transformed_cloud);

			viewer->addPointCloud<pcl::PointXYZ>(transformed_cloud, "sample cloud3");
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud3");
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "sample cloud3");
			while (!viewer->wasStopped())
			{
				viewer->spinOnce(100);
				std::this_thread::sleep_for(100ms);
			}
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
	}

	//Element 2 of exec params = OBJECT DETECTION
	if (std::get<1>(execution_params[2])) {
	get_all_file_names(query_directory, ".ply", query_names);
	get_all_file_names(target_directory, ".ply", target_names);

	string query_filename = query_directory + "/" + query_names[7].string();
	for (int neighbor = 0; neighbor < keypointdetector_nof_neighbors.size(); ++neighbor)
	{
		for (int threshold = 0; threshold < keypointdetector_threshold.size(); ++threshold)
		{
			for (int target_number = 0; target_number < target_names.size(); ++target_number)
			{
				vector<tuple<string, float>> processing_times, stats;
				Normals NormalEstimator;
				KeypointDetector KeypointDetector;
				Descriptor Describer;
				Matching Matcher;
				string target_filename = target_directory + "/" + target_names[target_number].string();

				int name_pos_query = query_filename.find("2", 0);
				int ext_pos_query = query_filename.find(".ply", 0);
				int name_pos = target_filename.find((target_names[target_number].string()), 0);
				int extension_pos = target_filename.find(".ply", 0);
				string query_identifier = get_identifier(query_filename, name_pos_query, ext_pos_query);
				string target_identifier = get_identifier(target_filename, name_pos, extension_pos);

				string pr_filename = pr_root + "/" + dataset + "/" + object + "/" + preprocessor_mode + "/" + descriptor
					+ "/iss" + std::to_string(keypointdetector_nof_neighbors[neighbor])
					+ "_th" + std::to_string(keypointdetector_threshold[threshold]).substr(0, 3)
					+ "/" + query_identifier + "_to_" + target_identifier + ".csv";
				string stats_filename = stats_root + "/" + dataset + "/" + object + "/" + preprocessor_mode + "/" + descriptor
					+ "/iss" + std::to_string(keypointdetector_nof_neighbors[neighbor])
					+ "_th" + std::to_string(keypointdetector_threshold[threshold]).substr(0, 3)
					+ "/" + query_identifier + "_to_" + target_identifier + "_stats.csv";

				std::string query_fileformat = get_fileformat(query_filename);
				std::string target_fileformat = get_fileformat(target_filename);
				query = load_3dmodel(query_filename, query_fileformat);
				target = load_3dmodel(target_filename, target_fileformat);
				queryResolution = static_cast<float> (compute_cloud_resolution(query.makeShared()));
				targetResolution = static_cast<float> (compute_cloud_resolution(target.makeShared()));

				//Estimate Normals
				time_meas();
				NormalEstimator.query = query;
				NormalEstimator.target = target;
				NormalEstimator.calculateNormals(5 * queryResolution, 5 * targetResolution);
				NormalEstimator.removeNaNNormals();
				query = NormalEstimator.query;
				target = NormalEstimator.target;
				processing_times.push_back(time_meas("normal estimation"));
				//Detect keypoints
				time_meas();
				KeypointDetector.calculateIssKeypoints(KeypointDetector.queryKeypoints_, query, NormalEstimator.queryNormals_, queryResolution,
					keypointdetector_threshold[threshold], keypointdetector_nof_neighbors[neighbor]);
				KeypointDetector.calculateIssKeypoints(KeypointDetector.targetKeypoints_, target, NormalEstimator.targetNormals_, targetResolution,
					keypointdetector_threshold[threshold], keypointdetector_nof_neighbors[neighbor]);
				processing_times.push_back(time_meas("detecting keypoints"));

				//Calculate descriptor for each keypoint
				time_meas();
				Describer.NormalEstimator = NormalEstimator;
				Describer.KeypointDetector = KeypointDetector;
				Describer.query_ = query;
				Describer.target_ = target;
				Describer.calculateDescriptor(supportRadius_ * queryResolution, supportRadius_ * targetResolution);
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
				ransac_rejection(Matcher.corresp, KeypointDetector.queryKeypoints_, KeypointDetector.targetKeypoints_, RansacRejector);
				Eigen::Matrix4f transformation_matrix = get_ransac_transformation_matrix(RansacRejector);
				pcl::transformPointCloud(KeypointDetector.queryKeypoints_, KeypointDetector.queryKeypoints_, transformation_matrix);
				pcl::transformPointCloud(query, query, transformation_matrix);
				processing_times.push_back(time_meas("Ransac Rejection"));

				// Iterative closest Point ICP
				time_meas();
				Eigen::Matrix4f icp_transformation_matrix = icp(query, target);
				pcl::transformPointCloud(query, query, icp_transformation_matrix);
				pcl::transformPointCloud(KeypointDetector.queryKeypoints_, KeypointDetector.queryKeypoints_, icp_transformation_matrix);
				processing_times.push_back(time_meas("ICP"));

				//Calculate euclidean distance of a query keypoint to its matched target keypoint: sqrt(delta_x^2 + delta_y^2 + delta_z^2)
				std::vector<float> euclidean_distance = calculate_euclidean_distance(KeypointDetector.queryKeypoints_, KeypointDetector.targetKeypoints_);
				float distance_threshold = shotRadius_ * queryResolution / 2;
				pcl::Correspondences true_positives = get_true_positives(distance_threshold, euclidean_distance);
				print_results(true_positives, KeypointDetector);

				//Enable if evaluation according to Guo et al.
#if 0
//A match is considered TP if the euclidean distance of a query keypoint to its matched target keypoint is less than half the supportradius
				pcl::Correspondences true_positives = get_true_positives(distance_threshold, euclidean_distance);
				//Store the NNDR and the euclidean distance for the evaluation according to Guo et al.
				pr_filename = "../../../../PR/Guo/" + query_name + "_to_" + target_name + ".csv";
				std::string results = concatenate_results(KeypointDetector, euclidean_distance, distance_threshold);
				filehandler.writeToFile(results, pr_filename);
				print_results(true_positives, KeypointDetector);
#endif

				//Enable if evaluation according to Buch et al.
#if 1
				accumulate_keypoints(KeypointDetector);
				std::string results = concatenate_distances(euclidean_distance);
				//FileHandler.writeToFile(results, pr_filename);
#endif
				std::string NOF_keypoints = std::to_string(accumulated_keypoints) + "," + std::to_string(distance_threshold) + "\n";
				//FileHandler.writeToFile(NOF_keypoints, pr_filename);
				NOF_keypoints = "";
				accumulated_keypoints = 0;
				//corr.clear();
			}
		}
	}
}

	//Enable if the visualization module should be used
#if 0
	//Enable if manual transformation matrix construction is desired
#if 0

				Eigen::Matrix3f	temp, Rx, Ry, Rz;
				Eigen::Matrix4f T;
				Eigen::Vector4f vec;
				vec << 0, 0, 0, 1;

				float psi = 0;
				float theta = 0;
				float phi = 0.262;

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

				pcl::transformPointCloud(query, query, T);
#endif
				boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
				viewer->setBackgroundColor(0, 0, 0);
				//viewer->addCoordinateSystem (0.1);
				viewer->initCameraParameters();

				//Move query so that it is separated from the target to see correspondences
				Eigen::Matrix4f t;
				t << 1, 0, 0, queryResolution * 200,
					0, 1, 0, 0,
					0, 0, 1, 0,
					0, 0, 0, 1;

				pcl::transformPointCloud(KeypointDetector.queryKeypoints_, KeypointDetector.queryKeypoints_, t);
				pcl::transformPointCloud(query, query, t);

				//Add query keypoints to visualizer
				pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(KeypointDetector.queryKeypoints_.makeShared(), 200, 0, 0);
				viewer->addPointCloud<pcl::PointXYZ>(KeypointDetector.queryKeypoints_.makeShared(), red, "sample cloud1");
				viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "sample cloud1");

				//add target keypoints to visualizer
				pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue(KeypointDetector.targetKeypoints_.makeShared(), 0, 0, 150);
				viewer->addPointCloud<pcl::PointXYZ>(KeypointDetector.targetKeypoints_.makeShared(), blue, "sample cloud2");
				viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "sample cloud2");
				//add lines between the correspondences
				viewer->addCorrespondences<pcl::PointXYZ>(KeypointDetector.queryKeypoints_.makeShared(), KeypointDetector.targetKeypoints_.makeShared(), corr, "correspondences");

				//add query points to visualizer
				viewer->addPointCloud<pcl::PointXYZ>(query.makeShared(), "sample cloud3");
				viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud3");
				viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "sample cloud3");

				//add target points to visualizer
				viewer->addPointCloud<pcl::PointXYZ>(target.makeShared(), "sample cloud4");
				viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud4");
				viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "sample cloud4");

				////add normals
				//viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(query.makeShared(), NormalEstimator.queryNormals_.makeShared(), 15, 0.005, "query_normals");
				//viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(target.makeShared(), NormalEstimator.targetNormals_.makeShared(), 15, 0.005, "target_normals");
				//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "query_normals");
				//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "target_normals");

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

#endif

	return 0;
}