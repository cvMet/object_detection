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
#include "../include/scene.h"
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
#include "../include/registrator.h"
#include "../include/parameter_handler.h"
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
std::vector<tuple<bshot_descriptor, int>> learned_descriptors;
std::vector<tuple<string, vector<double>>> angles;

std::clock_t start, end_time;
float shotRadius_ = 30;
float fpfhRadius_ = 20;

const int rows = 240, columns = 320;
bool running = false;
bool query_learning = false;

//Paths used during objectdetection
string query_directory = "../../../../clouds/query_clouds";
string target_directory = "../../../../clouds/target_clouds";
string pr_root = "../../../../PR/Buch";
string stats_root = "../../../../stats";
vector<fs::path> query_names;
vector<fs::path> target_names;
//Paths used during merging
string origin_directory = "../../../../clouds/merging/origin";
string permutation_directory = "../../../../clouds/merging/permutation";
vector<fs::path> origin_names;
vector<fs::path> permutation_names;
//Paths used during learning
string query_learning_directory = "../../../../clouds/learning";
vector<fs::path> query_learning_names;
//Paths used during processing
string processing_directory = "../../../../clouds/processing";
string processed_directory = "../../../../clouds/processing/processed";
vector<fs::path> processing_names;

#if isshot
const float supportRadius_ = shotRadius_;
string descriptor = "B_SHOT";
#else
const float supportRadius_ = fpfhRadius_;
string descriptor = "FPFH";
#endif

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

std::vector<double> get_angles(Eigen::Matrix4f transformation_matrix) {
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

std::tuple<string, float> time_meas(string action) {
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

std::string create_writable_stats(vector<tuple<string, float>> stats) {
	std::string data = "";
	for (int i = 0; i < stats.size(); ++i) {
		data += get<0>(stats[i]) + ',' + std::to_string(get<1>(stats[i]));
		data += "\n";
	}
	return data;
}

std::string create_writable_angles(vector<tuple<string, vector<double>>> angles) {
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

std::string get_identifier(string filename, int name_pos, int extension_pos) {
	return filename.substr(name_pos, (extension_pos - name_pos));
}

std::string get_input() {
	std::cout << "enter value: " << std::endl;
	string input;
	cin >> input;
	return input;
}

float get_median(std::vector<float> values) {
	std::vector<float>::iterator first = values.begin();
	std::vector<float>::iterator last = values.end();
	std::vector<float>::iterator middle = first + (last - first) / 2;
	std::nth_element(first, middle, last);
	return *middle;
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

void set_query_learning() {
	query_learning = true;
}

int main(int argc, char* argv[])
{
	FileHandler FileHandler;
	CloudCreator CloudCreator;
	ParameterHandler PH;
	ParameterHandler* ParameterHandler = &PH;

	BaseMenu* CurrentMenu = new MainMenu(ParameterHandler);
	bool quit = false;
	bool execute = false;
	while (!quit && !execute)
	{
		CurrentMenu->clearScreen();
		CurrentMenu->printFlavorText();
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

	if (ParameterHandler->get_exec_param_state("cloudcreation")) {
		//Paths used during cloudgen
		string depth_directory = "../../../../datasets/" + ParameterHandler->get_dataset() + "/raw_depth/" + ParameterHandler->get_object();
		string depth_extension = ".txt";
		string cloud_directory = "../../../../clouds/" + ParameterHandler->get_dataset() + "/" + ParameterHandler->get_object();
		vector<fs::path> depth_names;
		vector<fs::path> cloud_names;
		//Clouds used during cloud generation
		pcl::PointCloud<pcl::PointXYZ> query_cloud;
		pcl::PointCloud<pcl::PointXYZ> background_cloud;
		pcl::PointCloud<pcl::PointXYZ> filtered_query;
		pcl::PointCloud<pcl::PointXYZ> filtered_background;
		pcl::PointCloud<pcl::PointXYZ> final_cloud;

		FileHandler.get_all_file_names(depth_directory, depth_extension, depth_names);
		std::string bkgr_depth_filename = depth_directory + "/" + depth_names[0].string();
		std::vector<std::vector<float>> background_distance_array = FileHandler.melexis_txt_to_distance_array(bkgr_depth_filename, rows, columns);
		background_cloud = CloudCreator.distance_array_to_cloud(background_distance_array, 6.0f, 0.015f, 0.015f);
		if (ParameterHandler->get_filter_state("median")) {
			background_cloud = CloudCreator.median_filter_cloud(background_cloud, 5);
		}
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
			if (ParameterHandler->get_filter_state("median")) {
				time_meas();
				query_cloud = CloudCreator.median_filter_cloud(query_cloud, 5);
				creation_stats.push_back(time_meas("time_medianfilter"));
			}
			time_meas();
			final_cloud = CloudCreator.remove_background(query_cloud, background_cloud, 0.005f);
			creation_stats.push_back(time_meas("time_backgroundRemoval"));
			if (ParameterHandler->get_filter_state("roi")) {
				time_meas();
				final_cloud = CloudCreator.roi_filter(final_cloud, "x", -0.2f, 0.2f);
				final_cloud = CloudCreator.roi_filter(final_cloud, "y", -0.2f, 0.2f);
				creation_stats.push_back(time_meas("time_ROIfilter"));
			}
			if (ParameterHandler->get_filter_state("sor")) {
				time_meas();
				final_cloud = CloudCreator.remove_outliers(final_cloud.makeShared(), 10);
				creation_stats.push_back(time_meas("time_SORfilter"));
			}
			string filename = cloud_directory + "/" + depth_names[i].string();
			string substr = filename.substr(0, (filename.length() - 4)) + ".ply";
			pcl::io::savePLYFileASCII(substr, final_cloud);
			if (ParameterHandler->get_cloudgen_stats_state()) {
				string statistics = create_writable_stats(creation_stats);
				string stats_filename = filename.substr(0, (filename.length() - 4)) + "_stats.csv";
				FileHandler.writeToFile(statistics, stats_filename);
			}
		}
	}

	if (ParameterHandler->get_exec_param_state("merging")) {
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> transformed_clouds;
		std::vector<tuple<Eigen::Matrix4f, Eigen::Matrix4f>> transformation_matrices;
		pcl::PointCloud<pcl::PointXYZ>::Ptr origin_cloud(new pcl::PointCloud<pcl::PointXYZ>);

		//Load origin cloud
		FileHandler.get_all_file_names(origin_directory, ".ply", origin_names);
		string origin_filename = origin_directory + "/" + origin_names[0].string();
		FileHandler.load_ply_file(origin_filename, origin_cloud);
		float queryResolution = CloudCreator.compute_cloud_resolution(origin_cloud);

		//Load all permutation clouds into vector
		FileHandler.get_all_file_names(permutation_directory, ".ply", permutation_names);
		for (int cloud = 0; cloud < permutation_names.size(); ++cloud) {
			pcl::PointCloud<pcl::PointXYZ>::Ptr permutation_cloud(new pcl::PointCloud<pcl::PointXYZ>);
			string cloud_filename = permutation_directory + "/" + permutation_names[cloud].string();
			FileHandler.load_ply_file(cloud_filename, permutation_cloud);
			clouds.push_back(permutation_cloud);
		}

		for (int cloud = 0; cloud < clouds.size(); ++cloud) {
			Normals NormalEstimator;
			KeypointDetector KeypointDetector;
			Descriptor Describer;
			Matching Matcher;
			pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>);
			float targetResolution = CloudCreator.compute_cloud_resolution(clouds[cloud]);

			Scene Query("query", origin_cloud);
			Query.resolution = queryResolution;
			Scene Target("target", clouds[cloud]);
			Target.resolution = targetResolution;

			//Estimate Normals
			NormalEstimator.set_scalefactor(ParameterHandler->get_ne_scalefactor());
			NormalEstimator.calculateNormals(Query.resolution, Query.cloud);
			NormalEstimator.removeNaNNormals(Query.cloud);
			Query.normals = NormalEstimator.normals;
			NormalEstimator.calculateNormals(targetResolution, Target.cloud);
			NormalEstimator.removeNaNNormals(Target.cloud);
			Target.normals = NormalEstimator.normals;

			//Detect keypoints
			KeypointDetector.set_threshold(0.7f);
			KeypointDetector.set_neighbor_count(5);
			KeypointDetector.calculateIssKeypoints(Query);
			Query.keypoints = KeypointDetector.keypoints;
			KeypointDetector.calculateIssKeypoints(Target);
			Target.keypoints = KeypointDetector.keypoints;

			//Calculate descriptor for each keypoint
			Describer.calculateDescriptor(Query);
			Query.descriptors = Describer.descriptors;
			Describer.calculateDescriptor(Target);
			Target.descriptors = Describer.descriptors;

			//Matching
			Matcher.queryDescriptor_ = Query.descriptors;
			Matcher.targetDescriptor_ = Target.descriptors;
			Matcher.calculateCorrespondences(ParameterHandler->get_matcher_distance_threshold(), true);

			Registrator Registrator(Query, Target, Matcher.corresp);
			Registrator.set_distance_threshold(supportRadius_* Query.resolution / 2);
			Registrator.init_RANSAC();
			Eigen::Matrix4f ransac_transformation = Registrator.do_ransac();
			Eigen::Matrix4f icp_transformation = Registrator.do_icp();
			//Add inverse of transformation matrices to corresponding vector to enable target-to-source transformation
			transformation_matrices.push_back(make_tuple(ransac_transformation.inverse(), icp_transformation.inverse()));
		}
		if (ParameterHandler->get_visualization_state()) {
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
		}

		pcl::PointCloud<pcl::PointXYZ>::Ptr master_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		*master_cloud = *origin_cloud;
		for (int i = 0; i < transformed_clouds.size(); ++i) {
			*master_cloud += *transformed_clouds[i];
		}
		pcl::io::savePLYFileASCII("../../../../clouds/merging/merged/mastercloud.ply", *master_cloud);
		if (ParameterHandler->get_visualization_state()) {
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
		}
		*master_cloud = CloudCreator.remove_outliers(master_cloud, 50);
		pcl::io::savePLYFileASCII("../../../../clouds/merging/merged/mastercloud_SOR.ply", *master_cloud);
	}

	if (ParameterHandler->get_exec_param_state("detection")) {
		//Clouds used during objectdetection
		pcl::PointCloud<pcl::PointXYZ>::Ptr query(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
		FileHandler.get_all_file_names(query_directory, ".ply", query_names);
		FileHandler.get_all_file_names(target_directory, ".ply", target_names);

		for (int neighbor = 0; neighbor < ParameterHandler->sizeof_detector_nn(); ++neighbor)
		{
			for (int threshold = 0; threshold < ParameterHandler->sizeof_detector_threshold(); ++threshold)
			{
				for (int query_number = 0; query_number < query_names.size(); ++query_number)
				{
					Normals QueryNormalEstimator;
					KeypointDetector QueryKeypointDetector;
					Descriptor QueryDescriber;

					std::string log_filename = pr_root + "/" + ParameterHandler->get_dataset() + "/" + ParameterHandler->get_object() + "/" + ParameterHandler->get_preprocessor_mode() + "/" + descriptor + "/log.csv";
					std::string match_log_filename = pr_root + "/" + ParameterHandler->get_dataset() + "/" + ParameterHandler->get_object() + "/" + ParameterHandler->get_preprocessor_mode() + "/" + descriptor + "/match_log.csv";
					std::string query_filename = query_directory + "/" + query_names[query_number].string();
					int name_pos_query = query_filename.find((query_names[query_number].string()), 0);
					int ext_pos_query = query_filename.find(".ply", 0);
					std::string query_identifier = get_identifier(query_filename, name_pos_query, ext_pos_query);

					FileHandler.load_ply_file(query_filename, query);
					Scene Query(query_identifier, query);
					for (int p = 0; p < Query.cloud->points.size(); ++p) {
						Query.cloud->points[p].x = Query.cloud->points[p].x * 0.001;
						Query.cloud->points[p].y = Query.cloud->points[p].y * 0.001;
						Query.cloud->points[p].z = Query.cloud->points[p].z * 0.001;
					}
					Query.resolution = CloudCreator.compute_cloud_resolution(Query.cloud);
					//Query Normal Estimation
					QueryNormalEstimator.set_scalefactor(ParameterHandler->get_ne_scalefactor());
					QueryNormalEstimator.calculateNormals(Query.resolution, Query.cloud);
					QueryNormalEstimator.removeNaNNormals(Query.cloud);
					Query.normals = QueryNormalEstimator.normals;
					//Query Keypoint Detection
					QueryKeypointDetector.set_threshold(ParameterHandler->get_detector_threshold_at(threshold));
					QueryKeypointDetector.set_neighbor_count(ParameterHandler->get_detector_nn_at(neighbor));
					QueryKeypointDetector.calculateIssKeypoints(Query);
					Query.keypoints = QueryKeypointDetector.keypoints;
					//Query Description
					QueryDescriber.set_support_radius(supportRadius_);
					QueryDescriber.calculateDescriptor(Query);
					Query.descriptors = QueryDescriber.descriptors;

					for (int target_number = 0; target_number < target_names.size(); ++target_number)
					{
						Normals NormalEstimator;
						KeypointDetector KeypointDetector;
						Descriptor Describer;
						Matching Matcher;
						vector<tuple<string, float>> processing_times, stats;

						string target_filename = target_directory + "/" + target_names[target_number].string();
						int name_pos = target_filename.find((target_names[target_number].string()), 0);
						int extension_pos = target_filename.find(".ply", 0);
						string target_identifier = get_identifier(target_filename, name_pos, extension_pos);
						
						FileHandler.load_ply_file(target_filename, target);
						Scene Target(target_identifier, target);
						// Necessary if undistorted clouds are processed (since they are in mm but pcl works on m-basis)
						for (int p = 0; p < Target.cloud->points.size(); ++p) {
							Target.cloud->points[p].x = Target.cloud->points[p].x * 0.001;
							Target.cloud->points[p].y = Target.cloud->points[p].y * 0.001;
							Target.cloud->points[p].z = Target.cloud->points[p].z * 0.001;
						}
						Target.resolution = CloudCreator.compute_cloud_resolution(Target.cloud);

						//Target Normal Estimation
						time_meas();
						NormalEstimator.calculateNormals(Target.resolution, Target.cloud);
						NormalEstimator.removeNaNNormals(Target.cloud);
						Target.normals = NormalEstimator.normals;
						processing_times.push_back(time_meas("normal estimation"));

						//Target Keypoint Detection
						time_meas();
						KeypointDetector.set_threshold(ParameterHandler->get_detector_threshold_at(threshold));
						KeypointDetector.set_neighbor_count(ParameterHandler->get_detector_nn_at(neighbor));
						KeypointDetector.calculateIssKeypoints(Target);
						Target.keypoints = KeypointDetector.keypoints;
						processing_times.push_back(time_meas("detecting keypoints"));

						//Target Description
						time_meas();
						Describer.set_support_radius(supportRadius_);
						Describer.calculateDescriptor(Target);
						Target.descriptors = Describer.descriptors;
						processing_times.push_back(time_meas("calculating descriptor"));

						//Matching
						time_meas();
						Matcher.queryDescriptor_ = Query.descriptors;
						Matcher.targetDescriptor_ = Target.descriptors;
						Matcher.calculateCorrespondences(ParameterHandler->get_matcher_distance_threshold(), true);
						processing_times.push_back(time_meas("matching"));

						// Registration -> Output Preparation
						Registrator Registrator(Query, Target, Matcher.corresp);
						Registrator.set_distance_threshold(supportRadius_ * Query.resolution / 2);
						Registrator.init_RANSAC();
						Registrator.do_ransac();
						Registrator.do_icp();
						Registrator.print_precision_recall();
						std::string result = Registrator.get_result();

						string pr_filename = pr_root + "/" + ParameterHandler->get_dataset() + "/" + ParameterHandler->get_object() + "/" + ParameterHandler->get_preprocessor_mode() + "/" + descriptor
							+ "/" + query_identifier + "_to_" + target_identifier + ".csv";
						FileHandler.writeToFile(result, pr_filename);

						if (ParameterHandler->get_match_retrieval_state()) {
							std::string matches;
							if (ParameterHandler->get_ransac_state()) {
								matches = query_identifier + "_to_" + target_identifier + "," + std::to_string(Registrator.get_number_of_matches()) + "\n";
							}
							else {
								matches = query_identifier + "_to_" + target_identifier + "," + std::to_string(Matcher.corresp.size()) + "\n";
							}
							FileHandler.writeToFile(matches, match_log_filename);
						}
						if (ParameterHandler->get_detection_stats_state()) {
							string stats_filename = stats_root + "/" + ParameterHandler->get_dataset() + "/" + ParameterHandler->get_object() + "/" + ParameterHandler->get_preprocessor_mode() + "/" + descriptor
								+ "/" + query_identifier + "_to_" + target_identifier + "_stats.csv";
							std::string stats = create_writable_stats(processing_times);
							FileHandler.writeToFile(stats, stats_filename);
						}
						if (ParameterHandler->get_detection_logging_state()) {
							std::string log = query_identifier + "_to_" + target_identifier + "," + std::to_string((Registrator.get_number_of_tp() >= ParameterHandler->get_detection_threshold())) + "\n";
							FileHandler.writeToFile(log, log_filename);
						}
						if (ParameterHandler->get_visualization_state()) {
							boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
							viewer->setBackgroundColor(0, 0, 0);
							viewer->initCameraParameters();
							//Move query so that it is separated from the target to see correspondences
							Eigen::Matrix4f t;
							t << 1, 0, 0, Query.resolution * 200,
								0, 1, 0, 0,
								0, 0, 1, 0,
								0, 0, 0, 1;
							pcl::PointCloud<pcl::PointXYZ> visu_query;
							pcl::PointCloud<pcl::PointXYZ> visu_query_keypoints;
							if (Registrator.is_query_aligned()) {
								pcl::transformPointCloud(*Registrator.get_icp_aligned_keypoints(), visu_query_keypoints, t);
								pcl::transformPointCloud(*Registrator.get_icp_aligned_cloud(), visu_query, t);
							}
							else {
								pcl::transformPointCloud(Query.keypoints, visu_query_keypoints, t);
								pcl::transformPointCloud(*Query.cloud, visu_query, t);
							}
							//Add query keypoints to visualizer
							pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(visu_query_keypoints.makeShared(), 200, 0, 0);
							viewer->addPointCloud<pcl::PointXYZ>(visu_query_keypoints.makeShared(), red, "sample cloud1");
							viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "sample cloud1");
							//add target keypoints to visualizer
							pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue(Target.keypoints.makeShared(), 0, 0, 150);
							viewer->addPointCloud<pcl::PointXYZ>(Target.keypoints.makeShared(), blue, "sample cloud2");
							viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "sample cloud2");
							//add lines between the correspondences
							viewer->addCorrespondences<pcl::PointXYZ>(visu_query_keypoints.makeShared(), Target.keypoints.makeShared(), Matcher.corresp, "correspondences");
							//add query points to visualizer
							viewer->addPointCloud<pcl::PointXYZ>(visu_query.makeShared(), "sample cloud3");
							viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud3");
							viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "sample cloud3");
							//add target points to visualizer
							viewer->addPointCloud<pcl::PointXYZ>(Target.cloud, "sample cloud4");
							viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud4");
							viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "sample cloud4");
							while (!viewer->wasStopped())
							{
								viewer->spinOnce(100);
								std::this_thread::sleep_for(100ms);
							}
						}
					}
				}
			}
		}
	}

	if (ParameterHandler->get_exec_param_state("processing")) {
		if (ParameterHandler->get_background_removal_state()) {
			FileHandler.get_all_file_names(processing_directory, ".ply", processing_names);
			string background_filename = processing_directory + "/" + processing_names[0].string();
			int name_pos = background_filename.find((processing_names[0].string()), 0);
			int extension_pos = background_filename.find(".ply", 0);
			string background_identifier = get_identifier(background_filename, name_pos, extension_pos);
			pcl::PointCloud<pcl::PointXYZ> background;
			FileHandler.load_ply_file(background_filename, background.makeShared());
			//Pop first element (background without query)
			processing_names.erase(processing_names.begin());
			for (int i = 0; i < processing_names.size(); ++i) {
				string filename = processing_directory + "/" + processing_names[i].string();
				int name_pos = filename.find((processing_names[i].string()), 0);
				int extension_pos = filename.find(".ply", 0);
				string identifier = get_identifier(filename, name_pos, extension_pos);
				pcl::PointCloud<pcl::PointXYZ> scene;
				FileHandler.load_ply_file(filename, scene.makeShared());
				pcl::PointCloud<pcl::PointXYZ> processed_cloud = CloudCreator.remove_background(scene, background, ParameterHandler->get_background_removal_threshold());
				filename = processed_directory + "/" + processing_names[i].string();
				string substr = filename.substr(0, (filename.length() - 4)) + ".ply";
				pcl::io::savePLYFileASCII(substr, processed_cloud);
			}
		}
	}

	return 0;
}