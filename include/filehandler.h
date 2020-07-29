#ifndef FILEHANDLER_H
#define FILEHANDLER_H
#include <sstream>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/filters/median_filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <fstream>
#include "Descriptor.h"

namespace fs = boost::filesystem;

class FileHandler {
public:
	std::vector<std::vector<float>> melexis_txt_to_distance_array(std::string filename, int rows, int columns) {
		std::ifstream file(filename);
		if (!file.is_open()) {
			std::cout << "File: " << filename << " could not be read" << std::endl;
			std::vector<std::vector<float>> a;
			return a;
		}
		std::string line = "";
		std::vector<std::vector<float>> distance_array(rows, std::vector<float>(columns, 0));
		int indRow = 0;
		std::string word;
		while (getline(file, line)) {
			std::stringstream s(line);
			int indCol = 0;
			while (std::getline(s, word, ',')) {
				if (std::stof(word) < 3000) { //we assume that the distance is not greater than 3m
					distance_array[(indRow % rows)][indCol] += std::stof(word);
				}
				++indCol;
			}
			++indRow;
		}
		file.close();
		return distance_array;
	}

	std::string get_fileformat(string filename) {
		//Get fileformat of the query -> last three letters of filename
		return filename.std::string::substr(filename.length() - 3);
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

	bool path_valid(std::string path) {
		if (!boost::filesystem::exists(path))
		{
			std::cout << "Path entered was not valid!" << std::endl;
			return false;
		}
		return true;
	}

	void load_ply_file(std::string filename, pcl::PointCloud<PointXYZ>::Ptr cloud) {
		if (pcl::io::loadPLYFile(filename, *cloud) == -1)
		{
			std::cout << "Error loading query cloud." << std::endl;
		}
	}

	void writeToFile(std::string data, string filename) {
	//	if (path_valid(filename)) {
			ofstream myfile;
			myfile.open(filename, std::ofstream::app);
			myfile << data;
			myfile.close();
			std::cout << "Writing to File succeeded." << endl;
		//}
		//else {
		//	std::cout << "Writing to File did not succeeded." << endl;
		//}
	}

	void get_all_file_names(const fs::path& root, const string& ext, vector<fs::path>& ret)
	{
		ret.clear();
		if (!fs::exists(root) || !fs::is_directory(root)) return;

		fs::recursive_directory_iterator it(root);
		fs::recursive_directory_iterator endit;

		while (it != endit)
		{
			if (fs::is_regular_file(*it) && it->path().extension() == ext) ret.push_back(it->path().filename());
			++it;

		}
	}
};

#endif