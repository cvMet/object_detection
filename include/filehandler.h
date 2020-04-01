#ifndef FILEHANDLER_H
#define FILEHANDLER_H
#include <sstream>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/filters/median_filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <fstream>
#include "Descriptor.h"

class FileHandler {
public:

	void writeToFile(std::string data, string filename) {
		ofstream myfile;
		myfile.open(filename, std::ofstream::app);
		myfile << data;
		myfile.close();
		std::cout << "Writing to File succeeded." << endl;
	}

	std::vector<std::vector<float>> melexis_txt_to_distance_array(std::string filename, int rows, int columns) {
		std::ifstream file(filename);
		if (!file.is_open()) {
			std::cout << "File: " << filename << " could not be read" << std::endl;
			std::vector<std::vector<float>> a;
			return a;
		}
		std::string line = "";
		std::vector<std::vector<float>> pointcloud(rows, std::vector<float>(columns, 0));
		int indRow = 0;
		std::string word;
		while (getline(file, line)) {
			std::stringstream s(line);
			int indCol = 0;
			while (std::getline(s, word, ',')) {
				if (std::stof(word) < 3000) { //we assume that the distance is not greater than 3m
					pointcloud[(indRow % rows)][indCol] += std::stof(word);
				}
				++indCol;
			}
			++indRow;
		}
		file.close();
		return pointcloud;
	}
	
};

#endif