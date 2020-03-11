#ifndef CSVTOPOINTCLOUD_H
#define CSVTOPOINTCLOUD_H
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
		myfile.open(filename);
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

	std::vector<std::vector<float>> espros_csv_to_distance_array(std::string filename, int rows, int columns)
	{
		std::ifstream file(filename);
		if (!file.is_open()) {
			std::cout << "File: "<< filename <<" could not be read" << std::endl;
			std::vector<std::vector<float>> a;
			return a;
		}
		std::string line = "";
		std::vector<std::vector<float>> pointcloud(rows, std::vector<float>(columns, 0));
		int indRow = 0;
		std::string word;
		//the csv file consists of 25 datasets. Each set consists of three subsets (distance, amplitude, distance filtered). Each subset consists of 240x320pixel.
		//All unfiltered distance values get accumulated
		while (getline(file, line)) {
			if ((indRow / rows) % 3 == 0) {
				std::stringstream s(line);
				// convert string to float and store it in pointcloud
				int indCol = 0;
				while (std::getline(s, word, ',')) {
					if (std::stof(word) < 3000) { //we assume that the distance is not greater than 3m
						pointcloud[(indRow % rows)][indCol] += std::stof(word);
					}
					++indCol;
				}
			}
			++indRow;
		}
		// Close the File
		file.close();
		//Divide by number of subsets to get mean <---- BUG (SHOULDNT IT BE 25) ?!
		for (int i = 0; i < rows; ++i) {
			for (int j = 0; j < columns; ++j) {
				pointcloud[i][j] = pointcloud[i][j] /(indRow/rows);
			}
		}
		return pointcloud;
	}

	pcl::PointCloud<pcl::PointXYZ> getCloudFromPNG(char* filename) {
		// Load the depth input file
		vtkSmartPointer<vtkImageData> depthImageData;
		vtkSmartPointer<vtkPNGReader> depthReader;

		depthReader = vtkSmartPointer<vtkPNGReader>::New();
		depthReader->SetFileName(filename);
		depthReader->Update();
		depthImageData = depthReader->GetOutput();

		if (depthReader->GetNumberOfScalarComponents() != 1)
		{
			print_error("Component number of depth input file should be 1.\n");
			exit(-1);
		}
		//Get the dimensions of the image
		int dimensions[3];
		depthImageData->GetDimensions(dimensions);

		float depth;
		float z;
		PointCloud<PointXYZ> pointCloud;
		PointXYZ point;
		//Read all pixels and calculate the corresponding real world coordinate
		for (int x = 0; x < dimensions[0]; ++x) {
			for (int y = 0; y < dimensions[1]; ++y) {
				depth = depthImageData->GetScalarComponentAsFloat(x, y, 0, 0);			
				//Depth correction
				z = (depth * 1.0266 - 26.88)*0.1;
				//Calculate world coordinates using the Pinhole Modell and the Calibrationmatrix K
				point.x = (x-204.98)*z / 1076.74;
				point.y = (y-176.59)*z / 1075.17;
				point.z = z;

				pointCloud.push_back(point);
			}
		}
		return pointCloud;
	}

	
};

#endif