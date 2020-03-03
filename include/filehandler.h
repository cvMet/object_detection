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
	int nRows=240, nCols=320;
	void writeToFile(std::string data, string filename) {
		ofstream myfile;
		myfile.open(filename);
		myfile << data;
		myfile.close();
		std::cout << "Writing to File succeeded." << endl;
	}
	//from https://thispointer.com/how-to-read-data-from-a-csv-file-in-c/
	std::vector<std::vector<float>> readCsv(std::string filename)
	{
		std::ifstream file(filename);
		if (!file.is_open()) {
			std::cout << "File: "<< filename <<" could not be read" << std::endl;
			std::vector<std::vector<float>> a;
			return a;
		}

		std::string line = "";
		// Iterate through each line and split the content using delimeter

		std::vector<std::vector<float>> pointcloud(nRows, std::vector<float>(nCols, 0));
		int indRow = 0;
		std::string word;
		while (getline(file, line)) {


			if ((indRow / nRows) % 3 == 0) { // only add raw data -> row 0-239, 720-959 etc
				// used for breaking words 
				std::stringstream s(line);

				// read every column data of a row and 
				// store it in a string variable, 'word' 
				int indCol = 0;
				while (std::getline(s, word, ',')) {
					if (std::stof(word) < 3000) { //we assume that the distance is not greater than 3m
						pointcloud[(indRow % nRows)][indCol] += std::stof(word);
					}
					++indCol;
				}
			}
			++indRow;
		}
		// Close the File
		file.close();

		for (int i = 0; i < nRows; ++i) {
			for (int j = 0; j < nCols; ++j) {
				pointcloud[i][j] = pointcloud[i][j] /(indRow/nRows);
			}
		}
		return pointcloud;
	}

		pcl::PointCloud<pcl::PointXYZ> getCloud(std::vector<std::vector<float>> cloud, float f, float x_pixel, float y_pixel, float lowerBoundary)
	{
		pcl::PointCloud<pcl::PointXYZ> pointCloud;
		// Fill in the cloud data
		pointCloud.width = nCols;
		pointCloud.height = nRows;
		pointCloud.is_dense = false;
		pointCloud.points.resize(pointCloud.width * pointCloud.height);
		int ind = 0;
		double normalize = 0;
		for (int i = 0; i < nRows; ++i)
		{
			for (int j = 0; j < nCols; ++j)
			{
				//make sure the cloud is in m since the csv file is in mm
				normalize = 0.001 / sqrt(((j - nCols / 2) * x_pixel * (j - nCols / 2) * x_pixel) + ((i - nRows / 2) * y_pixel * (i - nRows / 2) * y_pixel) + (f * f));
				pointCloud.points[ind].x = normalize * cloud[i][j] * (j - nCols / 2) * x_pixel; //j / 1000.0f;//
				pointCloud.points[ind].y = normalize * cloud[i][j] * (i - nRows / 2) * y_pixel; //i/1000.0f;//(cloud[i][j] / (1000.0f * f)) * (i - nRows / 2) * y_pixel; //i
				pointCloud.points[ind].z = normalize * cloud[i][j] * f; //mm
				++ind;
			}
		}
		//Use Medianfilter with size 13x13
#if 1
		pcl::MedianFilter<pcl::PointXYZ> median;
		median.setInputCloud(pointCloud.makeShared());
		median.setWindowSize(13);
		median.applyFilter(pointCloud);
		std::cout << "Medianfilter size: " << median.getWindowSize() << std::endl;
#endif
		
		pcl::PointCloud<pcl::PointXYZ> finalCloud;
		for (int i = 0; i < pointCloud.size(); ++i) {
			if (pointCloud.at(i).z > lowerBoundary) {
				finalCloud.push_back(pointCloud.at(i));
			}
		}
		std::vector<int> vec;
		pcl::removeNaNFromPointCloud(finalCloud, finalCloud, vec);
		return finalCloud;
	}



	//from: https://github.com/PointCloudLibrary/pcl/blob/master/tools/png2pcd.cpp
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