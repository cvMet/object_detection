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

additional parts were written by Joël Carlen, Student at the Lucerne University of Applied Sciences and Arts
*/
#pragma once
#ifndef MATCHING_H
#define MATCHING_H

#include "descriptor.h"
#include "bshot_bits.h"
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

class Matching
{
public:
	Descriptor desc;
	pcl::Correspondences corresp;

	void calculateCorrespondences(float threshold) {
#if isshot
		int maxSize = std::max(desc.modelDescriptor_.size(), desc.sceneDescriptor_.size());
		int* dist = new int[maxSize];
		int* sceneNn = new int[desc.sceneDescriptor_.size()];
		int* modelNn = new int[desc.modelDescriptor_.size()];
		int* firstNnValue = new int[desc.modelDescriptor_.size()];;
		int* secondNnValue = new int[desc.modelDescriptor_.size()];;
		int minIndex;
		//Search nearest neighbor for every keypoint of the model
		for (int i = 0; i < (int)desc.modelDescriptor_.size(); ++i)
		{
			for (int k = 0; k < (int)desc.sceneDescriptor_.size(); ++k)
			{
				dist[k] = (int)(desc.modelDescriptor_[i].bits ^ desc.sceneDescriptor_[k].bits).count();	//XOR to get the Hamming distance
			}
			firstNnValue[i] = minVect(dist, (int)desc.sceneDescriptor_.size(), &minIndex); //get nearest neighbor
			modelNn[i] = minIndex;
			secondNnValue[i] = secondMinVect(dist, (int)desc.sceneDescriptor_.size(), firstNnValue[i], &minIndex); //get second nearest neighbor
			
		}
		//Search nearest neighbor for every keypoint of the scene
		for (int i = 0; i < (int)desc.sceneDescriptor_.size(); ++i)
		{
			for (int k = 0; k < (int)desc.modelDescriptor_.size(); ++k) {
				dist[k] = (int)(desc.sceneDescriptor_[i].bits ^ desc.modelDescriptor_[k].bits).count();
			}			
			minVect(dist, (int)desc.modelDescriptor_.size(), &minIndex);
			sceneNn[i] = minIndex;
		}

		for (int i = 0; i < (int)desc.modelDescriptor_.size(); ++i)
		{
			if (sceneNn[modelNn[i]] == i) //checks if points are "the same" to make sure no points are matched twice
			{
				//chech if NNDR is below threshold
				if (sqrt((float)firstNnValue[i]) / sqrt((float)secondNnValue[i]) < threshold)
				{
					pcl::Correspondence corr;
					corr.index_query = i;
					corr.index_match = modelNn[i];
					corr.distance = sqrt((float)firstNnValue[i]) / sqrt((float)secondNnValue[i]);
					corresp.push_back(corr);
				}
			}
		}
		
		delete[] dist;
		
#else
		//from http://pointclouds.org/documentation/tutorials/global_hypothesis_verification.php
		pcl::KdTreeFLANN<FPFHSignature33> matchSearch;
			
		std::vector<int> sceneNn,modelNn;
		std::vector<float> nnValue, secondNnValue;
		//Search nearest neighbor for every keypoint of the model
		matchSearch.setInputCloud(desc.sceneDescriptor_.makeShared());
		for (std::size_t i = 0; i < desc.modelDescriptor_.size(); ++i)
		{
			std::vector<int> neighborIndices(2);
			std::vector<float> neighborSquaredDistances(2);
			if (!std::isfinite(desc.modelDescriptor_.at(i).histogram[0]))  //skipping NaNs
			{
				continue;
			}
			int found_neighs = matchSearch.nearestKSearch(desc.modelDescriptor_.at(i), 2, neighborIndices, neighborSquaredDistances); //find nearest neighbour
			modelNn.push_back(neighborIndices[0]); //get index of NN
			nnValue.push_back(neighborSquaredDistances[0]); //get squared L2 distance to NN
			secondNnValue.push_back(neighborSquaredDistances[1]); //get squared L2 distance to second NN
		}
		//Search Nearest Neighbor for every Keypoint of the scene
		matchSearch.setInputCloud(desc.modelDescriptor_.makeShared());
		for (std::size_t i = 0; i < desc.sceneDescriptor_.size(); ++i)
		{
			std::vector<int> neighborIndices(1);
			std::vector<float> neighborSquaredDistances(1);
			if (!std::isfinite(desc.sceneDescriptor_.at(i).histogram[0]))  //skipping NaNs
			{
				continue;
			}
			int found_neighs = matchSearch.nearestKSearch(desc.sceneDescriptor_.at(i), 1, neighborIndices, neighborSquaredDistances); //find nearest neighbour
			sceneNn.push_back(neighborIndices[0]); //get index of NN

		}
		for (int i = 0; i < desc.modelDescriptor_.size(); ++i) {
			//check if points are the same and the NNDR below a threshold
			if (sceneNn[modelNn[i]] == i && (sqrt(nnValue[i]) / sqrt(secondNnValue[i])) <= threshold) {
				pcl::Correspondence corr(static_cast<int> (i), modelNn[i], sqrt(nnValue[i]) / sqrt(secondNnValue[i]));
				corresp.push_back(corr);
			}
		}
			
		
#endif

	}
};
#endif // !MATCHING_H