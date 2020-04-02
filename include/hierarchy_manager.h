#ifndef HIERARCHY_MANAGER_H
#define HIERARCHY_MANAGER_H

#include "tree.h"

class HierarchyManager
{
public:
	//Create directory path for results such as PR curves or stats
	//Example params: root = "../../../../stats", dataset = "20_03_20", object = "Gipfeli"
	void create_result_dirs(string root, string dataset, string objectname) {
		Tree DirectoryTree;
		std::string root_dir = root + "/" + dataset;

		Tree::node* rootNode = DirectoryTree.newNode(objectname);
		rootNode->child.push_back(DirectoryTree.newNode("B_SHOT"));
		rootNode->child.push_back(DirectoryTree.newNode("FPFH"));
		for (int i = 0; i < rootNode->child.size(); ++i) {
			rootNode->child[i]->child.push_back(DirectoryTree.newNode("iss3_th0.7"));
			rootNode->child[i]->child.push_back(DirectoryTree.newNode("iss3_th0.9"));
			rootNode->child[i]->child.push_back(DirectoryTree.newNode("iss5_th0.7"));
			rootNode->child[i]->child.push_back(DirectoryTree.newNode("iss5_th0.9"));
			for (int j = 0; j < rootNode->child[i]->child.size(); ++j) {
				rootNode->child[i]->child[j]->child.push_back(DirectoryTree.newNode("0_rot"));
				rootNode->child[i]->child[j]->child.push_back(DirectoryTree.newNode("30_rot"));
				rootNode->child[i]->child[j]->child.push_back(DirectoryTree.newNode("0_tran"));
				rootNode->child[i]->child[j]->child.push_back(DirectoryTree.newNode("30_tran"));
				for (int k = 0; k < rootNode->child[i]->child[j]->child.size(); ++k) {
					boost::filesystem::create_directories(root_dir + rootNode->data + "/" + rootNode->child[i]->data + "/" + rootNode->child[i]->child[j]->data + "/" + rootNode->child[i]->child[j]->child[k]->data);
				}
			}
		}
	}



};

#endif