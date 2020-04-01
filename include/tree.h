#ifndef TREE_H
#define TREE_H

#include "boost/filesystem.hpp"
#include "boost/filesystem/path.hpp"
#include <iostream>
#include <filesystem>


class Tree {
public:
    //declaration for new tree node
    struct node {
        std::string data;
        vector<node*>child;
    };

    struct node* newNode(std::string data) {
        node* temp = new node;
        temp->data = data;
        return temp;
    }
};
#endif