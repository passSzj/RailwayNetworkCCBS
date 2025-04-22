#ifndef RAILWAY_NETWORK_CCBS_MAP_H
#define RAILWAY_NETWORK_CCBS_MAP_H

#include <vector>
#include "struct.hpp"
#include "tinyxml2.h"
#include <sstream>
#include <fstream>
#include <iostream>
#pragma once
class Map{
    private:
        std::vector<Vertex> vertices;
        std::vector<std::vector<Node>> valid_moves;
        int size;

    public:
        Map(){}
        ~Map(){}
        bool getMap(const char* file_path);
        std::vector<Node> getValidMoves(int index,int id);
        subVertex getSubVertex(int id,int index);
        int getMapSize(){return size;}
        double getX(int id);
        double getY(int id);

};

#endif //RAILWAY_NETWORK_CCBS_MAP_H