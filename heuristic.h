#ifndef RAILWAY_NETWORK_CCBS_HEURISTIC_H
#define RAILWAY_NETWORK_CCBS_HEURISTIC_H

#include "struct.hpp"
#include "map.h"
#include <vector>
#include <unordered_map>

struct by_index_and_g{};


typedef multi_index_container<
        Node,
        indexed_by<
        ordered_non_unique<BOOST_MULTI_INDEX_MEMBER(Node , double, g)>,
        hashed_unique<BOOST_MULTI_INDEX_MEMBER(Node, int, id)>,
        ordered_non_unique<tag<by_index_and_g>,composite_key<Node, BOOST_MULTI_INDEX_MEMBER(Node,int,index),
        BOOST_MULTI_INDEX_MEMBER(Node,double ,g)>>
        >
> Open;


class heuristic {

private:
    std::vector<std::vector<double>> h_values;
    Open open;
    Node find_min();
    Node find_min_by_index(int target_index);
    double dist(Node&a,Node&b){return std::sqrt(pow(a.x-b.x,2)+pow(a.y-b.y,2));}
public:
    heuristic(){}
    void init(int size,int agents);
    void count(Map &map,Agent agent);
    double get_value(int id_node, int id_agent) {return h_values[id_node][id_agent];}

};


#endif //RAILWAY_NETWORK_CCBS_HEURISTIC_H
