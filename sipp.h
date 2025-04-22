#ifndef RAILWAY_NETWORK_CCBS_SIPP_H
#define RAILWAY_NETWORK_CCBS_SIPP_H

#include "struct.hpp"
#include "map.h"
#include "heuristic.h"

#include <unordered_map>
#include <map>
#include <set>

class SIPP {
public:
    SIPP() {}
    ~SIPP() {}
    Path find_path(Agent agent, Map &map, std::list<Constraint> cons, heuristic &h_values);   //constraint

private:
    Agent agent;
    void clear();
    Path pathPlan(Node start,Node goal, Map &map, heuristic &h_values, double agentSpeed, double max_f=CN_INFINITY);
    Node getNode(int index,int node_id, double node_x, double node_y, double t1, double t2);
    std::list<Node> open;
    std::unordered_map<int,Node> close;
    Path path;
    std::unordered_map<int, std::pair<double, bool>> visited;
    Node find_min();
    void add_open(Node newNode);
    void findSuccessors(Node curNode, Map map, std::list<Node> &succs, heuristic &h_values, Node goal, double agentSpeed);
    double dist(const Node& a, const Node& b);
    std::vector<Node> reconstructPath(Node curNode);
    void make_constraints(std::list<Constraint> &cons);
    void addCollisionInterval(int id, std::pair<double, double> interval);
    void addMoveConstraint(Move move);
    std::map<std::pair<int,int>,std::vector<Move>> constraints;
    std::unordered_map<int,std::vector<std::pair<double,double>>> collisionIntervals;
};


#endif //RAILWAY_NETWORK_CCBS_SIPP_H
