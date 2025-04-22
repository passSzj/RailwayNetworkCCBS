#ifndef RAILWAY_NETWORK_CCBS_STRUCT_H
#define RAILWAY_NETWORK_CCBS_STRUCT_H
#include <boost/multi_index_container.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/hashed_index.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/composite_key.hpp>
#include <vector>
#include "const.h"
#include <list>
#include <set>
#include <chrono>
#include <iostream>

using boost::multi_index_container;
using namespace boost::multi_index;

struct Agent {
    double start_x,start_y,goal_x,goal_y;
    int start_id,goal_id,s_index,g_index;
    int id;
    double speed;
    double length;
    Agent(int id=-1,int speed=-1,double length=5,int startID=-1,int goalID=-1,int sindex=-1,int gindex=-1):
    start_id(startID),goal_id(goalID),s_index(sindex),g_index(gindex),id(id), speed(speed),length(length){}
};



struct Node{
    int index;
    int id;
    double f,g,x,y;
    Node* parent;
    std::pair<double,double> interval;
    int interval_id;
    Node(int index=-1,int id=-1, double f=-1, double g=-1, double x=-1, double y=-1, Node* parent=nullptr, double begin=-1, double end=-1)
        : index(index),id(id), f(f), g(g), x(x), y(y), parent(parent), interval(std::make_pair(begin,end)) {interval_id=0;};

    void ConvertNode(){if(this->index==0) this->index=1; else if(this->index==1) this->index=0;}
};


struct solutionNode
{
    int id;
    double g;
    int index;
    solutionNode(int id_=-1, double g_=-1,int index=-1):id(id_),g(g_),index(index){}
    solutionNode(const Node &n)
    {
        index=n.index;
        id = n.id;
        g = n.g;
    }
};




struct subVertex{

    int index;
    double x;
    double y;  
    std::vector<int> neighbors;
    subVertex(int index=-1,double x=-1, double y=-1):index(index),x(x), y(y) {}
    ~subVertex()  {neighbors.clear();}
};



struct Vertex{
    std::vector<subVertex> subVertices;
    Vertex(){}
    Vertex(std::vector<subVertex> subVertices): subVertices(subVertices) {}
    ~Vertex() {subVertices.clear();}
};


struct Path{
    std::vector<Node> nodes;
    double cost;
    int agentID;
    int expanded;
    double agentLength;
    double agentSpeed;
    Path(std::vector<Node> nodes= std::vector<Node>(0), double cost=-1,int agentID=-1)
        : nodes(nodes), cost(cost), agentID(agentID) {expanded=0;agentLength=-1;agentSpeed=-1;}
};

struct solutionPath{

    std::vector<solutionNode> nodes;
    double cost;
    double agentLength;
    double agentSpeed;
    int agentID;
    int expanded;
    solutionPath(std::vector<solutionNode> _nodes = std::vector<solutionNode>(0), double _cost = -1, int _agentID = -1)
            : nodes(_nodes), cost(_cost), agentID(_agentID) {expanded = 0; agentLength=-1;agentSpeed=-1;}
    solutionPath operator= (const Path &path)
    {
        cost = path.cost;
        agentID = path.agentID;
        expanded = path.expanded;
        agentLength=path.agentLength;
        agentSpeed=path.agentSpeed;
        nodes.clear();
        for(auto n:path.nodes)
            nodes.push_back(solutionNode(n));
        return *this;
    }
};

struct Constraint{
    int agent;
    double t1,t2;
    int id1,id2;
    bool positive;
    Constraint(int agent=-1,double t1=-1,double t2=-1,int id1=-1,int id2=-1)
            :agent(agent),t1(t1),t2(t2),id1(id1),id2(id2) {}

};


struct Move{
    int index;
    double t1,t2;
    int id1,id2;
    int preId;
    double length;
    double speed;
    Move(int index=-1, double length=-1,double speed=-1, double t1 = -1, double t2 = -1, int id1 = -1, int id2 = -1)
            : index(index),t1(t1), t2(t2), id1(id1), id2(id2),preId(-1),length(length),speed(speed){}
    Move(solutionNode a, solutionNode b,double length,double speed): preId(-1),t1(a.g), t2(b.g), id1(a.id), id2(b.id),index(a.index),length(length),speed(speed){}
    Move(solutionNode pre, solutionNode a, solutionNode b,double length,double speed): preId(pre.id),t1(a.g), t2(b.g), id1(a.id), id2(b.id),index(a.index),length(length),speed(speed){}
    Move(Constraint& con): t1(con.t1), t2(con.t2), id1(con.id1), id2(con.id2) {}
};


struct Solution{
    bool found;
    double flowtime;
    double makespan;
    double initCost;
    int high_level_expanded;
    int low_level_expanded;
    std::chrono::duration<double> initTime;
    std::vector<solutionPath> paths;
    std::chrono::duration<double> time;
    Solution(double _flowtime = -1, double _makespan = -1, std::vector<solutionPath> _paths = {})
            : flowtime(_flowtime), makespan(_makespan), paths(_paths) { initCost = -1; low_level_expanded = 0;}
    ~Solution() { paths.clear(); found = false;}

};

struct Conflict{
    int agent1,agent2;
    double t;
    Move move1,move2;
    double overcost;
    Conflict(int agent1=-1,int agent2=-1,Move move1= Move(),Move move2=Move(),double t= CN_INFINITY):
        agent1(agent1),agent2(agent2),t(t),move1(move1),move2(move2){overcost=0;}
};




struct CBS_Node{
    std::vector<solutionPath> paths;
    CBS_Node *parent;
    Constraint constraint;
    int id;
    double cost;
    double h;
    int conflictsNum;
    int totalCons;
    int lowLevelExpanded;
    std::list<Conflict> conflicts;
    std::string idStr;

    CBS_Node(std::vector<solutionPath> paths={},CBS_Node* parent=nullptr, Constraint constraint = Constraint(),
             double cost=0, int conflictsNum=0,int totalCons=0) : paths(paths),parent(parent),constraint(constraint),cost(cost)
             ,conflictsNum(conflictsNum),totalCons(totalCons)
             {
                lowLevelExpanded=0;
                h=0;
                conflicts.clear();
            }

    ~CBS_Node()
    {
        parent = nullptr;
        paths.clear();
        conflicts.clear();
    }

};


class Vector2D {
public:
    Vector2D(double _i = 0.0, double _j = 0.0):x(_i),y(_j){}
    double x, y;
    inline Vector2D operator +(const Vector2D &vec) { return Vector2D(x + vec.x, y + vec.y); }
    inline Vector2D operator -(const Vector2D &vec) { return Vector2D(x - vec.x, y - vec.y); }
    inline Vector2D operator -() { return Vector2D(-x,-y); }
    inline Vector2D operator /(const double &num) { return Vector2D(x/num, y/num); }

    inline Vector2D operator *(const double &num) {
        return Vector2D(x*num, y*num);
    }

    inline double operator *(const Vector2D &vec){ return x*vec.x + y*vec.y; }
    inline void operator +=(const Vector2D &vec) { x += vec.x; y += vec.y; }
    inline void operator -=(const Vector2D &vec) { x -= vec.x; y -= vec.y; }



    inline Vector2D normalized() const {
        double length = sqrt(x * x + y * y);
        // 防止除以0
        if(length < CN_EPSILON) {
            return Vector2D(0, 0);
        }
        return Vector2D(x / length, y / length);
    }

    // 点积运算
    double dot(const Vector2D& other) const {
        return x * other.x + y * other.y;
    }

    inline double cross(const Vector2D& other) const {
        return x * other.y - y * other.x;
    }


    Vector2D round(){
        double value = 1e-8;
        if (std::abs(x - std::round(x)) < value) {
            x = std::round(x);
        }
        if (std::abs(y - std::round(y)) < value) {
            y = std::round(y);
        }

        return Vector2D(x, y);
    }


};











struct Open_Elem
{
    CBS_Node* tree_pointer;
    int id;
    double cost;
    unsigned int cons_num;
    unsigned int conflicts_num;

    Open_Elem(CBS_Node* _tree_pointer = nullptr, int _id = -1, double _cost = -1, unsigned int _cons_num = 0, unsigned int _conflicts_num = 0)
            : tree_pointer(_tree_pointer), id(_id), cost(_cost), cons_num(_cons_num), conflicts_num(_conflicts_num) {}
    ~Open_Elem()
    {
        tree_pointer = nullptr;
    }
};

struct id{};
typedef multi_index_container<
        Open_Elem,
        indexed_by<
                ordered_non_unique<composite_key<Open_Elem,
                        BOOST_MULTI_INDEX_MEMBER(Open_Elem, double, cost),
                        BOOST_MULTI_INDEX_MEMBER(Open_Elem, unsigned int, conflicts_num),
                        BOOST_MULTI_INDEX_MEMBER(Open_Elem, unsigned int, cons_num)>,
                        composite_key_compare<std::less<double>, std::less<int>, std::greater<int>>>,
                hashed_unique<tag<id>, BOOST_MULTI_INDEX_MEMBER(Open_Elem, int, id)>
        >
> CT_container;

struct Focal_Elem
{
    int id;
    unsigned int conflicts_num;
    unsigned int constraints;
    double cost;
    Focal_Elem(int id_=-1, unsigned int conflicts_num_ = 0, unsigned int constraints_ = 0, double cost_ = 0):id(id_), conflicts_num(conflicts_num_), constraints(constraints_), cost(cost_){}
    bool operator <(const Focal_Elem& other) const
    {
        if(this->conflicts_num < other.conflicts_num)
            return true;
        else if(this->conflicts_num > other.conflicts_num)
            return false;
        else if(this->constraints > other.constraints)
            return true;
        else if(this->constraints < other.constraints)
            return false;
        else if(this->cost < other.cost)
            return true;
        else
            return false;
    }
};


struct conflicts_num{};
typedef multi_index_container<
        Focal_Elem,
        indexed_by<

                ordered_non_unique<tag<conflicts_num>, identity<Focal_Elem>>,

                hashed_unique<tag<id>, BOOST_MULTI_INDEX_MEMBER(Focal_Elem, int, id)>
        >
> Focal_container;



class CBSTree{
    std::list<CBS_Node> tree;
    CT_container container;
    Focal_container focal;
    double focal_weight;
    int open_size;
    std::set<int> closed;
public:
    CBSTree(){open_size=0;focal_weight=1.0;}
    int getSize()   {return tree.size();}
    void setFocalWeight(double weight) {focal_weight=weight;}
    int getOpenSize() {return open_size;}
    void addNode(CBS_Node node){
        tree.push_back(node);
        container.insert(Open_Elem(&tree.back(),node.id,node.cost,node.totalCons,node.conflictsNum));
        open_size++;
        if(focal_weight>1.0)
            if(container.get<0>().begin()->cost*focal_weight>node.cost)
                focal.insert(Focal_Elem(node.id,node.conflictsNum,node.totalCons,node.cost));
    }

    CBS_Node* getFront(){
        open_size--;
        if(focal_weight>1.0){
            double cost=container.get<0>().begin()->cost;
            if(focal.empty())
                updateFocal(cost);
            auto min=container.get<1>().find(focal.get<0>().begin()->id);
            focal.get<0>().erase(focal.get<0>().begin());
            auto pointer=min->tree_pointer;
            container.get<1>().erase(min);
            if(container.get<0>().begin()->cost>cost+CN_EPSILON)
                updateFocal(cost);
            return pointer;
        }
        else{
            auto pointer=container.get<0>().begin()->tree_pointer;
            container.get<0>().erase(container.get<0>().begin());
            return pointer;
        }
    }

    void updateFocal(double cost){
        auto it0=container.get<0>().begin();
        auto it1=container.get<0>().upper_bound(cost*focal_weight+CN_EPSILON);
        for(auto it=it0;it!=it1;it++)
            focal.insert(Focal_Elem(it->id,it->conflicts_num,it->cons_num,it->cost));
    }

    std::vector<solutionPath> getPaths(CBS_Node node,int size){
        std::vector<solutionPath> paths(size);
        while(node.parent!= nullptr){
            if(paths.at(node.paths.begin()->agentID).nodes.empty())
                paths.at(node.paths.begin()->agentID)=*node.paths.begin();
            node=*node.parent;
        }
        for(unsigned int i=0;i<node.paths.size();i++)
            if(paths.at(i).nodes.empty())
                paths.at(i)=node.paths.at(i);
        return paths;
    }

    void printContainer() {
        std::cout << "Container contents (index 0 - sorted by cost, conflicts_num, cons_num):" << std::endl;
        for (const auto& elem : container.get<0>()) {
            std::cout << "id: " << elem.id
                      << ", cost: " << elem.cost
                      << ", conflicts_num: " << elem.conflicts_num
                      << ", cons_num: " << elem.cons_num
                      << std::endl;
        }
    }

};


class Point {
public:
    double i;
    double j;

    Point(double _i = 0.0, double _j = 0.0):i (_i), j (_j){}
    Point operator-(Point &p){return Point(i - p.i, j - p.j);}
    int operator== (Point &p){return (i == p.i) && (j == p.j);}
    int   classify(Point &pO, Point &p1)
    {
        Point p2 = *this;
        Point a = p1 - pO;
        Point b = p2 - pO;
        double sa = a.i * b.j - b.i * a.j;
        if (sa > 0.0)
            return 1;
        if (sa < 0.0)
            return 2;
        if ((a.i * b.i < 0.0) || (a.j * b.j < 0.0))
            return 3;
        if ((a.i*a.i + a.j*a.j) < (b.i*b.i + b.j*b.j))
            return 4;
        if (pO == p2)
            return 5;
        if (p1 == p2)
            return 6;
        return 7;
    }
};





#endif //RAILWAY_NETWORK_CCBS_STRUCT_H
