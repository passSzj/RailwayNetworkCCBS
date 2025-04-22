#ifndef RAILWAY_NETWORK_CCBS_CBS_H
#define RAILWAY_NETWORK_CCBS_CBS_H

#include "struct.hpp"
#include "map.h"
#include "task.h"
#include "heuristic.h"
#include "sipp.h"
#include <chrono>



class CBS {
public:
    CBS() {}
    Solution findSolution(Map &map,Task &task);
    bool initRoot(Map &map, Task &task);
    std::vector<Conflict> getAllConflicts(std::vector<solutionPath> &paths, int id);
    Conflict checkPaths(solutionPath &pathA, solutionPath &pathB);
    bool validateConstraints(std::list<Constraint> constraints, int agent);
    double getDist(solutionNode nodeA,solutionNode nodeB,double lengthA,double lengthB);
    bool conflictCheck(Move move1, Move move2);


    bool timeConflictCheck(Move move1, Move move2);
    std::pair<double, double> intersectCheck(Move move1, Move move2);
    double getDist(Vector2D a, Vector2D b);
    bool isSameEdge(Vector2D a, Vector2D b, Vector2D c, Vector2D d);
    bool isSameVertex(Vector2D a, Vector2D b);

    Conflict getConflict(std::list<Conflict> &conflicts);
    std::list<Constraint> getConstraints(CBS_Node *node, int agent_id);

    bool segmentsIntersect(Vector2D p1, Vector2D p2, Vector2D q1, Vector2D q2);
    bool detectCollision(Vector2D head1, Vector2D tail1, Vector2D velocity1,
                         Vector2D head2, Vector2D tail2, Vector2D velocity2,
                         double timeStart, double timeEnd);

    bool turnoutDetectCollision(bool m1Pre,bool m2Pre,Move move1,Move move2);  //道岔冲突检测


    bool isCollinearByDeternminant(double preX,double preY,double x1,double y1,double x2,double y2);

    Constraint getConstraint(int agent, Move move1, Move move2);
    double getCost(CBS_Node node, int agent_id);
    Constraint getWaitConstraint(int agent,Move move1,Move move2);
    std::vector<solutionPath> getPaths(CBS_Node *node, unsigned int agents_size);

    void findNewConflicts(const Map &map, const Task &task, CBS_Node &node, std::vector<solutionPath> &paths, const solutionPath &path,
                                 const std::list<Conflict> &conflicts, int &low_level_searches, int &low_level_expanded);


    Solution solution;

    Map *map;
    heuristic h_values;
    CBSTree tree;
    SIPP planner;

};


#endif //RAILWAY_NETWORK_CCBS_CBS_H
