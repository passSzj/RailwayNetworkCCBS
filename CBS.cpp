#include "CBS.h"

Solution CBS::findSolution(Map &map,Task &task){
    this->map=&map;
    h_values.init(map.getMapSize(),task.getAgentSize());

    for(int i=0;i<int(task.getAgentSize());i++){
        Agent agent =task.getAgent(i);
        h_values.count(map,agent);
    }


    auto t = std::chrono::high_resolution_clock::now();
    if(!(this->initRoot(map,task)))
        return solution;
    //solution
    solution.initTime=std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now()-t);
//    solution.found=true;
    CBS_Node node;
    std::chrono::duration<double> time_spent;
    int expanded=1;
    double time=0;
    std::list<Conflict> conflicts;
    Conflict conflict;
    std::vector<int> conflictingAgents;
    std::vector<std::pair<int,int>> conflictingPairs;
    int lowLevelSearches=0;
    int lowLevelExpanded=0;
    int id=2;

    do{
        auto parent = tree.getFront();
        node=*parent;
                node.cost-=node.h;
        parent->conflicts.clear();
        auto paths = getPaths(&node,task.getAgentSize());
        auto time_now = std::chrono::high_resolution_clock::now();
        conflicts=node.conflicts;
        if(conflicts.empty()){
            solution.found=true;
            break;
        }
        conflict = getConflict(conflicts);
        auto time_spent = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now()-time_now);
        time+=time_spent.count();
        expanded++;

        std::list<Constraint> constraintsA = getConstraints(&node,conflict.agent1);
        Constraint constraintA = getConstraint(conflict.agent1,conflict.move1,conflict.move2);
        constraintsA.push_back(constraintA);
        solutionPath pathA;
        pathA = planner.find_path(task.getAgent(conflict.agent1),map,constraintsA,h_values);
        lowLevelSearches++;
        lowLevelExpanded+=pathA.expanded;

        std::list<Constraint> constraintsB = getConstraints(&node,conflict.agent2);
        Constraint constraintB = getConstraint(conflict.agent2,conflict.move2,conflict.move1);
        constraintsB.push_back(constraintB);
        solutionPath pathB;
        pathB = planner.find_path(task.getAgent(conflict.agent2),map,constraintsB,h_values);
        lowLevelSearches++;
        lowLevelExpanded+=pathB.expanded;

        CBS_Node right({pathA},parent,constraintA,node.cost+pathA.cost-getCost(node,conflict.agent1),0,node.totalCons+1);
        CBS_Node left({pathB},parent,constraintB,node.cost+pathB.cost-getCost(node,conflict.agent2),0,node.totalCons+1);
        Constraint positive;
        bool left_ok=true,right_ok=true;

        right.idStr=node.idStr+"0";
        left.idStr=node.idStr+"1";
        right.id = id++;
        left.id = id++;

        if(right_ok && pathA.cost > 0 && validateConstraints(constraintsA, pathA.agentID))
        {
            time_now = std::chrono::high_resolution_clock::now();
            findNewConflicts(map, task, right, paths, pathA, conflicts, lowLevelSearches, lowLevelExpanded);
            time_spent = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - time_now);
            time += time_spent.count();
            if(right.cost > 0)
            {
                tree.addNode(right);
            }
            else{
                std::cout<<"right cost < 0"<<std::endl;
            }
        }

        if(left_ok && pathB.cost > 0 && validateConstraints(constraintsB, pathB.agentID))
        {
            time_now = std::chrono::high_resolution_clock::now();
            findNewConflicts(map, task, left, paths, pathB, conflicts, lowLevelSearches, lowLevelExpanded);
            time_spent = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - time_now);
            time += time_spent.count();
            if(left.cost > 0)
            {
                tree.addNode(left);

            }
            else{
                std::cout<<"left cost < 0"<<std::endl;
            }
        }

        time_spent = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t);


        if(right.conflicts.empty()||left.conflicts.empty()){

            if(right.conflicts.empty()){
                solution.found=true;
                solution.paths = getPaths(&right,task.getAgentSize());
                solution.flowtime = right.cost;
                solution.low_level_expanded = double(lowLevelExpanded)/std::max(lowLevelSearches,1);
                solution.time = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t);
                solution.high_level_expanded = expanded;
                for(auto path:solution.paths)
                    solution.makespan = (solution.makespan > path.cost) ? solution.makespan : path.cost;
                return solution;
            }

            else if(left.conflicts.empty()){
                solution.found=true;
                solution.paths = getPaths(&left,task.getAgentSize());
                solution.flowtime = left.cost;
                solution.low_level_expanded = double(lowLevelExpanded)/std::max(lowLevelSearches,1);
                solution.time = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t);
                solution.high_level_expanded = expanded;
                for(auto path:solution.paths)
                    solution.makespan = (solution.makespan > path.cost) ? solution.makespan : path.cost;
                return solution;
            }
        }

    }while(tree.getOpenSize()>0);
    solution.paths = getPaths(&node,task.getAgentSize());
    solution.flowtime = node.cost;
    solution.low_level_expanded = double(lowLevelExpanded)/std::max(lowLevelSearches,1);
    solution.time = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t);
    solution.high_level_expanded = expanded;
    for(auto path:solution.paths)
        solution.makespan = (solution.makespan > path.cost) ? solution.makespan : path.cost;
    return solution;

}



Conflict CBS::getConflict(std::list<Conflict> &conflicts){
    auto best_it = conflicts.begin();
    for(auto it = conflicts.begin();it!=conflicts.end();it++){
        if(it->overcost>0){
            if(best_it->overcost<it->overcost||(fabs(best_it->overcost-it->overcost)<CN_EPSILON&&best_it->t<it->t)){
                best_it=it;
            }
        }
        else if(best_it->t<it->t){
            best_it=it;
        }
    }

    Conflict conflict = *best_it;
    conflicts.erase(best_it);
    return conflict;
}


// 获取指定节点及其父节点的约束列表
std::list<Constraint> CBS::getConstraints(CBS_Node *node, int agent_id)
{
    CBS_Node* curNode = node;
    std::list<Constraint> constraints(0);


    while(curNode->parent != nullptr)
    {

        if(agent_id < 0 || curNode->constraint.agent == agent_id)
            constraints.push_back(curNode->constraint);
        curNode = curNode->parent;
    }


    return constraints;
}


bool CBS::validateConstraints(std::list<Constraint> constraints, int agent)
{

    std::list<Constraint> positives;
    for(auto c: constraints)
        if(c.positive && c.agent == agent)
            positives.push_back(c);


    for(auto p: positives)
        for(auto c: constraints)
        {
            if(c.positive)
                continue;
            if(p.agent == c.agent && p.id1 == c.id1 && p.id2 == c.id2)
                if(p.t1 > c.t1 - CN_EPSILON && p.t2 < c.t2 + CN_EPSILON)
                    return false;
        }
    return true;
}

void CBS::findNewConflicts(const Map &map, const Task &task, CBS_Node &node, std::vector<solutionPath> &paths, const solutionPath &path,
                      const std::list<Conflict> &conflicts, int &low_level_searches, int &low_level_expanded){
    auto oldpath = paths[path.agentID];
    paths[path.agentID] = path;
    auto new_conflicts = getAllConflicts(paths, path.agentID);
    paths[path.agentID] = oldpath;
    std::list<Conflict> conflictsA({});
    for(auto c: conflicts)
        if(c.agent1 != path.agentID && c.agent2 != path.agentID)
            conflictsA.push_back(c);
    node.conflicts = conflictsA;
    for(auto n:new_conflicts)
        node.conflicts.push_back(n);
    node.conflictsNum = node.conflicts.size();
    return;

}


bool CBS::initRoot(Map &map, Task &task){
    CBS_Node root;
    solutionPath path;

    for (int i=0; i<int(task.getAgentSize());i++){
        Agent agent = task.getAgent(i);
        path = planner.find_path(agent,map,{},h_values);
        if(path.cost<0){
            return false;
        }
        root.paths.push_back(path);
        root.cost+=path.cost;
    }

    root.lowLevelExpanded=0;
    root.parent= nullptr;
    root.id=1;
    root.idStr="1";
    auto conflicts = getAllConflicts(root.paths,-1);
    root.conflictsNum=conflicts.size();

    bool useCardinal = false;

    for(auto conflict:conflicts){
        if(!useCardinal){
            root.conflicts.push_back(conflict);
        }
        else {
            auto pathA = planner.find_path(task.getAgent(conflict.agent1), map,
                                           {getConstraint(conflict.agent1, conflict.move1, conflict.move2)}, h_values);
            auto pathB = planner.find_path(task.getAgent(conflict.agent2), map,
                                           {getConstraint(conflict.agent2, conflict.move2, conflict.move1)}, h_values);
        }
    }

    solution.initCost=root.cost;
    tree.addNode(root);
    return true;
}


std::vector<Conflict> CBS::getAllConflicts(std::vector<solutionPath> &paths, int id){
    std::vector<Conflict> conflicts;
    if(id<0){
        for(int i=0;i<paths.size();i++){
            for(int j=i+1;j<paths.size();j++){
                Conflict conflict = checkPaths(paths[i],paths[j]);
                if(conflict.agent1>=0){
                    conflicts.push_back(conflict);
                }
            }
        }
    }
    else{
        for(int i=0;i<paths.size();i++){
            if(i==id)
                continue;
            Conflict conflict = checkPaths(paths[i],paths[id]);
            if(conflict.agent1>=0)
                conflicts.push_back(conflict);
        }
    }
    return conflicts;
}


Conflict CBS::checkPaths(solutionPath &pathA, solutionPath &pathB){
    int a=0,b=0;
    auto nodesA = pathA.nodes;
    auto nodesB = pathB.nodes;
    double lengthA=pathA.agentLength;
    double lengthB=pathB.agentLength;
    double speedA=pathA.agentSpeed;
    double speedB=pathB.agentSpeed;

    while(a<nodesA.size()-1 || b<nodesB.size()-1){
        double dist= getDist(nodesA[a],nodesB[b],lengthA,lengthB);
        if(a<nodesA.size()-1 && b<nodesB.size()-1){
            if(dist<((nodesA[a+1].g-nodesA[a].g)*speedA+(nodesB[b+1].g-nodesB[b].g)*speedB)){

                if(a==0&&b!=0){
                    if(conflictCheck(Move(nodesA[a],nodesA[a+1],lengthA,speedA),Move(nodesB[b-1],nodesB[b],nodesB[b+1],lengthB,speedB))){
                        return Conflict(pathA.agentID,pathB.agentID,Move(nodesA[a],nodesA[a+1],lengthA,speedA),
                                        Move(nodesB[b-1],nodesB[b],nodesB[b+1],lengthB,speedB),std::min(nodesA[a].g,nodesB[b].g));
                    }
                }

                else if(b==0&&a!=0){
                    if(conflictCheck(Move(nodesA[a-1],nodesA[a],nodesA[a+1],lengthA,speedA),Move(nodesB[b],nodesB[b+1],lengthB,speedB))){
                        return Conflict(pathA.agentID,pathB.agentID,Move(nodesA[a-1],nodesA[a],nodesA[a+1],lengthA,speedA),
                                        Move(nodesB[b],nodesB[b+1],lengthB,speedB),std::min(nodesA[a].g,nodesB[b].g));
                    }
                }

                else if(a!=0&&b!=0){
                    if(conflictCheck(Move(nodesA[a-1],nodesA[a],nodesA[a+1],lengthA,speedA),Move(nodesB[b-1],nodesB[b],nodesB[b+1],lengthB,speedB))){
                        return Conflict(pathA.agentID,pathB.agentID,Move(nodesA[a-1],nodesA[a],nodesA[a+1],lengthA,speedA),
                                        Move(nodesB[b-1],nodesB[b],nodesB[b+1],lengthB,speedB),std::min(nodesA[a].g,nodesB[b].g));
                    }
                }

                else if(a==0&&b==0){
                    if(conflictCheck(Move(nodesA[a],nodesA[a+1],lengthA,speedA), Move(nodesB[b],nodesB[b+1],lengthB,speedB))){
                        return Conflict(pathA.agentID,pathB.agentID,Move(nodesA[a],nodesA[a+1],lengthA,speedA),
                                        Move(nodesB[b],nodesB[b+1],lengthB,speedB),std::min(nodesA[a].g,nodesB[b].g));
                    }
                }




            }
        }
        if(a==nodesA.size()-1)
            b++;
        else if(b==nodesB.size()-1)
            a++;
        else if(fabs(nodesA[a+1].g-nodesB[b+1].g)<Total_PRECISION){
            a++;
            b++;
        }
        else if(nodesA[a+1].g<nodesB[b+1].g)
            a++;
        else if(nodesB[b+1].g-CN_EPSILON<nodesA[a+1].g)
            b++;
    }
    return Conflict();
}

double CBS::getDist(solutionNode nodeA,solutionNode nodeB,double lengthA,double lengthB){

    double aX=map->getX(nodeA.id);
    double aY=map->getY(nodeA.id);
    double bX=map->getX(nodeB.id);
    double bY=map->getY(nodeB.id);


    if(nodeA.index==1&&nodeB.index==0){
        return sqrt(pow(aX-bX,2)+ pow(aY-bY,2));
    }

    if(nodeA.index==0&&nodeB.index==1){
        return sqrt(pow(aX-bX,2)+ pow(aY-bY,2));
    }

    if(nodeA.index==0&&nodeB.index==0){
        if(aX>=bX){
            return sqrt(pow(aX-bX,2)+ pow(aY-bY,2))-lengthA;
        }
        if(bX>=aX){
            return sqrt(pow(aX-bX,2)+ pow(aY-bY,2))-lengthB;
        }
    }

    if(nodeA.index==1&&nodeB.index==1){
        if(aX>=bX){
            return sqrt(pow(aX-bX,2)+ pow(aY-bY,2))-lengthA;
        }
        if(bX>=aX){
            return sqrt(pow(aX-bX,2)+ pow(aY-bY,2))-lengthB;
        }
    }

}



bool CBS::conflictCheck(Move move1, Move move2) {

    bool isCollision=false;
    bool m1Pre=true,m2Pre=true;


    double startTimeA(move1.t1),endTimeA(move1.t2),startTimeB(move2.t1),endTimeB(move2.t2);
    double m1PreX(map->getX(move1.preId)),m1PreY(map->getY(move1.preId));
    double m2PreX(map->getX(move2.preId)),m2PreY(map->getY(move2.preId));
    double m1x1(map->getX(move1.id1)), m1x2(map->getX(move1.id2)), m1y1(map->getY(move1.id1)), m1y2(map->getY(move1.id2));
    double m2x1(map->getX(move2.id1)), m2x2(map->getX(move2.id2)), m2y1(map->getY(move2.id1)), m2y2(map->getY(move2.id2));



    Vector2D VA((m1x2-m1x1)/(move1.t2-move1.t1),(m1y2-m1y1)/(move1.t2-move1.t1));
    Vector2D VB((m2x2-m2x1)/(move2.t2-move2.t1),(m2y2-m2y1)/(move2.t2-move2.t1));

    Vector2D reverseA = VA.normalized() ;
    Vector2D reverseB = VB.normalized() ;

    //除了起点，列车的位置表示为车头位于顶点
    //车头位置
    Vector2D HeadA(m1x1,m1y1);
    Vector2D HeadB(m2x1,m2y1);


    Vector2D TailA = HeadA - reverseA * move1.length;
    Vector2D TailB = HeadB - reverseB * move2.length;



    if(move1.preId==-1)  //preId==-1即在起点
    {
        Vector2D temp(m1x1,m1y1);
        TailA=temp;
        HeadA = TailA + reverseA * move1.length;
    }

    if(move2.preId==-1)
    {
        Vector2D temp(m2x1,m2y1);
        TailB=temp;
        HeadB = TailB + reverseB * move2.length;
    }


    //quickCheck 时间区间判断
   if(timeConflictCheck(move1,move2)){
       return true;
    }




    if(move1.preId!=-1){
        m1Pre=isCollinearByDeternminant(m1PreX,m1PreY,m1x1,m1y1,m1x2,m1y2);
    }
    if(move2.preId!=-1){
        m2Pre=isCollinearByDeternminant(m2PreX,m2PreY,m2x1,m2y1,m2x2,m2y2);
    }

    if(!m1Pre||!m2Pre){
        turnoutDetectCollision(m1Pre,m2Pre,move1,move2);
    }



    double turnoutTime1=move1.length/move1.speed;
    double turnoutTime2=move2.length/move2.speed;


    if(startTimeB>startTimeA){
        HeadA+=VA*(startTimeB-startTimeA);
        TailA+=VA*(startTimeB-startTimeA);
        startTimeA=startTimeB;
    }
    else if(startTimeA>startTimeB){
        HeadB+=VB*(startTimeA-startTimeB);
        TailB+=VB*(startTimeA-startTimeB);
        startTimeB=startTimeA;
    }

    isCollision=detectCollision(HeadA,TailA,VA,HeadB,TailB,VB,startTimeA,std::min(endTimeA,endTimeB));

    return isCollision;


}




bool CBS::segmentsIntersect(Vector2D p1, Vector2D p2, Vector2D q1, Vector2D q2) {
    Vector2D r = p2 - p1;
    Vector2D s = q2 - q1;
    Vector2D qp = q1 - p1;

    double rxs = r.cross(s);
    double t = qp.cross(s) / rxs;
    double u = qp.cross(r) / rxs;



    if (std::abs(rxs) < 1e-6) {

        double p_min_x = std::min(p1.x, p2.x);
        double p_max_x = std::max(p1.x, p2.x);
        double p_min_y = std::min(p1.y, p2.y);
        double p_max_y = std::max(p1.y, p2.y);

        double q_min_x = std::min(q1.x, q2.x);
        double q_max_x = std::max(q1.x, q2.x);
        double q_min_y = std::min(q1.y, q2.y);
        double q_max_y = std::max(q1.y, q2.y);

        bool overlap_x = (p_max_x >= q_min_x && p_min_x <= q_max_x);
        bool overlap_y = (p_max_y >= q_min_y && p_min_y <= q_max_y);

        if (overlap_x && overlap_y) {
            return true;
        } else {
            return false;
        }
    }



    return (t >= 0 && t <= 1) && (u >= 0 && u <= 1);
}



bool CBS::detectCollision(Vector2D head1, Vector2D tail1, Vector2D velocity1,
                     Vector2D head2, Vector2D tail2, Vector2D velocity2,
                     double timeStart, double timeEnd) {
    Vector2D head1End = (head1 + velocity1 * (timeEnd-timeStart)).round();
    Vector2D tail1End = (tail1 + velocity1 * (timeEnd-timeStart)).round();
    Vector2D head2End = (head2 + velocity2 * (timeEnd-timeStart)).round();
    Vector2D tail2End = (tail2 + velocity2 * (timeEnd-timeStart)).round();


    if (segmentsIntersect(head1, head1End, head2, head2End) ||
        segmentsIntersect(head1, head1End, tail2, tail2End) ||
        segmentsIntersect(tail1, tail1End, head2, head2End) ||
        segmentsIntersect(tail1, tail1End, tail2, tail2End)) {
        return true;
    }

    return false;
}



bool CBS::turnoutDetectCollision(bool m1Pre,bool m2Pre,Move move1,Move move2) {
    double startTimeA(move1.t1), endTimeA(move1.t2), startTimeB(move2.t1), endTimeB(move2.t2);
    double m1x1(map->getX(move1.id1)), m1x2(map->getX(move1.id2)), m1y1(map->getY(move1.id1)), m1y2(
            map->getY(move1.id2));
    double m2x1(map->getX(move2.id1)), m2x2(map->getX(move2.id2)), m2y1(map->getY(move2.id1)), m2y2(
            map->getY(move2.id2));


    Vector2D VA((m1x2 - m1x1) / (move1.t2 - move1.t1), (m1y2 - m1y1) / (move1.t2 - move1.t1));
    Vector2D VB((m2x2 - m2x1) / (move2.t2 - move2.t1), (m2y2 - m2y1) / (move2.t2 - move2.t1));

    Vector2D reverseA = VA.normalized() ;
    Vector2D reverseB = VB.normalized() ;


    Vector2D HeadA(m1x1, m1y1);
    Vector2D HeadB(m2x1, m2y1);

    if(move1.preId==-1)
    {
        HeadA = HeadA + reverseA * move1.length;
    }

    if(move2.preId==-1)
    {
        HeadB = HeadB + reverseB * move2.length;
    }


    if (startTimeB > startTimeA) {
        HeadA += VA * (startTimeB - startTimeA);
        startTimeA = startTimeB;
    } else if (startTimeA > startTimeB) {
        HeadB += VB * (startTimeA - startTimeB);
        startTimeB = startTimeA;
    }


    if (!m1Pre && !m2Pre) {
        double m1PreX(map->getX(move1.preId)), m1PreY(map->getY(move1.preId));
        double m2PreX(map->getX(move2.preId)), m2PreY(map->getY(move2.preId));
        double turnoutTime1 = move1.length / move1.speed;
        double turnoutTime2 = move2.length / move2.speed;
        double timeInterval1 = startTimeA - move1.t1;
        double timeInterval2 = startTimeB - move2.t1;

        bool toTimeEnd = false;

        if (timeInterval1 >= turnoutTime1) {

            Vector2D V1((m2PreX - m2x1), (m2PreY - m2y1));
            Vector2D revsV1 = V1.normalized();
            Vector2D obsTail1 = HeadB + revsV1 * move2.length;
            Vector2D obsStart1 = HeadB;


            Vector2D V2((m2x2 - m2x1), (m2y2 - m2y1));
            Vector2D revsV2 = V2.normalized();
            Vector2D obsTail2 = HeadB + revsV2 * move2.length;
            Vector2D obsStart2 = HeadB;

            Vector2D reverseA = VA.normalized();
            Vector2D TailA = HeadA - reverseA * move1.length;
            Vector2D headAEnd = HeadA + VA * turnoutTime2;
            Vector2D tailAEnd = TailA + VA * turnoutTime2;

            if (segmentsIntersect(HeadA, headAEnd, obsStart1, obsTail1) ||
                segmentsIntersect(TailA, tailAEnd, obsStart1, obsTail1) ||
                segmentsIntersect(HeadA, headAEnd, obsStart2, obsTail2) ||
                segmentsIntersect(TailA, tailAEnd, obsStart2, obsTail2)
                    ) {
                return true;
            }

            return false;
        }
        else if (timeInterval2 >= turnoutTime2) {
            Vector2D V1((m1PreX - m1x1), (m1PreY - m1y1));
            Vector2D revsV1 = V1.normalized();
            Vector2D obsTail1 = HeadA + revsV1 * move1.length;
            Vector2D obsStart1 = HeadA;


            Vector2D V2((m1x2 - m1x1), (m1y2 - m1y1));
            Vector2D revsV2 = V2.normalized();
            Vector2D obsTail2 = HeadA + revsV2 * move1.length;
            Vector2D obsStart2 = HeadA;


            Vector2D reverseB = VB.normalized();
            Vector2D TailB = HeadB - reverseB * move2.length;
            Vector2D headBEnd = HeadB + VB * turnoutTime1;
            Vector2D tailBEnd = TailB + VB * turnoutTime1;

            if (segmentsIntersect(HeadB, headBEnd, obsStart1, obsTail1) ||
                segmentsIntersect(TailB, tailBEnd, obsStart1, obsTail1) ||
                segmentsIntersect(HeadB, headBEnd, obsStart2, obsTail2) ||
                segmentsIntersect(TailB, tailBEnd, obsStart2, obsTail2)
                    ) {
                return true;
            }

            return false;
        }
        else {
            Vector2D reverseA = VA.normalized();
            Vector2D TailA = HeadA - reverseA * move1.length;
            Vector2D headAEnd = HeadA + VA * turnoutTime2;
            Vector2D tailAEnd = TailA + VA * turnoutTime2;

            Vector2D reverseB = VB.normalized();
            Vector2D TailB = HeadB - reverseB * move2.length;
            Vector2D headBEnd = HeadB + VB * turnoutTime1;
            Vector2D tailBEnd = TailB + VB * turnoutTime1;

            if (segmentsIntersect(HeadA, headAEnd, HeadB, headBEnd) ||
                segmentsIntersect(HeadA, headAEnd, TailB, tailBEnd) ||
                segmentsIntersect(TailA, tailAEnd, HeadB, headBEnd) ||
                segmentsIntersect(TailA, tailAEnd, TailB, tailBEnd)
                    ) {
                toTimeEnd = true;
                return true;
            }
            return false;
        }
        if(!toTimeEnd){

            Vector2D reverseA = VA.normalized();
            Vector2D TailA = HeadA - reverseA * move1.length;
            Vector2D headAEnd = HeadA + VA * turnoutTime2;
            Vector2D tailAEnd = TailA + VA * turnoutTime2;

            Vector2D reverseB = VB.normalized();
            Vector2D TailB = HeadB - reverseB * move2.length;
            Vector2D headBEnd = HeadB + VB * turnoutTime1;
            Vector2D tailBEnd = TailB + VB * turnoutTime1;

            return detectCollision(HeadA, TailA, VA, HeadB, TailB, VB, startTimeA, std::min(endTimeA, endTimeB));
        }

    }


    else if (!m2Pre) {
        double m2PreX(map->getX(move2.preId)), m2PreY(map->getY(move2.preId));
        double turnoutTime2 = move2.length / move2.speed;
        if (startTimeB - move2.t1 < turnoutTime2) {

            Vector2D V1((m2PreX - m2x1), (m2PreY - m2y1));
            Vector2D revsV1 = V1.normalized();
            Vector2D obsTail1 = HeadB + revsV1 * move2.length;
            Vector2D obsStart1 = HeadB;


            Vector2D V2((m2x2 - m2x1), (m2y2 - m2y1));
            Vector2D revsV2 = V2.normalized();
            Vector2D obsTail2 = HeadB + revsV2 * move2.length;
            Vector2D obsStart2 = HeadB;



            Vector2D reverseA = VA.normalized();
            Vector2D TailA = HeadA - reverseA * move1.length;
            Vector2D headAEnd = HeadA + VA * turnoutTime2;
            Vector2D tailAEnd = TailA + VA * turnoutTime2;

            if (segmentsIntersect(HeadA, headAEnd, obsStart1, obsTail1) ||
                segmentsIntersect(TailA, tailAEnd, obsStart1, obsTail1) ||
                segmentsIntersect(HeadA, headAEnd, obsStart2, obsTail2) ||
                segmentsIntersect(TailA, tailAEnd, obsStart2, obsTail2)
                    ) {
                return true;
            }

            return false;


        } else {

            Vector2D headB = HeadB ;

            Vector2D tailB = headB - VB.normalized() * move2.length;

            Vector2D reverseA = VA.normalized();
            Vector2D TailA = HeadA - reverseA * move1.length;
            return detectCollision(HeadA, TailA, VA, headB, tailB, VB, startTimeA, std::min(endTimeA, endTimeB));
        }

    }

    else if (!m1Pre) {
        double m1PreX(map->getX(move1.preId)), m1PreY(map->getY(move1.preId));
        double turnoutTime1 = move1.length / move1.speed;
        if (startTimeA - move1.t1 < turnoutTime1) {

            Vector2D V1((m1PreX - m1x1), (m1PreY - m1y1));
            Vector2D revsV1 = V1.normalized();
            Vector2D obsTail1 = HeadA + revsV1 * move1.length;
            Vector2D obsStart1 = HeadA;


            Vector2D V2((m1x2 - m1x1), (m1y2 - m1y1));
            Vector2D revsV2 = V2.normalized();
            Vector2D obsTail2 = HeadA + revsV2 * move1.length;
            Vector2D obsStart2 = HeadA;

            Vector2D reverseB = VB.normalized();
            Vector2D TailB = HeadB - reverseB * move2.length;
            Vector2D headBEnd = HeadB + VB * turnoutTime1;
            Vector2D tailBEnd = TailB + VB * turnoutTime1;


            if (segmentsIntersect(HeadB, headBEnd, obsStart1, obsTail1) ||
                segmentsIntersect(TailB, tailBEnd, obsStart1, obsTail1) ||
                segmentsIntersect(HeadB, headBEnd, obsStart2, obsTail2) ||
                segmentsIntersect(TailB, tailBEnd, obsStart2, obsTail2)
                    ) {
                return true;
            }
            return false;
        } else {

            Vector2D headA = HeadA;
            Vector2D tailA = headA - VA.normalized() * move1.length;
            Vector2D reverseB = VB.normalized();
            Vector2D TailB = HeadB - reverseB * move2.length;

            return detectCollision(headA, tailA, VA, HeadB, TailB, VB, startTimeA, std::min(endTimeA, endTimeB));
        }
    }
}



bool CBS::isCollinearByDeternminant(double preX,double preY,double x1,double y1,double x2,double y2){

    double determinant = preX * (y1 - y2) +
                         x1 * (y2 - preY) +
                         x2 * (preY - y1);

    return std::abs(determinant) < eps;
}



bool CBS::timeConflictCheck(Move move1, Move move2){

    bool isCollision=false;


    double startTimeA(move1.t1),endTimeA(move1.t2),startTimeB(move2.t1),endTimeB(move2.t2);
    double m1PreX(map->getX(move1.preId)),m1PreY(map->getY(move1.preId));
    double m2PreX(map->getX(move2.preId)),m2PreY(map->getY(move2.preId));
    double m1x1(map->getX(move1.id1)), m1x2(map->getX(move1.id2)), m1y1(map->getY(move1.id1)), m1y2(map->getY(move1.id2));
    double m2x1(map->getX(move2.id1)), m2x2(map->getX(move2.id2)), m2y1(map->getY(move2.id1)), m2y2(map->getY(move2.id2));

    Vector2D m1Start(m1x1,m1y1);
    Vector2D m1End(m1x2,m1y2);
    Vector2D m2Start(m2x1,m2y1);
    Vector2D m2End(m2x2,m2y2);


    Vector2D VA((m1x2-m1x1)/(move1.t2-move1.t1),(m1y2-m1y1)/(move1.t2-move1.t1));
    Vector2D VB((m2x2-m2x1)/(move2.t2-move2.t1),(m2y2-m2y1)/(move2.t2-move2.t1));


    Vector2D reverseA = VA.normalized() ;
    Vector2D reverseB = VB.normalized() ;


    Vector2D HeadA(m1x1,m1y1);
    Vector2D HeadB(m2x1,m2y1);

    Vector2D TailA = HeadA - reverseA * move1.length;
    Vector2D TailB = HeadB - reverseB * move2.length;



    if(move1.preId==-1)
    {
        Vector2D temp(m1x1,m1y1);
        TailA=temp;
        HeadA = TailA + reverseA * move1.length;
    }
    if(move2.preId==-1)
    {
        Vector2D temp(m2x1,m2y1);
        TailB=temp;
        HeadB = TailB + reverseB * move2.length;
    }





    std::pair<double, double> p=intersectCheck(move1,move2);


    if(p.first!=-1&&p.second!=-1){
        Vector2D Point(p.first,p.second);
        double distForMove1=getDist(Point,HeadA);
        double distForMove2=getDist(Point,HeadB);



        double timeMove1Enter=move1.t1+distForMove1/move1.speed;
        double timeMove1Tail=move1.t1+distForMove1/move1.speed+move1.length/move1.speed;
        double timeMove2Enter=move2.t1+distForMove2/move2.speed;
        double timeMove2Tail=move2.t1+distForMove2/move2.speed+move2.length/move2.speed;



        double max_start = std::max(timeMove1Enter, timeMove2Enter);
        double min_end = std::min(timeMove1Tail, timeMove2Enter);

        isCollision = (max_start < min_end);
        return isCollision;

    }
    else if(isSameEdge(m1Start,m1End,m2Start,m2End)){

            double timeMove1EnterEdge=(move1.t1 == 0) ? 0 : move1.t1;
            double timeMove1TailEdge=move1.t2+move1.length/move1.speed;
            double timeMove2EnterEdge=(move2.t1 == 0) ? 0 : (move2.t1);
            double timeMove2TailEdge=move2.t2+move2.length/move2.speed;

            double max_start = std::max(timeMove1EnterEdge, timeMove2EnterEdge);
            double min_end = std::min(timeMove1TailEdge, timeMove2TailEdge);

            isCollision=(max_start < min_end);
            return isCollision;
    }
    else if((isSameVertex(m1End,m2End))&&(!isSameVertex(m1Start,m2Start))){
        double timeMove1EnterVertex=move1.t2;
        double timeMove1TailVertex=move1.t2;
        double timeMove2EnterVertex=move2.t2;
        double timeMove2TailVertex=move2.t2;

        double max_start = std::max(timeMove1EnterVertex, timeMove2EnterVertex);
        double min_end = std::min(timeMove1TailVertex, timeMove2TailVertex);
        isCollision=(max_start < min_end);
        return isCollision;
    }else if(isSameVertex(m1End, m2Start) || isSameVertex(m2End, m1Start)){
        if(isSameVertex(m1End, m2Start)){
            double timeMove1EnterVertex=move1.t2;
            double timeMove2EnterVertex=move2.preId==-1?0:move2.t1;
            double timeMove2TailLeave=move2.preId==-1?0:move2.t1+move2.length/move2.speed;

            if(timeMove1EnterVertex>timeMove2EnterVertex&&timeMove1EnterVertex< timeMove2TailLeave){
                return true;
            }
            return isCollision;
        }else{
            double timeMove2EnterVertex=move2.t2;
            double timeMove1EnterVertex=move1.preId==-1?0:move1.t1;
            double timeMove1TailLeave=move1.preId==-1?0:move1.t1+move1.length/move1.speed;

            if(timeMove2EnterVertex>timeMove1EnterVertex&&timeMove2EnterVertex< timeMove1TailLeave){
                return true;
            }
            return isCollision;
        }
    }
    else{
        return false;
    }

    return isCollision;


}



bool CBS::isSameEdge(Vector2D a, Vector2D b, Vector2D c, Vector2D d){   //a,b为一条边的两个顶点，c,d为另一条边的两个顶点

    bool forwardMatch =
            ((std::abs(a.x - c.x) < eps && std::abs(a.y - c.y) < eps)&&
             (std::abs(d.x - b.x) < eps && std::abs(d.y - b.y) < eps));

    bool reverseMatch =
            ((std::abs(a.x - d.x) < eps && std::abs(a.y - d.y) < eps)&&
             (std::abs(b.x - c.x) < eps && std::abs(b.y - c.y) < eps));


    return forwardMatch||reverseMatch;
}


bool CBS::isSameVertex(Vector2D a, Vector2D b){
    return (std::abs(a.x - b.x) < eps) && (std::abs(a.y - b.y) < eps);
}


std::pair<double, double> CBS::intersectCheck(Move move1, Move move2){
    double m1x1(map->getX(move1.id1)), m1x2(map->getX(move1.id2)), m1y1(map->getY(move1.id1)), m1y2(map->getY(move1.id2));
    double m2x1(map->getX(move2.id1)), m2x2(map->getX(move2.id2)), m2y1(map->getY(move2.id1)), m2y2(map->getY(move2.id2));


    double denom = (m1x2 - m1x1) * (m2y2 - m2y1) - (m1y2 - m1y1) * (m2x2 - m2x1);

    if (std::fabs(denom) < eps) {
        return {-1, -1};
    }


    double t = ((m2x1 - m1x1) * (m2y2 - m2y1) - (m2y1 - m1y1) * (m2x2 - m2x1)) / denom;
    double u = ((m2x1 - m1x1) * (m1y2 - m1y1) - (m2y1 - m1y1) * (m1x2 - m1x1)) / denom;


    if (t > eps && t < 1 - eps && u > eps && u < 1 - eps) {
        double intersectX = m1x1 + t * (m1x2 - m1x1);
        double intersectY = m1y1 + t * (m1y2 - m1y1);
        return {intersectX, intersectY};
    }

    return {-1, -1};
}

double CBS::getDist(Vector2D a, Vector2D b){
    return std::sqrt((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y));
}

Constraint CBS::getConstraint(int agent, Move move1, Move move2){
    if(move1.id1==move1.id2)
        return getWaitConstraint(agent,move1,move2);
    double startTimeA(move1.t1),endTimeA(move1.t2);
    Vector2D A(map->getX(move1.id1),map->getY(move1.id1)),A2(map->getX(move1.id2),map->getY(move1.id2)),
             B(map->getX(move2.id1),map->getY(move2.id1)),B2(map->getX(move2.id2),map->getY(move2.id2));


    if(move2.t2==CN_INFINITY)
        return Constraint(agent,move1.t1,CN_INFINITY,move1.id1,move1.id2);

    double delta = move2.t2 - move1.t1;
    while(delta>CN_PRECISION/2.0){
        if(conflictCheck(move1,move2)){
            move1.t1+=delta;
            move1.t2+=delta;
        }
        else{
            move1.t1-=delta;
            move1.t2-=delta;
        }
        if(move1.t1>move2.t2+CN_EPSILON){
            move1.t1=move2.t2;
            move1.t2=move1.t1+endTimeA-startTimeA;
            break;
        }
        delta/=2.0;
    }

    if(delta<CN_PRECISION/2.0+CN_EPSILON&&conflictCheck(move1,move2)){
        move1.t1=fmin(move1.t1+delta*2,move2.t2);
        move1.t2=move1.t1+endTimeA-startTimeA;
    }

    return Constraint(agent,startTimeA,move1.t1,move1.id1,move1.id2);
}

Constraint CBS::getWaitConstraint(int agent,Move move1,Move move2){

    double length = move1.length;
    double x0(map->getX(move2.id1)),y0(map->getY(move2.id1));
    double x1(map->getX(move2.id2)),y1(map->getY(move2.id2));
    double x2(map->getX(move1.id1)),y2(map->getY(move1.id1));

    std::pair<double,double> interval;
    Point point(x2,y2),p0(x0,y0),p1(x1,y1);
    int cls = point.classify(p0,p1);
    double dist = fabs((x0 - x1)*y2 + (y1 - y0)*x2 + (y0*x1 - x0*y1))/sqrt(pow(x0 - x1, 2) + pow(y0 - y1, 2));
    double da = (x0 - x2)*(x0 - x2) + (y0 - y2)*(y0 - y2);
    double db = (x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2);
    double ha = sqrt(da - dist*dist);
    double size = sqrt(abs(length*length - dist*dist));

    if(cls==3){
        interval.first = move2.t1;
        interval.second = move2.t1+(sqrt(length*length-dist*dist)-ha)/move1.speed;
    }
    else if(cls==4){
        interval.first = move2.t2-sqrt(length*length-dist*dist)/move1.speed+sqrt(db-dist*dist)/move1.speed;
        interval.second = move2.t2;
    }
    else if(da<length*length){
        if(db < length*length){
            interval.first = move2.t1;
            interval.second = move2.t2;
        }
        else{
            double hb = sqrt(db - dist*dist);
            interval.first = move2.t1;
            interval.second = move2.t2 - hb/move1.speed+size/move1.speed;
        }
    }
    else{
        if(db < length*length){
            interval.first = move2.t1+ha/move1.speed-size/move1.speed;;
            interval.second = move2.t2;
        }
        else{
            interval.first = move2.t1+ha/move1.speed-size/move1.speed;
            interval.second = move2.t2+ha/move1.speed-size/move1.speed;
        }
    }

    return Constraint(agent,interval.first,interval.second,move1.id1,move1.id2);
}

double CBS::getCost(CBS_Node node, int agent_id)
{
    while(node.parent != nullptr)
    {
        if(node.paths.begin()->agentID == agent_id)
            return node.paths.begin()->cost;
        node = *node.parent;
    }
    return node.paths.at(agent_id).cost;
}

std::vector<solutionPath> CBS::getPaths(CBS_Node *node, unsigned int agents_size){
    CBS_Node *curNode=node;
    std::vector<solutionPath> paths(agents_size);
    while(curNode->parent!= nullptr){
        if(paths.at(curNode->paths.begin()->agentID).cost<0)
            paths.at(curNode->paths.begin()->agentID)=*curNode->paths.begin();
        curNode=curNode->parent;
    }
    for(int i=0;i<agents_size;i++){
        if(paths.at(i).cost<0)
            paths.at(i)=curNode->paths.at(i);
    }
    return paths;
}


