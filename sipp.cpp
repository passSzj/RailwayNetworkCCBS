#include "sipp.h"


void SIPP::clear(){
    open.clear();
    close.clear();
    visited.clear();
    path.cost = -1;
}


Path SIPP::find_path(Agent agent, Map &map, std::list<Constraint> cons, heuristic &h_values){

    this->clear();
    this->agent=agent;
    make_constraints(cons);
    Node start,goal;
    Path path;
    int expanded=0;

    start= getNode(agent.s_index,agent.start_id,agent.start_x,agent.start_y,0,CN_INFINITY);
    goal= getNode(agent.g_index,agent.goal_id,agent.goal_x,agent.goal_y,0,CN_INFINITY);
    path = pathPlan(start,goal,map,h_values,agent.speed);
    expanded=int(close.size());
    if(path.cost<0)
        return Path();
    path.nodes.shrink_to_fit();
    //path.cost=path.nodes.back().g-agent.length/agent.speed;    -agent.length/agent.speed  起点处列车长度占据轨道一部分
    path.cost=path.nodes.back().g;
    path.agentID=agent.id;
    path.expanded=expanded;
    path.agentLength=agent.length;
    path.agentSpeed=agent.speed;

    return path;

}


Node SIPP::getNode(int index, int node_id, double node_x, double node_y, double t1, double t2){
    Node node;
    node = Node{index,node_id, 0, 0, node_x, node_y, nullptr, t1, t2};
    return node;
}


Path SIPP::pathPlan(Node start,Node goal, Map &map,heuristic &h_values,double agentSpeed, double max_f){
    open.clear();
    close.clear();
    path.cost=-1;
    visited.clear();
    Path path;
    int pathFound=0;
    start.parent= nullptr;
    open.push_back(start);
    visited.insert({start.id+(start.interval_id+start.index)*map.getMapSize(),{start.g,false}});
    Node curNode;

    while(!open.empty()){
        curNode=find_min();
        /*----------*/

        /*----------*/
        auto v = visited.find(curNode.id+(curNode.interval_id+curNode.index)*map.getMapSize());
        if(v->second.second)
            continue;
        v->second.second = true;

        curNode.ConvertNode();   /*转换的位置在close前后？？？*/

        auto parent=&close.insert({curNode.id+(curNode.interval_id+curNode.index)*map.getMapSize(),curNode}).first->second;

        if(curNode.id==goal.id){
            if(curNode.g - CN_EPSILON < goal.interval.second && goal.interval.first - CN_EPSILON < curNode.interval.second)
            {
                path.nodes = reconstructPath(curNode);
                if(path.nodes.back().g < goal.interval.first){
                    curNode.g = goal.interval.first;
                    path.nodes.push_back(curNode);
                }
                path.cost = curNode.g;
                path.expanded = int(close.size());
                pathFound++;
            }
                return path;
        }

        std::list<Node> succs;
        succs.clear();

        findSuccessors(curNode, map, succs, h_values, goal, agentSpeed);
        std::list<Node>::iterator it =succs.begin();

        while(it != succs.end()){
            if(it->f>max_f){
                it++;
                continue;
            }
            it->parent=parent;
            add_open(*it);
            it++;
        }

    }
    return path;
}


Node SIPP::find_min()
{
    Node min = *open.begin();
    open.pop_front();
    return min;
}

void SIPP::add_open(Node newNode){
    if (open.empty() || open.back().f - CN_EPSILON < newNode.f)
    {
        open.push_back(newNode);
        return;
    }
    for(auto iter = open.begin(); iter != open.end(); ++iter)
    {
        if(iter->f > newNode.f + CN_EPSILON)
        {
            open.insert(iter, newNode);
            return;
        }
        else if(fabs(iter->f - newNode.f) < CN_EPSILON && newNode.g + CN_EPSILON > iter->g)
        {
            open.insert(iter, newNode);
            return;
        }
    }
    open.push_back(newNode);
    return;
}


void SIPP::findSuccessors(Node curNode, Map map, std::list<Node> &succs, heuristic &h_values, Node goal, double agentSpeed){
    Node newNode;
    std::vector<Node> valid_moves = map.getValidMoves(curNode.index,curNode.id);
    for(auto move: valid_moves){

        newNode.index=move.index;
        newNode.id=move.id;
        newNode.x=move.x;
        newNode.y=move.y;
        double cost = dist(curNode,newNode)/agentSpeed;
        newNode.g=curNode.g+cost;

        std::vector<std::pair<double, double>> intervals(0);


        auto colls_it = collisionIntervals.find(newNode.id);

        if(colls_it != collisionIntervals.end())
        {
            std::pair<double, double> interval = {0, CN_INFINITY};
            for(unsigned int i = 0; i < colls_it->second.size(); i++)
            {
                interval.second = colls_it->second[i].first;
                intervals.push_back(interval);
                interval.first = colls_it->second[i].second;
            }
            interval.second = CN_INFINITY;
            intervals.push_back(interval);
        }
        else
            intervals.push_back({0, CN_INFINITY});


        auto cons_it=constraints.find({curNode.id,newNode.id});


        int id=0;
        for(auto interval:intervals) {

            newNode.interval_id = id;
            id++;
            auto it = visited.find(newNode.id + (newNode.interval_id + newNode.index) * map.getMapSize());

            if (it != visited.end())
                if (it->second.second)
                    continue;


            if (interval.second < newNode.g)
                continue;

            if (interval.first > newNode.g)
                newNode.g = interval.first;

            if(cons_it != constraints.end()){
                for(int i=0; i<cons_it->second.size(); i++)

                    if (newNode.g-cost+CN_EPSILON>cons_it->second[i].t1 && newNode.g-cost<cons_it->second[i].t2)
                        newNode.g= cons_it->second[i].t2+cost;
            }

            newNode.interval = interval;

            if(newNode.g-cost>curNode.interval.second || newNode.g>newNode.interval.second){
                continue;
            }

            if(it != visited.end()){

                if(it->second.first - CN_EPSILON < newNode.g)
                    continue;
                else
                    it->second.first = newNode.g;
            }
            else
                visited.insert({newNode.id + (newNode.interval_id+newNode.index) * map.getMapSize(), {newNode.g, false}});

            newNode.f = newNode.g + h_values.get_value(newNode.id,agent.id);


            succs.push_back(newNode);

        }

    }

}


double SIPP::dist(const Node& a, const Node& b)
{
    return std::sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}


std::vector<Node> SIPP::reconstructPath(Node curNode){
    path.nodes.clear();
    if(curNode.parent!= nullptr){
        do{
            path.nodes.insert(path.nodes.begin(),curNode);
            curNode=*curNode.parent;
        } while (curNode.parent!= nullptr);
        path.nodes.insert(path.nodes.begin(),curNode);

        for(int i=0;i<path.nodes.size();i++){
            int j=i+1;
            if(j==path.nodes.size())
                break;
            if(fabs(path.nodes[j].g-path.nodes[i].g- (dist(path.nodes[j], path.nodes[i])/agent.speed))> CN_EPSILON){
                Node add = path.nodes[i];
                add.g = path.nodes[j].g-(dist(path.nodes[j], path.nodes[i])/agent.speed);
                path.nodes.emplace(path.nodes.begin()+j,add);
            }
        }
        
        return path.nodes;
    }
}


void SIPP::make_constraints(std::list<Constraint> &cons){
    for(auto con:cons){
        if(con.id1 == con.id2)
            addCollisionInterval(con.id1,std::make_pair(con.t1,con.t2));
        else
            addMoveConstraint(Move(con));
    }
}


void SIPP::addCollisionInterval(int id, std::pair<double, double> interval){
    std::vector<std::pair<double, double>> intervals(0);
    if(collisionIntervals.count(id)==0)
        collisionIntervals.insert({id,{interval}});
    else
        collisionIntervals[id].push_back(interval);

    std::sort(collisionIntervals[id].begin(),collisionIntervals[id].end());

    for(unsigned int i=0; i+1<collisionIntervals[id].size(); i++){
        if(collisionIntervals[id][i].second+CN_EPSILON>collisionIntervals[id][i+1].first){
            collisionIntervals[id][i].second=collisionIntervals[id][i+1].second;
            collisionIntervals[id].erase(collisionIntervals[id].begin()+i+1);
            i--;
        }
    }

}


void SIPP::addMoveConstraint(Move move){
    std::vector<Move> mCons(0);

    if(constraints.count({move.id1,move.id2})==0)
        constraints.insert({{move.id1,move.id2},{move}});
    else{
        mCons=constraints.at({move.id1,move.id2});
        bool inserted=false;
        for(unsigned int i=0; i<mCons.size(); i++){
            if(inserted)
                break;
            if(mCons[i].t1 > move.t1)
            {
                if(mCons[i].t1 < move.t2 + CN_EPSILON)
                {
                    mCons[i].t1 = move.t1;
                    if(move.t2 + CN_EPSILON > mCons[i].t2)
                        mCons[i].t2 = move.t2;
                    inserted = true;
                    if(i != 0)
                        if(mCons[i-1].t2 + CN_EPSILON > move.t1 && mCons[i-1].t2 < move.t2 + CN_EPSILON)
                        {
                            mCons[i-1].t2 = move.t2;
                            if(mCons[i-1].t2 + CN_EPSILON > mCons[i].t1 && mCons[i-1].t2 < mCons[i].t2 + CN_EPSILON)
                            {
                                mCons[i-1].t2 = mCons[i].t2;
                                mCons.erase(mCons.begin() + i);
                            }
                            inserted = true;
                        }
                }
                else
                {
                    if(i != 0)
                        if(mCons[i-1].t2 + CN_EPSILON > move.t1 && mCons[i-1].t2 < move.t2 + CN_EPSILON)
                        {
                            mCons[i-1].t2 = move.t2;
                            inserted = true;
                            break;
                        }
                    mCons.insert(mCons.begin() + i, move);
                    inserted = true;
                }
            }
        }
        if(mCons.back().t2 + CN_EPSILON > move.t1 && mCons.back().t2 < move.t2 + CN_EPSILON)
            mCons.back().t2 = move.t2;
        else if(!inserted)
            mCons.push_back(move);
        constraints.at({move.id1, move.id2}) = mCons;
    }
}