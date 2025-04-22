#include "heuristic.h"


void heuristic::init(int size,int agents){
    h_values.clear();
    h_values.resize(size);
    for(int i=0;i<size;i++){
        h_values[i].resize(agents,-1);
    }
}

void heuristic::count(Map &map, Agent agent) {
    Node curNode(agent.g_index,agent.goal_id,0,0,agent.goal_x,agent.goal_y),newNode;
    open.clear();
    int index=curNode.index;
    open.insert(curNode);
    while(!open.empty()){

        curNode = find_min_by_index(index);
        curNode.ConvertNode();
        h_values[curNode.id][agent.id]=curNode.g;
        std::vector<Node> valid_moves = map.getValidMoves(curNode.index,curNode.id);
        for(auto move:valid_moves){
            newNode.x=move.x;
            newNode.y=move.y;
            newNode.id=move.id;
            newNode.index=move.index;
            index=move.index;

            newNode.g=curNode.g+dist(curNode,newNode)/agent.speed;

            if(h_values[newNode.id][agent.id]<0){
                auto it = open.get<1>().find(newNode.id);
                if(it != open.get<1>().end()){
                    if(it->g>newNode.g)
                        open.get<1>().erase(it);
                    else
                        continue;
                }
                open.insert(newNode);
            }
        }
      }
}

Node heuristic::find_min(){
    Node min = *open.begin();
    open.erase(open.begin());
    return min;
}

Node heuristic::find_min_by_index(int target_index) {
    Node result;
    auto& index_g_index = open.get<by_index_and_g>();
    auto range = index_g_index.equal_range(
            boost::make_tuple(target_index)
    );
    if(range.first != range.second) {
        result = *range.first;

        index_g_index.erase(range.first);
        return result;
    }
    throw std::runtime_error("No node found with specified index");
}