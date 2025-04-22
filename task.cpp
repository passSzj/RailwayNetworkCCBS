#include "task.h"


bool Task::getTask(const char* FileName){
    tinyxml2::XMLElement *root = 0, *agent = 0;
    tinyxml2::XMLDocument doc;

    if (doc.LoadFile(FileName) != tinyxml2::XMLError::XML_SUCCESS)
    {
        std::cout << "Error opening XML file!" << std::endl;
        return false;
    }

    root = doc.FirstChildElement("root");
    if (!root)
    {
        std::cout << "Error! No '" << "root" << "' tag found in XML file!" << std::endl;
        return false;
    }


    for (agent = root->FirstChildElement(); agent; agent = agent->NextSiblingElement())
    {
        Agent a;
        a.start_x = agent->DoubleAttribute("start_x" );
        a.start_y = agent->DoubleAttribute("start_y");
        a.start_id = agent->IntAttribute("start_id");
        a.s_index = agent->IntAttribute("s_index");
        a.goal_x = agent->DoubleAttribute("goal_x");
        a.goal_y = agent->DoubleAttribute("goal_y");
        a.goal_id = agent->IntAttribute("goal_id");
        a.g_index = agent->IntAttribute("g_index");
        a.length = agent->DoubleAttribute("length");
        a.speed = agent->DoubleAttribute("speed");
        a.id = int(agents.size());
        agents.push_back(a);

    }
    return true;

}


void Task::getXY(Map &map){
    for(int i=0;i<agents.size();i++){
        subVertex start=map.getSubVertex(agents[i].start_id,agents[i].s_index);
        subVertex goal=map.getSubVertex(agents[i].goal_id,agents[i].g_index);
        agents[i].start_x=start.x;
        agents[i].start_y=start.y;
        agents[i].goal_x=goal.x;
        agents[i].goal_y=goal.y;
    }
}

