#include "map.h"

bool Map::getMap(const char* file_path){
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(file_path) != tinyxml2::XML_SUCCESS) {
        std::cerr << "Failed to load XML file: " << file_path << std::endl;
        return false;
    }

    tinyxml2::XMLElement* root = 0, *element = 0, *data;
    std::string value;
    std::stringstream stream;
    root = doc.FirstChildElement("graphml")->FirstChildElement("graph");
    for(element = root->FirstChildElement("node"); element; element = element->NextSiblingElement("node")){
        data = element->FirstChildElement();


        stream.str("");
        stream.clear();
        stream << data->GetText();
        stream >> value;

        auto it = value.find_first_of(",");

        stream.str("");
        stream.clear();
        stream << value.substr(0, it);
        double i;
        stream >> i;
        

        stream.str("");
        stream.clear();
        value.erase(0, ++it);
        stream << value;
        double j;
        stream >> j;


        std::vector<subVertex> subVertices;
        subVertices.push_back(subVertex(0,i,j));
        subVertices.push_back(subVertex(1,i,j));
        Vertex vertex(subVertices);
        vertices.push_back(vertex);
    }

    for(element = root->FirstChildElement("edge"); element; element = element->NextSiblingElement("edge")){

        std::string source = std::string(element->Attribute("source"));
        std::string target = std::string(element->Attribute("target"));
        source.erase(source.begin(),++source.begin());
        target.erase(target.begin(),++target.begin());

        int id1, id2;
        stream.str("");
        stream.clear();
        stream << source;
        stream >> id1;
        stream.str("");
        stream.clear();
        stream << target;
        stream >> id2;

        vertices[id1].subVertices[1].neighbors.push_back(id2);
        vertices[id2].subVertices[0].neighbors.push_back(id1);
    }

    for(Vertex vertex:vertices){
        Node node;
        std::vector<Node> neighbors;
        neighbors.clear();
        for(unsigned int i=0; i<vertex.subVertices[0].neighbors.size();i++){
            node.index=1;
            node.x=vertices[vertex.subVertices[0].neighbors[i]].subVertices[0].x;
            node.y=vertices[vertex.subVertices[0].neighbors[i]].subVertices[0].y;
            node.id=vertex.subVertices[0].neighbors[i];
            neighbors.push_back(node);
        }
        for(unsigned int i=0; i<vertex.subVertices[1].neighbors.size();i++){
            node.index=0;
            node.x=vertices[vertex.subVertices[1].neighbors[i]].subVertices[1].x;
            node.y=vertices[vertex.subVertices[1].neighbors[i]].subVertices[1].y;
            node.id=vertex.subVertices[1].neighbors[i];
            neighbors.push_back(node);
        }
        valid_moves.push_back(neighbors);
    }

    size=int(vertices.size());
    return true;

}

std::vector<Node> Map::getValidMoves(int index,int id){
    std::vector<Node> valid;
    if(index==0){
        std::vector<Node> allMoves=valid_moves[id];
        for(unsigned int i=0;i<allMoves.size();i++){
            if(allMoves[i].index==1){
                valid.push_back(allMoves[i]);
            }
            else if(allMoves[i].index==0)
                continue;
        }
    }
    if(index==1){
        std::vector<Node> allMoves=valid_moves[id];
        for(unsigned int i=0;i<allMoves.size();i++){
            if(allMoves[i].index==0){
                valid.push_back(allMoves[i]);
            }
            else if(allMoves[i].index==1)
                continue;
        }
    }
    return valid;
}


subVertex Map::getSubVertex(int id,int index){
    if(id<int(vertices.size()))
        return vertices[id].subVertices[index];
    return subVertex();
}

double Map::getX(int id){
    if(id==-1){
        return -0.01;
    }
    return vertices[id].subVertices[0].x;
}

double Map::getY(int id){
    if(id==-1){
        return -0.01;
    }
    return vertices[id].subVertices[0].y;
}
