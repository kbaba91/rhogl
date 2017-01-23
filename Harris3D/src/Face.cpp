#include "Face.h"

namespace Harris3D{

Face::Face(){

}

void Face::addVertex(int vertex){
  vertices.push_back(vertex);
}

std::vector<int>& Face::getVertices(){
  return vertices;
}

int Face::getVertex(int pos){
  return vertices[pos];
}

int Face::index(int v){
  if(vertices[0] == v) return 0;
  if(vertices[1] == v) return 1;
  if(vertices[2] == v) return 2;
  return -1;
}
}
