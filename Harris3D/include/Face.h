#ifndef __FACE_H_
#define __FACE_H_

#include <vector>

namespace Harris3D{

class Face{
 private:
     std::vector<int> vertices;
 public:
     Face();
     void addVertex(int vertex);
     std::vector<int>& getVertices();
     int getVertex(int pos);
     int index(int v);
     double normal[3];
};

}
#endif
