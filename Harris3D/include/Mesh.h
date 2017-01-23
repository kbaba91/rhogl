#ifndef __MESH_H_
#define __MESH_H_

#include "Vertex.h"
#include "Face.h"
#include <vector>
#include <cstring>
#include <openvdb/openvdb.h>
#include <openvdb/util/Util.h>
#include <openvdb/tools/MeshToVolume.h>

namespace Harris3D{

class Mesh{

      //Topological information
      std::vector<Vertex> vertices;
      std::vector<Face> faces;

      float** D;
      
      //BBox Information
      double xmin, xmax, ymin, ymax, zmin, zmax;
      double diag;
      openvdb::FloatGrid::Ptr grid;

  public:
      void cleanMesh();
      
      Mesh();
      Mesh(const char* nombreArchivo);
      Mesh(std::vector<std::string>& files);
      ~Mesh();

      void loadFromFile(const char* filename);
      void loadFromFiles(std::vector<std::string>& files);
             
      friend std::ostream& operator<<(std::ostream& os, Mesh &obj);

      inline std::vector<Vertex>& getVertices(){return vertices;}
      inline std::vector<Face>& getFaces(){return faces;}
      inline  int getNumVertices(){return vertices.size();}
      inline  int getNumFaces(){return faces.size();}

      openvdb::FloatGrid::Ptr getGrid(float factor);
      void        computeGrid(float factor);
			 
      inline  double getDiagonal(){return diag;}
      inline  double getArea();
      double computeDiagonal();
      void getPatch(int seed, std::vector<int> indices, double radius, Vertex center, Mesh* patch, std::set<int>& vertReturned);
      void getSpatialPatch(Mesh* patch, Vertex center, double radius);

      void setVertices(std::vector<Vertex>& vertices);
      void setFaces(std::vector<Face>& faces);
      void insertVertex(int pos, double x, double y, double z);
      void insertFace(int pos, int p1, int p2, int p3);
};

}
#endif
