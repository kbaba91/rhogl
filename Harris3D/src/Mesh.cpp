#include <map>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <math.h>
#include <fstream>
#include <cassert>
#include <cfloat>
#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>
#include "Mesh.h"
#include "util.h"
#include "Clock.h"

using namespace std;
using namespace boost;

namespace Harris3D{

Mesh::Mesh(){
       vertices.clear();
       faces.clear();
}

Mesh::Mesh(const char* filename)
{
       vertices.clear();
       faces.clear();

       xmin = xmax = ymin = ymax = zmin = zmax = 0.0;

       loadFromFile(filename);

       //Finding bounding box...
       for(register int i = 0; i < getNumVertices(); i++){
            if(vertices[i].getX() < xmin)
                xmin = vertices[i].getX();
            else if(vertices[i].getX() > xmax)
                xmax = vertices[i].getX();

            if(vertices[i].getY() < ymin)
                ymin = vertices[i].getY();
            else if(vertices[i].getY() > ymax)
                ymax = vertices[i].getY();

            if(vertices[i].getZ() < zmin)
                zmin = vertices[i].getZ();
            else if(vertices[i].getZ() > zmax)
                zmax = vertices[i].getZ();
       }

       diag = sqrt((xmax - xmin)*(xmax - xmin) + (ymax - ymin)*(ymax - ymin) + (zmax - zmin)*(zmax - zmin));
}

Mesh::Mesh(std::vector<string>& files)
{
       vertices.clear();
       faces.clear();

       xmin = xmax = ymin = ymax = zmin = zmax = 0.0;

       loadFromFiles(files);

       //Finding bounding box...
       for(register int i = 0; i < getNumVertices(); i++){
            if(vertices[i].getX() < xmin)
                xmin = vertices[i].getX();
            else if(vertices[i].getX() > xmax)
                xmax = vertices[i].getX();

            if(vertices[i].getY() < ymin)
                ymin = vertices[i].getY();
            else if(vertices[i].getY() > ymax)
                ymax = vertices[i].getY();

            if(vertices[i].getZ() < zmin)
                zmin = vertices[i].getZ();
            else if(vertices[i].getZ() > zmax)
                zmax = vertices[i].getZ();
       }

       diag = sqrt((xmax - xmin)*(xmax - xmin) + (ymax - ymin)*(ymax - ymin) + (zmax - zmin)*(zmax - zmin));
}

double Mesh::computeDiagonal(){
    xmin = xmax = ymin = ymax = zmin = zmax = 0.0;

    //Finding bounding box...
    for(register int i = 0; i < getNumVertices(); i++){
         if(vertices[i].getX() < xmin)
             xmin = vertices[i].getX();
         else if(vertices[i].getX() > xmax)
             xmax = vertices[i].getX();

         if(vertices[i].getY() < ymin)
             ymin = vertices[i].getY();
         else if(vertices[i].getY() > ymax)
             ymax = vertices[i].getY();

         if(vertices[i].getZ() < zmin)
             zmin = vertices[i].getZ();
         else if(vertices[i].getZ() > zmax)
             zmax = vertices[i].getZ();
    }

    diag = sqrt((xmax - xmin)*(xmax - xmin) + (ymax - ymin)*(ymax - ymin) + (zmax - zmin)*(zmax - zmin));

    return diag;
}

void Mesh :: computeGrid(float factor){

    openvdb::math::Transform::Ptr t = openvdb::math::Transform::createLinearTransform(factor);

    std::vector<openvdb::Vec3s> pointList;
    std::vector<openvdb::Vec4I> polygonList;

    for(int i = 0; i < getNumVertices(); i++){
        openvdb::Vec3s xyz(0.0, 0.0, 0.0);
        xyz = t->worldToIndex(openvdb::Vec3s(vertices[i].x(), vertices[i].y(), vertices[i].z()));
        pointList.push_back(xyz);
    }

    for(int i = 0; i < getNumFaces(); i++){
        std::vector<int> vert = faces[i].getVertices();
        polygonList.push_back(openvdb::Vec4I(vert[0], vert[1], vert[2], openvdb::util::INVALID_IDX));
    }

    grid = openvdb::tools::meshToLevelSet<openvdb::FloatGrid>(*t, pointList, polygonList);
}

openvdb::FloatGrid::Ptr Mesh :: getGrid(float factor){
    if(grid==NULL){
        computeGrid(factor);
    }
    return grid;
}

Mesh::~Mesh(){
    	cleanMesh();
}

/* EDIT */
void Mesh::setVertices(std::vector<Vertex>& vertices){
    this->vertices = vertices;
}

void Mesh::setFaces(std::vector<Face>& faces){
    this->faces = faces;
}

//Clean up the object
void Mesh::cleanMesh(){
    faces.clear();
    vertices.clear();
}
/* END */

void Mesh::loadFromFile(const char* filename){

    int numFaces,numVertices,numEdges;

	std::ifstream in(filename);

    std::string format(filename);

    if ((format.find(".OFF") != std::string::npos) || (format.find(".off") != std::string::npos)){
        std::getline(in, format);
        assert((format.find("OFF") != std::string::npos) || (format.find("off") != std::string::npos));

        in>>numVertices>>numFaces>>numEdges;
        Util::skipline(in);

        vertices.clear();
        faces.clear();

        for(register int i = 0; i < numVertices; i++){
            double x, y, z;
            in>>x>>y>>z;
            Util::skipline(in);
            insertVertex(i,x,y,z);
        }

        for(register int i = 0;  i < numFaces; i++){
            int numVert;
            in >> numVert;

            assert(numVert == 3);

            int p1, p2, p3;

            in>>p1>>p2>>p3;
            Util::skipline(in);

            if(p1==p2 || p2 == p3 || p1 == p3)
                std::cout << "Warning: face " << i << " contains a zero-length edge" << std::endl;

            insertFace(i,p1,p2,p3);
        }
    }
    else if ((format.find(".OBJ") != std::string::npos) || (format.find(".obj") != std::string::npos)){
        string str;

        int lineNumber = 1;
        while(!in.eof()){
            getline(in, str);
            //cout << lineNumber << ": " << str << endl;
            if(str.size()==0 || str[0]=='#')
                continue;

            //Extraer los tokens y guardarlos en un vector
            vector<string> tok;
            char_separator<char> sep(" ");
            tokenizer< char_separator<char> > tokens(str, sep);
            BOOST_FOREACH (const string& t, tokens){
                tok.push_back(t);
            }

            if(tok[tok.size()-1].find('\\')!=string::npos){
                string str1;
                lineNumber++;
                getline(in, str1);
                tokenizer< char_separator<char> > tokens3(str1, sep);
                BOOST_FOREACH (const string& t, tokens3){
                    tok.push_back(t);
                }
            }

            if(tok[0].compare("v")==0){
                if(tok.size()==4){
                    Vertex v;
                    v.setX(atof(tok[1].c_str()));
                    v.setY(atof(tok[2].c_str()));
                    v.setZ(atof(tok[3].c_str()));
                    v.setIndex(vertices.size());
                    vertices.push_back(v);
                }else if (tok.size()==7){
                    Vertex v;
                    v.setX(atof(tok[1].c_str()));
                    v.setY(atof(tok[2].c_str()));
                    v.setZ(atof(tok[3].c_str()));
                    v.color[0] = atof(tok[4].c_str());
                    v.color[1] = atof(tok[5].c_str());
                    v.color[2] = atof(tok[6].c_str());
                    if (v.color[0]<0.2 && v.color[1]<0.2 && v.color[2]>0.8){
                        v.setBlue(true);
                    }
                    v.setIndex(vertices.size());
                    vertices.push_back(v);
                }else{
                    cout << "Error while reading vertex in line:" << lineNumber << endl;
                //	exit(EXIT_FAILURE);
                }
            }else if(tok[0].compare("f") == 0){
                //cout << tok.size() << endl;
                int numVertex = tok.size() - 1;
                if(numVertex < 3){
                    cout << "Error while reading face on line:" << lineNumber << endl;
                    //exit(EXIT_FAILURE);
                }
                vector<int> indices;
                char_separator<char> sep2("/");
                for(size_t i = 1; i < tok.size(); i++){
                    tokenizer< char_separator<char> > tokens2(tok[i], sep2);
                    tokenizer< char_separator<char> >::iterator beg = tokens2.begin();
                    int index = atoi(beg->c_str());
                    //cout << index << " ";
                    vector<int>::iterator it = find(indices.begin(), indices.end(), index);
                    if(it == indices.end())
                        indices.push_back(index);
                    else{
                        cout << "Repeating vertices in face list on line:" << lineNumber << "(" << str << ")"<<endl;
                        //exit(EXIT_FAILURE);
                    }
                }

                insertFace(faces.size(),indices[0]-1,indices[1]-1,indices[2]-1);
            }
            lineNumber++;
        }
    }

	in.close();

    cout << "Number of vertices: " << vertices.size() << endl;
    cout << "Number of faces: " << faces.size() << endl;
}

void Mesh::loadFromFiles(std::vector<string>& files){

    int numFaces,numVertices,numEdges;
    vertices.clear();
    faces.clear();

    for (int i = 0; i<files.size(); i++){
        std::ifstream in(files.at(i));
        std::string format(files.at(i));
        int offset = vertices.size();

        if ((format.find(".OFF") != std::string::npos) || (format.find(".off") != std::string::npos)){
            std::getline(in, format);
            assert((format.find("OFF") != std::string::npos) || (format.find("off") != std::string::npos));

            in>>numVertices>>numFaces>>numEdges;
            Util::skipline(in);

            for(register int i = 0; i < numVertices; i++){
                double x, y, z;
                in>>x>>y>>z;
                Util::skipline(in);
                insertVertex(vertices.size(),x,y,z);
            }

            for(register int i = 0;  i < numFaces; i++){
                int numVert;
                in >> numVert;

                assert(numVert == 3);

                int p1, p2, p3;

                in>>p1>>p2>>p3;
                Util::skipline(in);

                if(p1==p2 || p2 == p3 || p1 == p3)
                    std::cout << "Warning: face " << i << " contains a zero-length edge" << std::endl;

                insertFace(faces.size(),p1+offset,p2+offset,p3+offset);
            }
        }
        else if ((format.find(".OBJ") != std::string::npos) || (format.find(".obj") != std::string::npos)){
            string str;

            int lineNumber = 1;
            while(!in.eof()){
                getline(in, str);
                //cout << lineNumber << ": " << str << endl;
                if(str.size()==0 || str[0]=='#')
                    continue;

                //Extraer los tokens y guardarlos en un vector
                vector<string> tok;
                char_separator<char> sep(" ");
                tokenizer< char_separator<char> > tokens(str, sep);
                BOOST_FOREACH (const string& t, tokens){
                    tok.push_back(t);
                }

                if(tok[tok.size()-1].find('\\')!=string::npos){
                    string str1;
                    lineNumber++;
                    getline(in, str1);
                    tokenizer< char_separator<char> > tokens3(str1, sep);
                    BOOST_FOREACH (const string& t, tokens3){
                        tok.push_back(t);
                    }
                }

                if(tok[0].compare("v")==0){
                    if(tok.size()==4){
                        Vertex v;
                        v.setX(atof(tok[1].c_str()));
                        v.setY(atof(tok[2].c_str()));
                        v.setZ(atof(tok[3].c_str()));
                        v.setIndex(vertices.size());
                        vertices.push_back(v);
                    }else if (tok.size()==7){
                        Vertex v;
                        v.setX(atof(tok[1].c_str()));
                        v.setY(atof(tok[2].c_str()));
                        v.setZ(atof(tok[3].c_str()));
                        v.color[0] = atof(tok[4].c_str());
                        v.color[1] = atof(tok[5].c_str());
                        v.color[2] = atof(tok[6].c_str());
                        if (v.color[0]<0.2 && v.color[1]<0.2 && v.color[2]>0.8){
                            v.setBlue(true);
                        }
                        v.setIndex(vertices.size());
                        vertices.push_back(v);
                    }else{
                        cout << "Error while reading vertex in line:" << lineNumber << endl;
                    //	exit(EXIT_FAILURE);
                    }
                }else if(tok[0].compare("f") == 0){
                    //cout << tok.size() << endl;
                    int numVertex = tok.size() - 1;
                    if(numVertex < 3){
                        cout << "Error while reading face on line:" << lineNumber << endl;
                        //exit(EXIT_FAILURE);
                    }
                    vector<int> indices;
                    char_separator<char> sep2("/");
                    for(size_t i = 1; i < tok.size(); i++){
                        tokenizer< char_separator<char> > tokens2(tok[i], sep2);
                        tokenizer< char_separator<char> >::iterator beg = tokens2.begin();
                        int index = atoi(beg->c_str());
                        //cout << index << " ";
                        vector<int>::iterator it = find(indices.begin(), indices.end(), index);
                        if(it == indices.end())
                            indices.push_back(index);
                        else{
                            cout << "Repeating vertices in face list on line:" << lineNumber << "(" << str << ")"<<endl;
                            //exit(EXIT_FAILURE);
                        }
                    }

                    insertFace(faces.size(),indices[0]-1+offset,indices[1]-1+offset,indices[2]-1+offset);
                }
                lineNumber++;
            }
        }

        in.close();
    }

    cout << "Number of vertices: " << vertices.size() << endl;
    cout << "Number of faces: " << faces.size() << endl;
}

void Mesh :: insertVertex(int pos, double x, double y, double z){
    Vertex vertex;
    vertex.setX(x);	vertex.setY(y);	vertex.setZ(z);
    vertex.setIndex(pos);
    vertices.push_back(vertex);
}

void Mesh :: insertFace(int pos, int p1, int p2, int p3){
    Face face;
    face.addVertex(p1);	face.addVertex(p2); 	face.addVertex(p3);

	vertices[p1].addFace(pos);			vertices[p2].addFace(pos);			vertices[p3].addFace(pos);
    vertices[p1].setPosBarycentric(0);	vertices[p2].setPosBarycentric(1);	vertices[p3].setPosBarycentric(2);
	vertices[p1].addVertex(p2);	vertices[p1].addVertex(p3);
	vertices[p2].addVertex(p1);	vertices[p2].addVertex(p3);
	vertices[p3].addVertex(p1);	vertices[p3].addVertex(p2);

    //compute normal
    face.normal[0] = (vertices[p2].getY() - vertices[p1].getY()) * (vertices[p3].getZ() - vertices[p1].getZ()) - (vertices[p2].getZ() - vertices[p1].getZ()) * (vertices[p3].getY() - vertices[p1].getY());
    face.normal[1] = (vertices[p2].getZ() - vertices[p1].getZ()) * (vertices[p3].getX() - vertices[p1].getX()) - (vertices[p2].getX() - vertices[p1].getX()) * (vertices[p3].getZ() - vertices[p1].getZ());
    face.normal[2] = (vertices[p2].getX() - vertices[p1].getX()) * (vertices[p3].getY() - vertices[p1].getY()) - (vertices[p2].getY() - vertices[p1].getY()) * (vertices[p3].getX() - vertices[p1].getX());

    //unit vector
    double size = std::sqrt(pow(face.normal[0],2) + pow(face.normal[1],2) + pow(face.normal[2],2));
    face.normal[0] /= size;
    face.normal[1] /= size;
    face.normal[2] /= size;

    faces.push_back(face);

    //compute vertex normal
    vertices[p1].computeNormal(faces);
    vertices[p2].computeNormal(faces);
    vertices[p3].computeNormal(faces);
}

void Mesh :: getPatch(int seed, std::vector<int> indices, double radius, Vertex center, Mesh* patch, std::set<int>& vertReturned){
	//set<int> vertReturned;
	std::set<int> faceReturned;

	std::cout << "Before the patch extraction" << std::endl;
	vertices[seed].getPatch(vertices, indices, vertReturned, faceReturned, radius, center);
	std::cout << "After the patch extraction" << std::endl;

	//Chequear que todos los vertices contenidos en faceReturned esten en vertReturned
	std::set<int>::iterator it;
	for(it = faceReturned.begin(); it!=faceReturned.end(); it++){
		int faceInd = *it;
		int ind1 = faces[faceInd].getVertex(0);
		int ind2 = faces[faceInd].getVertex(1);
		int ind3 = faces[faceInd].getVertex(2);

		vertReturned.insert(ind1);
		vertReturned.insert(ind2);
		vertReturned.insert(ind3);
	}

    patch->cleanMesh();

	std::map<int, int> mapping;
	it = vertReturned.begin();

	int i = 0;
	while(it!=vertReturned.end()){
		int ind = *it;
		mapping.insert( std::pair<int, int> (ind, i) );
		patch->insertVertex(i, vertices[ind].x(), vertices[ind].y(), vertices[ind].z());
		it++;
		i++;
	}

	it = faceReturned.begin();
	i = 0;
	while(it!=faceReturned.end()){
		int faceInd = *it;
		int ind1 = faces[faceInd].getVertex(0);
		int ind2 = faces[faceInd].getVertex(1);
		int ind3 = faces[faceInd].getVertex(2);

		patch->insertFace(i, mapping[ind1], mapping[ind2], mapping[ind3]);
		i++;
		it++;
	}
	//cout << "Size:" << vertReturned.size() << " - " << faceReturned.size() << endl;

}

void Mesh::getSpatialPatch(Mesh* patch, Vertex center, double radius){
	std::set<int> vertReturned;
	std::set<int> faceReturned;

    for(int i = 0; i < getNumFaces(); i++){
		std::vector<int> vert = faces[i].getVertices();

		double x1 = vertices[vert[0]].x();
		double y1 = vertices[vert[0]].y();
		double z1 = vertices[vert[0]].z();
		double dist1 = sqrt((x1 - center.x())*(x1 - center.x()) + (y1 - center.y())*(y1 - center.y()) + (z1 - center.z())*(z1 - center.z()));

		double x2 = vertices[vert[1]].x();
		double y2 = vertices[vert[1]].y();
		double z2 = vertices[vert[1]].z();
		double dist2 = sqrt((x2 - center.x())*(x2 - center.x()) + (y2 - center.y())*(y2 - center.y()) + (z2 - center.z())*(z2 - center.z()));

		double x3 = vertices[vert[2]].x();
		double y3 = vertices[vert[2]].y();
		double z3 = vertices[vert[2]].z();
		double dist3 = sqrt((x3 - center.x())*(x3 - center.x()) + (y3 - center.y())*(y3 - center.y()) + (z3 - center.z())*(z3 - center.z()));

		if(dist1 < radius && dist2 < radius && dist3 < radius){
			vertReturned.insert(vert[0]);
			vertReturned.insert(vert[1]);
			vertReturned.insert(vert[2]);
			faceReturned.insert(i);
		}
	}

	std::set<int>::iterator it;
	for(it = faceReturned.begin(); it!=faceReturned.end(); it++){
		int faceInd = *it;
		int ind1 = faces[faceInd].getVertex(0);
		int ind2 = faces[faceInd].getVertex(1);
		int ind3 = faces[faceInd].getVertex(2);

		vertReturned.insert(ind1);
		vertReturned.insert(ind2);
		vertReturned.insert(ind3);
	}

    patch->cleanMesh();

	std::map<int, int> mapping;
	it = vertReturned.begin();

	int i = 0;
	while(it!=vertReturned.end()){
		int ind = *it;
		mapping.insert( std::pair<int, int> (ind, i) );
		patch->insertVertex(i, vertices[ind].x(), vertices[ind].y(), vertices[ind].z());
		it++;
		i++;
	}

	it = faceReturned.begin();
	i = 0;
	while(it!=faceReturned.end()){
		int faceInd = *it;
		int ind1 = faces[faceInd].getVertex(0);
		int ind2 = faces[faceInd].getVertex(1);
		int ind3 = faces[faceInd].getVertex(2);

		patch->insertFace(i, mapping[ind1], mapping[ind2], mapping[ind3]);
		i++;
		it++;
	}
}

double Mesh::getArea(){
	double area = 0.0;

    for(int i = 0; i < getNumFaces(); i++){
		std::vector<int> indices = faces[i].getVertices();
		double x1 = vertices[indices[0]].x();
		double y1 = vertices[indices[0]].y();
		double z1 = vertices[indices[0]].z();

		double x2 = vertices[indices[1]].x();
		double y2 = vertices[indices[1]].y();
		double z2 = vertices[indices[1]].z();

		double x3 = vertices[indices[2]].x();
		double y3 = vertices[indices[2]].y();
		double z3 = vertices[indices[2]].z();

		double det1 = x1*y2 + x2*y3 + x3*y1 - x3*y2 - x2*y1 - x1*y3;
		double det2 = y1*z2 + y2*z3 + y3*z1 - y3*z2 - y2*z1 - y1*z3;
		double det3 = z1*x2 + z2*x3 + z3*x1 - z3*x2 - z2*x1 - z1*x3;

		area = area + 0.5*sqrt(det1*det1 + det2*det2 + det3*det3);
	}

	return area;
}

std::ostream& operator<<(std::ostream& os, Mesh& obj){
     os<<"OFF"<<std::endl;
     os<<obj.getNumVertices()<<" "<<obj.getNumFaces()<<" "<<0<<std::endl;
     for(int i = 0; i<obj.getNumVertices(); i++){
         os<<obj.vertices[i].getX()<<" "<<obj.vertices[i].getY()<<" "<<obj.vertices[i].getZ()<<std::endl;
     }
     for(int i = 0; i < obj.getNumFaces(); i++){
            os<<3<<" ";
            std::vector<int> vert = obj.faces[i].getVertices();
            for(size_t j = 0; j < (vert.size() - 1); j++){
                os<<vert[j]<<" ";
            }
            os<<vert[vert.size()-1]<<std::endl;
     }
     return os;
}

}
