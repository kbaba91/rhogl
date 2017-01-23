#include "Vertex.h"
#include "util.h"
#include <iostream>
#include <cmath>
#include <map>

namespace Harris3D{

void Vertex::getNeighborhood(int rad, std::vector<Vertex*>& V, std::vector<Vertex>& vertices){
	std::queue<Vertex*> Q;
	std::vector<Vertex*> marked;

	Q.push(this);
	this->setMark(true);
	this->setDepth(0);
	marked.push_back(this);

	while(!Q.empty()){
		Vertex* v0 = Q.front();
		Q.pop();
		V.push_back(new Vertex(v0->getX(), v0->getY(), v0->getZ())); //Indeed, copy vertex information rather than return the same vertex

		int dep = v0->getDepth();
		if(dep <= rad){
			std::set<int> listVertices = v0->getAdjacentVertices();
			std::set<int> :: iterator it;
			for(it = listVertices.begin(); it!=listVertices.end(); it++){
				Vertex* v1 = &vertices[*it];
                if(!v1->isMarked()){
                    Q.push(v1);
                    v1->setMark(true);
                    v1->setDepth(dep + 1);
                    marked.push_back(v1);
                }
            }
        }
	}

	std::vector<Vertex*>::iterator ini = marked.begin();

	while(ini<marked.end()){
		(*ini)->setMark(false);
		(*ini)->setDepth(0);
		ini++;
	}
}

bool Vertex :: hasBlueNeighbor(std::vector<Vertex>& vertices){
    bool aux = false;
    std::set<int> listVertices = this->getAdjacentVertices();
    std::set<int> :: iterator it;
    for(it = listVertices.begin(); it!=listVertices.end(); it++){
        Vertex* v1 = &vertices[*it];
        if(v1->getBlue()){
           aux = true;
           break;
        }
    }
    return aux;
}

int Vertex :: getRadius(std::vector<Vertex>& vertices, double radius, std::vector<Vertex*>& V){
	std::vector<Vertex*> marked; //Store the marked vertices
	std::map<int, double> distances; //Store the distances relatives to the current vertex
	std::map<int, Vertex*> markedRing; //Elements in a new ring
	std::queue<Vertex*> Q;
	double maxDistance = 0.0;
	int rad = -1;

	Q.push(this);
	this->setMark(true);
	markedRing.insert(std::pair<int, Vertex*>(this->index, this));

	distances[this->index] = 0.0;

	while(!Q.empty()){
		Vertex* v0 = Q.front();
		Q.pop();

		int dep = v0->getDepth();
		if(dep != rad){ //First vertex in the new ring
			std::map<int, Vertex*>::iterator it;
			double max = 0.0;

			//Mark the previous ring
			for(it = markedRing.begin(); it!=markedRing.end(); it++){
				Vertex* mar = (*it).second;
				mar->setMark(true);
				marked.push_back(mar);
				V.push_back(new Vertex(mar->getX(), mar->getY(), mar->getZ()));
				if(distances[(*it).first] > max)
					max = distances[(*it).first];
			}

			rad++;
			markedRing.clear();
			maxDistance = max;
			if(maxDistance > radius)
				break;
		}

		std::set<int> listVertices = v0->getAdjacentVertices();
		std::set<int> :: iterator it;

		for(it = listVertices.begin(); it!=listVertices.end(); it++){
				Vertex* v1 = &vertices[*it];
				if(!v1->isMarked()){
					if(distances[v1->getIndex()] == 0.0){ //Distance is not set
						Q.push(v1);
						v1->setDepth(dep + 1);
					}
					markedRing.insert(std::pair<int, Vertex*>(v1->getIndex(), v1));
					double dist = v0->distanceL2(v1);
					double newDistance = distances[v0->getIndex()] + dist;
					if(distances[v1->getIndex()] == 0.0){ //First time on this vertex
						distances[v1->getIndex()] = newDistance;
					}else if(newDistance  < distances[v1->getIndex()]){
						distances[v1->getIndex()] = newDistance;
					}
				}
			}
		//}
	}

	if(!markedRing.empty()){
			std::map<int, Vertex*>::iterator it;
			double max = 0.0;


			for(it = markedRing.begin(); it!=markedRing.end(); it++){
				Vertex* mar = (*it).second;
				mar->setMark(true);
				marked.push_back(mar);
				V.push_back(new Vertex(mar->getX(), mar->getY(), mar->getZ()));
				if(distances[(*it).first] > max)
					max = distances[(*it).first];
			}

			rad++;
			markedRing.clear();
			maxDistance = max;

	}

	//Unmark all vertices
	std::vector<Vertex*>::iterator ini = marked.begin();
	while(ini  < marked.end()){
		(*ini)->setMark(false);
		(*ini)->setDepth(0);
		ini++;
	}
	return rad;
}

void Vertex::processMaximum(std::vector<Vertex>& vertices, int numRings){
		std::set<int> :: iterator it;
		for(it = adjacentVertices.begin(); it!=adjacentVertices.end(); it++){
			Vertex* v1 = &vertices[*it];
			if(v1!=this){
				if(response < v1->getResponse())
					return;
			}
		}
	isInterest = true;
}

void Vertex :: getPatch(std::vector<Vertex>& vertices, std::vector<int> indices, std::set<int>& returned, std::set<int>& faceR, double radius, Vertex center){
	std::set<int> waiting;
	std::queue<int> visited;
	visited.push(this->index); //Este vertice va a la cola

	waiting.insert(indices.begin(), indices.end());
	waiting.erase(this->index); // Eliminamos este vertice del conjunto de faltantes

	while(!waiting.empty() && !visited.empty()){
		int ind = visited.front();
		visited.pop();

		if(!vertices[ind].isMarked()){
			returned.insert(ind);
			vertices[ind].setMark(true);
			waiting.erase(ind);

			std::set<int> listVertices = vertices[ind].getAdjacentVertices();
			std::set<int> :: iterator it;

			for(it = listVertices.begin(); it!=listVertices.end(); it++){
				int ind = *it;
				if(!vertices[ind].isMarked()){
					double distX = vertices[ind].x() - center.x();
					double distY = vertices[ind].y() - center.y();
					double distZ = vertices[ind].z() - center.z();
					double dist = sqrt(distX*distX + distY*distY + distZ*distZ);
					if(dist < radius)
						visited.push(*it);
				}
			}

			std::vector<int> fac = vertices[ind].getFaces();
			std::vector<int>::iterator it1;
			for(it1 = fac.begin(); it1!=fac.end(); it1++)
				faceR.insert(*it1);
		}

	}

	std::set<int>::iterator it2;
	for(it2 = returned.begin(); it2!=returned.end(); it2++)
		vertices[*it2].setMark(false);

}

std::ostream& operator<<(std::ostream& out, Vertex& point){
	out << point.x() <<" "<<point.y()<<" "<<point.z()<<std::endl;

	return out;
}

double Vertex::distanceL2(Vertex* v1){
    double val = (x()-v1->x())*(x()-v1->x()) + (y()-v1->y())*(y()-v1->y()) + (z()-v1->z())*(z()-v1->z());
    return sqrt(val);
}

void Vertex::computeNormal(std::vector<Face>& arr){
    normal[0] = 0.0;
    normal[1] = 0.0;
    normal[2] = 0.0;
    for (int i = 0; i<this->faces.size(); i++){
        normal[0] += arr[faces.at(i)].normal[0];
        normal[1] += arr[faces.at(i)].normal[1];
        normal[2] += arr[faces.at(i)].normal[2];
    }
    normal[0] = normal[0] / (double)this->faces.size();
    normal[1] = normal[1] / (double)this->faces.size();
    normal[2] = normal[2] / (double)this->faces.size();
}
}
