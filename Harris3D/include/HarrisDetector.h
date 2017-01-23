#ifndef __HARRISDETECTOR_H_
#define __HARRISDETECTOR_H_

#include "Mesh.h"
#include "Vertex.h"
#include "Properties.h"

namespace Harris3D{

class HarrisDetector{
	private:
		Mesh* object;
		Util::Properties* prop;
		
		int typeNeighborhood;
		double fractionDiagonal;
		int numberRingNeighbor;
		
		double k;
		
		int numberRingsDetection;
		
		int typeSelection;
		double paramSelection;
		int numberKeypoints;
		
		int filteringSteps;
		
		void processOptions();
		
	public:
		enum _type_neighborhood{SPATIAL, ADAPTIVE, RINGS};
		enum _type_selection{FRACTION, CLUSTERING, NUMBER};
		
		HarrisDetector();
		HarrisDetector(Mesh* obj, Util::Properties* pr);
		virtual ~HarrisDetector() {}
		
		inline void setMesh(Mesh* obj) { object = obj;}
		inline void setProperties(Util::Properties* pr) { prop = pr; processOptions();}
		
		void showOptions();
		
		void detectInterestPoints(std::vector<int>& interestPoints);
		
};
}
#endif
