
#ifndef DXUTPOLYGON
#define DXUTPOLYGON

#include "DxUtInclude.h"
#include "DxUtTriangle.h"
#include "DxUtArray.h"

namespace DxUt {

//TODO: Replae with generic structures

struct SSegment2F {
	Vector2F e1;
	Vector2F e2;
	UINT uiEdge;

	SSegment2F() {}
	SSegment2F(const Vector2F & _e1, const Vector2F & _e2):SSegment2F(_e1,_e2) {}
	bool Intersect(SSegment2F & line, Vector2F & intersection, double t);
};

struct SSegment3F {
	Vector3F e1;
	Vector3F e2;
	UINT uiEdge;

	SSegment3F() {}
	SSegment3F(const Vector3F & _e1, const Vector3F & _e2):e1(_e1), e2(_e2) {}
	//Finds the closest point of the *this line to l
	bool Intersect(SSegment3F & line, Vector3F & intersection, double t);
};

//2d polygon in 3d space
struct SPolygon3F {
public:
	Vector3F normal;

	CArray<SSegment3F> edges;
	UINT uiPoly;
public:
	SPolygon3F() {}
	SPolygon3F(Vector3F & _normal, UINT _uiPoly):normal(_normal),uiPoly(_uiPoly) {}
	~SPolygon3F() {edges.Clear(); }


	void ComputeAreaAndNormal(float & area, Vector3F & normal) {
		int N = edges.GetSize();
		SSegment3F * pEdge = edges.GetData();
		//Paul Bourke
		int i;
		float areaVec[3] = {0,0,0};

		for (i=0;i<N;i++) {
			areaVec[2] += pEdge->e1.x * pEdge->e2.y;
			areaVec[2] -= pEdge->e1.y * pEdge->e2.x;
	
			areaVec[1] += pEdge->e1.z * pEdge->e2.x;
			areaVec[1] -= pEdge->e1.x * pEdge->e2.z;

			areaVec[0] += pEdge->e1.y * pEdge->e2.z;
			areaVec[0] -= pEdge->e1.z * pEdge->e2.y;
			pEdge++;
		}

		areaVec[0] *= .5f;
		areaVec[1] *= .5f;
		areaVec[2] *= .5f;
		area = sqrt(areaVec[0]*areaVec[0] + areaVec[1]*areaVec[1] + areaVec[2]*areaVec[2]);
		float div = 1.f/area;
		normal = Vector3F((float)(areaVec[0]*div), (float)(areaVec[1]*div), (float)(areaVec[2]*div));
	}

	//Does not handle cases of overlap or intersection
	float ComputeArea() {
		float area;
		Vector3F normal;
		ComputeAreaAndNormal(area, normal);
		return area;
	}

	//Does not handle cases of overlap or intersection
	/*Vector3F ComputeNormal(CArray<SSegment3F> & _rgEdge) {
		float area;
		Vector3F normal;
		ComputeAreaAndNormal(area, normal);
		return normal;
	}*/

	/* Compute a polygon based upon a list of disorded edges */
	/* The first edge in edgeDisordereds will determine the direction of successive edges */
	void ComputePolygon(CArray<SSegment3F> & edgeDisordereds) {
		UINT nEdges = edgeDisordereds.GetSize();
		bool * usedEdges = new bool[nEdges];
		memset(usedEdges, 0, sizeof(bool)*nEdges);
	
		edges.Resize(0);
		edges.Reserve(nEdges);
		SSegment3F * pEdgesDisordered = edgeDisordereds.GetData();
		edges.PushBack(pEdgesDisordered[0]);
		usedEdges[0] = 1;
		// TODO: Optimize 
		for (UINT i=0; i<nEdges; i++) {
			SSegment3F & connection = edges.GetBack();
			for (UINT j=0; j<nEdges; j++) {
				if (!usedEdges[j]) {
					if (pEdgesDisordered[j].e1 == connection.e2) {
						usedEdges[j] = 1;
						edges.PushBack(SSegment3F(pEdgesDisordered[j].e1, pEdgesDisordered[j].e2));
					}
					if (pEdgesDisordered[j].e2 == connection.e2) {
						usedEdges[j] = 1;
						edges.PushBack(SSegment3F(pEdgesDisordered[j].e2, pEdgesDisordered[j].e1));
					}
				}
			}
		}
		delete[] usedEdges;
	}

};


};


#endif
