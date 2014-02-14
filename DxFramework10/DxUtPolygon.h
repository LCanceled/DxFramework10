
#ifndef DXUTPOLYGON
#define DXUTPOLYGON

#include "DxUtInclude.h"
#include "DxUtTriangle.h"
#include "DxUtArray.h"

namespace DxUt {

template <typename TVec, typename TLine>
struct SLine {
	TVec e1;
	TVec e2;
	DWORD dwEdge;
public:
	SLine() {}
	SLine(const TVec & _e1, const TVec & _e2):e1(_e1), e2(_e2),dwEdge(0) {}
	//~SLine() {}

	virtual bool Intersect(TLine & line, TVec & intersection, double dT)=0;

	void operator()(TVec & _e1, TVec & _e2) {
		e1 = _e1;
		e2 = _e2;
	}
};

struct SLine2F : public SLine<Vector2F, SLine2F> {
	SLine2F() {}
	SLine2F(const Vector2F & _e1, const Vector2F & _e2):SLine(_e1,_e2) {}
	bool Intersect(SLine2F & line, Vector2F & intersection, double dT);
};

struct SLine3F : public SLine<Vector3F, SLine3F> {
	SLine3F() {}
	SLine3F(const Vector3F & _e1, const Vector3F & _e2):SLine(_e1, _e2) {}
	//Finds the closest point of the *this line to l
	bool Intersect(SLine3F & line, Vector3F & intersection, double dT);
};

template <typename TPolygonEdge, typename TLine, typename TVector, typename TScalar>
struct SPolygon {
	//typename typedef std::list<PolyEdge >::iterator EdgeIterator;
	CArray<TPolygonEdge> rgEdge;
	DWORD dwPoly;

	template <typename _TScalar>
	_TScalar ComputeAreaOfPolygon(CArray<TPolygonEdge> & _rgEdge) {
		int N = _rgEdge.GetSize();
		TPolygonEdge * pEdge = _rgEdge.GetData();
		//Paul Bourke
		int i;
		_TScalar areaVec[3] = {0,0,0};

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
		return (_TScalar)sqrt(areaVec[0]*areaVec[0] + areaVec[1]*areaVec[1] + areaVec[2]*areaVec[2]);
	}
	template <typename _TScalar>
	TVector ComputeNormalOfPolygon(CArray<TPolygonEdge> & _rgEdge) {
		int N = _rgEdge.GetSize();
		TPolygonEdge * pEdge = _rgEdge.GetData();
		//Paul Bourke
		int i;
		_TScalar areaVec[3] = {0,0,0};

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
		_TScalar div = 1.f/sqrt(areaVec[0]*areaVec[0] + areaVec[1]*areaVec[1] + areaVec[2]*areaVec[2]);
		return TVector((TScalar)(areaVec[0]*div), (TScalar)(areaVec[1]*div), (TScalar)(areaVec[2]*div));
	}
public:
	SPolygon() {}
	SPolygon(DWORD _dwPoly):dwPoly(_dwPoly) {}
	~SPolygon() {rgEdge.Clear(); }

	//Clockwise clipping of the polygon cw
	//Returns true if the line did intersect the polygon
	//virtual bool ConvexClipAgainstLine(TLine & l, bool bCW=1)=0;

	//Does not handle cases of overlap or intersection
	virtual TScalar ComputeArea() {
		return ComputeAreaOfPolygon<TScalar>(rgEdge);
	}
};

template <typename TPolygonEdge>
struct SPolygon2F : public SPolygon<TPolygonEdge, SLine2F, Vector2F, float> {
public:
	SPolygon2F() {}

	virtual float ComputeArea() {Assert(0, "SPolygon2F::ComputeArea not implemented."); }
};

//2d polygon in 3d space
template <typename TPolygonEdge>
struct SPolygon3F : public SPolygon<TPolygonEdge, SLine3F, Vector3F, float> {
public:
	Vector3F normal;
public:
	SPolygon3F() {}
	SPolygon3F(Vector3F & _normal, DWORD _dwPoly):normal(_normal),SPolygon(_dwPoly) {}

	/* Compute a polygon based upon a list of disorded edges */
	/* The first edge in rgEdgeDisordered will determine the direction of successive edges */
	void ComputePolygon(CArray<TPolygonEdge> & rgEdgeDisordered) {
		DWORD nEdges = rgEdgeDisordered.GetSize();
		bool * rgUsedEdge = new bool[nEdges];
		memset(rgUsedEdge, 0, sizeof(bool)*nEdges);
	
		rgEdge.Resize(0);
		rgEdge.Reserve(nEdges);
		SLine3F * pEdgesDisordered = rgEdgeDisordered.GetData();
		rgEdge.PushBack(pEdgesDisordered[0]);
		rgUsedEdge[0] = 1;
		for (DWORD i=0; i<nEdges; i++) {
			SLine3F & connection = rgEdge.GetBack();
			for (DWORD j=0; j<nEdges; j++) {
				if (!rgUsedEdge[j]) {
					if (pEdgesDisordered[j].e1 == connection.e2) {
						rgUsedEdge[j] = 1;
						rgEdge.PushBack(TPolygonEdge(pEdgesDisordered[j].e1, pEdgesDisordered[j].e2));
					}
					if (pEdgesDisordered[j].e2 == connection.e2) {
						rgUsedEdge[j] = 1;
						rgEdge.PushBack(TPolygonEdge(pEdgesDisordered[j].e2, pEdgesDisordered[j].e1));
					}
				}
			}
		}
		delete[] rgUsedEdge;
	}


	virtual float ComputeArea() {
		return (float)ComputeAreaOfPolygon<double>(rgEdge);
	}
	virtual Vector3F ComputeNormal() {
		return ComputeNormalOfPolygon<double>(rgEdge);
	}
};


};


#endif










/*
template <typename TPolygon, typename TVector, typename TScalar>
struct SPolyhedron {
	std::vector<TPolygon> polys;
public:
	//Compute the area weighted normal for the polyhedron
	virtual TVector ComputeAreaNormal() {
		TVector area(0);
		TPolygon * rgPolygon = polys.data();
		for (DWORD i=0, end=polys.size(); i<end; i++) {
			area += (rgPolygon[i]->ComputeArea())*rgPolygon[i]->normal;
		}
		return area;
	}
};

struct SPolyhedron3F : public SPolyhedron<SPolygon3F*, Vector3F, float> {
};*/