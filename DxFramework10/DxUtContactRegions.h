
#ifndef DXUTCONTACTREGIONS_H
#define DXUTCONTACTREGIONS_H

#include "DxUtInclude.h"
#include "DxUtOBBox.h"
#include "DxUtTriangle.h"
#include "DxUtPolygon.h"

namespace DxUt {

class CContactRegions {
private:
public:
	struct SPolygonEdge {
		SPolygonEdge():bConvex(0), bCoplanar(0) {}
		SPolygonEdge(SLine3F & _edge, SPolygonEdge & cpyEdge):edge(_edge)
			{ 
				bConvex = cpyEdge.bConvex;
				bCoplanar = cpyEdge.bCoplanar;
				dwPoly = cpyEdge.dwPoly;
				coneDir = cpyEdge.coneDir;
				coneTheta = cpyEdge.coneTheta;
				
				//memcpy((char*)this+sizeof(SLine3F), (char*)(&cpyEdge)+sizeof(SLine3F), sizeof(SPolygonEdge)-sizeof(SLine3F));
		}
		SLine3F edge;
		bool bConvex;
		bool bCoplanar;
		/* Indices less that 0 correspond to edge indices */
		int dwPoly;

		/* Specify the admissible normal region */
		Vector3F coneDir;
		float coneTheta;
	};

	struct SPolygonBisection {
		unsigned char dwEdgeStart;
		unsigned char dwEdgeEnd;

		Vector3F intersectionStart;
		Vector3F intersectionEnd;
	};

	struct SPolygon3FEx : public SPolygon3F<SPolygonEdge> {
		CArray<SPolygonEdge> rgBoundaryEdges;

		bool bVisited			: 1;
		bool bConvex			: 1;
		SPolygonBisection bisection[2];
		DWORD nBisectors : 2;
	
		DWORD dwCoplanarPolygon;

		SPolygon3FEx():bVisited(0),bConvex(0),nBisectors(0) {}
		SPolygon3FEx(SPolygon3FEx & cpy):bVisited(0),bConvex(0),nBisectors(0) {}//Assert(0, "SPolygon3FEx::Spolygon3FEx copy constructor not implemented."); }
		SPolygon3FEx(STriangleF & tri):bVisited(0),bConvex(0),nBisectors(0) {
			DebugBreak();
			rgEdge.Resize(3);
			rgEdge[0].edge(tri.vPosW[0], tri.vPosW[1]);
			rgEdge[1].edge(tri.vPosW[1], tri.vPosW[2]);
			rgEdge[2].edge(tri.vPosW[2], tri.vPosW[0]);
			dwPoly = tri.dwTriangle;
		}
		void Init(STriangleF & tri, STriangleF & tri1, STriangleF & tri2,
			STriangleF & tri3, DWORD dwE0, DWORD dwE1, DWORD dwE2,
			float fTheta0, Vector3F & nor0,
			float fTheta1, Vector3F & nor1,
			float fTheta2, Vector3F & nor2) {
			rgEdge.Resize(3);
			rgEdge[0].edge(tri.vPosW[0], tri.vPosW[1]);
			rgEdge[0].dwPoly = dwE0;
			rgEdge[0].bCoplanar = AreTrianglesCoplanar(tri, tri1);
			rgEdge[0].bConvex = IsTriangleEdgeConvex(tri, tri1, rgEdge[0].edge, rgEdge[0].bCoplanar);
			rgEdge[0].coneDir = nor0;
			rgEdge[0].coneTheta = fTheta0;

			rgEdge[1].edge(tri.vPosW[1], tri.vPosW[2]);
			rgEdge[1].dwPoly = dwE1;
			rgEdge[1].bCoplanar = AreTrianglesCoplanar(tri, tri2);
			rgEdge[1].bConvex = IsTriangleEdgeConvex(tri, tri2, rgEdge[1].edge, rgEdge[1].bCoplanar);
			rgEdge[1].coneDir = nor1;
			rgEdge[1].coneTheta = fTheta1;

			rgEdge[2].edge(tri.vPosW[2], tri.vPosW[0]);
			rgEdge[2].dwPoly = dwE2;
			rgEdge[2].bCoplanar = AreTrianglesCoplanar(tri, tri3);
			rgEdge[2].bConvex = IsTriangleEdgeConvex(tri, tri3, rgEdge[2].edge, rgEdge[2].bCoplanar);
			rgEdge[2].coneDir = nor2;
			rgEdge[2].coneTheta = fTheta2;

			dwPoly = tri.dwTriangle;
			normal = tri.Normal();
		}
		void TransformPolygon(Matrix4x4F & T) {
			SPolygonEdge * _rgEdge = rgEdge.GetData();

			Vector3F e0(T * _rgEdge[0].edge.e1);
			Vector3F e1(T * _rgEdge[1].edge.e1);
			Vector3F e2(T * _rgEdge[2].edge.e1);
			rgBoundaryEdges.PushBack(SPolygonEdge(SLine3F(e0, e1), _rgEdge[0]));
			rgBoundaryEdges.PushBack(SPolygonEdge(SLine3F(e1, e2), _rgEdge[1]));
			rgBoundaryEdges.PushBack(SPolygonEdge(SLine3F(e2, e0), _rgEdge[2]));
		}
		bool AreTrianglesCoplanar(STriangleF & tri, STriangleF & tri1) {
			Vector3F & n0 = tri.Normal();
			Vector3F & n1 = tri1.Normal();
			return DotXYZ(n0, n1) > .999f;
		}
		bool IsTriangleEdgeConvex(STriangleF & tri, STriangleF & tri1, SLine3F & edge, bool bCoplanar) {
			Vector3F & n0 = tri.Normal();
			Vector3F & n1 = tri1.Normal();
			Vector3F n12(CrossXYZ(n0, n1));
			Vector3F dir(edge.e2 - edge.e1);
			return bCoplanar ? 1 : DotXYZ(n12, dir) > 0;
		}
	};

	struct SCoplanarPolygon3F : public SPolygon3F<SPolygonEdge> {
		SCoplanarPolygon3F():bVisited(0) {}
		SCoplanarPolygon3F(SCoplanarPolygon3F & cpy):bVisited(0) {}
		//~SCoplanarPolygon3F() {}
		Vector3F areaNormal;
		Vector3F avgNormal;
		bool bVisited;
	};
	
	struct SContactRegionVertex {
		SContactRegionVertex() {}
		~SContactRegionVertex() {}

		Vector3F v;
		SPolygon3FEx * pPolygon;
		/* First bit is 1 if there is a change in the iline vector for polygon[0] */
		/* Second bit is 1 if there is a change in the iline vector for polygon[1] */
		/* Third bit is 1 if the prior vertex and this one share the same polygon[0] */
		/* Fourth bit is 1 if the prior vertex and this one share the same polygon[1] */
		unsigned char rgFaceFlag[2];
	};
	struct SNormalRegion {

	};
private:
	CArray<SPolygon3FEx*> m_rgInteriorPolygon[2];
	CArray<SPolygon3FEx*> * m_pInteriorPolygon;

	CArray<SCoplanarPolygon3F> m_rgCoplanarPolygon[2];
	CArray<SCoplanarPolygon3F> * m_pCoplanarPolygon;

	CArray<SContactRegionVertex> m_rgContactRegionVertex[2];
	CArray<SContactRegionVertex> * m_pContactRegionVertex;

	/* Caller supplied variables */
	CArray<SContactPoint> * m_rgCP;
	CArray<SPolygon3FEx> * m_rgPolygon[2];
	CArray<SPolygon3FEx> * m_pPolygon;
	Matrix4x4F * m_pTriTransform;

	DWORD dwCounter;

	bool ArePolygonsPlanar(Vector3F & n0, Vector3F & n1) {
		return DotXYZ(n0, n1) > COPLANAR_EPS;
	}

	bool BisectEdgePolygon(SPolygon3FEx * pInnerPolygon, 
		SPolygon3FEx * pOuterPolygon, SLine3F * pVertexEdge, unsigned char & dwEdge, Vector3F &intersection);

	void ComputeNormals();
	void ComputeCoplanarPolygons();

	void ComputeCPsInterior(SCoplanarPolygon3F * pPolygon);
	void ComputeCPsBoundary(SCoplanarPolygon3F * pPolygon);

	void ResetInteriorPolygons();
	void FormInteriorPolygons(SPolygon3FEx * pPolygon);
	bool FormCoplanarPolygons(SPolygon3FEx * pPolygon, int dwBoundaryEdge, int dwIniBoundaryEdgeValue);

	void AddConvexEdgeToContactRegion(SCoplanarPolygon3F * pPolygon, SPolygonEdge * pEdge);
	void AddConcaveEdgeToContactRegion(SCoplanarPolygon3F * pPolygon, SPolygonEdge * pEdge, bool bPreEdge, bool bNexEdge);

	const double COPLANAR_EPS;

	bool IsCPValid(Vector3F & pos, Vector3F & nor);
	void AddValidContactPoint(Vector3F & pos, Vector3F & coneDir, float coneTheta,
		bool bVertex, Vector3F & edgeDir, CArray<SCoplanarPolygon3F> * pOtherPolygons);
	void ComputeEdges(SCoplanarPolygon3F * pPolygon, CArray<SCoplanarPolygon3F> * pOtherPolygons);

	void ResetAll();
public:
	CContactRegions():COPLANAR_EPS(.999),dwCounter(0) {}
	//~CContactRegions() {}

	/* WARNING: pPolygon must not go out of scope. */
	void PushVertex(Vector3F & _v, SPolygon3FEx * pPolygon, unsigned char * rgFaceFlag, DWORD dwBody) {
		m_rgContactRegionVertex[dwBody].PushBack();
		SContactRegionVertex & v = m_rgContactRegionVertex[dwBody].GetBack();
		v.v = _v;
		v.pPolygon = pPolygon;
		v.rgFaceFlag[0] = rgFaceFlag[0];
		v.rgFaceFlag[1] = rgFaceFlag[1];
	}
	void PushVertexForward(Vector3F & _v, SPolygon3FEx * pPolygon, unsigned char * rgFaceFlag, DWORD dwBody) {
		SContactRegionVertex v;
		v.v = _v;
		v.pPolygon = pPolygon;
		v.rgFaceFlag[0] = rgFaceFlag[0];
		v.rgFaceFlag[1] = rgFaceFlag[1];
		m_rgContactRegionVertex[dwBody].PushForward(v);
	}
	bool IsValid() {return m_rgContactRegionVertex[0].GetSize() > 2; }
	
	void Reset() {m_rgContactRegionVertex[0].Resize(0); m_rgContactRegionVertex[1].Resize(0);}

	bool FormContactRegion(DWORD dwBody, DWORD nFaces, CArray<SPolygon3FEx> * _rgPolygon, Matrix4x4F & T);
	//bool FormContactRegion(DWORD * nFaces, CArray<SContactPoint> * _rgCP, CArray<SPolygon3FEx> * _rgPolygon, Matrix4x4F * T);
	bool FormContactPoints(DWORD dwBody, CArray<SContactPoint> * rgCP);

	//void Reverse() {
	//	m_rgContactRegionVertex[1].Reverse();
	//}

	void FormContactPointsSD(CArray<SContactPoint> * rgCP);
};


};


#endif









/*bool ClipPolygonBegin(DWORD dwVertex, DWORD dwBodyIndex, SPolygon3FEx * pPrePolygon);
	bool ClipPolygonEnd(DWORD dwVertex, DWORD dwBodyIndex, SPolygon3FEx * pPrePolygon);

	bool ArePolygonsPlanar(DWORD dw0, DWORD dw1) {
		Vector3F & n0 = m_rgBVPolygon.data()[dw0].normal;
		Vector3F & n1 = m_rgBVPolygon.data()[dw1].normal;
		return DotXYZ(n0, n1) > .999;
	}
	bool IsPolygonEdgeConvex(Vector3F & n0, Vector3F & n1, SLine3F & edge) {
		Vector3F n12(CrossXYZ(n0, n1));
		Vector3F dir(edge.e2 - edge.e1);
		return DotXYZ(n0, n1) > .999 || DotXYZ(n12, dir) > 0; /* Handle the case for coplanar 
	}
	bool IsPolygonEdgeConvex(DWORD dw0, DWORD dw1, SLine3F & edge) {
		Vector3F & n0 = m_rgBVPolygon.data()[dw0].normal;
		Vector3F & n1 = m_rgBVPolygon.data()[dw1].normal;
		Vector3F n12(CrossXYZ(n0, n1));
		Vector3F dir(edge.e2 - edge.e1);
		return DotXYZ(n0, n1) > .999 || DotXYZ(n12, dir) > 0; /* Handle the case for coplanar 
	}
	bool IsPolygonLocallyConvex(SPolygon3FEx * pPolygon) {
		bool bLocallyConvex = 1;
		for (DWORD i=0, end=pPolygon->boundaryEdges.size(); i<end; i++) {
			bLocallyConvex &= (pPolygon->boundaryEdges[i].index == -1 || 
				IsPolygonEdgeConvex(pPolygon->polyIndex, pPolygon->boundaryEdges[i].index, pPolygon->boundaryEdges[i].edge));
		}
		return bLocallyConvex;
	}*/
	//bool FormPolyhedrons(SPolygon3FEx * pPolygon);
	//DWORD m_dwIniBoundaryPolygon;
	//DWORD m_dwIniBoundaryEdgePolygon;
	//void ComputeCPPolyhedronInterior(SPolyhedron3FEx * pPolyhedron);
	//void ComputeCPPolyhedronBoundary(SPolyhedron3FEx * pPolyhedron);

	/*struct SPolyhedron3FEx : public SPolyhedron<SPolygon3FEx*, Vector3F, float> {
		Vector3F areaNormal; 
		Vector3F avgNormal;
		bool convex;
		bool visited;

		SPolyhedron3FEx():visited(0),areaNormal(0),avgNormal(0) {}
	};*/