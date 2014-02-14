
#ifndef DXUTTRIANGLE_H
#define DXUTTRIANGLE_H

#include "DxUtVector.h"
#include "DxUtPlane.h"
#include "DxUtRay.h"

namespace DxUt {

struct STriangleF {
	STriangleF():dwTriangle(0) {}
	STriangleF(Vector3F & v1, Vector3F & v2, Vector3F & v3):dwTriangle(0) {
		vPosW[0] = v1; vPosW[1] = v2; vPosW[2] = v3; 
	}
	//~STriangleF() {}

	Vector3F vPosW[3];
	DWORD dwTriangle;

	//vPosW = Matrix4x4 * vPosW
	STriangleF & Transform(Matrix4x4F & m);
	STriangleF & operator=(STriangleF & tri); 

	FLOAT Area() {
		Vector3F v1(vPosW[1] - vPosW[0]);
		Vector3F v2(vPosW[2] - vPosW[0]);
		return .5f*Vector3F(CrossXYZ(v1, v2)).Length();
	}
	Vector3F Centroid() {
		Vector3F v(vPosW[0]+vPosW[1]+vPosW[2]);
		return .333333f*v;
	}
	Vector3F Normal() {
		Vector3F v1(vPosW[1] - vPosW[0]);
		Vector3F v2(vPosW[2] - vPosW[0]);
		return Vector3F(CrossXYZ(v1,v2)).Normalize();
	}
	Vector3F NormalSq() {
		Vector3F v1(vPosW[1] - vPosW[0]);
		Vector3F v2(vPosW[2] - vPosW[0]);
		return Vector3F(CrossXYZ(v1,v2));
	}
	PlaneF NormalPlane() {
		return PlaneF(vPosW[0], vPosW[1], vPosW[2]);
	}

	/* Compute radius of sphere at the centroid enclosing the vertices */
	float CentroidRadius() {
		Vector3F ctd(Centroid());
		float l0 = (vPosW[0] - ctd).LengthSq();
		float l1 = (vPosW[1] - ctd).LengthSq();
		float l2 = (vPosW[2] - ctd).LengthSq();
		float l = max(l0, l1);
		l = max(l, l2);
		return sqrtf(l);
	}

	/* Compute the maximum of the edge lengths */
	float MaxEdgeLength() {
		float l0 = (vPosW[1] - vPosW[0]).LengthSq();
		float l1 = (vPosW[2] - vPosW[1]).LengthSq();
		float l2 = (vPosW[0] - vPosW[2]).LengthSq();
		float l = max(l0, l1);
		l = max(l, l2);
		return sqrtf(l);
	}
};

//void ExtractVerticesFromTriangles();
//void ExtractEdgesFromTriangles();

bool TriSegmentIntersect(STriangleF & tri,Vector3F & e1, Vector3F & e2, Vector3F & iPos, float & t, Vector3F & norSq, float & u, float & v, float & w);
bool TriRayIntersect(STriangleF & tri, Vector3F & pos, Vector3F & vec, Vector3F & iPos, float & t, Vector3F & norSq, float & u, float & v, float & w);
bool TriRayIntersect(STriangleF & tri, SRay & ray, SRayIntersectData & data);
bool TriPlannarLineIntersect(STriangleF & tri, Vector3F & e1, Vector3F & e2);

/* The first 2 bits of dwType store the type of the closest feature
 * 0 = Face closest feature
 * 1 = Edge closest feature
 * 2 = Vertex closest feature
 * The next 2 store the index of the edge or vertex that is closest
*/
Vector3F ComputeClosestPoint(STriangleF & tri, Vector3F & pt, DWORD & dwType);

/* Divide a triangle into 4 subtris based upon loop subdivision */
void SubdivideTriangle(STriangleF & tri, STriangleF * rgSubdividedTri);

/* Triangle-Triangle Intersection */
struct STriTriIntersectData {
	/* The first 4 bits store the type of intersection:
	 * 0 = first in second 
	 * 1 = second in first
	 * 2 = partial containment
	 * 3 = coplanar
	 * The next 2 bits block is the edge index for iPos[0]
	 * The next 2 bits block after that is the edge index for iPos[1]
	 */
	DWORD dwType;
	Vector3F iPos[2];
	Vector3F iLine;
	/* The squared normals */
	Vector3F normals[2];
};

//Description of a contact point
struct SContactPoint {
	SContactPoint():dist(0) {}
	union {
		STriangleF * pTri[2];
		DWORD dwFaceIndex[2];
	};
	Vector3F iPos;
	Vector3F iNor;
	float dist;
};

/* Note the neither dwFaceIndex or pTri will be filled out by this function */
bool TriTriIntersect(STriangleF & t1, STriangleF & t2,  STriTriIntersectData & triData);
//bool TriTriIntersect(STriangleF & t1, STriangleF & t2,  SCollisionPair & cData);

};


#endif

