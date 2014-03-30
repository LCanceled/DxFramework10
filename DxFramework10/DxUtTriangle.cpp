
#include "DxUtTriangle.h"
#include "DxUtMatrix.h"
#include "TriTriTest.h"

#define Abs(x) ((x < 0) ? (-x) : x)
#define Cross3D(a, b) (Vector3F((float)(a[1]*b[2] - a[2]*b[1]), \
	(float)(a[2]*b[0] - a[0]*b[2]), (float)(a[0]*b[1] - a[1]*b[0])))

namespace DxUt {

//vPosW = Matrig2.xg2.x x vPosW
STriangleF & STriangleF::Transform(Matrix4x4F & m)
{ 
	vPosW[0] = m*vPosW[0];
	vPosW[1] = m*vPosW[1];
	vPosW[2] = m*vPosW[2];
	return *this;
}

STriangleF & STriangleF::operator=(STriangleF & tri)
{
	memcpy(this, &tri, sizeof(STriangleF));
	return *this;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////   Triangle Functions   /////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool TriangleSegmentIntersect(STriangleF & tri,
	Vector3F & e1, Vector3F & e2, Vector3F & iPos, float & _t, Vector3F & norSq, float & u, float & v, float & w)
{
	//Ericson, Christer, Real Time Collision Detection 
	//5.3.6 Intersecting Ray or Segment Against Triangle

	Vector3F & A = tri.vPosW[0], & B = tri.vPosW[1], & C = tri.vPosW[2];
	Vector3F q1(B - A), q2(C - A), eVec(e1 - e2);

	Vector3F cr1(CrossXYZ(q1,q2));
	float d = DotXYZ(eVec, cr1);
	if (d <= 0.f) return 0;

	Vector3F vAP(e1-A);
	float t = DotXYZ(vAP, cr1);
	if (t < 0.f || t > d) return 0;

	Vector3F cr2(CrossXYZ(eVec,vAP));
	v = DotXYZ(q2, cr2);
	if (v < 0.f || v > d) return 0;
	w = -DotXYZ(q1, cr2);
	if (w < 0.f || (v+w) > d) return 0;

	float ood = 1.f/d;
	_t = t*ood;
	v *= ood;
	w *= ood;
	u = 1.f - v - w;

	iPos = e2 + _t*(eVec); //May not be correct
	norSq = cr1;

	return 1;
}

bool TriangleRayIntersect(STriangleF & tri, Vector3F & pos,
	Vector3F & vec, Vector3F & iPos, float & _t, Vector3F & norSq, float & u, float & v, float & w)
{
	//Ericson, Christer, Real Time Collision Detection 
	//5.3.6 Intersecting Ray or Segment Against Triangle

	Vector3F & A = tri.vPosW[0], & B = tri.vPosW[1], & C = tri.vPosW[2];
	Vector3F q1(B - A), q2(C - A), eVec(-vec);

	Vector3F cr1(CrossXYZ(q1,q2));
	float d = DotXYZ(eVec, cr1);
	if (d <= 0.f) return 0;

	Vector3F vAP(pos-A);
	float t = DotXYZ(vAP, cr1);
	if (t < 0.f) return 0;

	Vector3F cr2(CrossXYZ(eVec,vAP));
	v = DotXYZ(q2, cr2);
	if (v < 0.f || v > d) return 0;
	w = -DotXYZ(q1, cr2);
	if (w < 0.f || (v+w) > d) return 0;

	float ood = 1.f/d;
	_t = t*ood;
	v *= ood;
	w *= ood;
	u = 1.f - v - w;

	iPos = pos + _t*(vec);
	norSq = cr1;

	return 1;
}

 bool TriRayIntersect(STriangleF & tri, SRay & ray, SRayIntersectData & data)
 {
	 return TriangleRayIntersect(tri, ray.p, ray.d, data.pos, data.t, data.nor, data.u, data.v, data.w);
 }

/*bool TriPlannarLineIntersect(STriangleF & tri, Vector2F & e1, Vector2F & e2)
{

}*/

bool TriTriIntersect(STriangleF & t1, STriangleF & t2,  STriTriIntersectData & triData)
{
	/*PlaneF pl1 = t1.NormalPlane();
	float rgDist[3] = {0};
	bool rgUnder[3] = {
		pl1.PointUnderPlane(t2.vPosW[0], rgDist[0]),
		pl1.PointUnderPlane(t2.vPosW[1], rgDist[1]),
		pl1.PointUnderPlane(t2.vPosW[2], rgDist[2])
	};

	if (rgUnder[0] == rgUnder[1] && rgUnder[1] == rgUnder[2]) return 0; 
	
	
	
	
	//Sort
	int rgSortedIndex[3];
	if (rgUnder[0] + rgUnder[1] + rgUnder[2] == 1) {
		//rgSortedIndex[0] = rgUnder[0] 
	}
	else {

	}
	return 0;*/

	int c = 0; 
	UINT uiType = 0;
	double iPos1[3], iPos2[3];
	double pl1[4], pl2[4], iLine[3];

	double p1[3] = {t1.vPosW[0].x, t1.vPosW[0].y, t1.vPosW[0].z};
	double p2[3] = {t1.vPosW[1].x, t1.vPosW[1].y, t1.vPosW[1].z};
	double p3[3] = {t1.vPosW[2].x, t1.vPosW[2].y, t1.vPosW[2].z};

	double q1[3] = {t2.vPosW[0].x, t2.vPosW[0].y, t2.vPosW[0].z};
	double q2[3] = {t2.vPosW[1].x, t2.vPosW[1].y, t2.vPosW[1].z};
	double q3[3] = {t2.vPosW[2].x, t2.vPosW[2].y, t2.vPosW[2].z};

	if (!tri_tri_intersection_test_3d(p1, p2, p3, q1, q2, q3,
		&c, iPos1, iPos2, pl1, pl2, triData.uiType, iLine) || c) return 0;

	triData.iPos[0] = Vector3F((float)iPos1[0], (float)iPos1[1], (float)iPos1[2]);
	triData.iPos[1] = Vector3F((float)iPos2[0], (float)iPos2[1], (float)iPos2[2]);
	triData.iLine = Vector3F((float)iLine[0], (float)iLine[1], (float)iLine[2]);
	Vector3F dir(triData.iPos[1] - triData.iPos[0]);
	if (DotXYZ(dir, triData.iLine) < 0) {
		Vector3F tmp(triData.iPos[0]);
		triData.iPos[0] = triData.iPos[1];
		triData.iPos[1] = tmp;

		BYTE b = ((BYTE*)&triData.uiType)[1];
		((BYTE*)&triData.uiType)[1] = ((BYTE*)&triData.uiType)[2];
		((BYTE*)&triData.uiType)[2] = b;
		((BYTE*)&triData.uiType)[3] = 1;
	}
	triData.normals[0] = Vector3F((float)pl1[0], (float)pl1[1], (float)pl1[2]).Normalize();
	triData.normals[1] = Vector3F((float)pl2[0], (float)pl2[1], (float)pl2[2]).Normalize();

	return 1;
}

Vector3F ComputeClosestPoint(STriangleF & tri, Vector3F & p, UINT & uiType)
{
	Vector3F & a = tri.vPosW[0];
	Vector3F & b = tri.vPosW[1];
	Vector3F & c = tri.vPosW[2];

	Vector3F ab = b - a;
	Vector3F ac = c - a;
	Vector3F ap = p - a;

	// Check if P in vertex region outside A
	float d1 = DotXYZ(ab, ap);
	float d2 = DotXYZ(ac, ap);
	if (d1 <= 0.f && d2 <= 0.f) {
		uiType = (0x0 << 2) | (0x2);
		return a;
	}

	// Check if P in vertex region outside B
	Vector3F bp = p - b;
	float d3 = DotXYZ(ab, bp);
	float d4 = DotXYZ(ac, bp);
	if (d3 >= 0.f && d4 <= d3) {
		uiType = (0x1 << 2) | (0x2);
		return b;
	}

	// Check if P in edge region of AB, if so return projection of P onto AB
	float vc = d1*d4 - d3*d2;
	if (vc <= 0.f && d1 >= 0.f && d3 <= 0.f) {
		uiType = (0x0 << 2) | (0x1);
		float v = d1 / (d1 - d3);
		return a + v * ab;
	}

	// Check if P in vertex region outside C
	Vector3F cp = p - c;
	float d5 = DotXYZ(ab, cp);
	float d6 = DotXYZ(ac, cp);
	if (d6 >= 0.f && d5 <= d6) {
		uiType = (0x2 << 2) | (0x2);
		return c;
	}

	// Check if P in edge region of AC, if so return projection of P onto AC
	float vb = d5*d2 - d1*d6;
	if (vb <= 0.f && d2 >= 0.f && d6 <= 0.f) {
		uiType = (0x2 << 2) | (0x1);
		float w  = d2 / (d2 - d6);
		return a + w * ac;
	}

	// Check if P in edge region of BC, if so return projection  of P onto BC
	float va = d3*d6 - d5*d4;
	if (va <= 0.f && (d4 - d3) >= 0.f && (d5 - d6) >= 0.f)  {
		uiType = (0x1 << 2) | (0x1);
		float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
		return b + w * (c - b);
	}

	// P inside face region. Compute X through its barycentric coordinates (u, v, w)
	uiType = (0x0 << 2) | (0x0);
	float denom = 1.f / (va + vb + vc);
	float v = vb * denom;
	float w = vc * denom;
	return a + v * ab + w * ac;
}


void SubdivideTriangle(STriangleF & tri, STriangleF * subdividedTris)
{
	Vector3F e0(tri.vPosW[1] - tri.vPosW[0]);
	Vector3F e1(tri.vPosW[2] - tri.vPosW[1]);
	Vector3F e2(tri.vPosW[0] - tri.vPosW[2]);

	Vector3F midP0(tri.vPosW[0] + .5f*e0);
	Vector3F midP1(tri.vPosW[1] + .5f*e1);
	Vector3F midP2(tri.vPosW[2] + .5f*e2);

	subdividedTris[0].vPosW[0] = tri.vPosW[0];
	subdividedTris[0].vPosW[1] = midP0;
	subdividedTris[0].vPosW[2] = midP2;

	subdividedTris[1].vPosW[0] = midP0;
	subdividedTris[1].vPosW[1] = tri.vPosW[1];
	subdividedTris[1].vPosW[2] = midP1;

	subdividedTris[2].vPosW[0] = midP2;
	subdividedTris[2].vPosW[1] = midP1;
	subdividedTris[2].vPosW[2] = tri.vPosW[2];

	subdividedTris[3].vPosW[0] = midP0;
	subdividedTris[3].vPosW[1] = midP1;
	subdividedTris[3].vPosW[2] = midP2;
}



};



















/*struct SCollisionPair {
	UINT uiType;
	UINT uiIndx[2];
	Vector3F iPos[2];
	Vector3F iLine;
};*/


	/*int c = 0; 
	UINT uiType = 0;
	double iPos1[3], iPos2[3];
	double pl1[4], pl2[4], iLine[3];

	double p1[3] = {t1.vPosW[0].x, t1.vPosW[0].y, t1.vPosW[0].z};
	double p2[3] = {t1.vPosW[1].x, t1.vPosW[1].y, t1.vPosW[1].z};
	double p3[3] = {t1.vPosW[2].x, t1.vPosW[2].y, t1.vPosW[2].z};

	double q1[3] = {t2.vPosW[0].x, t2.vPosW[0].y, t2.vPosW[0].z};
	double q2[3] = {t2.vPosW[1].x, t2.vPosW[1].y, t2.vPosW[1].z};
	double q3[3] = {t2.vPosW[2].x, t2.vPosW[2].y, t2.vPosW[2].z};

	if (!tri_tri_intersection_test_3d(p1, p2, p3, q1, q2, q3,
		&c, iPos1, iPos2, pl1, pl2, triData.uiType, iLine) || c) return 0;

	triData.iPos[0] = Vector3F((float)iPos1[0], (float)iPos1[1], (float)iPos1[2]);
	triData.iPos[1] = Vector3F((float)iPos2[0], (float)iPos2[1], (float)iPos2[2]);
	triData.iLine = Vector3F((float)iLine[0], (float)iLine[1], (float)iLine[2]);
	Vector3F dir(triData.iPos[1] - triData.iPos[0]);
	if (DotXYZ(dir, triData.iLine) < 0) {
		Vector3F tmp(triData.iPos[0]);
		triData.iPos[0] = triData.iPos[1];
		triData.iPos[1] = tmp;

		BYTE b = ((BYTE*)&triData.uiType)[1];
		((BYTE*)&triData.uiType)[1] = ((BYTE*)&triData.uiType)[2];
		((BYTE*)&triData.uiType)[2] = b;
	}
	triData.normals[0] = Vector3F((float)pl1[0], (float)pl1[1], (float)pl1[2]).Normalize();
	triData.normals[1] = Vector3F((float)pl2[0], (float)pl2[1], (float)pl2[2]).Normalize();

	return 1;
}

bool TriTriIntersect(STriangleF & t1, STriangleF & t2,  STriTriIntersectData & cData)
{
	int c = 0;
	UINT uiType = 0;
	double iPos1[3], iPos2[3];
	double pl1[4], pl2[4], iLine[3];

	double p1[3] = {t1.vPosW[0].x, t1.vPosW[0].y, t1.vPosW[0].z};
	double p2[3] = {t1.vPosW[1].x, t1.vPosW[1].y, t1.vPosW[1].z};
	double p3[3] = {t1.vPosW[2].x, t1.vPosW[2].y, t1.vPosW[2].z};

	double q1[3] = {t2.vPosW[0].x, t2.vPosW[0].y, t2.vPosW[0].z};
	double q2[3] = {t2.vPosW[1].x, t2.vPosW[1].y, t2.vPosW[1].z};
	double q3[3] = {t2.vPosW[2].x, t2.vPosW[2].y, t2.vPosW[2].z};

	if (!tri_tri_intersection_test_3d(p1, p2, p3, q1, q2, q3,
		&c, iPos1, iPos2, pl1, pl2, uiType, iLine) || c) return 0;

	if (!_finite(iPos1[0]) || !_finite(iPos2[0])) {
		DebugBreak();
		return 0;
	}
	cData.uiType = uiType;
	cData.iPos[0] = Vector3F((float)iPos1[0], (float)iPos1[1], (float)iPos1[2]);
	cData.iPos[1] = Vector3F((float)iPos2[0], (float)iPos2[1], (float)iPos2[2]);
	cData.iLine = Vector3F((float)iLine[0], (float)iLine[1], (float)iLine[2]).Normalize();

	return 1;




}
*/