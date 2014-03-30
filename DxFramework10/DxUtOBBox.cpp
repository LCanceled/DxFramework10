
#include "DxUtOBBox.h"
#include "DxUtTriangle.h"
#include "DxUtMesh.h"
#include "DxUtPlane.h"

//For use in the COBBox Intersect Test
#define _Abs(x, t) ((t=x) < 0 ? -t : t)

namespace DxUt {

COBBox::COBBox():m_CenterW(0, 0, 0), m_HalfWidthsW(0, 0, 0)
{
	ZeroMemory(&m_RotVecW[0], sizeof(Vector3F));
	ZeroMemory(&m_RotVecW[1], sizeof(Vector3F));
	ZeroMemory(&m_RotVecW[2], sizeof(Vector3F));
}

void COBBox::ComputeOBB(CMesh * pMesh, OBBComputeMethod method)
{
	UINT nVert = 0;
	Vector3F * verts = NULL;

	nVert = 3*pMesh->GetNumTriangles();
	verts = pMesh->GetNewVertexTriangleList();

	ComputeOBB(verts, nVert, method);

	delete[] verts;
	verts = NULL;
}

void COBBox::ComputeOBB(Vector3F * verts, UINT nVert, OBBComputeMethod method)
{
	if (method == CVTriangles && (nVert%3))
		DxUtSendError("COBBox::ComputeOBB nVert must be divisible by 3.");

	Matrix4x4F cov;
	Vector3F mean;
	if (method == CVVertices) 
		CovarianceVertices3x3F(verts, nVert, cov, mean);	
	else CovarianceTriangles3x3F(verts, nVert, cov, mean);

	Matrix4x4F A;								//Eigen vectors
	Vector3F lambda;							//Eigen values
	JacobiTransformation3x3F(cov, A, lambda);

	m_RotVecW[0] = (A.GetColumnVec3F(0)).Normalize();
	m_RotVecW[1] = (A.GetColumnVec3F(1)).Normalize();
	m_RotVecW[2] = (A.GetColumnVec3F(2)).Normalize();

	FLOAT dot = 0;
	FLOAT dMin[3] = {FLT_MAX, FLT_MAX, FLT_MAX};
	FLOAT dMax[3] = {-FLT_MAX, -FLT_MAX, -FLT_MAX};
	for (UINT i=0; i<nVert; i++) {
		Vector3F & v = verts[i];
		dot = DotXYZ(m_RotVecW[0], v);
		if (dot < dMin[0]) 
			dMin[0] = dot;
		if (dot > dMax[0])
			dMax[0] = dot;

		dot = DotXYZ(m_RotVecW[1], v);
		if (dot < dMin[1]) 
			dMin[1] = dot;
		if (dot > dMax[1])
			dMax[1] = dot;

		dot = DotXYZ(m_RotVecW[2], v);
		if (dot < dMin[2]) 
			dMin[2] = dot;
		if (dot > dMax[2])
			dMax[2] = dot;
	}

	m_HalfWidthsW.x = .5f*(dMax[0] - dMin[0]);
	m_HalfWidthsW.y = .5f*(dMax[1] - dMin[1]);
	m_HalfWidthsW.z = .5f*(dMax[2] - dMin[2]);

	m_CenterW = 
		.5f*(dMax[0] + dMin[0])*m_RotVecW[0] + 
		.5f*(dMax[1] + dMin[1])*m_RotVecW[1] + 
		.5f*(dMax[2] + dMin[2])*m_RotVecW[2];

	memcpy(m_RotVecL, m_RotVecW, sizeof(Vector3F)*3);
	m_HalfWidthsL = m_HalfWidthsW;
	m_CenterL = m_CenterW;
}

BOOL COBBox::PointInOBBW(Vector3F & pt)
{
	if (fabs(DotXYZ(m_RotVecW[0], pt)) > m_HalfWidthsW.x)
		return FALSE;
	if (fabs(DotXYZ(m_RotVecW[1], pt)) > m_HalfWidthsW.y)
		return FALSE;
	if (fabs(DotXYZ(m_RotVecW[2], pt)) > m_HalfWidthsW.z)
		return FALSE;

	return TRUE;
}

BOOL COBBox::OBBoxIntersectW(COBBox & oBB)
{
	//See Ericson, Christer, Real Time Collision Detection, page 101, and
	//See Gottschalk, Stefan, Collision Queries using Oriented Bounding Boxes

	//a is assumed to be the this COBBox
	//b is assumed to be oBB
	FLOAT fRA=0, fRB=0, t=0;
	const FLOAT ep = 1e-4f;
	Vector3F * rVA = m_RotVecW;
	Vector3F * rVB = oBB.m_RotVecW;
	Vector3F & hWA = m_HalfWidthsW;
	Vector3F & hWB = oBB.m_HalfWidthsW;

	//Rotation matrices in a's coordinate frame
    Matrix4x4F rot, aRt;
	rot.m[0][0] = DotXYZ(rVA[0], rVB[0]); aRt.m[0][0] = Abs(rot.m[0][0]) + ep;
	rot.m[0][1] = DotXYZ(rVA[0], rVB[1]); aRt.m[0][1] = Abs(rot.m[0][1]) + ep;
	rot.m[0][2] = DotXYZ(rVA[0], rVB[2]); aRt.m[0][2] = Abs(rot.m[0][2]) + ep;

	rot.m[1][0] = DotXYZ(rVA[1], rVB[0]); aRt.m[1][0] = Abs(rot.m[1][0]) + ep;
	rot.m[1][1] = DotXYZ(rVA[1], rVB[1]); aRt.m[1][1] = Abs(rot.m[1][1]) + ep;
	rot.m[1][2] = DotXYZ(rVA[1], rVB[2]); aRt.m[1][2] = Abs(rot.m[1][2]) + ep;

	rot.m[2][0] = DotXYZ(rVA[2], rVB[0]); aRt.m[2][0] = Abs(rot.m[2][0]) + ep;
	rot.m[2][1] = DotXYZ(rVA[2], rVB[1]); aRt.m[2][1] = Abs(rot.m[2][1]) + ep;
	rot.m[2][2] = DotXYZ(rVA[2], rVB[2]); aRt.m[2][2] = Abs(rot.m[2][2]) + ep;

    //Translation vector T in a's coordinate frame
	Vector3F T(oBB.m_CenterW - m_CenterW);
	T = Vector3F(DotXYZ(T, rVA[0]), DotXYZ(T, rVA[1]), DotXYZ(T, rVA[2]));

	//Axis rotVecA[0]
	fRB = hWB.c[0] * aRt.m[0][0] + hWB.c[1] * aRt.m[0][1] + hWB.c[2] * aRt.m[0][2];
	if (Abs(T.c[0]) > (hWA.c[0] + fRB)) return 0;

	//Axis rotVecA[1]
	fRB = hWB.c[0] * aRt.m[1][0] + hWB.c[1] * aRt.m[1][1] + hWB.c[2] * aRt.m[1][2];
	if (Abs(T.c[1]) > (hWA.c[1] + fRB)) return 0;

	//Axis rotVecA[2]
	fRB = hWB.c[0] * aRt.m[2][0] + hWB.c[1] * aRt.m[2][1] + hWB.c[2] * aRt.m[2][2];
	if (Abs(T.c[2]) > (hWA.c[2] + fRB)) return 0;

	//Axis rotVecB[0]
	fRA = hWA.c[0] * aRt.m[0][0] + hWA.c[1] * aRt.m[1][0] + hWA.c[2] * aRt.m[2][0];
	if (_Abs(T.c[0] * rot.m[0][0] + T.c[1] * rot.m[1][0] + T.c[2] * rot.m[2][0], t) > (fRA + hWB.c[0])) return 0;

	//Axis rotVecB[1]
	fRA = hWA.c[0] * aRt.m[0][1] + hWA.c[1] * aRt.m[1][1] + hWA.c[2] * aRt.m[2][1];
	if (_Abs(T.c[0] * rot.m[0][1] + T.c[1] * rot.m[1][1] + T.c[2] * rot.m[2][1], t) > (fRA + hWB.c[1])) return 0;

	//Axis rotVecB[2]
	fRA = hWA.c[0] * aRt.m[0][2] + hWA.c[1] * aRt.m[1][2] + hWA.c[2] * aRt.m[2][2];
	if (_Abs(T.c[0] * rot.m[0][2] + T.c[1] * rot.m[1][2] + T.c[2] * rot.m[2][2], t) > (fRA + hWB.c[2])) return 0;

	//Axes rotVecA[0] X rotVecB[0];
	fRA = hWA.c[1] * aRt.m[2][0] + hWA.c[2] * aRt.m[1][0];
	fRB = hWB.c[1] * aRt.m[0][2] + hWB.c[2] * aRt.m[0][1];
	if (_Abs(T.c[2] * rot.m[1][0] - T.c[1] * rot.m[2][0], t) > (fRA + fRB)) return 0;

    //Axes rotVecA[0] X rotVecB[1];
	fRA = hWA.c[1] * aRt.m[2][1] + hWA.c[2] * aRt.m[1][1];
	fRB = hWB.c[0] * aRt.m[0][2] + hWB.c[2] * aRt.m[0][0];
	if (_Abs(T.c[2] * rot.m[1][1] - T.c[1] * rot.m[2][1], t) > (fRA + fRB)) return 0;

	//Axes rotVecA[0] X rotVecB[2];
	fRA = hWA.c[1] * aRt.m[2][2] + hWA.c[2] * aRt.m[1][2];
	fRB = hWB.c[0] * aRt.m[0][1] + hWB.c[1] * aRt.m[0][0];
	if (_Abs(T.c[2] * rot.m[1][2] - T.c[1] * rot.m[2][2], t) > (fRA + fRB)) return 0;

	//Axes rotVecA[1] X rotVecB[0];
	fRA = hWA.c[0] * aRt.m[2][0] + hWA.c[2] * aRt.m[0][0];
	fRB = hWB.c[1] * aRt.m[1][2] + hWB.c[2] * aRt.m[1][1];
	if (_Abs(T.c[0] * rot.m[2][0] - T.c[2] * rot.m[0][0], t) > (fRA + fRB)) return 0;

	//Axes rotVecA[1] X rotVecB[1];
	fRA = hWA.c[0] * aRt.m[2][1] + hWA.c[2] * aRt.m[0][1];
	fRB = hWB.c[0] * aRt.m[1][2] + hWB.c[2] * aRt.m[1][0];
	if (_Abs(T.c[0] * rot.m[2][1] - T.c[2] * rot.m[0][1], t) > (fRA + fRB)) return 0;

	//Axes rotVecA[1] X rotVecB[2];
	fRA = hWA.c[0] * aRt.m[2][2] + hWA.c[2] * aRt.m[0][2];
	fRB = hWB.c[0] * aRt.m[1][1] + hWB.c[1] * aRt.m[1][0];
	if (_Abs(T.c[0] * rot.m[2][2] - T.c[2] * rot.m[0][2], t) > (fRA + fRB)) return 0;

	//Axes rotVecA[2] X rotVecB[0];
    fRA = hWA.c[0] * aRt.m[1][0] + hWA.c[1] * aRt.m[0][0];
    fRB = hWB.c[1] * aRt.m[2][2] + hWB.c[2] * aRt.m[2][1];
    if (_Abs(T.c[1] * rot.m[0][0] - T.c[0] * rot.m[1][0], t) > fRA + fRB) return 0;

    //Axes rotVecA[2] X rotVecB[1];
    fRA = hWA.c[0] * aRt.m[1][1] + hWA.c[1] * aRt.m[0][1];
    fRB = hWB.c[0] * aRt.m[2][2] + hWB.c[2] * aRt.m[2][0];
    if (_Abs(T.c[1] * rot.m[0][1] - T.c[0] * rot.m[1][1], t) > fRA + fRB) return 0;

    //Axes rotVecA[2] X rotVecB[2];
    fRA = hWA.c[0] * aRt.m[1][2] + hWA.c[1] * aRt.m[0][2];
    fRB = hWB.c[0] * aRt.m[2][1] + hWB.c[1] * aRt.m[2][0];
    if (_Abs(T.c[1] * rot.m[0][2] - T.c[0] * rot.m[1][2], t) > fRA + fRB) return 0;

	return 1;
}

/*

#define SEP_AXIS(proj1, proj2, tProj, nx, ny, nz)	\
{													\
	fDepth = (proj1 + proj2) - _Abs(tProj, t);		\
	if (fDepth < 0) return 0;						\
	else if (fDepth < fSmallestDepth) {				\
		fSmallestDepth = fDepth;					\
		contactNormal = Vector3F(nx, ny, nz);		\
		axisIndexA = count;							\
	} count++;										\
}		

#define TRANSFORM_NORMAL(n)													\
{																			\
	n = Vector3F(rVA[0].x*n.x + rVA[1].x*n.y + rVA[2].x*n.z,				\
				 rVA[0].y*n.x + rVA[1].y*n.y + rVA[2].y*n.z,				\
				 rVA[0].z*n.x + rVA[1].z*n.y + rVA[2].z*n.z).Normalize();	\
}

bool FindClosestPoint(Vector3F & p1, Vector3F & p2, Vector3F & p3, Vector3F & p4, Vector3F & point)
{
	//Paul Bourke
   Vector3F p13,p43,p21, pa, pb;
   double d1343,d4321,d1321,d4343,d2121;
   double numer,denom, mua, mub;
   const float EPS = 1e-5f;

   p13.x = p1.x - p3.x;
   p13.y = p1.y - p3.y;
   p13.z = p1.z - p3.z;
   p43.x = p4.x - p3.x;
   p43.y = p4.y - p3.y;
   p43.z = p4.z - p3.z;
   if (Abs(p43.x) < EPS && Abs(p43.y) < EPS && Abs(p43.z) < EPS)
      return 0;
   p21.x = p2.x - p1.x;
   p21.y = p2.y - p1.y;
   p21.z = p2.z - p1.z;
   if (Abs(p21.x) < EPS && Abs(p21.y) < EPS && Abs(p21.z) < EPS)
      return 0;

   d1343 = p13.x * p43.x + p13.y * p43.y + p13.z * p43.z;
   d4321 = p43.x * p21.x + p43.y * p21.y + p43.z * p21.z;
   d1321 = p13.x * p21.x + p13.y * p21.y + p13.z * p21.z;
   d4343 = p43.x * p43.x + p43.y * p43.y + p43.z * p43.z;
   d2121 = p21.x * p21.x + p21.y * p21.y + p21.z * p21.z;

   denom = d2121 * d4343 - d4321 * d4321;
   if (Abs(denom) < EPS)
      return 0;
   numer = d1343 * d4321 - d1321 * d4343;

   mua = numer / denom;
   mub = (d1343 + d4321 * (mua)) / d4343;

   pa.x = p1.x + mua * p21.x;
   pa.y = p1.y + mua * p21.y;
   pa.z = p1.z + mua * p21.z;
   pb.x = p3.x + mub * p43.x;
   pb.y = p3.y + mub * p43.y;
   pb.z = p3.z + mub * p43.z;

   point = .5f*(pb-pa) + pa;

   return 1;
}

bool FindClosestFace(Vector3F * rotVecB, Vector3F & contactNormal, const float faceEps, int & axisIndexB) 
{
	//Find the axis of B most parallel to contactNormal
	int closestIndex = 0;
	float m[3], t1, t2;
	
	m[0] = DotXYZ(rotVecB[0], contactNormal);
	m[1] = DotXYZ(rotVecB[1], contactNormal); if (Abs(m[1]) > (t1=Abs(m[0]))) closestIndex = 1;
	m[2] = DotXYZ(rotVecB[2], contactNormal); if ((t2=Abs(m[2])) > Abs(m[1]) && t2 > t1) closestIndex = 2;
	axisIndexB = closestIndex;

	return Abs(m[axisIndexB]) > (1.f-faceEps);
}

void FindClosestVertices(Vector3F * rotVec, Vector3F & halfWidths, Vector3F & center, Vector3F & contactNormal, Vector3F & v1, Vector3F & v2, int & axisIndexB)
{
	//Find the closest vertex to the direction of the contactNormal
	int closestIndex = 0;
	float m[3], s[3] = {1.f, 1.f, 1.f};
	m[0] = DotXYZ(rotVec[0], contactNormal); if (m[0] < 0) {m[0] = -m[0], s[0] = -1.f; }
	m[1] = DotXYZ(rotVec[1], contactNormal); if (m[1] < 0) {m[1] = -m[1], s[1] = -1.f; } if (m[1] > m[0])					closestIndex = 1;
	m[2] = DotXYZ(rotVec[2], contactNormal); if (m[2] < 0) {m[2] = -m[2], s[2] = -1.f; } if (m[2] > m[1] && m[2] > m[0])	closestIndex = 2;
	axisIndexB = closestIndex;
	v1 = center + (s[0]*halfWidths.x)*rotVec[0] + (s[1]*halfWidths.y)*rotVec[1] + (s[2]*halfWidths.z)*rotVec[2];

	//Find the second closest vertex to the direction of the contactNormal
	float m2[3];
	closestIndex = 0;
	m[0] *= halfWidths.x, m[1] *= halfWidths.y, m[2] *= halfWidths.z;
	m2[0] = -m[0] + m[1] + m[2];
	m2[1] = +m[0] - m[1] + m[2]; if (m2[1] > m2[0])						closestIndex = 1;
	m2[2] = +m[0] + m[1] - m[2]; if (m2[2] > m2[1] && m2[2] > m2[0])	closestIndex = 2;
	v2 = v1 - (2.f*s[closestIndex]*halfWidths[closestIndex])*rotVec[closestIndex];

	//float s2[3] = {s[0], s[1], s[2]};
	//s2[closestIndex] *= -1.f;
	//v2 = oBBB.m_CenterW + (s2[0]*oBBB.m_HalfWidthsW.x)*oBBB.m_RotVecW[0] + (s2[1]*oBBB.m_HalfWidthsW.y)*oBBB.m_RotVecW[1] + (s2[2]*oBBB.m_HalfWidthsW.z)*oBBB.m_RotVecW[2];
}

inline bool VertexUnderPlane(PlaneF & planeA, Vector3F & v, Vector3F & vCopy) 
{
	Vector3F dif(v - planeA.p);
	if (DotXYZ(dif, planeA.n) <= 0) {vCopy = v; return 1; }
	return 0;
}

void IntersectLineAABB(double hWX, double hWY, Vector2F & rayPos, Vector2F & rayDir, Vector3F & rayWPos, Vector3F & rayWDir, Vector3F & cn, std::vector<SContactPoint> & CPs)
{	
	static const float eps=1e-4f;
	hWX += eps;
	hWY += eps;

	//Line
	if (rayDir.x == 0) rayDir.x = 1e-8f;
	if (rayDir.y == 0) rayDir.y = 1e-8f;
	double slope = rayDir.y/rayDir.x;
	double constant = rayPos.y - slope*rayPos.x; 
	double t=0;

	//Clip line against horizontal planes
	Vector3F rgP[8]; int nP = 0;
	double xClip0 = (-hWY - constant)/slope; t = (xClip0-rayPos.x)/rayDir.x; if (xClip0 >= -hWX && xClip0 <= hWX && t >= 0 && t <= 1) rgP[nP++] = rayWPos+t*rayWDir;
	double xClip1 = (+hWY - constant)/slope; t = (xClip1-rayPos.x)/rayDir.x; if (xClip1 >= -hWX && xClip1 <= hWX && t >= 0 && t <= 1) rgP[nP++] = rayWPos+t*rayWDir;

	//Clip line against vertical planes
	double yClip0 = -slope*hWX + constant;	t = (yClip0-rayPos.y)/rayDir.y; if (yClip0 >= -hWY && yClip0 <= hWY && t >= 0 && t <= 1) rgP[nP++] = rayWPos+t*rayWDir;
	double yClip1 = +slope*hWX + constant;	t = (yClip1-rayPos.y)/rayDir.y; if (yClip1 >= -hWY && yClip1 <= hWY && t >= 0 && t <= 1) rgP[nP++] = rayWPos+t*rayWDir;

	//Test for rayPos contained in the box
	if (rayPos.x >= -hWX && rayPos.x <= hWX && rayPos.y >= -hWY && rayPos.y <= hWY)
		rgP[nP++] = rayWPos;

	for (int i=0; i<nP; i++) {
		SContactPoint cp;
		cp.iPos = rgP[i];
		cp.iNor = cn;
		CPs.push_back(cp);
	}
}

inline void PointInOBB(Vector2F pt, Vector3F & vAW, Vector2F hWB, Vector2F & rVB1, Vector2F & rVB2, Vector2F & centerB, Vector3F & contactNormal, std::vector<SContactPoint> & CPs)
{
	pt -= centerB;
	float cmp1 = DotXY(pt, rVB1);
	if (cmp1 < -hWB.x || cmp1 > hWB.x) return;
	float cmp2 = DotXY(pt, rVB2);
	if (cmp2 < -hWB.y || cmp2 > hWB.y) return;

	SContactPoint cp;
	cp.iPos = vAW;
	cp.iNor = contactNormal;
	CPs.push_back(cp);
}

void IntersectOBB2D(
	Vector2F & hWA,
	Vector3F & vAW0, Vector3F & vAW1, Vector3F & vAW2, Vector3F & vAW3,
	Vector2F  hWB,
	Vector2F & rVB1, Vector2F & rVB2,
	Vector2F & vBL0, Vector2F & vBL1, Vector2F & vBL2, Vector2F & vBL3,
	Vector3F & vBW0, Vector3F & vBW1, Vector3F & vBW2, Vector3F & vBW3,
	Vector2F & centerB,
	Vector3F & contactNormal, std::vector<SContactPoint> & CPs)
{
	IntersectLineAABB(hWA.x, hWA.y, vBL0, (vBL1-vBL0), vBW0, (vBW1-vBW0), contactNormal, CPs);
	IntersectLineAABB(hWA.x, hWA.y, vBL1, (vBL2-vBL1), vBW1, (vBW2-vBW1), contactNormal, CPs);
	IntersectLineAABB(hWA.x, hWA.y, vBL2, (vBL3-vBL2), vBW2, (vBW3-vBW2), contactNormal, CPs);
	IntersectLineAABB(hWA.x, hWA.y, vBL3, (vBL0-vBL3), vBW3, (vBW0-vBW3), contactNormal, CPs);

	hWB = hWB * hWB;
	PointInOBB(Vector2F(+hWA.x, +hWA.y), vAW0, hWB, rVB1, rVB2, centerB, contactNormal, CPs);
	PointInOBB(Vector2F(+hWA.x, -hWA.y), vAW1, hWB, rVB1, rVB2, centerB, contactNormal, CPs);
	PointInOBB(Vector2F(-hWA.x, -hWA.y), vAW2, hWB, rVB1, rVB2, centerB, contactNormal, CPs);
	PointInOBB(Vector2F(-hWA.x, +hWA.y), vAW3, hWB, rVB1, rVB2, centerB, contactNormal, CPs);
}

BOOL COBBox::OBBoxIntersectW(COBBox & oBB, std::vector<SContactPoint> & CPs)
{
	//See Ericson, Christer, Real Time Collision Detection, page 101, and

	//a is assumed to be the this COBBox
	//b is assumed to be oBB
	FLOAT fRA=0, fRB=0, t=0, fProj=0, fDepth=0, fSmallestDepth=FLT_MAX;
	const FLOAT ep = 1e-3f;
	Vector3F * rVA = m_RotVecW;
	Vector3F * rVB = oBB.m_RotVecW;
	Vector3F hWA(m_HalfWidthsW);
	Vector3F hWB(oBB.m_HalfWidthsW);
	Vector3F ctA(m_CenterW);
	Vector3F ctB(oBB.m_CenterW);

	Vector3F contactNormal;
	int axisIndexA=0, axisIndexB=0;
	int count=0;
	float dirA=1.f, dirB=1.f;

	static Vector3F zeroVec(0, 0, 0);
	static Vector3F idRotVec[] = {Vector3F(1,0,0), Vector3F(0,1,0), Vector3F(0,0,1) };

	//Rotation matrices in a's coordinate frame
    Matrix4x4F rot, aRt;
	rot.m[0][0] = DotXYZ(rVA[0], rVB[0]); aRt.m[0][0] = Abs(rot.m[0][0]) + ep;
	rot.m[0][1] = DotXYZ(rVA[0], rVB[1]); aRt.m[0][1] = Abs(rot.m[0][1]) + ep;
	rot.m[0][2] = DotXYZ(rVA[0], rVB[2]); aRt.m[0][2] = Abs(rot.m[0][2]) + ep;

	rot.m[1][0] = DotXYZ(rVA[1], rVB[0]); aRt.m[1][0] = Abs(rot.m[1][0]) + ep;
	rot.m[1][1] = DotXYZ(rVA[1], rVB[1]); aRt.m[1][1] = Abs(rot.m[1][1]) + ep;
	rot.m[1][2] = DotXYZ(rVA[1], rVB[2]); aRt.m[1][2] = Abs(rot.m[1][2]) + ep;

	rot.m[2][0] = DotXYZ(rVA[2], rVB[0]); aRt.m[2][0] = Abs(rot.m[2][0]) + ep;
	rot.m[2][1] = DotXYZ(rVA[2], rVB[1]); aRt.m[2][1] = Abs(rot.m[2][1]) + ep;
	rot.m[2][2] = DotXYZ(rVA[2], rVB[2]); aRt.m[2][2] = Abs(rot.m[2][2]) + ep;

    //Translation vector T in a's coordinate frame
	Vector3F T(ctB - ctA);
	T = Vector3F(DotXYZ(T, rVA[0]), DotXYZ(T, rVA[1]), DotXYZ(T, rVA[2]));

	//Check A's axes
	SEP_AXIS(hWB.c[0] * aRt.m[0][0] + hWB.c[1] * aRt.m[0][1] + hWB.c[2] * aRt.m[0][2], hWA.c[0], T.c[0], 1, 0, 0);
	SEP_AXIS(hWB.c[0] * aRt.m[1][0] + hWB.c[1] * aRt.m[1][1] + hWB.c[2] * aRt.m[1][2], hWA.c[1], T.c[1], 0, 1, 0);
	SEP_AXIS(hWB.c[0] * aRt.m[2][0] + hWB.c[1] * aRt.m[2][1] + hWB.c[2] * aRt.m[2][2], hWA.c[2], T.c[2], 0, 0, 1);

	//Check B's axes
	SEP_AXIS(hWA.c[0] * aRt.m[0][0] + hWA.c[1] * aRt.m[1][0] + hWA.c[2] * aRt.m[2][0], hWB.c[0],
		T.c[0] * rot.m[0][0] + T.c[1] * rot.m[1][0] + T.c[2] * rot.m[2][0], rot.m[0][0], rot.m[1][0], rot.m[2][0]);
	SEP_AXIS(hWA.c[0] * aRt.m[0][1] + hWA.c[1] * aRt.m[1][1] + hWA.c[2] * aRt.m[2][1], hWB.c[1],
		T.c[0] * rot.m[0][1] + T.c[1] * rot.m[1][1] + T.c[2] * rot.m[2][1], rot.m[0][1], rot.m[1][1], rot.m[2][1]);
	SEP_AXIS(hWA.c[0] * aRt.m[0][2] + hWA.c[1] * aRt.m[1][2] + hWA.c[2] * aRt.m[2][2], hWB.c[2],
		T.c[0] * rot.m[0][2] + T.c[1] * rot.m[1][2] + T.c[2] * rot.m[2][2], rot.m[0][2], rot.m[1][2], rot.m[2][2]);

	//The separating axis test may incorrectly classify a Face-Face contact as an Edge-Edge contact
	//hence, these values are saved and used to check weather there are two faces of the boxes that are aligned (Face-Face contact)
	float fSmallestFaceDepth = fSmallestDepth;
	TRANSFORM_NORMAL(contactNormal); //World space transform
	Vector3F faceContactNormal(contactNormal);

	//If the smallest separating axis is not from a transform b to be at the origin
	float bBoxFlipSign = 1;
	if (axisIndexA > 2) {
		bBoxFlipSign = -1;

		rVA = oBB.m_RotVecW;
		rVB = m_RotVecW;
		hWA =  oBB.m_HalfWidthsW;
		hWB = m_HalfWidthsW;
		ctA = oBB.m_CenterW;
		ctB = m_CenterW;

		rot.MTranspose();
		aRt.MTranspose();

		T = ctB - ctA;
		T = Vector3F(DotXYZ(T, rVA[0]), DotXYZ(T, rVA[1]), DotXYZ(T, rVA[2]));

		axisIndexA -= 3;
	}
	int faceAxisIndexA = axisIndexA;

	//Check the other axes
	//Axes rotVecA[0] X rotVecB[0], rotVecB[1], rotVecB[1]
	Vector3F ref(CrossXYZ(rVA[0], rVB[0]));
	SEP_AXIS(hWA.c[1] * aRt.m[2][0] + hWA.c[2] * aRt.m[1][0], hWB.c[1] * aRt.m[0][2] + hWB.c[2] * aRt.m[0][1],
		T.c[2] * rot.m[1][0] - T.c[1] * rot.m[2][0], 0, -rot.m[2][0], rot.m[1][0]);
	SEP_AXIS(hWA.c[1] * aRt.m[2][1] + hWA.c[2] * aRt.m[1][1], hWB.c[0] * aRt.m[0][2] + hWB.c[2] * aRt.m[0][0],
		T.c[2] * rot.m[1][1] - T.c[1] * rot.m[2][1], 0, -rot.m[2][1], rot.m[1][1]);
	SEP_AXIS(hWA.c[1] * aRt.m[2][2] + hWA.c[2] * aRt.m[1][2], hWB.c[0] * aRt.m[0][1] + hWB.c[1] * aRt.m[0][0],
		T.c[2] * rot.m[1][2] - T.c[1] * rot.m[2][2], 0, -rot.m[2][2], rot.m[1][2]);

	//Axes rotVecA[1] X rotVecB[0], rotVecB[1], rotVecB[2]
	SEP_AXIS(hWA.c[0] * aRt.m[2][0] + hWA.c[2] * aRt.m[0][0], hWB.c[1] * aRt.m[1][2] + hWB.c[2] * aRt.m[1][1],
		T.c[0] * rot.m[2][0] - T.c[2] * rot.m[0][0], rot.m[2][0], 0, -rot.m[0][0]);
	SEP_AXIS(hWA.c[0] * aRt.m[2][1] + hWA.c[2] * aRt.m[0][1], hWB.c[0] * aRt.m[1][2] + hWB.c[2] * aRt.m[1][0],
		T.c[0] * rot.m[2][1] - T.c[2] * rot.m[0][1], rot.m[2][1], 0, -rot.m[0][1]);
	SEP_AXIS(hWA.c[0] * aRt.m[2][2] + hWA.c[2] * aRt.m[0][2], hWB.c[0] * aRt.m[1][1] + hWB.c[1] * aRt.m[1][0],
		T.c[0] * rot.m[2][2] - T.c[2] * rot.m[0][2], rot.m[2][2], 0, -rot.m[0][2]);

	//Axes rotVecA[2] X rotVecB[0], rotVecB[1], rotVecB[2]
	SEP_AXIS(hWA.c[0] * aRt.m[1][0] + hWA.c[1] * aRt.m[0][0], hWB.c[1] * aRt.m[2][2] + hWB.c[2] * aRt.m[2][1],
		T.c[1] * rot.m[0][0] - T.c[0] * rot.m[1][0], -rot.m[1][0], rot.m[0][0], 0);
	SEP_AXIS(hWA.c[0] * aRt.m[1][1] + hWA.c[1] * aRt.m[0][1], hWB.c[0] * aRt.m[2][2] + hWB.c[2] * aRt.m[2][0],
		T.c[1] * rot.m[0][1] - T.c[0] * rot.m[1][1], -rot.m[1][1], rot.m[0][1], 0);
	SEP_AXIS(hWA.c[0] * aRt.m[1][2] + hWA.c[1] * aRt.m[0][2], hWB.c[0] * aRt.m[2][1] + hWB.c[1] * aRt.m[2][0],
		T.c[1] * rot.m[0][2] - T.c[0] * rot.m[1][2], -rot.m[1][2], rot.m[0][2], 0);

	//World space, then compare normals
	TRANSFORM_NORMAL(contactNormal);
	if (_Abs(DotXYZ(faceContactNormal, contactNormal), t) > (1.f-.0001f)) {
		axisIndexA = faceAxisIndexA;
		contactNormal = faceContactNormal;
		fSmallestDepth = fSmallestFaceDepth;
	}

	//Normal should point from a to b and be in world space
	Vector3F vab(ctB - ctA);
	faceContactNormal = DotXYZ(faceContactNormal, vab) > 0 ? faceContactNormal : -faceContactNormal;

	const float faceEps = .001f; //Eps for deciding if it is a Face-Face contact
	if (FindClosestFace(rVB, faceContactNormal, faceEps, axisIndexB)) {
		axisIndexA = faceAxisIndexA;

		int axisIndexA1 = axisIndexA ? 0 : 1;
		int axisIndexA2 = axisIndexA==2 ? 1 : 2;
		int axisIndexB1 = axisIndexB ? 0 : 1;
		int axisIndexB2 = axisIndexB==2 ? 1 : 2;	

		//A World space vertices
		Vector3F lengths1AW(hWA[axisIndexA1]*rVA[axisIndexA1]);
		Vector3F lengths2AW(hWA[axisIndexA2]*rVA[axisIndexA2]);
		float ss = DotXYZ(faceContactNormal, rVA[axisIndexA]) > 0 ? 1.f : -1.f;
		Vector3F offsetAW(ctA + (ss*hWA[axisIndexA])*rVA[axisIndexA]);

		Vector3F vAW0(offsetAW + lengths1AW + lengths2AW);
		Vector3F vAW1(offsetAW + lengths1AW - lengths2AW);
		Vector3F vAW2(offsetAW - lengths1AW - lengths2AW);
		Vector3F vAW3(offsetAW - lengths1AW + lengths2AW);

		//B Local space vertices
		Vector2F hWBL(hWB[axisIndexB1], hWB[axisIndexB2]);
		Vector2F lengths1BL(hWBL.x*rot.m[axisIndexA1][axisIndexB1], hWBL.x*rot.m[axisIndexA2][axisIndexB1]);
		Vector2F lengths2BL(hWBL.y*rot.m[axisIndexA1][axisIndexB2], hWBL.y*rot.m[axisIndexA2][axisIndexB2]);
		Vector2F offsetBL(T[axisIndexA1], T[axisIndexA2]);
		
		Vector2F vBL0(offsetBL + lengths1BL + lengths2BL);
		Vector2F vBL1(offsetBL + lengths1BL - lengths2BL);
		Vector2F vBL2(offsetBL - lengths1BL - lengths2BL);
		Vector2F vBL3(offsetBL - lengths1BL + lengths2BL);

		//B World space vertices
		Vector3F lengths1BW(hWB[axisIndexB1]*rVB[axisIndexB1]);
		Vector3F lengths2BW(hWB[axisIndexB2]*rVB[axisIndexB2]);
		ss = DotXYZ(faceContactNormal, rVB[axisIndexB]) < 0 ? 1.f : -1.f;
		Vector3F offsetBW(ctB + (ss*hWB[axisIndexB])*rVB[axisIndexB]);

		Vector3F vBW0(offsetBW + lengths1BW + lengths2BW);
		Vector3F vBW1(offsetBW + lengths1BW - lengths2BW);
		Vector3F vBW2(offsetBW - lengths1BW - lengths2BW);
		Vector3F vBW3(offsetBW - lengths1BW + lengths2BW);

		IntersectOBB2D(
			Vector2F(hWA[axisIndexA1], hWA[axisIndexA2]),
			vAW0, vAW1, vAW2,  vAW3,
			hWBL,
			lengths1BL, lengths2BL,//may be problem
			vBL0, vBL1, vBL2, vBL3,
			vBW0, vBW1, vBW2, vBW3,
			offsetBL,
			bBoxFlipSign*faceContactNormal, CPs);

		return CPs.size();
	}
	else if (axisIndexA > 5) { //Edge-Edge contact
		contactNormal = faceContactNormal;
		contactNormal = (DotXYZ(contactNormal, vab) > 0 ? contactNormal : -contactNormal);

		Vector3F vA1, vA2, vB1, vB2;
		FindClosestVertices(rVA, hWA, ctA, contactNormal, vA1, vA2, axisIndexB);
		FindClosestVertices(rVB, hWB, ctB, -contactNormal, vB1, vB2, axisIndexB);

		Vector3F contactPoint;
		FindClosestPoint(vA2, vA1, vB2, vB1, contactPoint);

		SContactPoint cp;
		cp.iPos = contactPoint;
		cp.iNor = bBoxFlipSign*contactNormal;
		CPs.push_back(cp);

		return 1;
	}
	//At this point, it is either Face-Face, Face-Edge, Face-Vertex
	axisIndexA = faceAxisIndexA;
	//fSmallestDepth = fFaceSmallestDepth
	contactNormal = faceContactNormal;
	
	//Find the two vertices closest to the origin of B along the faceContactNormal
	Vector3F v1, v2, v;
	FindClosestVertices(rVB, hWB, ctB, -contactNormal, v1, v2, axisIndexB);

	const float fHWExpansion = 1.1f;
	__Plane planeA, planeB;
	planeA.n = contactNormal;
	planeA.p = ctA + (fHWExpansion*hWA[axisIndexA])*faceContactNormal;
	bool cond0 = VertexUnderPlane(planeA, v1, v);
	bool cond1 = VertexUnderPlane(planeA, v2, v);
	
	//Flip the contact normal back to a->b
	contactNormal *= bBoxFlipSign;

	//Case Face-Edge contact
	if (cond0 && cond1) {
		//Transform into the coordinate frame of A in order to intersect against an AABB
		int axisIndexA1 = axisIndexA ? 0 : 1;
		int axisIndexA2 = axisIndexA==2 ? 1 : 2;
		Vector3F & axisVecA1 = rVA[axisIndexA1];
		Vector3F & axisVecA2 = rVA[axisIndexA2];
		Vector3F v1_(v1-ctA);
		Vector3F v2_(v2-ctA);
		Vector2F v1Proj(DotXYZ(axisVecA1, v1_), DotXYZ(axisVecA2, v1_));
		Vector2F v2Proj(DotXYZ(axisVecA1, v2_), DotXYZ(axisVecA2, v2_));
		float hWX = hWA[axisIndexA1];
		float hWY = hWA[axisIndexA2];

		IntersectLineAABB(hWX, hWY, v1Proj, (v2Proj-v1Proj), v1, (v2-v1), contactNormal, CPs);

		//Test for weather the other end point of the ray segments is in the box
		if (v2Proj.x >= -hWX && v2Proj.x <= hWX && v2Proj.y >= -hWY && v2Proj.y <= hWY) {
			SContactPoint cp;
			cp.iNor = contactNormal;
			cp.iPos = v2;
			CPs.push_back(cp);
		}

		return CPs.size();
	}
	else if (!cond0 && !cond1) return 0;

	//In the final case of a face-vertex contact, there 
	//Case Face-Vertex contact
	//It is necessary at this point to consider all the vertices int the convex hull of 
	SContactPoint cp;
	cp.iPos = v;
	cp.iNor = contactNormal;
	CPs.push_back(cp);

	return 1;
}
*/


	//The smallest axis will either be from a or from b; make it so that A is the smallest axis box, and B is the other
	/*COBBox * oBBA, * oBBB;
	float signFlip = 1;
	if (faceAxisIndexA < 3) {
		oBBA = this;
		oBBB = &oBB;
	} else {
		oBBA = &oBB;
		oBBB = this;
		faceContactNormal = -faceContactNormal;
		faceAxisIndexA -= 3;
		signFlip = -1;
	}*/


	//Fudge factor to make sure we have an intersection



/*
void COBBox::IntersectOBB2D(COBBox & oBBA, int axisIndexA, COBBox & oBBB, int axisIndexB, __Plane & planeA, Vector3F & contactNormal, std::vector<SContactPoint> & CPs) 
{
	int axisIndex1 = (axisIndexA-1) < 0 ? 2 : (axisIndexA-1); 
	int axisIndex2 = (axisIndexA+1) > 2 ? 0 : (axisIndexA+1);
	Vector3F temp1(oBBA.m_HalfWidthsW[axisIndex1]*oBBA.m_RotVecW[axisIndex1]);
	Vector3F temp2(oBBA.m_HalfWidthsW[axisIndex2]*oBBA.m_RotVecW[axisIndex2]);
	Vector3F edgePtsA[5] = {
		Vector3F(oBBA.m_CenterW + temp1 + temp2),
		Vector3F(oBBA.m_CenterW + temp1 - temp2),
		Vector3F(oBBA.m_CenterW - temp1 - temp2),
		Vector3F(oBBA.m_CenterW - temp1 + temp2),
	};
	edgePtsA[4] = edgePtsA[0];

	axisIndex1 = (axisIndexB-1) < 0 ? 2 : (axisIndexB-1); 
	axisIndex2 = (axisIndexB+1) > 2 ? 0 : (axisIndexB+1);
	temp1 = (oBBB.m_HalfWidthsW[axisIndex1]*oBBB.m_RotVecW[axisIndex1]);
	temp2 = (oBBB.m_HalfWidthsW[axisIndex2]*oBBB.m_RotVecW[axisIndex2]);
	Vector3F edgePtsB[5] = {
		Vector3F(oBBB.m_CenterW + temp1 + temp2),
		Vector3F(oBBB.m_CenterW + temp1 - temp2),
		Vector3F(oBBB.m_CenterW - temp1 - temp2),
		Vector3F(oBBB.m_CenterW - temp1 + temp2),
	};
	edgePtsB[4] = edgePtsB[0];

	bool isIntersectEdgePt;
	Vector3F intersectPt, intersectEdgePt;
	for (int i=0; i<4; i++) {
		bool lastIntersect=0;
		for (int j=0; j<4; j++) {
			if (EdgeEdgeIntersection(edgePtsA[i], edgePtsA[i+1], edgePtsB[j], edgePtsB[j+1], intersectPt, intersectEdgePt, isIntersectEdgePt)) {
				SContactPoint cp;
				cp.iNor = contactNormal;
				cp.iPos = intersectPt;
				CPs.push_back(cp);

				//Do not want to duplicate points
				if (isIntersectEdgePt && !lastIntersect) {
					intersectEdgePt = intersectEdgePt;
					CPs.push_back(cp);
				}
				lastIntersect++;
			}
		}
	}
}*/

//Rotation, Translation, Scaling must be expressed in this oBB's cordinate frame
BOOL COBBox::OBBoxIntersectW(COBBox & oBB, Matrix4x4F & rot, Vector3F & tns, FLOAT scl)
{
	//See Ericson, Christer, Real Time Collision Detection, page 101, and
	//Gottschalk, Stefan, Collision Queries using Oriented Bounding Boxes

	//a is assumed to be the this COBBox
	//b is assumed to be oBB
	float fRA=0, fRB=0, t=0;
	const float ep = 1e-4f;
	Vector3F & hWA = m_HalfWidthsW;
	Vector3F hWB(scl*oBB.m_HalfWidthsW);

	//Rotation matrices in a's coordinate frame
    Matrix4x4F aRt;
	aRt.m[0][0] = Abs(rot.m[0][0]) + ep;
	aRt.m[0][1] = Abs(rot.m[0][1]) + ep;
	aRt.m[0][2] = Abs(rot.m[0][2]) + ep;

	aRt.m[1][0] = Abs(rot.m[1][0]) + ep;
	aRt.m[1][1] = Abs(rot.m[1][1]) + ep;
	aRt.m[1][2] = Abs(rot.m[1][2]) + ep;

	aRt.m[2][0] = Abs(rot.m[2][0]) + ep;
	aRt.m[2][1] = Abs(rot.m[2][1]) + ep;
	aRt.m[2][2] = Abs(rot.m[2][2]) + ep;

	//Axis rotVecA[0]
	fRB = hWB.c[0] * aRt.m[0][0] + hWB.c[1] * aRt.m[0][1] + hWB.c[2] * aRt.m[0][2];
	if (Abs(tns.c[0]) > (hWA.c[0] + fRB)) return 0;

	//Axis rotVecA[1]
	fRB = hWB.c[0] * aRt.m[1][0] + hWB.c[1] * aRt.m[1][1] + hWB.c[2] * aRt.m[1][2];
	if (Abs(tns.c[1]) > (hWA.c[1] + fRB)) return 0;

	//Axis rotVecA[2]
	fRB = hWB.c[0] * aRt.m[2][0] + hWB.c[1] * aRt.m[2][1] + hWB.c[2] * aRt.m[2][2];
	if (Abs(tns.c[2]) > (hWA.c[2] + fRB)) return 0;

	//Axis rotVecB[0]
	fRA = hWA.c[0] * aRt.m[0][0] + hWA.c[1] * aRt.m[1][0] + hWA.c[2] * aRt.m[2][0];
	if (_Abs(tns.c[0] * rot.m[0][0] + tns.c[1] * rot.m[1][0] + tns.c[2] * rot.m[2][0], t) > (fRA + hWB.c[0])) return 0;

	//Axis rotVecB[1]
	fRA = hWA.c[0] * aRt.m[0][1] + hWA.c[1] * aRt.m[1][1] + hWA.c[2] * aRt.m[2][1];
	if (_Abs(tns.c[0] * rot.m[0][1] + tns.c[1] * rot.m[1][1] + tns.c[2] * rot.m[2][1], t) > (fRA + hWB.c[1])) return 0;

	//Axis rotVecB[2]
	fRA = hWA.c[0] * aRt.m[0][2] + hWA.c[1] * aRt.m[1][2] + hWA.c[2] * aRt.m[2][2];
	if (_Abs(tns.c[0] * rot.m[0][2] + tns.c[1] * rot.m[1][2] + tns.c[2] * rot.m[2][2], t) > (fRA + hWB.c[2])) return 0;

	//Axes rotVecA[0] X rotVecB[0];
	fRA = hWA.c[1] * aRt.m[2][0] + hWA.c[2] * aRt.m[1][0];
	fRB = hWB.c[1] * aRt.m[0][2] + hWB.c[2] * aRt.m[0][1];
	if (_Abs(tns.c[2] * rot.m[1][0] - tns.c[1] * rot.m[2][0], t) > (fRA + fRB)) return 0;

    //Axes rotVecA[0] X rotVecB[1];
	fRA = hWA.c[1] * aRt.m[2][1] + hWA.c[2] * aRt.m[1][1];
	fRB = hWB.c[0] * aRt.m[0][2] + hWB.c[2] * aRt.m[0][0];
	if (_Abs(tns.c[2] * rot.m[1][1] - tns.c[1] * rot.m[2][1], t) > (fRA + fRB)) return 0;

	//Axes rotVecA[0] X rotVecB[2];
	fRA = hWA.c[1] * aRt.m[2][2] + hWA.c[2] * aRt.m[1][2];
	fRB = hWB.c[0] * aRt.m[0][1] + hWB.c[1] * aRt.m[0][0];
	if (_Abs(tns.c[2] * rot.m[1][2] - tns.c[1] * rot.m[2][2], t) > (fRA + fRB)) return 0;

	//Axes rotVecA[1] X rotVecB[0];
	fRA = hWA.c[0] * aRt.m[2][0] + hWA.c[2] * aRt.m[0][0];
	fRB = hWB.c[1] * aRt.m[1][2] + hWB.c[2] * aRt.m[1][1];
	if (_Abs(tns.c[0] * rot.m[2][0] - tns.c[2] * rot.m[0][0], t) > (fRA + fRB)) return 0;

	//Axes rotVecA[1] X rotVecB[1];
	fRA = hWA.c[0] * aRt.m[2][1] + hWA.c[2] * aRt.m[0][1];
	fRB = hWB.c[0] * aRt.m[1][2] + hWB.c[2] * aRt.m[1][0];
	if (_Abs(tns.c[0] * rot.m[2][1] - tns.c[2] * rot.m[0][1], t) > (fRA + fRB)) return 0;

	//Axes rotVecA[1] X rotVecB[2];
	fRA = hWA.c[0] * aRt.m[2][2] + hWA.c[2] * aRt.m[0][2];
	fRB = hWB.c[0] * aRt.m[1][1] + hWB.c[1] * aRt.m[1][0];
	if (_Abs(tns.c[0] * rot.m[2][2] - tns.c[2] * rot.m[0][2], t) > (fRA + fRB)) return 0;

	//Axes rotVecA[2] X rotVecB[0];
    fRA = hWA.c[0] * aRt.m[1][0] + hWA.c[1] * aRt.m[0][0];
    fRB = hWB.c[1] * aRt.m[2][2] + hWB.c[2] * aRt.m[2][1];
    if (_Abs(tns.c[1] * rot.m[0][0] - tns.c[0] * rot.m[1][0], t) > fRA + fRB) return 0;

    //Axes rotVecA[2] X rotVecB[1];
    fRA = hWA.c[0] * aRt.m[1][1] + hWA.c[1] * aRt.m[0][1];
    fRB = hWB.c[0] * aRt.m[2][2] + hWB.c[2] * aRt.m[2][0];
    if (_Abs(tns.c[1] * rot.m[0][1] - tns.c[0] * rot.m[1][1], t) > fRA + fRB) return 0;

    //Axes rotVecA[2] X rotVecB[2];
    fRA = hWA.c[0] * aRt.m[1][2] + hWA.c[1] * aRt.m[0][2];
    fRB = hWB.c[0] * aRt.m[2][1] + hWB.c[1] * aRt.m[2][0];
    if (_Abs(tns.c[1] * rot.m[0][2] - tns.c[0] * rot.m[1][2], t) > fRA + fRB) return 0;

	return 1;
}

BOOL COBBox::OBBoxIntersectRelativeW(SRay & rayRelative, bool bSegment, SRayIntersectData * pInter0, SRayIntersectData * pInter1)
{
	/* Real Time Collision Detection */
	Vector3F & p = rayRelative.p;
	Vector3F & d = rayRelative.d;
	float tMin = 0;
	float tMax = bSegment ? 1.f : FLT_MAX;
	const float eps = 1e-4;

	for (UINT i=0; i<3; i++) {
		if (Abs(d[i]) < eps) {
			if (p[i] < -m_HalfWidthsW[i] || p[i] > m_HalfWidthsW[i]) return 0;
		} else {
			float ood = 1.f/d[i];
			float t1 = ood*(-m_HalfWidthsW[i] - p[i]);
			float t2 = ood*(m_HalfWidthsW[i] - p[i]);

			if (t1 > t2) {
				Swap(t1, t2);
			}

			tMin = Max(tMin, t1);
			tMax = Min(tMax, t2);

			if (tMin > tMax) return 0;
		}
	}
	if (pInter0) {
		pInter0->t = tMin;
		pInter0->pos = rayRelative.p + tMin*rayRelative.d;
	}
	if (pInter1) {
		pInter1->t = tMax;
		pInter1->pos = rayRelative.p + tMax*rayRelative.d;
	}

	return 1;
}

COBBox & COBBox::operator=(COBBox & ref)
{
	memcpy(this, &ref, sizeof(COBBox));
	return *this;
}


};
