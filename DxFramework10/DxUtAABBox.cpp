
#include "DxUtAABBox.h"
#include "DxUtMesh.h"

//For use in Transform CAABBox
//#define _Min(m, a, b, t, u) ((t=m*a) < (u=m*b) ? t : u)
//#define _Max(t, u) (t > u ? t : u)

namespace DxUt {

CAABBox::CAABBox(): m_MinPL(0, 0, 0), m_MaxPL(0, 0, 0), m_MinPW(0, 0, 0), m_MaxPW(0, 0, 0)
{
}

CAABBox::CAABBox(Vector3F & minP, Vector3F & maxP): m_MinPL(minP), m_MaxPL(maxP), m_MinPW(minP), m_MaxPW(maxP)
{
}

void CAABBox::ComputeAABBox(CMesh * pMesh)
{
	UINT nVert = 3*pMesh->GetNumTriangles();
	Vector3F * verts = pMesh->GetNewVertexTriangleList();

	ComputeAABBox(verts, nVert);
	delete[] verts;
}

void CAABBox::ComputeAABBox(Vector3F * verts, DWORD nVert)
{
	DWORD minX=0, maxX=0, minY=0, maxY=0, minZ=0, maxZ=0;
	for (DWORD i=1; i<nVert; i++) {
		if (verts[i].x < verts[minX].x) minX = i;
		if (verts[i].x > verts[maxX].x) maxX = i;
		if (verts[i].y < verts[minY].y) minY = i;
		if (verts[i].y > verts[maxY].y) maxY = i;
		if (verts[i].z < verts[minZ].z) minZ = i;
		if (verts[i].z > verts[maxZ].z) maxZ = i;
	}

	m_MinPL = Vector3F(verts[minX].x, verts[minY].y, verts[minZ].z);
	m_MaxPL = Vector3F(verts[maxX].x, verts[maxY].y, verts[maxZ].z);

	m_MinPW = m_MinPL;
	m_MaxPW = m_MaxPL;
}

bool CAABBox::PointInAABBoxW(Vector3F & pt)
{
	return (
		pt.x >= m_MinPW.x && pt.x <= m_MaxPW.x &&
		pt.y >= m_MinPW.y && pt.y <= m_MaxPW.y &&
		pt.z >= m_MinPW.z && pt.z <= m_MaxPW.z);
}

bool CAABBox::AABBoxIntersectW(CAABBox & box)
{
	return
		(m_MinPW.x <= box.m_MaxPW.x && m_MaxPW.x >= box.m_MinPW.x) &&
		(m_MinPW.y <= box.m_MaxPW.y && m_MaxPW.y >= box.m_MinPW.y) &&
		(m_MinPW.z <= box.m_MaxPW.z && m_MaxPW.z >= box.m_MinPW.z);
}

float CAABBox::ClosestPointSq(Vector3F & pt, Vector3F & closestPt)
{
	closestPt = pt;
	if (pt.x < m_MinPW.x) closestPt.x = m_MinPW.x;
	if (pt.x > m_MaxPW.x) closestPt.x = m_MaxPW.x;
	
	if (pt.y < m_MinPW.y) closestPt.y = m_MinPW.y;
	if (pt.y > m_MaxPW.y) closestPt.y = m_MaxPW.y;

	if (pt.z < m_MinPW.z) closestPt.z = m_MinPW.z;
	if (pt.z > m_MaxPW.z) closestPt.z = m_MaxPW.z;

	return (pt - closestPt).LengthSq();
}

bool CAABBox::InteresectFromInside(SRay & ray, SRayIntersectData & data)
{
	float t = FLT_MAX, tTmp;
	static const float eps = 1e-4;
	if (ray.d.x > 0.f) tTmp = (m_MaxPW.x - ray.p.x) / (ray.d.x);
	else				tTmp = (m_MinPW.x - ray.p.x) / (ray.d.x);
	if (Abs(ray.d.x) > eps) {
		if (tTmp < 0.f) return 0;
		t = Min(t, tTmp);
	}

	if (ray.d.y > 0.f) tTmp = (m_MaxPW.y - ray.p.y) / (ray.d.y);
	else				tTmp = (m_MinPW.y - ray.p.y) / (ray.d.y);
	if (Abs(ray.d.y) > eps) {
		if (tTmp < 0.f) return 0;
		t = Min(t, tTmp);
	}

	if (ray.d.z > 0.f) tTmp = (m_MaxPW.z - ray.p.z) / (ray.d.z);
	else				tTmp = (m_MinPW.z - ray.p.z) / (ray.d.z);
	if (Abs(ray.d.z) > eps) {
		if (tTmp < 0.f) return 0;
		t = Min(t, tTmp);
	}
	/* Assume that at least one coordinate of ray.d is > eps */

	data.pos = ray.p + t*ray.d; 
	data.t = t;

	return 1;
}

void CAABBox::TransformAABBoxW(Matrix4x4F & T)
{
	/* Fix this so that it works for more than just a trans matrix */
	m_MinPW = T*m_MinPL;
	m_MaxPW = T*m_MaxPL;
}

void CAABBox::TransformAABBoxW(Vector3F & trans, Matrix4x4F & rot, float scale)
{
	Vector3F points[2] = {Vector3F(scale*(rot*m_MinPL) + trans), Vector3F(scale*(rot*m_MaxPL) + trans) };
	
	DWORD minX=0, maxX=0, minY=0, maxY=0, minZ=0, maxZ=0;
	for (DWORD i=1; i<2; i++) {
		if (points[i].x < points[minX].x) minX = i;
		if (points[i].x > points[maxX].x) maxX = i;
		if (points[i].y < points[minY].y) minY = i;
		if (points[i].y > points[maxY].y) maxY = i;
		if (points[i].z < points[minZ].z) minZ = i;
		if (points[i].z > points[maxZ].z) maxZ = i;
	}

	m_MinPW = Vector3F(points[minX].x, points[minY].y, points[minZ].z);
	m_MaxPW = Vector3F(points[maxX].x, points[maxY].y, points[maxZ].z);
}


};



