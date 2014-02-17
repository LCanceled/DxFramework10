
#include "DxUtBSphere.h"
#include "DxUtAABBox.h"
#include "DxUtMesh.h"

namespace DxUt {

CBSphere::CBSphere():m_PosL(0, 0, 0), m_RadiusL(1.f), m_PosW(0, 0, 0), m_RadiusW(1.f)
{
}

CBSphere::CBSphere(Vector3F & posL, float RadiusL):
	m_PosL(posL), m_RadiusL(RadiusL), m_PosW(posL), m_RadiusW(RadiusL)
{
}

void CBSphere::ComputeBSphere(ID3DX10Mesh * pMesh, DWORD dwStride)
{
	UINT nVert = pMesh->GetVertexCount();
	Vector3F * verts = new Vector3F[nVert];
	ExtractVerticesFromMesh(pMesh, verts, dwStride);

	ComputeBSphere(verts, nVert);
	delete[] verts;
}

void CBSphere::ComputeBSphere(Vector3F * verts, DWORD nVert)
{
	//See Christer, Ericson, Real Time Collision Detection, page 89
	DWORD minX=0, maxX=0, minY=0, maxY=0, minZ=0, maxZ=0;
	for (DWORD i=1; i<nVert; i++) {
		if (verts[i].x < verts[minX].x) minX = i;
		if (verts[i].x > verts[maxX].x) maxX = i;
		if (verts[i].y < verts[minY].y) minY = i;
		if (verts[i].y > verts[maxY].y) maxY = i;
		if (verts[i].z < verts[minZ].z) minZ = i;
		if (verts[i].z > verts[maxZ].z) maxZ = i;
	}

	float dX = (verts[maxX] - verts[minX]).LengthSq();
	float dY = (verts[maxY] - verts[minY]).LengthSq();
	float dZ = (verts[maxZ] - verts[minZ]).LengthSq();

	Vector3F minD, maxD;
	if (dX > dY) {
		if (dX > dZ) {
			minD = verts[minX];
			maxD = verts[maxX]; }
		else {
			minD = verts[minZ];
			maxD = verts[maxZ]; }
	} else if (dY > dZ){
		minD = verts[minY];
		maxD = verts[maxY]; }
	else {
		minD = verts[minZ];
		maxD = verts[maxZ]; 
	}

	m_PosW = .5f*(minD + maxD);
	m_RadiusW = (m_PosW - maxD).Length();
	
	float len = 0;
	for (DWORD i=0; i<nVert; i++) {
		Vector3F vec(verts[i] - m_PosW);
		if ((len = vec.LengthSq()) > m_RadiusW*m_RadiusW) {
			float dist = (float)sqrt((double)len);
			float newR = .5f*(m_RadiusW + dist);
			m_PosW += (newR - m_RadiusW)*vec/dist;
			m_RadiusW = newR;
		}
	}

	m_PosL = m_PosW;
	m_RadiusL = m_RadiusL;
}

bool CBSphere::PointInBSphereW(Vector3F & pt)
{
	return ((pt-m_PosW).LengthSq() <= (m_RadiusW*m_RadiusW));
}

bool CBSphere::BSphereIntersectW(CBSphere & bSph)
{
	float d = (m_PosW - bSph.m_PosW).LengthSq();
	float r = m_RadiusW + bSph.m_RadiusW;

	return d <= r*r;
}

bool CBSphere::AABBoxIntersectW(CAABBox & box)
{
	float d=0, m=0;

	if (m_PosW.x > box.m_MaxPW.x) {
		m = m_PosW.x - box.m_MaxPW.x; d += m*m; }
	else if (m_PosW.x < box.m_MinPW.x) {
		m = box.m_MinPW.x - m_PosW.x; d += m*m; }

	if (m_PosW.y > box.m_MaxPW.y) {
		m = m_PosW.y - box.m_MaxPW.y; d += m*m; }
	else if (m_PosW.y < box.m_MinPW.y) {
		m = box.m_MinPW.y - m_PosW.y; d += m*m; }

	if (m_PosW.z > box.m_MaxPW.z) {
		m = m_PosW.z - box.m_MaxPW.z; d += m*m; }
	else if (m_PosW.z < box.m_MinPW.z) {
		m = box.m_MinPW.z - m_PosW.z; d += m*m; }

	return (d <= m_RadiusW*m_RadiusW);
}


};