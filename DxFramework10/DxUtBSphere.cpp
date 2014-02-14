
#include "DxUtBSphere.h"
#include "DxUtAABBox.h"
#include "DxUtMesh.h"

namespace DxUt {

CBSphere::CBSphere():m_PosL(0, 0, 0), m_fRadiusL(1.f), m_PosW(0, 0, 0), m_fRadiusW(1.f)
{
}

CBSphere::CBSphere(Vector3F & posL, float fRadiusL):
	m_PosL(posL), m_fRadiusL(fRadiusL), m_PosW(posL), m_fRadiusW(fRadiusL)
{
}

void CBSphere::ComputeBSphere(ID3DX10Mesh * pMesh, DWORD dwStride)
{
	UINT nVert = pMesh->GetVertexCount();
	Vector3F * rgVert = new Vector3F[nVert];
	ExtractVerticesFromMesh(pMesh, rgVert, dwStride);

	ComputeBSphere(rgVert, nVert);
	delete[] rgVert;
}

void CBSphere::ComputeBSphere(Vector3F * rgVert, DWORD nVert)
{
	//See Christer, Ericson, Real Time Collision Detection, page 89
	DWORD minX=0, maxX=0, minY=0, maxY=0, minZ=0, maxZ=0;
	for (DWORD i=1; i<nVert; i++) {
		if (rgVert[i].x < rgVert[minX].x) minX = i;
		if (rgVert[i].x > rgVert[maxX].x) maxX = i;
		if (rgVert[i].y < rgVert[minY].y) minY = i;
		if (rgVert[i].y > rgVert[maxY].y) maxY = i;
		if (rgVert[i].z < rgVert[minZ].z) minZ = i;
		if (rgVert[i].z > rgVert[maxZ].z) maxZ = i;
	}

	float dX = (rgVert[maxX] - rgVert[minX]).LengthSq();
	float dY = (rgVert[maxY] - rgVert[minY]).LengthSq();
	float dZ = (rgVert[maxZ] - rgVert[minZ]).LengthSq();

	Vector3F minD, maxD;
	if (dX > dY) {
		if (dX > dZ) {
			minD = rgVert[minX];
			maxD = rgVert[maxX]; }
		else {
			minD = rgVert[minZ];
			maxD = rgVert[maxZ]; }
	} else if (dY > dZ){
		minD = rgVert[minY];
		maxD = rgVert[maxY]; }
	else {
		minD = rgVert[minZ];
		maxD = rgVert[maxZ]; 
	}

	m_PosW = .5f*(minD + maxD);
	m_fRadiusW = (m_PosW - maxD).Length();
	
	float len = 0;
	for (DWORD i=0; i<nVert; i++) {
		Vector3F vec(rgVert[i] - m_PosW);
		if ((len = vec.LengthSq()) > m_fRadiusW*m_fRadiusW) {
			float dist = (float)sqrt((double)len);
			float newR = .5f*(m_fRadiusW + dist);
			m_PosW += (newR - m_fRadiusW)*vec/dist;
			m_fRadiusW = newR;
		}
	}

	m_PosL = m_PosW;
	m_fRadiusL = m_fRadiusL;
}

bool CBSphere::PointInBSphereW(Vector3F & pt)
{
	return ((pt-m_PosW).LengthSq() <= (m_fRadiusW*m_fRadiusW));
}

bool CBSphere::BSphereIntersectW(CBSphere & bSph)
{
	float d = (m_PosW - bSph.m_PosW).LengthSq();
	float r = m_fRadiusW + bSph.m_fRadiusW;

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

	return (d <= m_fRadiusW*m_fRadiusW);
}


};