
#ifndef DXUTAABBOX_H
#define DXUTAABBOX_H

#include "DxUtInclude.h"
#include "DxUtRay.h"
#include "DxUtMesh.h"

namespace DxUt {

class CAABBox {
private:
	friend class CCamera;
	friend class CBSphere;

	/* The local space AABB */
	Vector3F m_MinPL;
	Vector3F m_MaxPL;

	/* The world space AABB */
	Vector3F m_MinPW;
	Vector3F m_MaxPW;
public:
	CAABBox();
	CAABBox(Vector3F & minPL, Vector3F & maxPL);
	//~CAABBox() {}

	//It is assumed that a vertex's position is stored
	//in the first 12 bytes of a vertex's data sturcture.
	//Furthermore, the CAABBox will be computed with the
	//vertex buffer before it has been commited to a device.
	void ComputeAABBox(CMesh * pMesh);
	void ComputeAABBox(Vector3F * verts, UINT nVert);

	bool PointInAABBoxW(Vector3F & pt);
	bool AABBoxIntersectW(CAABBox & box);
	/* Transform from local space to world space */
	/* T must be just a translation matrix */
	void TransformAABBoxW(Matrix4x4F & T);
	void TransformAABBoxW(Vector3F & trans, Matrix4x4F & rot, float scale);

	float ClosestPointSq(Vector3F & pt, Vector3F & closestPt);

	/* Intersect AABB with a ray coming form the inside */
	bool InteresectFromInside(SRay & ray, SRayIntersectData & data);

	float VolumeW();
	float SurfaceAreaW();

	Vector3F CenterPointW() {
		return .5f*(m_MaxPW+m_MinPW);
	}
	Vector3F HalfWidthsW() {
		return .5f*(m_MaxPW - m_MinPW);
	}

	void SetMinP(Vector3F & minPL) {m_MinPW = m_MinPL = minPL; }
	void SetMaxP(Vector3F & maxPL) {m_MaxPW = m_MaxPL = maxPL; }
	void SetAABB(Vector3F & center, Vector3F & hW) {
		m_MinPL = m_MinPW = center-hW;
		m_MaxPL = m_MaxPW = center+hW;
	}
	Vector3F & MinPW() {return m_MinPW;}
	Vector3F & MaxPW() {return m_MaxPW;}
};

inline float CAABBox::VolumeW()
{
	return 
		(m_MaxPW.x-m_MinPW.x)*
		(m_MaxPW.y-m_MinPW.y)*
		(m_MaxPW.z-m_MinPW.z); 
}

inline float CAABBox::SurfaceAreaW()
{
	return 2.f*(
		(m_MaxPW.x-m_MinPW.x)*(m_MaxPW.y-m_MinPW.y) +
		(m_MaxPW.x-m_MinPW.x)*(m_MaxPW.z-m_MinPW.z) +
		(m_MaxPW.z-m_MinPW.z)*(m_MaxPW.y-m_MinPW.y));
}


};


#endif