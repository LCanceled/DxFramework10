
#ifndef DXUTBSPHERE_H
#define DXUTBSPHERE_H

#include "DxUtInclude.h"
#include "DxUtMesh.h"

namespace DxUt {

class CAABBox;

class CBSphere {
private:
	friend class CCamera;
	friend class CAABBox;

	/* The world spae BS */
	Vector3F m_PosW;
	float m_RadiusW;

	/* The local spae BS */
	Vector3F m_PosL;
	float m_RadiusL;
public:
	CBSphere();
	CBSphere(Vector3F & posL, 
		float RadiusL); 
	//~CBSphere() {}

	//It is assumed that a vertex's position is stored
	//in the first 12 bytes of a vertex's data sturcture.
	//Furthermore, the CAABBox will be computed with the
	//vertex buffer before it has been commited to a device.
	void ComputeBSphere(CMesh * pMesh);
	void ComputeBSphere(Vector3F * verts, DWORD nVert);

	bool PointInBSphereW(Vector3F & pt);
	bool BSphereIntersectW(CBSphere & bSph);
	bool AABBoxIntersectW(CAABBox & box);
	//T should be orthogonal. If there is scaling, the max
	//scaling of the three axes should be given as scl
	//This function will transform from local space to world space
	virtual void TransformBSphereL(Matrix4x4F & T, float scl);

	float VolumeW() {return 4.f*D3DX_PI*(m_RadiusW*m_RadiusW*m_RadiusW/3); }
	float SurfaceAreaW() {return (4.f*D3DX_PI*m_RadiusW*m_RadiusW); }

	Vector3F & PosW() {return m_PosW;}
	float & RadiusW() {return m_RadiusW;}
};

//The axis vectors of rotTrans must be the columns of rotTrans
inline void CBSphere::TransformBSphereL(Matrix4x4F & T, float scl)
{
	m_RadiusW = scl*m_RadiusL;
	m_PosW = T*m_PosL;
}


};

#endif