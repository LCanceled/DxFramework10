
#ifndef DXUTOBBOX_H
#define DXUTOBBOX_H

#include "DxUtInclude.h"
#include "DxUtTriangle.h"
#include "DxUtRay.h"
#include <vector> 

namespace DxUt {

struct __Plane {
	Vector3F n, p;
};

class COBBox {
public:
	//The OBB may be computed by finding the covariance matrix of the vertices (CVVertices)
	//or of the triangles (CVTriangles) of the mesh. CVTriangles usually is a better fit OOB.  
	enum OBBComputeMethod {
		CVVertices,			
		CVTriangles
	};
protected:
	friend class CRigidBody;
	friend class BVTree;

	Vector3F m_CenterL;			//The center of the OBB
	Vector3F m_HalfWidthsL;		//The axis length of the OBB
	Vector3F m_RotVecL[3];		//The axis vectors of the OBB

	Vector3F m_CenterW;			//The center of the OBB
	Vector3F m_HalfWidthsW;		//The axis length of the OBB
	Vector3F m_RotVecW[3];		//The axis vectors of the OBB

	/*void IntersectOBB2D(Vector2F & hWA, int axisIndexA1, int axisIndexA2,
		Vector2F & bT0, Vector2F & bT1, Vector2F & bT2, Vector2F & bT3,
		Vector3F & b0, Vector3F & b1, Vector3F & b2, Vector3F & b3,  
		//Vector3F & offset, Vector3F * invRot, Vector3F & invTrans,
		Vector3F & contactNormal, std::vector<SContactPoint> & rgCP);*/
public:
	COBBox();
	//~COBBox() {}

	//pMesh must have adjancey
	void ComputeOBB(ID3DX10Mesh * pMesh, DWORD dwStride, OBBComputeMethod method);

	//rgVert must be a triangle list 
	void ComputeOBB(Vector3F * rgVert, DWORD nVert, OBBComputeMethod method);

	BOOL PointInOBBW(Vector3F & pt);
	BOOL OBBoxIntersectW(COBBox & oBB);
	BOOL OBBoxIntersectW(COBBox & oBB, std::vector<SContactPoint> & rgCPs);  
	//Rotation, Translation, Scaling of oBB must be expressed relative to the function caller's OBB frame
	BOOL OBBoxIntersectW(COBBox & oBB, Matrix4x4F & rot, Vector3F & trans, FLOAT fScl);
	BOOL OBBoxIntersectRelativeW(SRay & rayRelative, bool bSegment=0, SRayIntersectData * pInter0=0, SRayIntersectData * pInter1=0);

	//TransformOBB is done as follows:
	//m_CenterW = center,
	//m_RotVecW[i] = rot*m_RotVecW[i],
	//m_HalfWidthsW -= fScl*m_HalfWidthsW
	//The transformation is clearly from world space to world space
	void TransformOBBW(Vector3F & center, Matrix4x4F & rot, FLOAT fScl);
	void TransformOBBW(Matrix4x4F & rT);

	float VolumeW();
	float SurfaceAreaW();

	Vector3F & GetCenterW() {return m_CenterW;}
	Vector3F & GetHalfWidthsW() {return m_HalfWidthsW;}
	Vector3F & GetRotVecW(DWORD dwIndex);

	float GetMaxRadiusSq() {return m_HalfWidthsW.Length(); }
	
	/* Converts the axis vectors into a rotation matrix of column form */
	void RotationW(Matrix4x4F & rot);
	/* Converts the axis vectors into a rotation matrix of row form */
	void RotationWT(Matrix4x4F & rot);

	void SetOBB(Vector3F & center, Vector3F halfWidths, Vector3F * rotVec) {
		m_CenterL = m_CenterW = center;
		m_HalfWidthsL = m_HalfWidthsW = halfWidths;
		memcpy(m_RotVecL, rotVec, 3*sizeof(Vector3F));
		memcpy(m_RotVecW, rotVec, 3*sizeof(Vector3F));
	}

	COBBox & operator=(COBBox & ref);
};

inline float COBBox::VolumeW()
{
	return 8.f*
		m_HalfWidthsW.x*
		m_HalfWidthsW.y*
		m_HalfWidthsW.z;
}

inline float COBBox::SurfaceAreaW()
{
	return 8.f*(
		m_HalfWidthsW.x*m_HalfWidthsW.y + 
		m_HalfWidthsW.y*m_HalfWidthsW.z + 
		m_HalfWidthsW.x*m_HalfWidthsW.z);
}

inline void COBBox::TransformOBBW(Vector3F & center, Matrix4x4F & rot, FLOAT fScl)
{
	m_CenterW = m_CenterL.MulNormal(rot, m_CenterL) + center;
	m_RotVecW[0] = m_RotVecL[0].MulNormal(rot, m_RotVecL[0]);
	m_RotVecW[1] = m_RotVecL[1].MulNormal(rot, m_RotVecL[1]);
	m_RotVecW[2] = m_RotVecL[2].MulNormal(rot, m_RotVecL[2]);

	m_HalfWidthsW = fScl*m_HalfWidthsL;
}

inline void COBBox::TransformOBBW(Matrix4x4F & rT)
{
	m_CenterW = m_CenterL.MulNormal(rT, m_CenterL) + Vector3F(rT.m[0][3], rT.m[1][3], rT.m[2][3]);
	m_RotVecW[0] = m_RotVecL[0].MulNormal(rT, m_RotVecL[0]);
	m_RotVecW[1] = m_RotVecL[0].MulNormal(rT, m_RotVecL[1]); 
	m_RotVecW[2] = m_RotVecL[0].MulNormal(rT, m_RotVecL[2]);
}

inline Vector3F & COBBox::GetRotVecW(DWORD dwIndex)
{
	Assert(dwIndex < 3, "COBBox::GetRotVecW index is out of range.");

	return m_RotVecW[dwIndex];
}

inline void COBBox::RotationW(Matrix4x4F & rot)
{
	rot.m[0][0] = m_RotVecW[0].x, rot.m[1][0] = m_RotVecW[0].y, rot.m[2][0] = m_RotVecW[0].z, rot.m[3][0] = 0;
	rot.m[0][1] = m_RotVecW[1].x, rot.m[1][1] = m_RotVecW[1].y, rot.m[2][1] = m_RotVecW[1].z, rot.m[3][1] = 0;
	rot.m[0][2] = m_RotVecW[2].x, rot.m[1][2] = m_RotVecW[2].y, rot.m[2][2] = m_RotVecW[2].z, rot.m[3][2] = 0;
	rot.m[0][3] = 0,              rot.m[1][3] = 0,              rot.m[2][3] = 0,              rot.m[3][3] = 1.f;
}

inline void COBBox::RotationWT(Matrix4x4F & rot)
{
	rot.m[0][0] = m_RotVecW[0].x, rot.m[0][1] = m_RotVecW[0].y, rot.m[0][2] = m_RotVecW[0].z, rot.m[0][3] = 0;
	rot.m[1][0] = m_RotVecW[1].x, rot.m[1][1] = m_RotVecW[1].y, rot.m[1][2] = m_RotVecW[1].z, rot.m[1][3] = 0;
	rot.m[2][0] = m_RotVecW[2].x, rot.m[2][1] = m_RotVecW[2].y, rot.m[2][2] = m_RotVecW[2].z, rot.m[2][3] = 0;
	rot.m[3][0] = 0,              rot.m[3][1] = 0,              rot.m[3][2] = 0,              rot.m[3][3] = 1.f;
}


};

#endif
