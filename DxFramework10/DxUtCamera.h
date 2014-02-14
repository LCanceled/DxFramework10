
#ifndef DXUTCAMERA_H
#define DXUTCAMERA_H

#include "DxUtInclude.h"

namespace DxUt {

//class CAABBox;
//class CBSphere;

class CCamera {
protected:
	Vector3F m_Pos;
	Vector3F m_RightVec;
	Vector3F m_UpVec;
	Vector3F m_ForwardVec;

	//VERY IMPORTANT!
	//These matrices have row vectors not column
	//The world view proj should be constructed as
	//wvp = w^T*v*p assuming a column vector world 
	Matrix4x4F m_View;
	Matrix4x4F m_Proj;

	float m_fFov;
	float m_fAspect;
	float m_fWidth, m_fHeight;
	float m_fNearField, m_fFarField;
	float m_fAngleYZ, m_fTanYZ, m_fCosYZ;
	float m_fAngleXZ, m_fTanXZ, m_fCosXZ;
	float m_fAlpha, m_fPhi, m_fTheta; 

	WORD m_wKeyPX, m_wKeyMX;
	WORD m_wKeyPY, m_wKeyMY;
	WORD m_wKeyPZ, m_wKeyMZ;

	BOOL m_bClickActivate;
	BOOL m_bKeyActivate;

	void SetDefaultKeys();
	void SetDefaultMouse();

	friend class CRayTracer;
public:
	CCamera();
	virtual ~CCamera() {}

	//Creates a left handed coordinate system camera
	void CreateCameraLH(float fFov, UINT uiWidth, UINT uiHeight,
		float fNearField, float fFarField);

	virtual void SetKeys(
		WORD keyPX, WORD keyMX, WORD keyPY, WORD keyMY, WORD keyPZ, WORD keyMZ);
	virtual void SetMouse(BOOL bClickActivate, BOOL bKeyActivate);

	bool InFrustum(Vector3F & point);
	//bool InFrustum(CBSphere & bSph);
	//bool InFrustum(CAABBox & aABBox);
	/* Get frustum ray from cursor position	*/
	void GetFrustumRay(Vector3F & rayPos, Vector3F & rayVector);
	void GetFrustumRay(Vector2F & windowCoord, Vector3F & rayPos, Vector3F & rayVector);

	/* Helpers for effects */
	/*Note that worldT must be the transpose of the column major world matrix */
	void SetEffectCamPos(ID3D10EffectVariable * eCamPos);
	void SetEffectMatrices(Matrix4x4F & worldT, ID3D10EffectMatrixVariable * eWVP, 
		ID3D10EffectMatrixVariable * eWorld);

	Vector3F & GetPosition() {return m_Pos;}
	Vector3F & GetRightVector() {return m_RightVec;}
	Vector3F & GetUpVector() {return m_UpVec;}
	Vector3F & GetForwardVector() {return m_ForwardVec;}
	Matrix4x4F & GetView() {return m_View;}
	Matrix4x4F & GetProjection() {return m_Proj;}
	float GetWidth() {return m_fWidth;}
	float GetHeight() {return m_fHeight;}
	float GetNearField() {return m_fNearField;}
	float GetFarField() {return m_fFarField;}
	float GetAngleXZ() {return m_fAngleXZ;}
	float GetAngleYZ() {return m_fAngleYZ;}
	float GetAlpha() {return m_fAlpha;}
	float GetPhi() {return m_fPhi;}
	float GetTheta() {return m_fTheta;}
	float GetTanXZ() {return m_fTanXZ;}
	float GetTanYZ() {return m_fTanYZ;}

	virtual void OnSize() {
		CreateCameraLH(m_fFov, g_uiWndWidth, g_uiWndHeight, m_fNearField, m_fFarField); }

	virtual CCamera operator=(CCamera & ref);
};

inline void CCamera::SetEffectCamPos(ID3D10EffectVariable * eCamPos)
{
	eCamPos->SetRawValue(&m_Pos, 0, sizeof(Vector3F));
}

inline void CCamera::SetEffectMatrices(Matrix4x4F & worldT, ID3D10EffectMatrixVariable * eWVP, 
		ID3D10EffectMatrixVariable * eWorld) 
{
	eWVP->SetMatrix((float*)&(worldT*m_View*m_Proj));
	eWorld->SetMatrix((float*)&worldT);
}


};

#endif