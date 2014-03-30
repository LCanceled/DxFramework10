
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

	float m_Fov;
	float m_Aspect;
	float m_Width, m_Height;
	float m_NearField, m_FarField;
	float m_AngleYZ, m_TanYZ, m_CosYZ;
	float m_AngleXZ, m_TanXZ, m_CosXZ;
	float m_Alpha, m_Phi, m_Theta; 

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
	float GetWidth() {return m_Width;}
	float GetHeight() {return m_Height;}
	float GetNearField() {return m_NearField;}
	float GetFarField() {return m_FarField;}
	float GetAngleXZ() {return m_AngleXZ;}
	float GetAngleYZ() {return m_AngleYZ;}
	float GetAlpha() {return m_Alpha;}
	float GetPhi() {return m_Phi;}
	float GetTheta() {return m_Theta;}
	float GetTanXZ() {return m_TanXZ;}
	float GetTanYZ() {return m_TanYZ;}

	virtual void OnSize() {
		CreateCameraLH(m_Fov, g_uiWnuiIdth, g_uiWndHeight, m_NearField, m_FarField); }

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