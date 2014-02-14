
#ifndef DXUTFPCAMERA_H
#define DXUTFPCAMERA_H

#include "DxUtCamera.h"

namespace DxUt {

//First person camera
class CFPCamera : public CCamera {
private:
	Vector3F m_Velocity;
	float m_fRotSpeed;
	BOOL m_bTransEnable;
	BOOL m_bRotEnable;
public:
	CFPCamera():m_fRotSpeed(0), m_bTransEnable(1), m_bRotEnable(1) {}
	~CFPCamera() {}

	//Create a left handded coordinate system FPCamera
	void CreateFPCameraLH(float fFov, UINT uiWidth, UINT uiHeight, 
		float fNearField, float fFarField, Vector3F & vel, float fRotSpeed);

	void SetFPCamera(Vector3F & pos, float fPhi, float fTheta);
	void SetFPCamera(Vector3F & vel, float fRotSpeed);
	void EnableTranslation() {m_bTransEnable = 1;}
	void DisableTranslation() {m_bTransEnable = 0;}
	void EnableRotation() {m_bRotEnable = 1;}
	void DisableRotation() {m_bRotEnable = 0;}

	void UpdateFPCamera(float dt);

	void OnSize() {
		CreateFPCameraLH(m_fFov, g_uiWndWidth, g_uiWndHeight, m_fNearField, m_fFarField, m_Velocity, m_fRotSpeed); }

	CFPCamera operator=(CFPCamera & ref);
};

};

#endif

