
#ifndef DXUTFPCAMERA_H
#define DXUTFPCAMERA_H

#include "DxUtCamera.h"

namespace DxUt {

//First person camera
class CFPCamera : public CCamera {
private:
	Vector3F m_Velocity;
	float m_RotSpeed;
	bool m_bTransEnable;
	bool m_bRotEnable;
public:
	CFPCamera():m_RotSpeed(0), m_bTransEnable(1), m_bRotEnable(1) {}
	~CFPCamera() {}

	//Create a left handded coordinate system FPCamera
	void CreateFPCameraLH(float fov, UINT uiWidth, UINT uiHeight, 
		float nearField, float farField, const Vector3F & vel, float rotSpeed);

	void SetFPCamera(const Vector3F & pos, float phi, float theta);
	void SetFPCamera(const Vector3F & vel, float rotSpeed);
	void EnableTranslation() {m_bTransEnable = 1;}
	void DisableTranslation() {m_bTransEnable = 0;}
	void EnableRotation() {m_bRotEnable = 1;}
	void DisableRotation() {m_bRotEnable = 0;}

	void UpdateFPCamera(float dt);

	void OnSize() {
		CreateFPCameraLH(m_Fov, g_uiWnuiIdth, g_uiWndHeight, m_NearField, m_FarField, m_Velocity, m_RotSpeed); }

	CFPCamera operator=(CFPCamera & ref);
};

};

#endif

