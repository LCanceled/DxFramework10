
#include "DxUtFPCamera.h"

namespace DxUt {

void CFPCamera::CreateFPCameraLH(float fFov, UINT uiWidth, UINT uiHeight, 
	float fNearField, float fFarField, const Vector3F & vel, float fRotSpeed)
{
	CCamera::CreateCameraLH(fFov, uiWidth, uiHeight, fNearField, fFarField);
	m_Velocity = vel;
	m_RotSpeed = fRotSpeed;
}

void CFPCamera::SetFPCamera(const Vector3F & pos, float fPhi, float fTheta)
{
	m_Pos = pos;
	m_Phi = fPhi;
	m_Theta = fTheta;

	m_RightVec.x = cosf(m_Phi-D3DX_PI/2.);
	m_RightVec.y = 0;
	m_RightVec.z = sinf(m_Phi-D3DX_PI/2.);

	m_ForwardVec.x = cos(m_Phi)*cos(m_Theta);
	m_ForwardVec.y = sin(m_Theta);
	m_ForwardVec.z = sin(m_Phi)*cos(m_Theta);
	m_ForwardVec = m_ForwardVec.Normalize();

	m_UpVec =  CrossXYZ(m_ForwardVec, m_RightVec);

	float x = -DotXYZ(m_Pos, m_RightVec);
	float y = -DotXYZ(m_Pos, m_UpVec);
	float z = -DotXYZ(m_Pos, m_ForwardVec);

	m_View.m[0][0] = m_RightVec.x,	m_View.m[1][0] = m_RightVec.y,	m_View.m[2][0] = m_RightVec.z,	m_View.m[3][0] = x;
	m_View.m[0][1] = m_UpVec.x,		m_View.m[1][1] = m_UpVec.y,		m_View.m[2][1] = m_UpVec.z,		m_View.m[3][1] = y;
	m_View.m[0][2] = m_ForwardVec.x, m_View.m[1][2] = m_ForwardVec.y, m_View.m[2][2] = m_ForwardVec.z, m_View.m[3][2] = z;
	m_View.m[0][3] = 0,					m_View.m[1][3] = 0,					m_View.m[2][3] = 0,					m_View.m[3][3] = 1;
}

void CFPCamera::SetFPCamera(const Vector3F & vel, float rotSpeed)
{
	m_Velocity = vel;
	m_RotSpeed = rotSpeed;
}
	
void CFPCamera::UpdateFPCamera(float dt)
{
	Vector3F dir(0, 0, 0);

	if (m_bTransEnable) {
		if (g_KeysState[m_wKeyPX]) {
			dir += m_RightVec;}
		if (g_KeysState[m_wKeyMX]) {
			dir -= m_RightVec;}
		if (g_KeysState[m_wKeyPY]) {
			dir += m_UpVec;}
		if (g_KeysState[m_wKeyMY]) {
			dir -= m_UpVec;}
		if (g_KeysState[m_wKeyPZ]) {
			dir += m_ForwardVec;}
		if (g_KeysState[m_wKeyMZ]) {
			dir -= m_ForwardVec;}
	}
	if (m_bRotEnable) {
		if (g_MouseState.rgbButtons[m_bClickActivate] || g_KeysState[m_bKeyActivate]) {
			if (g_MouseState.lX > 0) 
				m_Phi -= g_MouseState.lX*m_RotSpeed;
			if (g_MouseState.lX < 0) 
				m_Phi -= g_MouseState.lX*m_RotSpeed;
			if (g_MouseState.lY > 0) 
				m_Theta -= g_MouseState.lY*m_RotSpeed;
			if (g_MouseState.lY < 0) 
				m_Theta -= g_MouseState.lY*m_RotSpeed;} 
	}
	if (dir.Length() > 0) dir = dir.Normalize();

	m_Pos.x += dir.x*m_Velocity.x*dt;
	m_Pos.y += dir.y*m_Velocity.y*dt;
	m_Pos.z += dir.z*m_Velocity.z*dt;

	m_RightVec.x = cosf(m_Phi-D3DX_PI/2.);
	m_RightVec.y = 0;
	m_RightVec.z = sinf(m_Phi-D3DX_PI/2.);

	m_ForwardVec.x = cos(m_Phi)*cos(m_Theta);
	m_ForwardVec.y = sin(m_Theta);
	m_ForwardVec.z = sin(m_Phi)*cos(m_Theta);
	m_ForwardVec = m_ForwardVec.Normalize();

	m_UpVec = CrossXYZ(m_ForwardVec, m_RightVec);

	float x = -DotXYZ(m_Pos, m_RightVec);
	float y = -DotXYZ(m_Pos, m_UpVec);
	float z = -DotXYZ(m_Pos, m_ForwardVec);

	m_View.m[0][0] = m_RightVec.x,	m_View.m[1][0] = m_RightVec.y,	m_View.m[2][0] = m_RightVec.z,	m_View.m[3][0] = x;
	m_View.m[0][1] = m_UpVec.x,		m_View.m[1][1] = m_UpVec.y,		m_View.m[2][1] = m_UpVec.z,		m_View.m[3][1] = y;
	m_View.m[0][2] = m_ForwardVec.x, m_View.m[1][2] = m_ForwardVec.y, m_View.m[2][2] = m_ForwardVec.z, m_View.m[3][2] = z;
	m_View.m[0][3] = 0,					m_View.m[1][3] = 0,					m_View.m[2][3] = 0,					m_View.m[3][3] = 1;
}

CFPCamera CFPCamera::operator=(CFPCamera & ref)
{
	CCamera::operator =(ref);

	m_Velocity = ref.m_Velocity;
	m_RotSpeed = ref.m_RotSpeed;

	return *this;
}


};

