
#include "DxUtCamera.h"
//#include "DxUtAABBox.h"
//#include "DxUtBSphere.h"

namespace DxUt {

CCamera::CCamera():m_RightVec(0, 0, 1.f), m_UpVec(0, 1.f, 0), 
	m_ForwardVec(1.f, 0, 0), m_fAlpha(0), m_fPhi(0), m_fTheta(0)
{
	m_View.MIdenity();
	m_Proj.MIdenity();

	SetDefaultKeys();
	SetDefaultMouse();
}

void CCamera::CreateCameraLH(float fFov, UINT uiWidth, UINT uiHeight, float fNearField, float fFarField)
{
	m_fFov = fFov;
	m_fWidth = (float)uiWidth;
	m_fHeight = (float)uiHeight;
	m_fAspect = m_fWidth/m_fHeight;
	m_fNearField = fNearField;
	m_fFarField = fFarField;
	m_fAngleXZ = atan(tan(fFov/2.f)*m_fAspect);
	m_fAngleYZ = fFov/2.f;
	m_fTanXZ = tanf(m_fAngleXZ);
	m_fTanYZ = tanf(m_fAngleYZ);
	m_fCosXZ = cosf(m_fAngleXZ);
	m_fCosYZ = cosf(m_fAngleYZ);

	float yScl = cosf(fFov/2.f)/sinf(fFov/2.f);
	float xScl = yScl/m_fAspect;
	float zn = fNearField;
	float zf = fFarField;
	m_Proj.m[0][0] = xScl,	m_Proj.m[1][0] = 0,		m_Proj.m[2][0] = 0,			 m_Proj.m[3][0] = 0;
	m_Proj.m[0][1] = 0,		m_Proj.m[1][1] = yScl,	m_Proj.m[2][1] = 0,			 m_Proj.m[3][1] = 0;
	m_Proj.m[0][2] = 0,		m_Proj.m[1][2] = 0,		m_Proj.m[2][2] = zf/(zf-zn), m_Proj.m[3][2] = -zn*zf/(zf-zn);
	m_Proj.m[0][3] = 0,		m_Proj.m[1][3] = 0,		m_Proj.m[2][3] = 1,			 m_Proj.m[3][3] = 0;
}

void CCamera::SetKeys(
	WORD keyPX, WORD keyMX, WORD keyPY, WORD keyMY, WORD keyPZ, WORD keyMZ)
{
	m_wKeyPX = keyPX;
	m_wKeyMX = keyMX;
	m_wKeyPY = keyPY;
	m_wKeyMY = keyMY;
	m_wKeyPZ = keyPZ;
	m_wKeyMZ = keyMZ;
}

void CCamera::SetDefaultKeys()
{
	m_wKeyPX = DIK_RIGHT;
	m_wKeyMX = DIK_LEFT;
	m_wKeyPY = DIK_NUMPADPLUS;
	m_wKeyMY = DIK_NUMPADMINUS;
	m_wKeyPZ = DIK_UP;
	m_wKeyMZ = DIK_DOWN;
}

void CCamera::SetMouse(BOOL bClickActivate, BOOL bKeyActivate)
{
	m_bClickActivate = bClickActivate;
	m_bKeyActivate = bKeyActivate;
}

void CCamera::SetDefaultMouse()
{
	m_bClickActivate = 0;
	m_bKeyActivate = DIK_RCONTROL;
}

bool CCamera::InFrustum(Vector3F & point)
{
	Vector3F vec = point - m_Pos;

	float forward = DotXYZ(vec, m_ForwardVec);
	if (forward <= m_fNearField || forward >= m_fFarField) return 0;

	float right = DotXYZ(vec, m_RightVec);
	float rightLimit = forward*m_fTanXZ;
	if (right < (-rightLimit) || right > rightLimit) return 0;

	float up = DotXYZ(vec, m_UpVec);
	float upLimit = forward*m_fTanYZ;
	if (up <= (-upLimit) || up >= upLimit) return 0;

	return 1;
}
/*
bool CCamera::InFrustum(CBSphere & bSph)
{
	Vector3F vec = bSph.m_PosW - m_Pos;

	float forward = DotXYZ(vec, m_ForwardVec);
	if (forward <= (m_fNearField-bSph.m_RadiusW) || forward >= (m_fFarField+bSph.m_RadiusW)) return 0;

	float right = DotXYZ(vec, m_RightVec);
	float rightLimit = forward*m_fTanXZ;
	float incRadius = bSph.m_RadiusW/m_fCosXZ;
	if (right < (-rightLimit-incRadius) || right > (rightLimit+incRadius)) return 0;

	float up = DotXYZ(vec, m_UpVec);
	float upLimit = forward*m_fTanYZ;
	incRadius = bSph.m_RadiusW/m_fCosYZ;
	if (up <= (-upLimit-incRadius) || up >= (upLimit+incRadius)) return 0;

	return 1;
}

bool CCamera::InFrustum(CAABBox & aABBox)
{
	Vector3F point;
	int numOutOfLeft=0, numOutOfRight=0;
	int numOutOfFar=0,numOutOfNear=0;
	int numOutOfTop=0, numOutOfBottom=0;
	int outSide0=0, outSide1=0;
	bool isInRightTest, isInUpTest, isInFrontTest;

	Vector3F corners[2];
	corners[0] = aABBox.m_MinPW-m_Pos;
	corners[1] = aABBox.m_MaxPW-m_Pos;

	for (WORD i=0; i<8; i++) {
		isInRightTest = isInUpTest = isInFrontTest = FALSE;
		point.x = corners[i&1].x;
		point.y = corners[(i>>2)&1].y;
		point.z = corners[(i>>1)&1].z;

		float forward = DotXYZ(point, m_ForwardVec);
		outSide0 = (forward < m_fNearField), numOutOfNear += outSide0;		
		outSide1 = (forward > m_fFarField), numOutOfFar += outSide1;
		isInFrontTest = (outSide0+outSide1) == 0;

		float right = DotXYZ(point, m_RightVec);
		outSide0 = (right < (-forward*m_fTanXZ)), numOutOfLeft += outSide0;
		outSide1 = (right > (forward*m_fTanXZ)), numOutOfRight += outSide1;
		isInRightTest = (outSide0+outSide1) == 0;
		
		float up = DotXYZ(point, m_UpVec);
		outSide0 = (up < (-forward*m_fTanYZ)), numOutOfTop += outSide0;
		outSide1 = (up > (forward*m_fTanYZ)), numOutOfBottom += outSide1;
		isInUpTest = (outSide0+outSide1) == 0;

		if (isInRightTest && isInFrontTest && isInUpTest) return 1;
	}

	if (numOutOfFar==8 || numOutOfNear==8 || numOutOfLeft==8 || numOutOfRight==8  ||
		numOutOfTop==8 || numOutOfBottom==8 ) return 0;

	return 1;
}
*/
void CCamera::GetFrustumRay(Vector3F & rayPos, Vector3F & rayVector)
{
	POINT point;
	GetCursorPos(&point);
	ScreenToClient(g_hWnd, &point);
	Vector2F point2F((float)point.x, (float)point.y);

	GetFrustumRay(point2F, rayPos, rayVector);
}

void CCamera::GetFrustumRay(Vector2F & windowCoord, Vector3F & rayPos, Vector3F & rayVector)
{
	rayPos = m_Pos;

	rayVector.x = (2.f*windowCoord.x/m_fWidth - 1.f)/m_Proj.m[0][0];
	rayVector.y = (-2.f*windowCoord.y/m_fHeight + 1.f)/m_Proj.m[1][1];
	rayVector.z = 1.f;

	rayVector = rayVector*m_View;
	rayVector = rayVector.Normalize();
}

CCamera CCamera::operator=(CCamera & ref)
{
	memcpy(this, &ref, sizeof(CCamera));
	return *this;
}


};
