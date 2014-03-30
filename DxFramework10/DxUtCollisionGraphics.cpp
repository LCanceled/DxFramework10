
#include "DxUtCollisionGraphics.h"

namespace DxUt {

CMeshPNT CCollisionGraphics::m_Box;
CMeshPNT CCollisionGraphics::m_Sphere;
ID3D10Buffer * CCollisionGraphics::m_pTriVertex;

CCamera * CCollisionGraphics::m_pCam;
SLightDir CCollisionGraphics::m_Light;

CCollisionGraphics::CCollisionGraphics()
{
	m_Light.amb = D3DXCOLOR(.5f, .5f, .5f, 1.f); 
	m_Light.dif = D3DXCOLOR(.8f, .8f, .8f, 1.f); 
	m_Light.spe = D3DXCOLOR(.6f, .6f, .6f, 1.f);
	m_Light.vec = DxUt::Vector3F(-.707f, -.707f, 0);
}

void CCollisionGraphics::CreateGraphics()
{
	m_Box.LoadMeshFromFile("Box.txt", D3DX10_MESH_32_BIT, Vector3F(1.f));
	m_Sphere.LoadMeshFromFile("Sphere.txt", D3DX10_MESH_32_BIT, Vector3F(.5f));

	/* Create the triangle */
    D3D10_BUFFER_DESC bufferDesc;
    bufferDesc.Usage            = D3D10_USAGE_DYNAMIC;
    bufferDesc.ByteWidth        = sizeof(SVertexPNT) * 3;
    bufferDesc.BindFlags        = D3D10_BIND_VERTEX_BUFFER;
    bufferDesc.CPUAccessFlags   = D3D10_CPU_ACCESS_WRITE;
    bufferDesc.MiscFlags        = 0;

	SVertexPNT verts[] = {
		SVertexPNT(0, 0, 0, 0, 0, -1.f, 0, 1),
		SVertexPNT(20.f, 0, 0, 0, 0, -1.f, 1, 1),
		SVertexPNT(0, 20.f, 0, 0, 0, -1.f, 0, 0),
	};
	
    D3D10_SUBRESOURCE_DATA InitData;
    InitData.pSysMem = verts;
	InitData.SysMemPitch = 0;
	InitData.SysMemSlicePitch = 0;
    if (FAILED(g_pD3DDevice->CreateBuffer(&bufferDesc, &InitData, &m_pTriVertex))) {
		DxUtSendError("CCollisionGraphics::CreateGraphics() could not create the vertex buffer for the triangle.");
	}
}

void CCollisionGraphics::SetupDraw()
{
	m_Box.SetupDraw(m_pCam, m_Light);
}

void CCollisionGraphics::DrawPoint(Vector3F & pos, float scale, Vector3F & color)
{
	DxUt::Matrix4x4F world, scaling, trans;
	scaling.MScaling(Vector3F(scale));	
	trans.MTranslation(pos);
	world = (trans*scaling);
	m_Box.DrawAllSubsets(m_pCam, world, 0);
}

void CCollisionGraphics::DrawPointArray(CArray<Vector3F> & positions, float scale, Vector3F & color)
{
	DxUt::Matrix4x4F world, scaling, trans;
	scaling.MScaling(Vector3F(scale));
		
	Vector3F * pPos = positions.GetData();
	for (DWORD i=0, end=positions.GetSize(); i<end; i++) {
		trans.MTranslation(pPos[i]);

		world = (trans*scale);
		m_Sphere.DrawAllSubsets(m_pCam, world, 0);
	}
}

void CCollisionGraphics::DrawNormal(Vector3F & pos, Vector3F & dir, float crossSection, float len, Vector3F & color)
{
	DxUt::Matrix4x4F world, scale, trans, rot;
	scale.MScaling(crossSection, len, crossSection);
		
	Vector3F up(0, 1.f, 0);
	Vector3F rotVec = -CrossXYZ(up, dir);
	float angle = DotXYZ(up, dir);
	rot.MRotationAxisLH(rotVec, acos(angle));
	trans.MTranslation(pos + len * dir);
	world = (trans*rot*scale);

	m_Box.DrawAllSubsets(m_pCam, world, 0);
}

void CCollisionGraphics::DrawContactPoints(CArray<SContactPoint> * CPs)
{
	float fHalfNormalLen = .5f * .4f;
	DxUt::Matrix4x4F world, scale, trans, rot;
	scale.MScaling(.02f, 2.f*fHalfNormalLen + .01f, .02f);
	SMaterial mat;
	mat.amb = D3DXCOLOR(.2f, .5f, .3f, 1.f);
	mat.dif = D3DXCOLOR(.6f, 1.f, .6f, 1.f);
	mat.spe = D3DXCOLOR(.2f, .2f, .2f, 1.f);
	mat.pow = 20.f;
	
	SContactPoint * pCP = CPs->GetData();
	for (DWORD i=0, end=CPs->GetSize(); i<end; i++) {
		
		Vector3F up(0, 1.f, 0);
		Vector3F dir = pCP[i].iNor;
		Vector3F rotVec = -CrossXYZ(up, dir);
		float angle = DotXYZ(up, dir);
		rot.MRotationAxisLH(rotVec, acos(angle));
		trans.MTranslation(pCP[i].iPos + 2.f*fHalfNormalLen * dir);
		world = (trans*rot*scale);
		SMaterial _mat = mat;
		if (pCP[i].dwFaceIndex[0] == 1) {
			_mat.amb = D3DXCOLOR(.5f, .3f, .3f, 1.f);
			_mat.dif = D3DXCOLOR(1.f, .6f, .6f, 1.f);
		}
		else if (pCP[i].dwFaceIndex[0] == 2) {
			_mat.amb = D3DXCOLOR(.3f, .3f, .5f, 1.f);
			_mat.dif = D3DXCOLOR(.6f, .6f, 1.f, 1.f);
		}
		m_Box.DrawAllSubsets(m_pCam, world, 0, &_mat);
	}
}

void CCollisionGraphics::DrawBox(Vector3F & pos, Vector3F & halfWidths, Matrix4x4F & rot)
{
	DxUt::Matrix4x4F world, scale, trans;
	scale.MScaling(halfWidths);
	trans.MTranslation(pos);
	world = (trans*rot*scale);

	m_Box.DrawAllSubsets(m_pCam, world, 0);
}

void CCollisionGraphics::DrawBox(Matrix4x4F & rotTrans, Vector3F & halfWidths)
{
	DxUt::Matrix4x4F world, scale, trans;
	scale.MScaling(halfWidths);
	world = (rotTrans*scale);

	m_Box.DrawAllSubsets(m_pCam, world, 0);
}

void CCollisionGraphics::SetupTriangleDraw(Vector3F & color)
{
	CPNTPhongFx & effect = m_Box.GetEffect();
	UINT stride = sizeof(SVertexPNT);
	UINT offset = 0;
	g_pD3DDevice->IASetVertexBuffers(0, 1, &m_pTriVertex, &stride, &offset);
	g_pD3DDevice->IASetInputLayout(effect.eVertexLayout);
	g_pD3DDevice->IASetPrimitiveTopology(D3D10_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

	effect.eCamPos->SetRawValue(&m_pCam->GetPosition(), 0, sizeof(DxUt::Vector3F));
	
	effect.eLight->SetRawValue(&m_Light, 0, sizeof(DxUt::SLightDir));

	effect.eTexture->SetResource(m_Box.GetSHView(0));

	DxUt::SMaterial mat;
	mat.spe = D3DXCOLOR(.8f, .8f, .8f, 1.f);
	mat.pow = 16.f;
	mat.amb = D3DXCOLOR(.8f*color.x, .8f*color.y, .8f*color.z, 1.f);
	mat.dif = D3DXCOLOR(color.x, color.y, color.z, 1.f);
	effect.eMaterial->SetRawValue(&mat, 0, sizeof(SMaterial));
}

void CCollisionGraphics::DrawTriangle(Vector3F v1, Vector3F v2, Vector3F v3, Vector3F * color, float scale)
{
	Vector3F dir1(v2 - v1);
	Vector3F dir2(v3 - v1);
	Vector3F n(CrossXYZ(dir1, dir2).Normalize());
	Vector3F ctd((v1+v2+v3)/3.f);
	v1 = ctd + scale*(v1 - ctd);
	v2 = ctd + scale*(v2 - ctd);
	v3 = ctd + scale*(v3 - ctd);
	SVertexPNT verts[] = {
		SVertexPNT(v1.x, v1.y, v1.z, n.x, n.y, n.z, 0, 1),
		SVertexPNT(v2.x, v2.y, v2.z, n.x, n.y, n.z, 1, 1),
		SVertexPNT(v3.x, v3.y, v3.z, n.x, n.y, n.z, 0, 0),
	};
	void * pData;
	if (FAILED(m_pTriVertex->Map(D3D10_MAP_WRITE_DISCARD, 0, (void**)&pData)))
		DebugBreak();
	memcpy(pData, verts, sizeof(verts));
	m_pTriVertex->Unmap();

	CPNTPhongFx & effect = m_Box.GetEffect();
	if (color) {
		DxUt::SMaterial mat;
		mat.spe = D3DXCOLOR(.8f, .8f, .8f, 1.f);
		mat.pow = 16.f;
		mat.amb = D3DXCOLOR(.8f*color->x, .8f*color->y, .8f*color->z, 1.f);
		mat.dif = D3DXCOLOR(color->x, color->y, color->z, 1.f);
		effect.eMaterial->SetRawValue(&mat, 0, sizeof(SMaterial));
	}
	
	effect.eWVP->SetMatrix((float*)&(m_pCam->GetView()*m_pCam->GetProjection()));
	effect.eWorld->SetMatrix((float*)&Matrix4x4F(1.f));

	effect.eTech->GetPassByIndex(0)->Apply(0);

	g_pD3DDevice->Draw(3, 0);
}

void CCollisionGraphics::DestroyGraphics()
{
	m_Box.Destroy();
	m_Sphere.Destroy();
	ReleaseX(m_pTriVertex);
}


};