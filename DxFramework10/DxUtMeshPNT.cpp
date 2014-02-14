
#include "DxUtMeshPNT.h"
#include "DxUtEffectPool.h"

namespace DxUt {
	
void CMeshPNT::LoadMeshFromFile(char * szMeshFile, DWORD dwOptions, Vector3F & scale, char * szFxFile)
{
	CMesh::LoadMeshFromFile(szMeshFile, dwOptions, scale);
	SetEffect(szFxFile);
}

 void CMeshPNT::SetEffect(char * szFxFile)
 {
	if (!CEffectPool::GetEffect(szFxFile ? szFxFile : "/PNT_Phong.fx", &m_Effect)) {
		CPNTPhongFx & fx = *m_Effect;
		fx.eTech		= fx->GetTechniqueByIndex(0);
		fx.eWVP			= fx->GetVariableByName("g_WVP")->AsMatrix();
		fx.eWorld		= fx->GetVariableByName("g_World")->AsMatrix();
		fx.eMaterial	= fx->GetVariableByName("g_Mat");
		fx.eLight		= fx->GetVariableByName("g_Light");
		fx.eCamPos		= fx->GetVariableByName("g_CamPos");
		fx.eTexture		= fx->GetVariableByName("g_Tex")->AsShaderResource();

		DxUt::CreateInputLayout(fx.eTech, DxUt::GetVertexElementDescPNT(), 3, 0, fx.eVertexLayout);
	}
 }

 
void CMeshPNT::SetupDraw(CCamera * pCam, SLightDir & light)
{
	g_pD3DDevice->IASetInputLayout(m_Effect->eVertexLayout);
	g_pD3DDevice->IASetPrimitiveTopology(D3D10_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

	m_Effect->eCamPos->SetRawValue(&pCam->GetPosition(), 0, sizeof(DxUt::Vector3F));
	
	m_Effect->eLight->SetRawValue(&light, 0, sizeof(DxUt::SLightDir));
}
	
void CMeshPNT::DrawAllSubsets(CCamera * pCam, Matrix4x4F & world, DWORD dwShaderPass, SMaterial * pOverrideMaterial)
{
	Matrix4x4F & wT = world.MTranspose();
	m_Effect->eWVP->SetMatrix((float*)&(wT*pCam->GetView()*pCam->GetProjection()));
	m_Effect->eWorld->SetMatrix((float*)&wT); //Not correct under translations

	m_Effect->eTexture->SetResource(m_rgSRView[0]);

	for (DWORD i=0; i<m_nSubsets; i++) {
		if (pOverrideMaterial) {
			m_Effect->eMaterial->SetRawValue(pOverrideMaterial, 0, sizeof(SMaterial));
		} else {
			m_Effect->eMaterial->SetRawValue(&m_rgMat[i], 0, sizeof(SMaterial));
		}

		m_Effect->eTech->GetPassByIndex(dwShaderPass)->Apply(0);

		m_pMesh->DrawSubset(i);
	}
}


};