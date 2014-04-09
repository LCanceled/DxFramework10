
#include "DxUtMeshPNT.h"
#include "DxUtEffectPool.h"

namespace DxUt {
	
void CMeshPNT::LoadMeshFromFile(char * szMeshFile, UINT uiOptions, const Vector3F & scale, char * szFxFile)
{
	CMesh::LoadMeshFromFile(szMeshFile, uiOptions, scale);
	m_Effect.CreateEffect(szFxFile);
}
 
void CMeshPNT::SetupDraw(CCamera * pCam, SLightDir & light)
{
	g_pD3DDevice->IASetInputLayout(m_Effect.eVertexLayout);
	g_pD3DDevice->IASetPrimitiveTopology(D3D10_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

	m_Effect.eCamPos->SetRawValue(&pCam->GetPosition(), 0, sizeof(DxUt::Vector3F));
	
	m_Effect.eLight->SetRawValue(&light, 0, sizeof(DxUt::SLightDir));
}
	
void CMeshPNT::DrawAllSubsets(CCamera * pCam, const Matrix4x4F & world, UINT uiShaderPass, SMaterial * pOverrideMaterial)
{
	world.Transpose();
	Matrix4x4F wT = world.Transpose();
	m_Effect.eWVP->SetMatrix((float*)&(wT*pCam->GetView()*pCam->GetProjection()));
	m_Effect.eWorld->SetMatrix((float*)&wT); //Not correct under translations

	m_Effect.eTexture->SetResource(m_SRViews[0]);

	for (UINT i=0; i<m_nSubsets; i++) {
		if (pOverrideMaterial) {
			m_Effect.eMaterial->SetRawValue(pOverrideMaterial, 0, sizeof(SMaterial));
		} else {
			m_Effect.eMaterial->SetRawValue(&m_Materials[i], 0, sizeof(SMaterial));
		}

		m_Effect.eTech->GetPassByIndex(uiShaderPass)->Apply(0);

		m_pMesh->DrawSubset(i);
	}
}


};