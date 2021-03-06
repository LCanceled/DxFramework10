
#include "DxUtMeshTypes.h"

namespace DxUt {

void CMeshParallaxMapped::TextureCreationHook(char * szTexFile, UINT i)
{
	char szTexFileNH[MAX_PATH];
	strcpy(szTexFileNH, szTexFile);
	char * pExt = strchr(szTexFile, '.');
	char * pExtNH = szTexFileNH + (pExt - szTexFile); 
	pExtNH[0] = 'N';
	pExtNH[1] = 'H';

	int j=0;
	for (; pExt[j] != '\0'; j++)
		pExtNH[j+2] = pExt[j];
	pExtNH[j+2] = '\0';

	if (!m_SRViewsNH) {
		m_SRViewsNH = new ID3D10ShaderResourceView*[m_nSubsets];
	}
	if (FAILED(D3DX10CreateShaderResourceViewFromFileA(g_pD3DDevice, szTexFileNH, 0, 0, &m_SRViewsNH[i], 0))) {
		DxUtSendErrorEx("LoadMeshFromFile could not load mesh HN texture.", szTexFile);
	}
}

//Loads a PNT mesh from a txt file. It is required that for each texture
//of the mesh there is a corresponding normal/height texture with the ending
//NH before the extension. Also, Note that the shader creates the TBN frame.
void CMeshParallaxMapped::LoadMeshFromFile(char * szMeshFile, UINT uiOptions)
{
	CMesh::LoadMeshFromFile(szMeshFile, uiOptions, Vector3F(1.f));

	DebugBreak();

	/*m_Effect.CreateEffect("PNT_Phong.fx");
	m_Effect.eTech		= m_Effect->GetTechniqueByIndex(0);
	m_Effect.eWVP		= m_Effect->GetVariableByName("g_WVP")->AsMatrix();
	m_Effect.eWorld		= m_Effect->GetVariableByName("g_World")->AsMatrix();
	m_Effect.eMaterial	= m_Effect->GetVariableByName("g_Mat");
	m_Effect.eLight		= m_Effect->GetVariableByName("g_Light");
	m_Effect.eCamPos	= m_Effect->GetVariableByName("g_CamPos");
	m_Effect.eTexture	= m_Effect->GetVariableByName("g_Tex")->AsShaderResource();
	m_Effect.eNHTexture = m_Effect->GetVariableByName("g_TexNH")->AsShaderResource();
	m_Effect.eTextureTile = m_Effect->GetVariableByName("g_TexTile")->AsScalar();
	m_Effect.eHeightScale = m_Effect->GetVariableByName("g_HeightScale")->AsScalar();*/

	//Input layout
	//DxUt::CreateInputLayout(m_Effect.eTech, DxUt::GetVertexElementDescPNT(), 3, 0, m_Effect.eVertexLayout);
}

void CMeshParallaxMapped::SetShaderVariablesPerScene(SLightDir & light, float texTile, float heightScale)
{
	DebugBreak();
	/*m_Effect.eLight->SetRawValue(&light, 0, sizeof(DxUt::SLightDir));
	m_Effect.eTextureTile->SetFloat(texTile);
	m_Effect.eHeightScale->SetFloat(heightScale);*/
}

void CMeshParallaxMapped::Draw(Matrix4x4F & world, Matrix4x4F & worldViewProj, Vector3F & camPos)
{
	DebugBreak();
	/*g_pD3DDevice->IASetInputLayout(m_Effect.eVertexLayout);

	m_Effect.eCamPos->SetRawValue(&camPos, 0, sizeof(Vector3F));

	g_pD3DDevice->IASetInputLayout(m_Effect.eVertexLayout);
	g_pD3DDevice->IASetPrimitiveTopology(D3D10_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

	m_Effect.eWVP->SetMatrix((float*)&(worldViewProj));
	m_Effect.eWorld->SetMatrix((float*)&world);

	for (UINT i=0; i<m_nSubsets; i++) {
		m_Effect.eMaterial->SetRawValue(&m_Materials[i], 0, sizeof(SMaterial));
		m_Effect.eTexture->SetResource(m_SRViews[i]);
		m_Effect.eNHTexture->SetResource(m_SRViewsNH[i]);

		m_Effect.eTech->GetPassByIndex(0)->Apply(0);

		m_pMesh->DrawSubset(i);
	}*/
}

void CMeshParallaxMapped::DestroyMesh()
{
	CMesh::Destroy();

	if (m_SRViewsNH) {
		for (UINT i=0; i<m_nSubsets; i++) ReleaseX(m_SRViews[i]);
		delete[] m_SRViews;
		m_SRViews = NULL;
	}
}


};