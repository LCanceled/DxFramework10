
#ifndef DXUTMESH_H
#define DXUTMESH_H

#include "DxUtInclude.h"
#include "DxUtLight.h"
#include "DxUtMaterial.h"
#include "DxUtVertex.h"
#include "DxUtCamera.h"

namespace DxUt {

class CMesh {
protected:
	ID3DX10Mesh * m_pMesh;
	DWORD m_dwStride;
	DWORD m_nSubsets; 

	SMaterial * m_rgMat; 
	ID3D10ShaderResourceView ** m_rgSRView;

	//Creates an empty mesh
	//virtual void CreateMesh(DWORD nTri, DWORD nVert, DWORD dwOptions, 
	//	const D3D10_INPUT_ELEMENT_DESC * aDesc, DWORD cDesc, DWORD nSubsets);

	virtual void TextureCreationHook(char * szTexFile, DWORD i) {}
public:
	CMesh();
	virtual ~CMesh() {}

	void CMesh::CreateMesh(DWORD nTri, DWORD nVert, DWORD dwOptions, 
		const D3D10_INPUT_ELEMENT_DESC * aDesc, DWORD cDesc, DWORD nSubsets);

	/* Loads a PNT mesh from a txt file scaled by fScale */
	virtual void LoadMeshFromFile(char * szMeshFile, DWORD dwOptions, Vector3F & scale);

	virtual void SetupDraw(CCamera * pCam, SLightDir & light) {}
	virtual void DrawAllSubsets(CCamera * pCam, Matrix4x4F & world, DWORD dwShaderPass, SMaterial * pOverrideMaterial=NULL) {}

	/*//Sets the SMaterial and sRV of a subset to effect handels
	virtual void SetMatEffectHandels(ID3D10EffectVariable *& eMat,
		ID3D10EffectShaderResourceVariable *& eSRV, DWORD dwSubset);

	//For each subset sets the SMaterial and sRV to effect handels,
	//commits the changes on dwShaderPass , and draws that subset
	virtual void DrawAllSubsets(ID3D10EffectTechnique *& eTech, ID3D10EffectVariable *& eMat, 
		ID3D10EffectShaderResourceVariable *& eSRV, DWORD dwShaderPass);
	*/

	ID3DX10Mesh * GetMesh() {return m_pMesh;}
	DWORD GetNumTriangles() {return m_pMesh->GetFaceCount();}
	DWORD GetNumVertices() {return m_pMesh->GetVertexCount();}
	DWORD GetStride() {return m_dwStride; }

	DWORD & GetNumSubsets() {return m_nSubsets;}
	DWORD GetNumMaterial() {return m_nSubsets;}
	SMaterial & GetMaterial(DWORD dwIndex);
	ID3D10ShaderResourceView * GetSHView(DWORD dwIndex); 

	ID3DX10Mesh *& operator->() {
		return m_pMesh; }

	virtual void DestroyMesh();
};

inline SMaterial & CMesh::GetMaterial(DWORD dwIndex)
{
	Assert(dwIndex < m_nSubsets, "CMesh::GetMaterial the material index is out of range.");

	return m_rgMat[dwIndex];
}

inline ID3D10ShaderResourceView * CMesh::GetSHView(DWORD dwIndex)
{
	Assert(dwIndex < m_nSubsets, " CMesh::GetSHView the SHView index is out of range.");

	return m_rgSRView[dwIndex];
}

void ExtractVerticesFromMesh(ID3DX10Mesh * pMesh, Vector3F * rgVert, DWORD dwStride);

void ExtractVertexTriangleListFromMesh(ID3DX10Mesh * pMesh, Vector3F * rgVert, DWORD dwStride);

/* pMesh must have adjancey information */
void ExtractAdjanceyFromMesh(ID3DX10Mesh * pMesh, DWORD * rgAdj);


};


#endif
