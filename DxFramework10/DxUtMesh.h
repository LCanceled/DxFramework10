
#ifndef DXUTMESH_H
#define DXUTMESH_H

#include "DxUtInclude.h"
#include "DxUtLight.h"
#include "DxUtMaterial.h"
#include "DxUtVertex.h"
#include "DxUtCamera.h"
#include "DxUtTriangle.h"
#include "DxUtArray.h"
#include "DxUtResourcePool.h"
#include <string>

namespace DxUt {

class CMesh : public CPooledResource<CMesh> {
protected:
	std::string m_Name;
	ID3DX10Mesh * m_pMesh;
	UINT m_uiStride;
	UINT m_nSubsets; 
	UINT m_nVertices;
	UINT m_nFaces;

	SMaterial * m_Materials; 
	// TODO: Add a Texture class on
	ID3D10ShaderResourceView ** m_SRViews;

	/* Note that some of these vertices may be degenerate */
	Vector3F * m_Vertices;
	STriangleF * m_Tris;
	UINT * m_Adj;

	virtual void TextureCreationHook(char * szTexFile, UINT i) {}

	void LoadMeshFromFileTXT(char * szMeshFile, UINT uiOptions, const Vector3F & scale);
	void LoadMeshFromFileOFF(char * szMeshFile, UINT uiOptions, const Vector3F & scale);
public:
	CMesh();
	virtual ~CMesh() {}

	//void CMesh::CreateMesh(UINT nTri, UINT nVert, UINT uiOptions, 
	//	const D3D10_INPUT_ELEMENT_DESC * aDesc, UINT cDesc, UINT nSubsets, char * szName);

	/* Loads a PNT mesh from a txt file scaled by scale */
	virtual void LoadMeshFromFile(char * szMeshFile, UINT uiOptions, const Vector3F & scale);

	virtual void SetupDraw(CCamera * pCam, SLightDir & light) {}
	virtual void DrawAllSubsets(CCamera * pCam, const Matrix4x4F & world, UINT uiShaderPass, SMaterial * pOverrideMaterial=NULL) {}

	/*//Sets the SMaterial and sRV of a subset to effect handels
	virtual void SetMatEffectHandels(ID3D10EffectVariable *& eMat,
		ID3D10EffectShaderResourceVariable *& eSRV, UINT uiSubset);

	//For each subset sets the SMaterial and sRV to effect handels,
	//commits the changes on uiShaderPass , and draws that subset
	virtual void DrawAllSubsets(ID3D10EffectTechnique *& eTech, ID3D10EffectVariable *& eMat, 
		ID3D10EffectShaderResourceVariable *& eSRV, UINT uiShaderPass);
	*/

	ID3DX10Mesh * GetMesh() {return m_pMesh;}
	UINT GetNumTriangles() {return m_nFaces;}
	UINT GetNumVertices() {return m_nVertices;}
	UINT GetStride() {return m_uiStride; }

	Vector3F * GetVertices() const {return m_Vertices;}
	STriangleF * GetTriangles() const {return m_Tris;}
	UINT * GetAdjancey() const {return m_Adj;}

	Vector3F * GetNewVertexTriangleList() {
		Vector3F * vert = new Vector3F[3*m_nFaces];
		for (UINT i=0; i<3*m_nFaces; i++) {vert[i] = m_Tris[i/3].vPosW[i%3]; }
		return vert;
	}

	UINT & GetNumSubsets() {return m_nSubsets;}
	UINT GetNumMaterial() {return m_nSubsets;}
	SMaterial & GetMaterial(UINT uiIndex);
	ID3D10ShaderResourceView * GetSHView(UINT uiIndex); 

	//ID3DX10Mesh *& operator->() {
	//	return m_pMesh; }

	virtual void ShallowCopy(CMesh & rhs);

	virtual void Destroy();
};

inline SMaterial & CMesh::GetMaterial(UINT uiIndex)
{
	Assert(uiIndex < m_nSubsets, "CMesh::GetMaterial the material index is out of range.");

	return m_Materials[uiIndex];
}

inline ID3D10ShaderResourceView * CMesh::GetSHView(UINT uiIndex)
{
	Assert(uiIndex < m_nSubsets, "CMesh::GetSHView the SHView index is out of range.");

	return m_SRViews[uiIndex];
}



};


#endif
