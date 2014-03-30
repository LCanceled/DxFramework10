
#ifndef DXUTMESH_H
#define DXUTMESH_H

#include "DxUtInclude.h"
#include "DxUtLight.h"
#include "DxUtMaterial.h"
#include "DxUtVertex.h"
#include "DxUtCamera.h"
#include "DxUtTriangle.h"
#include "DxUtArray.h"
#include <string>

namespace DxUt {

class CMesh {
protected:
	std::string m_Name;
	ID3DX10Mesh * m_pMesh;
	DWORD m_dwStride;
	DWORD m_nSubsets; 
	DWORD m_nVertices;
	DWORD m_nFaces;

	SMaterial * m_Materials; 
	// TODO: Add a Texture class on
	ID3D10ShaderResourceView ** m_SRViews;

	/* Note that some of these vertices may be degenerate */
	Vector3F * m_Vertices;
	STriangleF * m_Tris;
	DWORD * m_Adj;

	virtual void TextureCreationHook(char * szTexFile, DWORD i) {}
public:
	CMesh();
	virtual ~CMesh() {}

	//void CMesh::CreateMesh(DWORD nTri, DWORD nVert, DWORD dwOptions, 
	//	const D3D10_INPUT_ELEMENT_DESC * aDesc, DWORD cDesc, DWORD nSubsets, char * szName);

	/* Loads a PNT mesh from a txt file scaled by scale */
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
	DWORD GetNumTriangles() {return m_nFaces;}
	DWORD GetNumVertices() {return m_nVertices;}
	DWORD GetStride() {return m_dwStride; }

	Vector3F * GetVertices() const {return m_Vertices;}
	STriangleF * GetTriangles() const {return m_Tris;}
	DWORD * GetAdjancey() const {return m_Adj;}

	Vector3F * GetNewVertexTriangleList() {
		Vector3F * vert = new Vector3F[3*m_nFaces];
		for (int i=0; i<3*m_nFaces; i++) {vert[i] = m_Tris[i/3].vPosW[i%3]; }
		return vert;
	}

	DWORD & GetNumSubsets() {return m_nSubsets;}
	DWORD GetNumMaterial() {return m_nSubsets;}
	SMaterial & GetMaterial(DWORD dwIndex);
	ID3D10ShaderResourceView * GetSHView(DWORD dwIndex); 

	//ID3DX10Mesh *& operator->() {
	//	return m_pMesh; }

	virtual void ShallowCopy(CMesh * pMesh);

	virtual void Destroy();
};

inline SMaterial & CMesh::GetMaterial(DWORD dwIndex)
{
	Assert(dwIndex < m_nSubsets, "CMesh::GetMaterial the material index is out of range.");

	return m_Materials[dwIndex];
}

inline ID3D10ShaderResourceView * CMesh::GetSHView(DWORD dwIndex)
{
	Assert(dwIndex < m_nSubsets, "CMesh::GetSHView the SHView index is out of range.");

	return m_SRViews[dwIndex];
}



};


#endif
