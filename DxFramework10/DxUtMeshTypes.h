
#ifndef DXUTMESHTYPES_H
#define DXUTMESHTYPES_H

#include "DxUtMesh.h"
#include "DxUtShaders.h"
#include "DxUtLight.h"

namespace DxUt {

class CMeshParallaxMapped : public CMesh {
private:
	ID3D10ShaderResourceView ** m_SRViewsNH;
	CParallaxMappingFx m_Effect;

	void TextureCreationHook(char * szTexFile, UINT i);
public:
	//Loads a PNT mesh from a txt file. It is required that for each texture
	//of the mesh there is a corresponding normal/height texture with the ending
	//NH before the extension. Also, Note that the shader creates the TBN frame.
	void LoadMeshFromFile(char * szMeshFile, UINT uiOptions);

	ID3D10ShaderResourceView * GetSRViewHN(UINT uiIndex); 

	void SetShaderVariablesPerScene(SLightDir & light, float texTile, float heightScale);
	void Draw(Matrix4x4F & world, Matrix4x4F & worldViewProj, Vector3F & camPos);

	void DestroyMesh();
};

inline ID3D10ShaderResourceView * CMeshParallaxMapped::GetSRViewHN(UINT uiIndex)
{
#if defined (DEBUG) | defined (_DEBUG)
	if (uiIndex >= m_nSubsets) {
		DxUtSendError("The texture index for the mesh is out of range."); }
#endif

	return m_SRViewsNH[uiIndex];
}


};


#endif

