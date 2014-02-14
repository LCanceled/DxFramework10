
#ifndef DXUTMESHPNT_H
#define DXUTMESHPNT_H

#include "DxUtMesh.h"
#include "DxUtShaders.h"
#include "DxUtCamera.h"

namespace DxUt {

class CMeshPNT : public CMesh {
private:
	SLightDir m_Light;
	CPNTPhongFx * m_Effect;
public:
	void LoadMeshFromFile(char * szMeshFile, DWORD dwOptions, Vector3F & scale, char * szFxFile=NULL);
	void SetEffect(char * szFxFile);

	void SetupDraw(CCamera * pCam, SLightDir & light);
	void DrawAllSubsets(CCamera * pCam, Matrix4x4F & world, DWORD dwShaderPass, SMaterial * pOverrideMaterial=NULL);

	CPNTPhongFx & GetEffect() {return *m_Effect; }
};


};


#endif

