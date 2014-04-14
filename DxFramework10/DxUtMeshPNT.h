
#ifndef DXUTMESHPNT_H
#define DXUTMESHPNT_H

#include "DxUtMesh.h"
#include "DxUtShaders.h"
#include "DxUtCamera.h"

namespace DxUt {

class CMeshPNT : public CMesh {
protected:
	SLightDir m_Light;
	CPNTPhongFx m_Effect;
public:
	void LoadMeshFromFile(char * szMeshFile, UINT uiOptions, const Vector3F & scale, char * szFxFile=NULL);

	void SetupDraw(CCamera * pCam, SLightDir & light);
	void DrawAllSubsets(CCamera * pCam, const Matrix4x4F & world, UINT uiShaderPass, SMaterial * pOverrideMaterial=NULL);

	CPNTPhongFx & GetEffect() {return m_Effect; }

	virtual void Destroy();
};


};


#endif

