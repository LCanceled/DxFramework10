
#ifndef COLLISIONGRAPHICS_H
#define COLLISIONGRAPHICS_H

#include "DxUtInclude.h"
#include "DxUtTriangle.h"
#include "DxUtArray.h"
#include "DxUtFPCamera.h"
#include "DxUtMeshPNT.h"

namespace DxUt {

class CCollisionGraphics {
private:
	friend class CD3DApp;

	static CMeshPNT m_Box;
	static CMeshPNT m_Sphere;
	static ID3D10Buffer * m_pTriVertex;

	static CCamera * m_pCam;
	
	static SLightDir m_Light;

	CCollisionGraphics();
	//~CCollisionGraphics() {}

	void CreateGraphics();
public:
	static void SetCamera(CCamera * pCam) {m_pCam = pCam; }
	static void SetLight(SLightDir & light) {m_Light = light; }

	static void SetupDraw();

	static void DrawPoint(Vector3F & pos, float scale, Vector3F & color);
	static void DrawPointArray(CArray<Vector3F> & positions, float scale, Vector3F & color);

	static void DrawNormal(Vector3F & pos, Vector3F & dir, float crossSection, float len, Vector3F & color);

	static void DrawContactPoints(CArray<SContactPoint> * CPs);

	static void DrawBox(Vector3F & pos, Vector3F & halfWidths, Matrix4x4F & rot);
	static void DrawBox(Matrix4x4F & rotTrans, Vector3F & halfWidths);

	static void SetupTriangleDraw(Vector3F & color);
	static void DrawTriangle(Vector3F v1, Vector3F v2, Vector3F v3, Vector3F * color=NULL, float scale = .9f);

	static void DestroyGraphics();
};


};

#endif
