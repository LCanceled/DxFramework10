
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
	CMeshPNT m_Box;
	CMeshPNT m_Sphere;
	ID3D10Buffer * m_pTriVertex;

	CCamera * m_pCam;
public:
	CCollisionGraphics() {}
	//~CCollisionGraphics() {}

	void CreateGraphics();

	void SetCamera(CCamera * pCam) {m_pCam = pCam; }

	void SetupDraw();

	void DrawPoint(Vector3F & pos, float fSize, Vector3F & color);
	void DrawPointArray(CArray<Vector3F> & rgPos, float fSize, Vector3F & color);

	void DrawNormal(Vector3F & pos, Vector3F & dir, float fCrossSection, float fLen, Vector3F & color);

	void DrawContactPoints(CArray<SContactPoint> * rgCP);

	void DrawBox(Vector3F & pos, Vector3F & halfWidths, Matrix4x4F & rot);
	void DrawBox(Matrix4x4F & rotTrans, Vector3F & halfWidths);

	void SetupTriangleDraw(Vector3F & color);
	void DrawTriangle(Vector3F v1, Vector3F v2, Vector3F v3, Vector3F * color=NULL, float fScale = .9f);

	void DestroyGraphics();
};


};

#endif
