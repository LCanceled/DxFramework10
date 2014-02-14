
#ifndef RAYTRACER_H
#define RAYTRACER_H

#include "DxUtBVTree.h"
#include "DxUtCamera.h"
#include "DxUtMesh.h"
#include "DxUtTriangle.h"
#include "DxUtRigidBodyWorld.h"
#include "DxUtCollisionGraphics.h"
#include "DxUtRay.h"

namespace DxUt {

struct SLightSource {
	Vector3F amb;
	Vector3F dif;
	Vector3F spe;
	Vector3F vec;
};

struct SVolume {
	/* Indices of refraction */
	Vector4F n;
};

struct SSurface {
	SSurface():bInterpolateNormal(1), bTraceable(1), bReflect(1), bTransmit(0), n(1.f) {}
	//~SSurface() {}

	bool bInterpolateNormal;
	bool bTraceable;
	bool bReflect;
	bool bTransmit;
	/* Indices of refraction */
	Vector4F n;

	virtual Vector3F ComputeEmmitedRadiance(SRay & ray, SRayIntersectData & rayData)=0;
	virtual Vector3F ComputeRaidanceDirect(SRay & ray, SRayIntersectData & rayData, CCamera * pCam, CArray<SLightSource> & rgLight)=0;
	virtual Vector3F ComputeReflectionVec(SRay & ray, SRayIntersectData & data, Vector4F & n12)=0;
	virtual Vector3F ComputeTransmissionVec(SRay & ray, SRayIntersectData & data, Vector4F & n12)=0;
	virtual Vector4F ComputeReflectance(SRay & ray, SRayIntersectData & rayData, Vector4F & n1, Vector4F & n2)=0;
};

struct SSurfaceLambert : public SSurface {
	SSurfaceLambert() {bTraceable = 0; bReflect = 0; }
	//~SSurfaceLambert() {}

	Vector3F amb;
	Vector3F dif;
	Vector3F spe;
	float sPow;

	Vector3F ComputeEmmitedRadiance(SRay & ray, SRayIntersectData & rayData) {return .4f*amb; }
	Vector3F ComputeRaidanceDirect(SRay & ray, SRayIntersectData & rayData, CCamera * pCam, CArray<SLightSource> & rgLight);
	Vector3F ComputeReflectionVec(SRay & ray, SRayIntersectData & data, Vector4F & n12) {return Vector3F(0); }
	Vector3F ComputeTransmissionVec(SRay & ray, SRayIntersectData & data, Vector4F & n12) {return Vector3F(0); }
	Vector4F ComputeReflectance(SRay & ray, SRayIntersectData & rayData, Vector4F & n1, Vector4F & n2) {return 1; }
};

struct SSurfaceFresnel : public SSurface {
	SSurfaceFresnel() {bTraceable = 1; bReflect = 1; bTransmit = 1; }
	//~SSurfaceFresnel() {}

	Vector3F amb;

	Vector3F ComputeEmmitedRadiance(SRay & ray, SRayIntersectData & rayData) {return Vector3F(0); }//Vector3F(0, 0, .4f); }//.4f*amb; }
	Vector3F ComputeRaidanceDirect(SRay & ray, SRayIntersectData & rayData, CCamera * pCam, CArray<SLightSource> & rgLight);
	Vector3F ComputeReflectionVec(SRay & ray, SRayIntersectData & data, Vector4F & n12);
	Vector3F ComputeTransmissionVec(SRay & ray, SRayIntersectData & data, Vector4F & n12);
	Vector4F ComputeReflectance(SRay & ray, SRayIntersectData & rayData, Vector4F & n1, Vector4F & n2);
};

class CRayTracer {
private:
	struct STriangleFEx : public STriangleF {
		Vector3F vNor[3];
		Vector3F vNorWorld[3];

		STriangleFEx & operator=(STriangleF & tri) {
			memcpy(vPosW, tri.vPosW, sizeof(Vector3F)*3);
			dwTriangle = tri.dwTriangle;
			return *this;
		}
	};

	struct SMeshData {
		SMeshData():rgTri(0), nTri(0), dwRBWorldBodyId(-1), pSurface(0) {}
		//~SMeshData() {}
		
		BVTree * m_BVTree;
		/* Pointer directly to m_BVTree.rgAdj */
		DWORD * rgAdj;
		
		/* Enhanced version of triangles found in m_BVTree */
		DWORD nTri;
		STriangleFEx * rgTri;

		/* The mesh instance is independent from RBWorld if dwRBWorldId is -1 */
		DWORD dwRBWorldBodyId;

		/* Rotation and translation */
		Matrix4x4F transform;

		SSurface * pSurface;
	};

	char * m_szScreenShotFile;
	DWORD m_nPixelsX, m_nPixelsY;
	DWORD m_dwScreenShotKey;
	CArray<SMeshData> m_rgMesh;
	
	CCamera m_Camera;
	CArray<SLightSource> m_rgLight;

	SVolume m_AirVolume;

	CRigidBodyWorld * m_pRBWorld;

	DWORD m_nSamples;

	void ComputeNormals(STriangleFEx * rgTri, DWORD nTri, DWORD * rgAdj);
	void TraceScreenShot();
	void DrawToTexture(Vector3F * rgTexValues);
	Vector3F TraceRays(SRay & ray, SVolume & vol, DWORD dwDepth);
	Vector3F InterpolateNormal(SRayIntersectData & data) {
		Vector3F * vNorWorld = m_rgMesh[data.dwBody].rgTri[data.dwTri].vNorWorld;
		return (data.u*vNorWorld[0] + data.v*vNorWorld[1] + data.w*vNorWorld[2]).Normalize();
	}

	friend DWORD WINAPI ThreadFunc(LPVOID lpParameter);

	bool IntersectRay(SRay & ray, SRayIntersectData & data);
public:
	CRayTracer();
	//~CRayTracer() {}

	void CreateRayTracer(char * szScreenShotFile, DWORD nPixelsX, DWORD nPixelsY,
		DWORD nSamples, DWORD dwScreenShotKey, DWORD nHintMeshes, CRigidBodyWorld * rBWorld=NULL);
	
	/* dwMeshId represents an index into an array */
	void AddMesh(CMesh & mesh, SSurface * pSurface);
	void AddMesh(DWORD dwRBWorldBodyId, SSurface * pSurface);
	/* Gives all bodies standard lambert surface model */
	void AddAllRigidBodyWorldMeshes();

	void AddLight(SLightSource & light);

	void UpdateRaytracer(CCamera * pCam);

	void DrawRays(CCollisionGraphics * pInstance);

	void SetMeshTransform(DWORD dwMesh, Matrix4x4F & transform) {m_rgMesh[dwMesh].transform = transform; }
	void SetPixels(DWORD nPixelsX, DWORD nPixelsY) {m_nPixelsX = nPixelsX; m_nPixelsY = nPixelsY;};
	void SetNumSamples(DWORD nSamples) {m_nSamples = nSamples; }
	void SetScreenShotKey(DWORD dwScreenShotKey) {m_dwScreenShotKey = dwScreenShotKey; }

	void DestroyRayTracer();
};


inline Vector3F Reflect(Vector3F & v, Vector3F & nor) 
{
	return Vector3F(v - (2.f * DotXYZ(v, nor)) * nor);
}


};


#endif