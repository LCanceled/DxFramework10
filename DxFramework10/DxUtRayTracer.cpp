

#include "DxUtRayTracer.h"

namespace DxUt {

CRayTracer::CRayTracer():m_pRBWorld(NULL), m_szScreenShotFile(NULL)
{
}

void CRayTracer::CreateRayTracer(char * szScreenShotFile, DWORD nPixelsX,
	DWORD nPixelsY, DWORD nSamples, DWORD dwScreenShotKey, DWORD nHintMeshes, CRigidBodyWorld * m_RBWorld)
{
	m_szScreenShotFile = new char[strlen(szScreenShotFile)+1];
	strcpy(m_szScreenShotFile, szScreenShotFile);

	m_nPixelsX = nPixelsX;
	m_nPixelsY = nPixelsY;
	m_nSamples = nSamples;

	m_dwScreenShotKey = dwScreenShotKey;

	m_Camera.CreateCameraLH(.5*D3DX_PI, nPixelsX, nPixelsY, 1.f, 1000.f);

	m_rgMesh.Reserve(nHintMeshes);

	m_pRBWorld = m_RBWorld;

	m_AirVolume.n = Vector4F(1.f);
}

#define SELECT_TRIANGLE_NORMAL(_i) (rgTri[_i].Normal())

void CRayTracer::ComputeNormals(STriangleFEx * rgTri, DWORD nTri, DWORD * rgAdj)
{
	for (DWORD i=0; i<nTri; i++) {
		DWORD dwInitialTri = i;
		for (DWORD j=0; j<3; j++) {
			DWORD dwInitialEdge = j;

			DWORD nNor = 0;
			Vector3F angleWeightedNor(0);
			DWORD dwNextEdge = dwInitialEdge;
			DWORD dwNextTri = dwInitialTri;
			do {
				/* Compute the angle weighted normal */
				Vector3F & n = SELECT_TRIANGLE_NORMAL(dwNextTri);
				int iPreEdge = (int)dwNextEdge-1 < 0 ? 2 : (int)dwNextEdge-1;
				if (rgTri[dwNextTri].vPosW[dwNextEdge] == Vector3F(-1, 1, 1))
					int a=0;
				Vector3F e1(Vector3F(rgTri[dwNextTri].vPosW[iPreEdge]			- rgTri[dwNextTri].vPosW[dwNextEdge]).Normalize());
				Vector3F e2(Vector3F(rgTri[dwNextTri].vPosW[(dwNextEdge+1)%3]	- rgTri[dwNextTri].vPosW[dwNextEdge]).Normalize());
				float fAngle = abs(acosf(DotXYZ(e1, e2)));
				angleWeightedNor += fAngle*n;

				DWORD dwPreTri = dwNextTri;
				dwNextTri = rgAdj[3*dwNextTri + dwNextEdge];

				/* Check the three edges */
				dwNextEdge = 0;
				while (rgAdj[3*dwNextTri+dwNextEdge] != dwPreTri) {
					dwNextEdge++;
					if (dwNextEdge > 2) {
						dwNextTri = dwInitialTri;
						break;
					}
				}
				dwNextEdge = (dwNextEdge + 1) % 3;

				nNor++;
			} while (dwNextTri != dwInitialTri);
			
			rgTri[i].vNor[j] = angleWeightedNor.Normalize();
		}
	}
}

void CRayTracer::AddMesh(CMesh & mesh, SSurface * pSurface)
{
	m_rgMesh.PushBack();
	SMeshData & data = m_rgMesh.GetBack();
	data.m_BVTree = new BVTree;
	BVTree & bVTree = *data.m_BVTree;
	bVTree.CreateBVTree(mesh.GetMesh(), mesh.GetStride());

	DWORD nTri = bVTree.m_nTri;
	data.nTri = nTri;
	data.rgAdj = bVTree.m_rgAdj;
	data.rgTri = new STriangleFEx[nTri];
	STriangleFEx * rgTri = data.rgTri;
	for (DWORD i=0; i<nTri; i++) {
		rgTri[i] = bVTree.m_rgTri[i];
	}
	ComputeNormals(data.rgTri, data.nTri, data.rgAdj);
	data.transform.MIdenity();
	data.pSurface = pSurface;
}

void CRayTracer::AddMesh(DWORD dwRBWorldBodyId, SSurface * pSurface)
{
	Assert(m_pRBWorld, "CRayTracer::AddMesh cannot add a mesh from a CRigidBodyWorld that does not exist.");

	m_rgMesh.PushBack();
	SMeshData & data = m_rgMesh.GetBack();
	data.m_BVTree = new BVTree;
	BVTree & bVTree = *data.m_BVTree;
	CMesh * pMesh = m_pRBWorld->GetRigidBody(dwRBWorldBodyId)->GetMesh();
	bVTree.CreateBVTree(pMesh->GetMesh(), pMesh->GetStride());

	DWORD nTri = bVTree.m_nTri;
	data.nTri = nTri;
	data.rgAdj = bVTree.m_rgAdj;
	data.rgTri = new STriangleFEx[nTri];
	STriangleFEx * rgTri = data.rgTri;
	for (DWORD i=0; i<nTri; i++) {
		rgTri[i] = bVTree.m_rgTri[i];
	}
	ComputeNormals(data.rgTri, data.nTri, data.rgAdj);
	data.dwRBWorldBodyId = dwRBWorldBodyId;
	data.transform.MIdenity();
	data.pSurface = pSurface;
}

void CRayTracer::AddAllRigidBodyWorldMeshes()
{
	Assert(m_pRBWorld, "CRayTracer::AddAllRigidBodyWorldMeshes cannot add a meshs from a CRigidBodyWorld that does not exist.");

	CRigidBodyWorld::SRBObject * pRBObjects = m_pRBWorld->m_rgRBObject.GetData();
	for (DWORD i=0, end=2; i<end; i++) {
		SMaterial mat = m_pRBWorld->m_rgRBObject[i].rB.GetMesh()->GetMaterial(0);
		SSurfaceLambert * desc =  new SSurfaceLambert;
		desc->amb = Vector3F(mat.amb.r, mat.amb.g, mat.amb.b);
		desc->dif = Vector3F(mat.dif.r, mat.dif.g, mat.dif.b);
		desc->spe = Vector3F(mat.spe.r, mat.spe.g, mat.spe.b);
		desc->sPow = mat.pow;
		desc->bInterpolateNormal = 1;
		AddMesh(i, desc);
	}
	for (DWORD i=2, end=m_pRBWorld->m_rgRBObject.GetSize(); i<end; i++) {
		SMaterial mat = m_pRBWorld->m_rgRBObject[i].rB.GetMesh()->GetMaterial(0);
		SSurfaceFresnel * desc =  new SSurfaceFresnel;
		desc->amb = Vector3F(mat.amb.r, mat.amb.g, mat.amb.b);
		if (!(i%2))
			desc->n = Vector4F(.5f, .5f, 1.f, 1.3);
		if (!(i%3))
			desc->n = Vector4F(1.f, .5f, .5f, 1.3);
		if (!(i%4))
			desc->n = Vector4F(.5f, 1.f, .5f, 1.3);
		desc->bInterpolateNormal = 1;
		AddMesh(i, desc);
	}
}

void CRayTracer::AddLight(SLightSource & light)
{
	m_rgLight.PushBack(light);
}

void CRayTracer::DrawToTexture(Vector3F * rgTexValues)
{
	CHAR file[MAX_PATH];
	InsertDirectory(g_szFileDir, m_szScreenShotFile, file);

	D3D10_TEXTURE2D_DESC desc;
	desc.Width = m_nPixelsX;
	desc.Height = m_nPixelsY;
	desc.MipLevels = desc.ArraySize = 1;
	desc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
	desc.SampleDesc.Count = 1;
	desc.SampleDesc.Quality = 0;
	desc.Usage = D3D10_USAGE_DYNAMIC;
	desc.MiscFlags = 0;
	desc.BindFlags = D3D10_BIND_SHADER_RESOURCE;
	desc.CPUAccessFlags = D3D10_CPU_ACCESS_WRITE;
	ID3D10Texture2D *pTexture = NULL;
	g_pD3DDevice->CreateTexture2D(&desc, NULL, &pTexture);

	ID3D10Texture2D *pRenderTarget = NULL;
	g_pD3DDevice->CreateTexture2D(&desc, NULL, &pRenderTarget);

	D3D10_MAPPED_TEXTURE2D mappedTex;
	pTexture->Map(D3D10CalcSubresource(0, 0, 1), D3D10_MAP_WRITE_DISCARD, 0, &mappedTex);

	UCHAR* pTexels = (UCHAR*)mappedTex.pData;
	for( UINT row = 0, idx = 0; row < desc.Height; row++)
	{
		UINT rowStart = row * mappedTex.RowPitch;
		for( UINT col = 0; col < desc.Width; col++ )
		{
			UINT colStart = col * 4;
			pTexels[rowStart + colStart + 0] = (UCHAR)rgTexValues[idx].x;
			pTexels[rowStart + colStart + 1] = (UCHAR)rgTexValues[idx].y;
			pTexels[rowStart + colStart + 2] = (UCHAR)rgTexValues[idx++].z;
			pTexels[rowStart + colStart + 3] = 255;
		}
	}
	pTexture->Unmap(D3D10CalcSubresource(0, 0, 1));

	D3DX10SaveTextureToFileA(pTexture, D3DX10_IFF_PNG, file);

	ReleaseX(pTexture);
}

bool CRayTracer::IntersectRay(SRay & ray, SRayIntersectData & data)
{
	float tMin = FLT_MAX;
	bool bIntersect = 0;
	SMeshData * pMeshes = m_rgMesh.GetData();
	for (DWORD i=0, end=m_rgMesh.GetSize(); i<end; i++) {
		SRayIntersectData tmp;
		if (pMeshes[i].m_BVTree->IntersectRay(ray, tmp) && tmp.t < tMin) {
			bIntersect = 1;
			tMin = tmp.t;
			data = tmp;
			data.dwBody = i;
		}
		tmp.t = 0;
	}
	return bIntersect;
}

static const float eps = 1e-2;

Vector3F CRayTracer::TraceRays(SRay & ray, SVolume & vol, DWORD dwDepth)
{
	Vector3F color(0.f, 0.f, 0.f);
	SRayIntersectData data;
	if (IntersectRay(ray, data)) {
		SSurface * pSurface = m_rgMesh[data.dwBody].pSurface;
		data.nor = data.nor.Normalize();		
		Vector3F nor(data.nor);
		if (pSurface->bInterpolateNormal)
			data.nor = InterpolateNormal(data);

		Vector3F & emit = Vector3F(0);//pSurface->ComputeEmmitedRadiance(ray, data);
		Vector3F & direct = pSurface->ComputeRaidanceDirect(ray, data, &m_Camera, m_rgLight);
		if (!pSurface->bTraceable) {return direct; }

		/* Existing or entering volume */
		//may be wrong for volumes going into volumes where one is not air
		SVolume newVol;
		if (DotXYZ(ray.d, nor) < 0) {
			newVol.n = pSurface->n;
		} else {
			newVol = m_AirVolume;
		}
		Vector4F & n1 = vol.n;
		Vector4F & n2 = newVol.n;
		Vector4F & n12 = vol.n/newVol.n;
		
		SRay newRay;
		float d1 = DotXYZ(ray.d, nor);
		float d2 = DotXYZ(ray.d, data.nor);
		if (d2 > 0) {data.nor = -data.nor; }
		if (dwDepth >= 24) {
			newRay.d = ray.d;
			newRay.p = data.pos + eps*newRay.d;
			//Vector4F fReflectance = m_rgMesh[data.dwBody].pSurface->ComputeReflectance(ray, data, n1, n2);
			//return Vector3F(1.f-fReflectance.x, 1.f-fReflectance.y, 1.f-fReflectance.z)*TraceRays(newRay, newVol, dwDepth-1); 
			//return TraceRays(newRay, newVol, dwDepth-1); 
			return Vector3F(0);
		}
		if (d1 * d2 < 0) {
			if (DotXYZ(data.nor, nor) < 0) {
				nor = -nor;
			}
			float dotND = DotXYZ(nor, ray.d);
			float dotSD = DotXYZ(data.nor, ray.d);
			float t = -dotSD/(dotND - dotSD);
			t = min(1.f, t+.1f);
			data.nor = (t * nor + (1.f - t) * data.nor).Normalize();

			newRay.p = data.pos;// - eps*ray.d;
			newRay.d = pSurface->ComputeReflectionVec(ray, data, n12);
			return TraceRays(newRay, vol, dwDepth+1);
		}

		Vector4F fReflectance = m_rgMesh[data.dwBody].pSurface->ComputeReflectance(ray, data, n1, n2);
		Vector3F fReflectanceColor(n2.x, n2.y, n2.z);//fReflectance.x, fReflectance.y, fReflectance.z);

		/* Trace reflected */
		Vector3F reflect(0);
		if (pSurface->bReflect && fReflectance.w > .05) {
			newRay.d = pSurface->ComputeReflectionVec(ray, data, n12);
			newRay.p = data.pos + eps*newRay.d;
			reflect = TraceRays(newRay, vol, dwDepth+1); }

		/* Trace transmitted */
		Vector3F transmit(0);
		if (pSurface->bTransmit && fReflectance.w < .95) {
			newRay.d = pSurface->ComputeTransmissionVec(ray, data, n12);
			newRay.p = data.pos + eps*newRay.d;
			transmit = TraceRays(newRay, newVol, dwDepth+1); 
		}
		color = fReflectance.w*reflect + (1.f - (fReflectance.w))*transmit;
		color = fReflectanceColor*color;
	}
	return color;
}

struct SThreadData
{
	DWORD nTilesX;
	DWORD nTilesY;

	DWORD nTilePixX;
	DWORD nTilePixY;
	DWORD nImagePixX;
	DWORD nImagePixY;

	DWORD nSamples;

	CRayTracer * pInst;
	SVolume vol;
	CCamera * pCam;
	Vector3F * rgTexValues;
};

DWORD g_dwTile = 0;
CRITICAL_SECTION g_CriticalSection;

DWORD WINAPI ThreadFunc(LPVOID lpParameter)
{
	SThreadData & data = *((SThreadData*)lpParameter);
	
	srand(0);
	float fOONSamples = 1.f/(float)data.nSamples;
	float fCellSizeX = 1.f/data.nImagePixX;
	float fCellSizeY = 1.f/data.nImagePixY;
	DWORD nTiles = data.nTilesX*data.nTilesY;
	bool bCond = 1;
	while (bCond) {
		EnterCriticalSection(&g_CriticalSection);
		if (g_dwTile >= nTiles) {LeaveCriticalSection(&g_CriticalSection); break; }
		DWORD dwOffsetX = (g_dwTile % data.nTilesX)*data.nTilePixX;
		DWORD dwOffsetY = (g_dwTile / data.nTilesY)*data.nTilePixY;
		g_dwTile++;
		LeaveCriticalSection(&g_CriticalSection);

		for (DWORD i=0; i<data.nTilePixY; i++) {
			for (DWORD j=0; j<data.nTilePixX; j++) {
				Vector3F finalColor(0);
				for (DWORD k=0; k<data.nSamples; k++) {
					//EnterCriticalSection(&g_CriticalSection);
					float fPixelX = dwOffsetX + j + .5f + .01f*((int)(rand() % 100) - 50);
					float fPixelY = dwOffsetY + i + .5f + .01f*((int)(rand() % 100) - 50);
					//LeaveCriticalSection(&g_CriticalSection);

					SRay ray;
					data.pCam->GetFrustumRay(Vector2F(fPixelX, fPixelY), ray.p, ray.d);

					Vector3F color(0);
					color = data.pInst->TraceRays(ray, data.vol, 0);
					if (color.x > 1.f) color.x = 1.f;
					if (color.y > 1.f) color.y = 1.f;
					if (color.z > 1.f) color.z = 1.f;
					finalColor += 255*color;
				}
				data.rgTexValues[(dwOffsetY + i)*data.nImagePixX + (dwOffsetX + j)] = fOONSamples*finalColor;
			}
		}
	}
	return 0;
}

void CRayTracer::TraceScreenShot()
{
	/* Transform all the bodies */
	SMeshData * pMeshes = m_rgMesh.GetData();
	for (DWORD i=0, end=m_rgMesh.GetSize(); i<end; i++) {
		Matrix4x4F * triTransform;
		if (pMeshes[i].dwRBWorldBodyId != -1) {
			triTransform = &m_pRBWorld->GetRigidBody(pMeshes[i].dwRBWorldBodyId)->GetWorldMatrix();
			pMeshes[i].m_BVTree->SetTransform(*triTransform, 1.f);
		} else {
			triTransform = &pMeshes[i].transform;
			pMeshes[i].m_BVTree->SetTransform(*triTransform, 1.f); }
		pMeshes[i].m_BVTree->TransformTriangles();

		STriangleFEx * pTris = pMeshes[i].rgTri;
		for (DWORD j=0, end2=pMeshes[i].nTri; j<end2; j++) {
			pTris[j].vNorWorld[0] = pTris[j].vNor[0].MulNormal((*triTransform), pTris[j].vNor[0]);
			pTris[j].vNorWorld[1] = pTris[j].vNor[1].MulNormal((*triTransform), pTris[j].vNor[1]);
			pTris[j].vNorWorld[2] = pTris[j].vNor[2].MulNormal((*triTransform), pTris[j].vNor[2]);
		}
	}
	
	/* Setup the threads */
	static bool bInitCriticalSection = 0;
	if (!bInitCriticalSection) {
		InitializeCriticalSection(&g_CriticalSection);
		bInitCriticalSection = 1;
	}
	const DWORD nThreads = 4;
	SThreadData rgThread[nThreads];
	HANDLE rghThread[nThreads];

	g_dwTile = 0;
	Vector3F * rgTexValues = new Vector3F[m_nPixelsX*m_nPixelsY];
	//memset(rgTexValues, 0, sizeof(Vector3F)*m_nPixelsX*m_nPixelsY);
	for (DWORD i=0; i<m_nPixelsX*m_nPixelsY; i++) {
		rgTexValues[i] = Vector3F(0);
	}
	DWORD nTilesX = 16;
	DWORD nTilesY = 16;
	DWORD dwTilePixX = m_nPixelsX/nTilesX;
	DWORD dwTilePixY = m_nPixelsY/nTilesY;
	for (DWORD idx=0; idx<nThreads; idx++) {
		rgThread[idx].nTilesX = nTilesX;
		rgThread[idx].nTilesY = nTilesY;

		rgThread[idx].nTilePixX = dwTilePixX;
		rgThread[idx].nTilePixY = dwTilePixY;
		rgThread[idx].nImagePixX = m_nPixelsX;
		rgThread[idx].nImagePixY = m_nPixelsY;
		rgThread[idx].nSamples = m_nSamples;
				
		rgThread[idx].pInst = this;
		rgThread[idx].vol = m_AirVolume;
		rgThread[idx].pCam = &m_Camera;
		rgThread[idx].rgTexValues = rgTexValues;

		rghThread[idx] = CreateThread(NULL, 0, ThreadFunc, &rgThread[idx], 0, 0);
	}
	//WaitForMultipleObjects(1, &rghThread[0], TRUE, INFINITE);
	WaitForMultipleObjects(nThreads, rghThread, TRUE, INFINITE);// supposedly a problem here
	for(int idx=0; idx<nThreads; idx++) CloseHandle(rghThread[idx]);
	
	/*float fOONSamples = 1.f/m_nSamples;
	for (DWORD i=0; i<m_nPixelsY; i++) {
		for (DWORD j=0; j<m_nPixelsX; j++) {
			Vector3F finalColor(0);
			for (DWORD k=0; k<m_nSamples; k++) {
				float fPixelX = j + .5f + .01f*((int)(rand() % 100) - 50);
				float fPixelY = i + .5f + .01f*((int)(rand() % 100) - 50);
				SRay ray;
				m_Camera.GetFrustumRay(Vector2F((float)j, (float)i), ray.p, ray.d);

				//if (j == 100 && i == 300) {
				//	while (1) {
				Vector3F color(0);
				color = TraceRays(ray, m_AirVolume, 0);
				if (color.x > 1.f) color.x = 1.f;
				if (color.y > 1.f) color.y = 1.f;
				if (color.z > 1.f) color.z = 1.f;
				finalColor += 255*color;
				//}}
			}
			rgTexValues[i*m_nPixelsX + j] = fOONSamples*finalColor;
		}
	}*/
	DrawToTexture(rgTexValues);

	delete[] rgTexValues;
}

void CRayTracer::UpdateRaytracer(CCamera * pCam)
{
	m_Camera.m_Pos = pCam->m_Pos;
	m_Camera.m_ForwardVec = pCam->m_ForwardVec;
	m_Camera.m_RightVec = pCam->m_RightVec;
	m_Camera.m_UpVec = pCam->m_UpVec;
	m_Camera.m_View = pCam->m_View;
	
	if (g_KeysState[m_dwScreenShotKey]) {
		TraceScreenShot();
		OutputDebugStringA("\nTook screen shot\n\n");
	}
}

void CRayTracer::DrawRays(CCollisionGraphics * pInstance)
{
	/* Transform all the bodies */
	SMeshData * pMeshes = m_rgMesh.GetData();
	for (DWORD i=0, end=m_rgMesh.GetSize(); i<end; i++) {
		Matrix4x4F * triTransform;
		if (pMeshes[i].dwRBWorldBodyId != -1) {
			triTransform = &m_pRBWorld->GetRigidBody(pMeshes[i].dwRBWorldBodyId)->GetWorldMatrix();
			pMeshes[i].m_BVTree->SetTransform(*triTransform, 1.f);
		} else {
			triTransform = &pMeshes[i].transform;
			pMeshes[i].m_BVTree->SetTransform(*triTransform, 1.f); }
		pMeshes[i].m_BVTree->TransformTriangles();
	}
	static DWORD dwKey = 1;
	for (DWORD i=0; i<8; i++) {
		if (g_KeysState[DIK_1 + i])
			dwKey = i;
	}
	m_rgMesh[6].m_BVTree->DrawBVTree(pInstance, dwKey);
}

void CRayTracer::DestroyRayTracer()
{
	m_pRBWorld = NULL;

	delete[] m_szScreenShotFile;
	m_szScreenShotFile = NULL;

	for (DWORD i=0; i<m_rgMesh.GetSize(); i++) {
		if (m_rgMesh[i].pSurface) {
			m_rgMesh[i].m_BVTree->DestroyBVTree();
			delete m_rgMesh[i].pSurface;
			m_rgMesh[i].pSurface = NULL;
			delete[] m_rgMesh[i].rgTri;
			m_rgMesh[i].rgTri = NULL;
		}
	}
	m_rgMesh.Clear();

	m_rgLight.Clear();
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////Surfaces////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Vector3F SSurfaceLambert::ComputeRaidanceDirect(SRay & ray, SRayIntersectData & rayData, CCamera * pCam, CArray<SLightSource> & rgLight)
{
	Vector3F color(0);
	
	Vector3F & nor = rayData.nor.Normalize();
	Vector3F vVec = (rayData.pos - pCam->GetPosition()).Normalize();
	
	for (DWORD i=0, end=rgLight.GetSize(); i<end; i++) {
		Vector3F distVec(rayData.pos);//- Vector3F(-16.f, 22.f, 8.f));
		float dist = distVec.Length();
		Vector3F & lightVec = distVec.Normalize();//rgLight[i].vec;
		
		/* Diffuse */
		float perDif = max(0, -DotXYZ(lightVec, nor));
		Vector3F difColor = perDif*(dif*rgLight[i].dif);
	
		/* Specular */
		Vector3F rVec = Reflect(-lightVec, nor);
		float perSpe = pow(abs(max(0, DotXYZ(vVec, rVec))), sPow);
		Vector3F speColor = perSpe*(spe*rgLight[i].spe);

		color += 1.5*exp(-dist*dist*.00045)*(difColor + speColor);
		if (color.x > 1.f) color.x = 1.f;
		if (color.y > 1.f) color.y = 1.f;
		if (color.z > 1.f) color.z = 1.f;
	}
	return color;
}

Vector3F SSurfaceFresnel::ComputeRaidanceDirect(SRay & ray, SRayIntersectData & rayData, CCamera * pCam, CArray<SLightSource> & rgLight)	
{
	Vector3F color(0);
	
	Vector3F & nor = rayData.nor.Normalize();
	Vector3F vVec = (rayData.pos - pCam->GetPosition()).Normalize();
	
	for (DWORD i=0, end=rgLight.GetSize(); i<end; i++) {
		Vector3F & lightVec = rgLight[i].vec;
	
		/* Specular */
		Vector3F rVec = Reflect(-lightVec, nor);
		float perSpe = pow(abs(max(0, DotXYZ(vVec, rVec))), 8.f);
		Vector3F spe(.7f, .7f, .7f);
		Vector3F speColor = perSpe*(spe*rgLight[i].spe);

		color += speColor;
		if (color.x > 1.f) color.x = 1.f;
		if (color.y > 1.f) color.y = 1.f;
		if (color.z > 1.f) color.z = 1.f;
	}
	return color;
}

Vector3F SSurfaceFresnel::ComputeReflectionVec(SRay & ray, SRayIntersectData & data, Vector4F & n12)
{
	return Reflect(ray.d, data.nor);
}

Vector3F SSurfaceFresnel::ComputeTransmissionVec(SRay & ray, SRayIntersectData & data, Vector4F & n12)
{
	Vector3F & nor = data.nor;
	Vector3F & l = ray.d;
	float cosTheta1 = -DotXYZ(nor, l);
	float cosTheta2 = (1.f - n12.w*n12.w*(1.f - cosTheta1*cosTheta1));
	if (cosTheta2 < 0) return Vector3F(0);
	cosTheta2 = sqrtf(cosTheta2);
	if (cosTheta1 < 0) {
		PostQuitMessage(0);
	}
	return n12.w*l + (n12.w*cosTheta1 - cosTheta2)*nor;
}

Vector4F SSurfaceFresnel::ComputeReflectance(SRay & ray, SRayIntersectData & data, Vector4F & n1, Vector4F & n2)
{
	Vector3F & nor = data.nor;
	Vector3F & l = ray.d;
	float cosTheta1 = -DotXYZ(nor, l);
	float omCosThetaSq = (1.f - cosTheta1*cosTheta1);
	Vector4F reflect;
	for (DWORD i=0; i<4; i++) {
		float n12 = n1.c[i]/n2.c[i];
		float cosTheta2 = 1.f - n12*n12*omCosThetaSq;
		if (cosTheta2 < 0) {reflect.c[i] = 1; continue; }
		cosTheta2 = sqrt(cosTheta2);

		float rs = (n1.c[i]*cosTheta1 - n2.c[i]*cosTheta2)/(n1.c[i]*cosTheta1 + n2.c[i]*cosTheta2);
		float rp = (n1.c[i]*cosTheta2 - n2.c[i]*cosTheta1)/(n1.c[i]*cosTheta2 + n2.c[i]*cosTheta1);
		reflect.c[i] = .5f * (rs*rs + rp*rp);
	}
	return reflect;
}



};



	/*DWORD nTilesX = 4, nTilesY = 4;
	DWORD dwTileSizeX = m_nPixelsX/nTilesX;
	DWORD dwTileSizeY = m_nPixelsY/nTilesY;
	for (DWORD i=0; i<nTilesY/2; i++) {
		for (DWORD j=0; j<nTilesX/2; j++) {
			for (DWORD k=0; k<nThreads; k++) {
				rgThread[k].pixX = dwTileSizeX*j;
				rgThread[k].pixY = dwTileSizeY*i;
				rgThread[k].id = k;
				rgThread[k].nPixX = dwTileSizeX;
				rgThread[k].nPixY = dwTileSizeY;
				
				rgThread[k].nPixTotalX = m_nPixelsX;
				rgThread[k].nPixTotalY = m_nPixelsY;

				
				rgThread[k].pInst = this;
				rgThread[k].vol = m_AirVolume;
				rgThread[k].pCam = &m_Camera;
				rgThread[k].rgTexValues = rgTexValues;

				rghThread[k] = CreateThread(NULL, 0, ThreadFunc, &rgThread[k], 0, 0);
				if (rghThread[k] == NULL) DebugBreak();
				WaitForMultipleObjects(1, &rghThread[k], TRUE, INFINITE);
				CloseHandle(rghThread[k]);
			}

			//WaitForMultipleObjects(nThreads, rghThread, TRUE, INFINITE);
			//for(int k=0; k<4; k++) CloseHandle(rghThread[k]);
		}
	}*/

	/*
DWORD WINAPI ThreadFunc(LPVOID lpParameter)
{
	SThreadData & data = *((SThreadData*)lpParameter);
	DWORD nPixSubTileX = data.nPixX/4; DWORD nPixSubSubTileX = nPixSubTileX/NTHREADS; 
	DWORD nPixSubTileY = data.nPixY/4; DWORD nPixSubSubTileY = nPixSubTileY/NTHREADS;
	for (DWORD dwSubTile=0; dwSubTile<4; dwSubTile++) {	
		for (DWORD i=0; i<data.nPixY; i++) {
			for (DWORD j=0; j<data.nPixX; j++) {
				DWORD offsetX = data.pixX + nPixSubTileX*dwSubTile + nPixSubSubTileX*data.id + j;
				DWORD offsetY = data.pixY + nPixSubTileY*dwSubTile + nPixSubSubTileY*data.id + i;

				SRay ray;
				data.pCam->GetFrustumRay(Vector2F((float)offsetX, (float)offsetY), ray.p, ray.d);

				Vector3F color(0);
				color = data.pInst->TraceRays(ray, data.vol, 0);
				if (color.x > 1.f) color.x = 1.f;
				if (color.y > 1.f) color.y = 1.f;
				if (color.z > 1.f) color.z = 1.f;
				Vector3F finalColor = 255*color;
				data.rgTexValues[offsetY*data.nPixTotalX + offsetX] = finalColor;
			}
		}
	}
	return 0;
}*/
