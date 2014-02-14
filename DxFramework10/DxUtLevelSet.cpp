
#include "DxUtLevelset.h"
#include "DxUtMesh.h"
#include "DxUtAABBox.h"
#include "DxUtPlane.h"
#include "DxUtCollisionGraphics.h"
#include <fstream>
#include "DxUtTimer.h"
#include "DxUtConvexHull.h"
#include "DxUtPlane.h"
#include "DxUtD3DApp.h"

#define SELECT_TRIANGLE_NORMAL(_i) (STriangleF(rgVert[3*_i + 0], rgVert[3*_i + 1], rgVert[3*_i + 2]).Normal())

namespace DxUt {

CLevelSet::CLevelSet():m_fValidParticleDistance(1e-4), m_bUseDumbScheme(0)
{
	m_FaceVertexScaling = Vector3F(.75f);
	m_TransformWorldToObj.MIdenity();
	m_TransformObjToWorld.MIdenity();
	m_TransformNormalGridToWorld.MIdenity();

	//g_pSwapChain->GetBuffer(
}

/*
void CLevelSet::AverageNormals(Vector3F & nextNor, Vector3F & preNor, Vector3F & avgNor, DWORD & nNor)
{
	/* Do not average normals that share the same normal 
	if (DotXYZ(nextNor, preNor) < .99f) {
		avgNor += nextNor;
		nNor++;
	}
}
*/

void CLevelSet::Initialize(Vector3F & gravity, float fTimeStepSize, float fMaxVelocity)
{
	m_Gravity = gravity;
	m_fTimeStepSize = fTimeStepSize;

	/* Compute minimum edge sample length */
	float fScaling = 1.f;
	m_fMinEdgeLen = fScaling*2.f*(gravity.Length() * fTimeStepSize);

	/* Compute the maximum face penetration */
	m_fMaxFacePenetrationDepth = fMaxVelocity * fTimeStepSize;
}

int CLevelSet::ComputeAdmissibleZone(DWORD dwInitialTri, DWORD dwInitialEdge,
	Vector3F * rgVert, DWORD * rgAdj, Vector3F & angleWeightedNor, Vector3F & edgeNor, float & fConeAngle)
{
	DWORD nNor = 0;
	angleWeightedNor = Vector3F(0,0,0);
	DWORD dwNextEdge = dwInitialEdge;
	DWORD dwNextTri = dwInitialTri;
	do {
		/* Compute the angle weighted normal */
		Vector3F & n = SELECT_TRIANGLE_NORMAL(dwNextTri);
		int iPreEdge = (int)dwNextEdge-1 < 0 ? 2 : (int)dwNextEdge-1;
		Vector3F e1(Vector3F(rgVert[3*dwNextTri + iPreEdge]			- rgVert[3*dwNextTri + dwNextEdge]).Normalize());
		Vector3F e2(Vector3F(rgVert[3*dwNextTri + (dwNextEdge+1)%3] - rgVert[3*dwNextTri + dwNextEdge]).Normalize());
		float fAngle = acosf(DotXYZ(e1, e2));
		angleWeightedNor += fAngle*n;

		DWORD dwPreTri = dwNextTri;
		dwNextTri = rgAdj[3*dwNextTri + dwNextEdge];
		if (dwNextTri == -1)
			return -1;

		/* Check the three edges */
		dwNextEdge = 0;
		while (rgAdj[3*dwNextTri+dwNextEdge] != dwPreTri) {
			dwNextEdge++;
			if (3*dwNextTri+dwNextEdge == -1) return -1;//goto errorCase;
			if (dwNextEdge > 2) {
				dwNextTri = dwInitialTri;
				return -1;
			}
		}
		dwNextEdge = (dwNextEdge + 1) % 3;

		nNor++;
	} while (dwNextTri != dwInitialTri);
	angleWeightedNor = angleWeightedNor.Normalize();
	
	/* Compute the edge normal */ {
		Vector3F n1 = SELECT_TRIANGLE_NORMAL(dwInitialTri);
		DWORD dwAdjacentTri = rgAdj[3*dwInitialTri + dwInitialEdge];
		if (3*dwInitialTri + dwInitialEdge == -1) return -1;
		Vector3F n2 = SELECT_TRIANGLE_NORMAL(dwAdjacentTri);
		edgeNor = (n1 + n2).Normalize();
	}

	/* Compute the minimum angle (fConeAngle) between the angleWeightedNor and the polygon normals */
	fConeAngle = 1.;
	dwNextTri = dwInitialTri;
	dwNextEdge = dwInitialEdge;
	do {
		/* Find the max of the angle */
		Vector3F & n = SELECT_TRIANGLE_NORMAL(dwNextTri);
		float d = DotXYZ(n, angleWeightedNor);
		fConeAngle = min(fConeAngle, d);

		DWORD dwPreTri = dwNextTri;
		dwNextTri = rgAdj[3*dwNextTri + dwNextEdge];
		if (dwNextTri == -1)
			return -1;

		/* Check the three edges */
		dwNextEdge = 0;
		while (rgAdj[3*dwNextTri+dwNextEdge] != dwPreTri) {
			dwNextEdge++;
			if (3*dwNextTri+dwNextEdge == -1) return -1;
			if (dwNextEdge > 2) {
				dwNextTri = dwInitialTri;
				return -1;
			}
		}
		dwNextEdge = (dwNextEdge + 1) % 3;
		nNor++;
	} while (dwNextTri != dwInitialTri);

	return nNor;
}

void CLevelSet::ComputeNondegenerateVertices(DWORD nVert, DWORD nTri, Vector3F * rgVert, DWORD * rgAdj, CFinitePointGrid3F<DWORD> & vertToIdx)
{
	m_rgVertexParticle.Reserve(nVert);
	SVertexParticle * pVP = m_rgVertexParticle.GetData();
	for (DWORD i=0; i<nVert; i++) {
		SVertexParticle vp = {rgVert[i], Vector3F(0,0,0), 0.f };
		DWORD dwEntry = 0, dwVal = 0;
		if (!m_rgVertexParticle.FindIfEquals(vp, dwEntry)) {
			Vector3F dummy;
			if (ComputeAdmissibleZone(i / 3, i % 3, rgVert, rgAdj, vp.coneDir, dummy, vp.coneAngle) > 0) {
				dwVal = m_rgVertexParticle.GetSize();
				vertToIdx.AddPoint(vp.v, dwVal);
				
				vp.bIntersected = 0;
				vp.bAdmissible = 0;
				m_rgVertexParticle.PushBack(vp);
			}
			else if (!vertToIdx.PointInGrid(vp.v, &dwVal)) {
				dwVal = -1;
				vertToIdx.AddPoint(vp.v, dwVal);
			}
		}
	}
}

void CLevelSet::AddEdgeVertices(Vector3F & v1, Vector3F & v2, float t,
	DWORD nVertPerEdge, CFinitePointGrid3F<DWORD> & vertToIdx, Vector3F & normal, Vector3F * rgAdjTriVert, DWORD & dwVal)
{
	Vector3F & tV1 = rgAdjTriVert[1] - rgAdjTriVert[0];
	Vector3F & tV2 = rgAdjTriVert[2] - rgAdjTriVert[0];
	Vector3F tNor(CrossXYZ(tV1, tV2).Normalize());
	// Do not add interior edges of coplanar triangles
	if (DotXYZ(normal, tNor) > .99f) {
		dwVal = -1; return;
	}

	Vector3F dif(v2-v1);
	//for (DWORD i=1; i<=nVertPerEdge; i++) m_rgEdgeParticle.PushBack(SEdgeParticle(v1 + (t*(float)i)*dif));

	SEdge edge;
	dwVal = -1;
	edge.nEdgeParticles = nVertPerEdge;
	//edge.rgEdgeParticle = &m_rgEdgeParticle.GetBack() - (nVertPerEdge-1);
	// The admissible zones of some particles could not computed correctly
	if (!vertToIdx.PointInGrid(v1, &dwVal)) {
		DebugBreak();
		dwVal = -1; return; 
	} 
	if (dwVal == -1) return;
	edge.pVP1 = &m_rgVertexParticle[dwVal];
	if (!vertToIdx.PointInGrid(v2, &dwVal)) {
		DebugBreak();
		dwVal = -1; return; 
	}
	if (dwVal == -1) return;
	edge.pVP2 = &m_rgVertexParticle[dwVal];
	edge.coneDir = (normal + tNor).Normalize();
	edge.coneAngle = DotXYZ(normal, edge.coneDir);
	edge.edgeLength = (edge.pVP1->v - edge.pVP2->v).Length();
	if (edge.edgeLength > m_fMinEdgeLen) {
		dwVal = m_rgEdge.GetSize();
		m_rgEdge.PushBack(edge);
	} else dwVal = -1;
}

float CLevelSet::ComputeClosestDistance(Vector3F & pt, Vector3F * rgVert, DWORD nVert, STriangleFEx * rgTri)
{
	static const float eps = 1e-4;
	float fBestDist = FLT_MAX;
	DWORD dwIdx = 0;
	DWORD dwType = 0;
	Vector3F closestPt;
	for (DWORD i=0, end=nVert/3; i<end; i++) {
		DWORD type = 0;
		Vector3F cPt = ComputeClosestPoint(rgTri[i], pt, type);
		if (rgTri[i].bBadTriangle) type = 0x3;
		Vector3F dist(pt - cPt);
		float fDist = dist.Length();

		if (fDist < fBestDist) {
			fBestDist = fDist;
			dwIdx = i;
			dwType = type;
			closestPt = cPt;
		}
	}

	STriangleFEx & tri = rgTri[dwIdx];
	Vector3F closestFeatureNor;
	Vector3F triNormal = tri.Normal();
	float area = tri.Area();
	/* Face closest feature */
	if ((dwType & 0x3) == 0)		closestFeatureNor = tri.nor;
	/* Edge closest feature */
	else if ((dwType & 0x3) == 1)	closestFeatureNor = tri.edgeNor[dwType >> 2];
	/* Vertex closest feature */
	else							closestFeatureNor = tri.vertNor[dwType >> 2];

	Vector3F dir(pt - closestPt);
	return DotXYZ(dir, closestFeatureNor) > 0 ? fBestDist : -fBestDist;
}

void CLevelSet::ComputeMeshParticles(DWORD nVert, DWORD nTri, Vector3F * rgVert, DWORD * rgAdj, char * szLevelSetFile)
{
	if (!_LoadLevelSet(szLevelSetFile)) {
		Assert(0, "CLevelSet::ComputerMeshParticles levelSet could not be loaded.");
		ComputeSDF(rgVert, nVert, rgAdj);
		WriteLevelSet(szLevelSetFile);
	}

	CAABBox aabb;
	aabb.ComputeAABBox(rgVert, nVert);
	/*aabb.MaxPW() = aabb.MinPW() + fCellSize*Vector3F(m_nCellsX-4, m_nCellsY-4, m_nCellsZ-4);
	float flt=(m_GridMin - aabb.MinPW()).LengthSq();
	float flt2=(m_GridMax - aabb.MaxPW()).LengthSq();
	if ((m_GridMin - aabb.MinPW()).LengthSq() > 1e-1 || (m_GridMax - aabb.MaxPW()).LengthSq() > 1e-1) {
		m_fCellSize = fCellSize;
		ComputeSDF(rgVert, nVert, rgAdj);
		WriteLevelSet(szLevelSetFile);
	}	
	flt=(m_GridMin - aabb.MinPW()).LengthSq();
	flt2=(m_GridMax - aabb.MaxPW()).LengthSq();*/

	/* Compute the vertex particles */
	const float eps = 1e-3;
	CFinitePointGrid3F<DWORD> vertToIdx;
	Vector3F gridMinP = m_GridMin;
	int gridFactor = 1; //Unstable code here
	vertToIdx.CreateGrid(m_nCellsX/gridFactor, m_nCellsY/gridFactor, m_nCellsZ/gridFactor, gridFactor*m_fCellSize, gridMinP);
	ComputeNondegenerateVertices(nVert, nTri, rgVert, rgAdj, vertToIdx);
	
	/* Compute the edge particles */
	CFinitePointGrid3F<DWORD> edgeToIdx;
	edgeToIdx.CreateGrid(m_nCellsX/gridFactor, m_nCellsY/gridFactor, m_nCellsZ/gridFactor, (gridFactor)*(m_fCellSize+eps), gridMinP);

	/* V + F - E = 2 */
	DWORD nVertPerEdge = 1;
	float t = 1.f/(nVertPerEdge+1);
	DWORD nEdges = (m_rgVertexParticle.GetSize() + nTri - 2);
	//m_rgEdgeParticle.Reserve(nVertPerEdge*nEdges);
	m_rgEdge.Reserve(nEdges);  
	for (DWORD i=0, dwSize=nTri*3; i<dwSize; i+=3) {
		SVisitedEdge e1(rgVert[i+0], rgVert[i+1]);
		SVisitedEdge e2(rgVert[i+1], rgVert[i+2]);
		SVisitedEdge e3(rgVert[i+2], rgVert[i+0]);
		Vector3F v1(rgVert[i+1] - rgVert[i+0]);
		Vector3F v2(rgVert[i+2] - rgVert[i+0]);
		Vector3F normal(CrossXYZ(v1, v2).Normalize());

		DWORD dwTmp = 0, dwVal = 0;
		if (!edgeToIdx.PointInGrid(e1.vAvg, &dwTmp)) {
			if (rgAdj[i+0] != -1) {
				AddEdgeVertices(rgVert[i+0], rgVert[i+1], t, nVertPerEdge, vertToIdx, normal, &rgVert[3*rgAdj[i+0]], dwVal);
			} else dwVal = -1;
			edgeToIdx.AddPoint(e1.vAvg, dwVal);
		}

		if (!edgeToIdx.PointInGrid(e2.vAvg, &dwTmp)) {
			if (rgAdj[i+1] != -1) {
				AddEdgeVertices(rgVert[i+1], rgVert[i+2], t, nVertPerEdge, vertToIdx, normal, &rgVert[3*rgAdj[i+1]], dwVal);
			} else dwVal = -1;
			edgeToIdx.AddPoint(e2.vAvg, dwVal);
		}
		
		if (!edgeToIdx.PointInGrid(e3.vAvg, &dwTmp)) {
			if (rgAdj[i+2] != -1) {
				AddEdgeVertices(rgVert[i+2], rgVert[i+0], t, nVertPerEdge, vertToIdx, normal, &rgVert[3*rgAdj[i+2]], dwVal);
			} else dwVal = -1;
			edgeToIdx.AddPoint(e3.vAvg, dwVal); 
		}
	}

	/* Compute the face vertices  */
	if (nTri < 33) {
		SVisitedTriangle * rgTri = new SVisitedTriangle[nTri];
		for (DWORD i=0; i<nTri; i++) {
			rgTri[i].tri.vPosW[0] = rgVert[3*i+0];
			rgTri[i].tri.vPosW[1] = rgVert[3*i+1];
			rgTri[i].tri.vPosW[2] = rgVert[3*i+2];
		}
		//edgeToIdx.Reset();
		m_rgFace.Reserve(nTri);
		CArray<Vector3F> rgFaceVert;
		CArray<SLine3F> rgEdge;
		for (DWORD i=0; i<nTri; i++) {
			if (!rgTri[i].bVisited) {
				m_rgFace.PushBack();
				Vector3F & nor = rgTri[i].tri.Normal();
				m_rgFace.GetBack().nor = nor;
				rgEdge.Resize(0);

				//m_rgFace.GetBack().rgBoundaryVert.PushBack(SFace::SFaceVertex(Vector2F(0), rgTri[i].tri.vPosW[0], NULL));
				//m_rgFace.GetBack().rgBoundaryVert.PushBack(SFace::SFaceVertex(Vector2F(0), rgTri[i].tri.vPosW[1], NULL));
				//m_rgFace.GetBack().rgBoundaryVert.PushBack(SFace::SFaceVertex(Vector2F(0), rgTri[i].tri.vPosW[2], NULL));
				ComputeFaceVertices(i, rgTri, rgAdj, vertToIdx, edgeToIdx, rgEdge);
				ComputeInsetVertices(nor, vertToIdx, rgEdge);

				//m_rgFace.GetBack().v /= m_rgFace.GetBack().rgVP.GetSize();
				/*rgFaceVert.Resize(m_rgFace.GetBack().rgInteriorVert.GetSize());
				for (DWORD j=0, end=rgFaceVert.GetSize(); j<end; j++)
					rgFaceVert[j] = m_rgFace.GetBack().rgInteriorVert[j].v;
				m_rgFace.GetBack().oBB.ComputeOBB(rgFaceVert.GetData(), rgFaceVert.GetSize(), COBBox::CVVertices);*/
			}
		}
		delete[] rgTri;
		rgFaceVert.Clear();

		m_rgIsFaceProcessed.Resize(m_rgFace.GetSize());
		for (DWORD i=0, end=m_rgIsFaceProcessed.GetSize(); i<end; i++) {
			m_rgIsFaceProcessed[i] = 0;
		}
		m_rgToProcessFace.Reserve(m_rgIsFaceProcessed.GetSize());
	}

	m_OBBox.ComputeOBB(rgVert, nVert, COBBox::OBBComputeMethod::CVTriangles);
	m_AABBox.ComputeAABBox(rgVert, nVert);

	
	Vector3F rotVecs[] = {		
		m_OBBox.GetRotVecW(0) = Vector3F(1,0,0),
		m_OBBox.GetRotVecW(1) = Vector3F(0,1,0),
		m_OBBox.GetRotVecW(2) = Vector3F(0,0,1) };
	m_OBBox.SetOBB(m_AABBox.CenterPointW(), m_AABBox.HalfWidthsW(), rotVecs);//*/

	m_fBSphereRadius = 2.f*m_OBBox.GetHalfWidthsW().Length();

	DoAdditionalProcessing(szLevelSetFile, rgVert, nVert, rgAdj, vertToIdx, edgeToIdx);

	vertToIdx.DestroyGrid();
	edgeToIdx.DestroyGrid();

	/* Collision indices */
	m_rgDefaultVertexParticleIndex.Resize(m_rgVertexParticle.GetSize());
	for (DWORD i=0, end=m_rgVertexParticle.GetSize(); i<end; i++) m_rgDefaultVertexParticleIndex[i] = i;
	m_pVertexParticleIndex = &m_rgDefaultVertexParticleIndex;
	
	m_rgDefaultEdgeIndex.Resize(m_rgEdge.GetSize());
	for (DWORD i=0, end=m_rgEdge.GetSize(); i<end; i++) m_rgDefaultEdgeIndex[i] = i;
	m_pEdgeIndex = &m_rgDefaultEdgeIndex;
}

void CLevelSet::ComputeInsetVertices(Vector3F & nor, CFinitePointGrid3F<DWORD> & vertToIdx, CArray<SLine3F> & rgEdgeDisordered)
{
	SPolygon3F<SLine3F> poly;
	CArray<SLine3F> rgEdgeOrdered;
	poly.ComputePolygon(rgEdgeDisordered);
	CArray<SLine3F> & rgEdge = poly.rgEdge;

	Vector3F o(0,0,0), B1(0), B2(0);
	//Compute2DPlaneBasis(nor, B1, B2);
	CArray<SFace::SFaceVertex> & rgInteriorVert = m_rgFace.GetBack().rgInteriorVert;
	for (DWORD i=0, end=rgEdge.GetSize(); i<end; i++) {
		SLine3F & edge1 = rgEdge[i];
		SLine3F & edge2 = rgEdge[(i+1)%end];
		Vector3F & v1 = (edge1.e1 - edge1.e2).Normalize();
		Vector3F & v2 = (edge2.e2 - edge2.e1).Normalize();
		Vector3F & v = (v1 + v2).Normalize();

		float dist = .125;//m_fMaxFacePenetrationDepth/CrossXYZ(v, v1).Length();
		Vector3F & pt = edge1.e2 + abs(dist)*v;

		DWORD dwIdx = 0;
		if (!vertToIdx.PointInGrid(edge1.e2, &dwIdx))
			DebugBreak();
		m_rgVertexParticle[dwIdx].rgInsetV.PushBack(SFaceVertex(pt, nor, (edge1.e2 - pt).Length()));
		//rgInteriorVert.PushBack(SFace::SFaceVertex(ProjectPointTo2DPlaneBasis(pt, o, B1, B2), pt, NULL));
		rgInteriorVert.PushBack(SFace::SFaceVertex(Vector2F(0), pt, NULL));
	}
}

void CLevelSet::ComputeFaceVertices(DWORD dwTri, SVisitedTriangle * rgTri, DWORD * rgAdj,
	CFinitePointGrid3F<DWORD> & vertToIdx, CFinitePointGrid3F<DWORD> & bVisitedEdges, CArray<SLine3F> & rgEdge)
{
	SVisitedTriangle * pVTri = &rgTri[dwTri];
	pVTri->bVisited = 1;	
	Vector3F & nor = m_rgFace.GetBack().nor;
	//PlaneF plane(pVTri->tri.vPosW[0], pVTri->tri.vPosW[1], pVTri->tri.vPosW[2]);
	Vector3F o(0,0,0), B1(0), B2(0);
	//Compute2DPlaneBasis(nor, B1, B2);

	/* Add the exterior edges */
	for (DWORD j=0; j<3; j++) {
		SVisitedTriangle * nextVTri = &rgTri[rgAdj[3*dwTri + j]];
		Vector3F nextNor(nextVTri->tri.Normal());
		//if (DotXYZ(nor, nextNor) < .999f) {
			rgEdge.PushBack(SLine3F(pVTri->tri.vPosW[j], pVTri->tri.vPosW[(j+1)%3]));
		//}
	}

	/* Do stuff */
	for (DWORD i=0; i<3; i++) {
		DWORD dwIdx, dwDummy;
		SVisitedTriangle * nextVTri = &rgTri[rgAdj[3*dwTri + i]];
		Vector3F nextNor(nextVTri->tri.Normal());
		//if (DotXYZ(nor, nextNor) < .999f) {
			Vector3F & pt = pVTri->tri.vPosW[i];
			vertToIdx.PointInGrid(pt, &dwIdx);

			SFace & f = m_rgFace.GetBack();
			//f.rgBoundaryVert.PushBack(SFace::SFaceVertex(ProjectPointTo2DPlaneBasis(
			//	m_rgVertexParticle[dwIdx].v, o, B1, B2), m_rgVertexParticle[dwIdx].v, &m_rgVertexParticle[dwIdx]));
			f.rgBoundaryVert.PushBack(SFace::SFaceVertex(Vector2F(0), pt, &m_rgVertexParticle[dwIdx]));
			m_rgVertexParticle[dwIdx].rgFace.PushBack(&f);
			f.v += pt;
			f.dwFace = m_rgFace.GetSize()-1;
		//} else if (!rgTri[rgAdj[3*dwTri + i]].bVisited) 
		//	ComputeFaceVertices(rgAdj[3*dwTri + i], rgTri, rgAdj, vertToIdx, bVisitedEdges, rgEdge);
	}
}

void CLevelSet::ComputeSDF(ID3DX10Mesh * pMesh, DWORD dwStride, char * szLevelSetFile, float fCellSize)
{
	DWORD nVert = 3*pMesh->GetFaceCount();
	Vector3F * rgVert = new Vector3F[nVert];
	ExtractVertexTriangleListFromMesh(pMesh, rgVert, dwStride);

	DWORD * rgAdj = new DWORD[3*nVert];
	ExtractAdjanceyFromMesh(pMesh, rgAdj);

	ComputeSDF(rgVert, nVert, rgAdj, szLevelSetFile, fCellSize);
}

void CLevelSet::ComputeSDF(STriangleF * rgTri, DWORD nTri, DWORD * rgAdj, char * szLevelSetFile, float fCellSize)
{
	Assert(0, "CLevelSet::ComputeSDF function not implemented.");
}

void CLevelSet::ComputeSDF(Vector3F * rgVert, DWORD nVert, DWORD * rgAdj, char * szLevelSetFile, float fCellSize)
{
	m_fCellSize = fCellSize;
	ComputeSDF(rgVert, nVert, rgAdj);
	WriteLevelSet(szLevelSetFile);
}

void CLevelSet::RecomputeSDFFast()
{
	// compute intersected tets
	// compute other stuff 

	for (int i=0; i<m_nLSVertsZ; i++) {
		for (int j=0; j<m_nLSVertsY; j++) {
			for (int k=0; k<m_nLSVertsX; k++) {

			}
		}
	}
}

void CLevelSet::CreateParticleRepresentation(ID3DX10Mesh * pMesh, DWORD dwStride, char * szLevelSetFile)
{
	//Assert(!m_pTree, "BVTree::CreateBVTree tree must be destroyed before creating a new one.");

	UINT nVert = 3*pMesh->GetFaceCount();
	UINT nTri = pMesh->GetFaceCount();

	Vector3F * rgVert = new Vector3F[nVert];
	DWORD * rgAdj = new DWORD[3*nTri];

	ExtractVertexTriangleListFromMesh(pMesh, rgVert, dwStride);
	ExtractAdjanceyFromMesh(pMesh, rgAdj);

	ComputeMeshParticles(nVert, nTri, rgVert, rgAdj, szLevelSetFile);

	delete[] rgVert;
	rgVert = NULL;
	delete[] rgAdj;
	rgAdj = NULL;
}

void CLevelSet::CreateParticleRepresentation(STriangleF * rgTri, DWORD nTri, DWORD * rgAdj, char * szLevelSetFile)
{
}

void CLevelSet::CreateParticleRepresentation(Vector3F * rgVert, DWORD nVert, DWORD * rgAdj, char * szLevelSetFile)
{
	ComputeMeshParticles(nVert, nVert/3, rgVert, rgAdj, szLevelSetFile);
}

void CLevelSet::ComputeSDF(Vector3F * rgVert, DWORD nVert, DWORD * rgAdj)
{
	/* Compute the triangles */
	STriangleFEx * rgTri = new STriangleFEx[nVert/3];
	int counter =0;
	for (DWORD i=0, end=nVert/3; i<end; i++) {
		float fDummy = 0;
		rgTri[i].vPosW[0] = rgVert[3*i + 0];
		rgTri[i].vPosW[1] = rgVert[3*i + 1];
		rgTri[i].vPosW[2] = rgVert[3*i + 2];
		rgTri[i].nor = SELECT_TRIANGLE_NORMAL(i);
		rgTri[i].bBadTriangle |= ComputeAdmissibleZone(i, 0, rgVert, rgAdj, rgTri[i].vertNor[0], rgTri[i].edgeNor[0], fDummy) == -1;
		rgTri[i].bBadTriangle |= ComputeAdmissibleZone(i, 1, rgVert, rgAdj, rgTri[i].vertNor[1], rgTri[i].edgeNor[1], fDummy) == -1;
		rgTri[i].bBadTriangle |= ComputeAdmissibleZone(i, 2, rgVert, rgAdj, rgTri[i].vertNor[2], rgTri[i].edgeNor[2], fDummy) == -1;
		float triArea = rgTri[i].Area();
		rgTri[i].bBadTriangle |= triArea < 1e-5;
		if (triArea < 1e-5)
			int a=0;
		if (rgTri[i].bBadTriangle)
			int a=0;
	}

	/* Compute the signed distance field */
	const float eps = 1e-3;
	CAABBox aabb;
	m_GridBox.ComputeAABBox(rgVert, nVert);
	m_GridMin = m_GridBox.MinPW() - Vector3F(eps);
	m_GridMax = m_GridBox.MaxPW() + Vector3F(eps);
	m_GridBox.SetMinP(m_GridMin);
	m_GridBox.SetMaxP(m_GridMax);
	m_TransformObjToGrid = m_GridMin;
	m_nCellsX = (int)ceilf_ASM((m_GridMax.x - m_GridMin.x)/m_fCellSize); m_nLSVertsX = m_nCellsX+1;
	m_nCellsY = (int)ceilf_ASM((m_GridMax.y - m_GridMin.y)/m_fCellSize); m_nLSVertsY = m_nCellsY+1;
	m_nCellsZ = (int)ceilf_ASM((m_GridMax.z - m_GridMin.z)/m_fCellSize); m_nLSVertsZ = m_nCellsZ+1;

	m_rgLSVertex = new SLevelSetVertex[m_nLSVertsX*m_nLSVertsY*m_nLSVertsZ];
	for (int k=0; k<m_nLSVertsZ; k++) {
		for (int j=0; j<m_nLSVertsY; j++) {
			for (int i=0; i<m_nLSVertsX; i++) {
				m_rgLSVertex[k*m_nLSVertsY*m_nLSVertsX + j*m_nLSVertsX + i].dist = ComputeClosestDistance(
					Vector3F((float)i * m_fCellSize, (float)j * m_fCellSize, (float)k * m_fCellSize) + m_TransformObjToGrid, rgVert, nVert, rgTri);
			}
		}
	}

	delete[] rgTri;
}

bool CLevelSet::_LoadLevelSet(char * szFile)
{
	char file[MAX_PATH];
	ChangeExtension(szFile, "lvset", file);
	InsertDirectoryEx(g_szFileDir, file, file);  

	std::ifstream stream(file, std::ios::binary);
	if (!stream) return 0;
	//Assert(stream, "CLevelSet::_LoadLevelSet invalid lvset file specified."); 

	stream.read((char*)&m_nLSVertsX, sizeof(DWORD));
	stream.read((char*)&m_nLSVertsY, sizeof(DWORD));
	stream.read((char*)&m_nLSVertsZ, sizeof(DWORD));
	m_nCellsX = m_nLSVertsX-1;
	m_nCellsY = m_nLSVertsY-1;
	m_nCellsZ = m_nLSVertsZ-1;

	stream.read((char*)&m_GridMin.x, sizeof(float));
	stream.read((char*)&m_GridMin.y, sizeof(float));
	stream.read((char*)&m_GridMin.z, sizeof(float));

	stream.read((char*)&m_GridMax.x, sizeof(float));
	stream.read((char*)&m_GridMax.y, sizeof(float));
	stream.read((char*)&m_GridMax.z, sizeof(float));

	stream.read((char*)&m_fCellSize, sizeof(float));

	m_GridBox.SetMinP(m_GridMin);
	m_GridBox.SetMaxP(m_GridMax);
	m_TransformObjToGrid = m_GridMin;

	m_rgLSVertex = new SLevelSetVertex[m_nLSVertsX*m_nLSVertsY*m_nLSVertsZ];
	for (int k=0; k<m_nLSVertsZ; k++) {
		for (int j=0; j<m_nLSVertsY; j++) {
			for (int i=0; i<m_nLSVertsX; i++) {
				stream.read((char*)&m_rgLSVertex[k*m_nLSVertsY*m_nLSVertsX + j*m_nLSVertsX + i].dist, sizeof(float)*1);
			}
		}
	}
	stream.close();

	return 1;
}

bool CLevelSet::LoadLevelSet(char * szFile)
{
	return _LoadLevelSet(szFile);
}

void CLevelSet::WriteLevelSet(char * szFile)
{
	char file[MAX_PATH];
	ChangeExtension(szFile, "lvset", file);
	InsertDirectoryEx(g_szFileDir, file, file);  

	std::ofstream stream(file, std::ios::out | std::ios::binary); stream.close();
	stream.open(file, std::ios::out | std::ios::binary);

	stream.write((char*)&m_nLSVertsX, sizeof(DWORD));
	stream.write((char*)&m_nLSVertsY, sizeof(DWORD));
	stream.write((char*)&m_nLSVertsZ, sizeof(DWORD));
	m_nCellsX = m_nLSVertsX-1;
	m_nCellsY = m_nLSVertsY-1;
	m_nCellsZ = m_nLSVertsZ-1;

	stream.write((char*)&m_GridMin.x, sizeof(float));
	stream.write((char*)&m_GridMin.y, sizeof(float));
	stream.write((char*)&m_GridMin.z, sizeof(float));

	stream.write((char*)&m_GridMax.x, sizeof(float));
	stream.write((char*)&m_GridMax.y, sizeof(float));
	stream.write((char*)&m_GridMax.z, sizeof(float));

	stream.write((char*)&m_fCellSize, sizeof(float));

	for (int k=0; k<m_nLSVertsZ; k++) {
		for (int j=0; j<m_nLSVertsY; j++) {
			for (int i=0; i<m_nLSVertsX; i++) {
				float dist = m_rgLSVertex[k*m_nLSVertsY*m_nLSVertsX + j*m_nLSVertsX + i].dist;
				stream.write((char*)&dist, sizeof(float)*1);
			}
		}
	}
	stream.close();
}

bool CLevelSet::ParticleInLevelSet(Vector3F & particle, float & fDist)
{
	Vector3F gridPt(m_TransformWorldToObj*particle), cpt;
	fDist = sqrt(m_GridBox.ClosestPointSq(gridPt, cpt));
	/* Check if the particle is in the grid */
	if (gridPt.x < m_GridMin.x || gridPt.y < m_GridMin.y || gridPt.z < m_GridMin.z || 
		gridPt.x > m_GridMax.x || gridPt.y > m_GridMax.y || gridPt.z > m_GridMax.z) return 0; 
	gridPt -= m_TransformObjToGrid;

	/* Check if the particle is in the polyhedron */
	int cellX = (int)floorf_ASM(gridPt.x/m_fCellSize);
	int cellY = (int)floorf_ASM(gridPt.y/m_fCellSize);
	int cellZ = (int)floorf_ASM(gridPt.z/m_fCellSize);
	if (cellX < 0 || cellY < 0 || cellZ < 0) 
		DebugBreak();
	if (cellX >= m_nCellsX || cellY >= m_nCellsY || cellZ >= m_nCellsZ) 
		DebugBreak();

	float t = ComputeSignedDistance(
		Vector3F(fracf(gridPt.x/m_fCellSize), fracf(gridPt.y/m_fCellSize), fracf(gridPt.z/m_fCellSize)),
		cellX, cellY, cellZ);

	fDist = t;
	return t < -m_fValidParticleDistance;
}

bool CLevelSet::ParticleInLevelSet_Fast(Vector3F & particle, float & fDist)
{
	Vector3F gridPt(particle), cpt;
	fDist = sqrt(m_GridBox.ClosestPointSq(gridPt, cpt));

	/* Check if the particle is in the grid */
	if (gridPt.x < m_GridMin.x || gridPt.y < m_GridMin.y || gridPt.z < m_GridMin.z || 
		gridPt.x > m_GridMax.x || gridPt.y > m_GridMax.y || gridPt.z > m_GridMax.z) return 0; 
	gridPt -= m_TransformObjToGrid;

	/* Check if the particle is in the polyhedron */
	int cellX = (int)floorf_ASM(gridPt.x/m_fCellSize);
	int cellY = (int)floorf_ASM(gridPt.y/m_fCellSize);
	int cellZ = (int)floorf_ASM(gridPt.z/m_fCellSize);
	if (cellX < 0 || cellY < 0 || cellZ < 0) 
		DebugBreak();
	if (cellX >= m_nCellsX || cellY >= m_nCellsY || cellZ >= m_nCellsZ) 
		DebugBreak();

	float t = ComputeSignedDistance(
		Vector3F(fracf(gridPt.x/m_fCellSize), fracf(gridPt.y/m_fCellSize), fracf(gridPt.z/m_fCellSize)),
		cellX, cellY, cellZ);

	fDist = t;
	return t < -m_fValidParticleDistance;
}

float CLevelSet::ComputeSignedDistance(Vector3F & dxyz, int cellX, int cellY, int cellZ)
{
	int CZ0 = cellZ*m_nLSVertsY*m_nLSVertsX;
	int CZ1 = (cellZ+1)*m_nLSVertsY*m_nLSVertsX;
	int CY0 = cellY*m_nLSVertsX;
	int CY1 = (cellY+1)*m_nLSVertsX;

	return (float)InterpolateTrilinear(dxyz.x, dxyz.y, dxyz.z,
			m_rgLSVertex[CZ0 + CY0 + cellX+0].dist,
			m_rgLSVertex[CZ0 + CY0 + cellX+1].dist,
			m_rgLSVertex[CZ0 + CY1 + cellX+0].dist,
			m_rgLSVertex[CZ0 + CY1 + cellX+1].dist,
		
			m_rgLSVertex[CZ1 + CY0 + cellX+0].dist,
			m_rgLSVertex[CZ1 + CY0 + cellX+1].dist,
			m_rgLSVertex[CZ1 + CY1 + cellX+0].dist,
			m_rgLSVertex[CZ1 + CY1 + cellX+1].dist);
}

#define PRINT

DWORD CLevelSet::LevelSetCollision(CLevelSet & collideLevelSet, CArray<SContactPoint> * rgCP)
{
	float radius = m_fBSphereRadius + collideLevelSet.m_fBSphereRadius;
	if ((m_OBBox.GetCenterW() - collideLevelSet.m_OBBox.GetCenterW()).LengthSq() > radius*radius) return 0;

	if (!m_OBBox.OBBoxIntersectW(collideLevelSet.m_OBBox)) {
		return 0;
	}
	//if (!m_AABBox.AABBoxIntersectW(collideLevelSet.m_AABBox)) return 0;

#ifdef PRINT
	CTimer g_Timer(1000);
	g_Timer.StartTimer(0);
#endif
	DWORD oldSize = rgCP->GetSize();

	/* Vertex containment */
#ifdef PRINT
	g_Timer.StartTimer(1);
#endif
	static int counter = 0;
	if (++counter == 347)
		int a=0;

	/* Body 1 */
	SContactPoint cp;
	Matrix4x4F T1 = collideLevelSet.m_TransformWorldToObj*m_TransformObjToWorld;
	for (DWORD i=0, end=m_pVertexParticleIndex->GetSize(); i<end; i++) {
		break;
		SVertexParticle & vP = m_rgVertexParticle[(*m_pVertexParticleIndex)[i]];
		SVertexParticle wVP = {
			T1*vP.v,
			m_TransformNormalGridToWorld*vP.coneDir,
			vP.coneAngle
		};
		//rgVP[i].tV = wVP.v;
		//if (collideLevelSet.ParticleInLevelSet(wVP.v)) {
		vP.tV = wVP.v;
		if (collideLevelSet.ParticleInLevelSet_Fast(wVP.v, vP.closestDist)) {
			vP.bIntersected = 1;
			cp.dist = vP.closestDist;
			cp.iNor=collideLevelSet.ComputeGradient_Fast(wVP.v);
			if (cp.iNor.LengthSq() < .5) 
				continue;
			//if (IsNormalAdmissible(wVP.coneDir, wVP.coneAngle, (cp.iNor))) {
			vP.bAdmissible = 1;

			cp.iPos = collideLevelSet.m_TransformObjToWorld*wVP.v;
			rgCP->PushBack(cp);
			/*//} else {
				for (DWORD j=0, end=vP.rgFace.GetSize(); j<end; j++) {
					if (!m_rgIsFaceProcessed[vP.rgFace[j]->dwFace]) {
						m_rgIsFaceProcessed[vP.rgFace[j]->dwFace] = 1;
						m_rgToProcessFace.PushBack(vP.rgFace[j]);
					}
				}*/
			}//}
		/*else if (vP.closestDist < m_fCellSize) {
			for (DWORD j=0, end=vP.rgFace.GetSize(); j<end; j++) {
				if (!m_rgIsFaceProcessed[vP.rgFace[j]->dwFace]) {
					m_rgIsFaceProcessed[vP.rgFace[j]->dwFace] = 1;
					m_rgToProcessFace.PushBack(vP.rgFace[j]);
				}
			}
		}*/
	}
	
	/* Body 2 */
	Matrix4x4F T2 = m_TransformWorldToObj*collideLevelSet.m_TransformObjToWorld;
	for (DWORD i=0, end=collideLevelSet.m_pVertexParticleIndex->GetSize(); i<end; i++) {
		SVertexParticle & vP = collideLevelSet.m_rgVertexParticle[(*collideLevelSet.m_pVertexParticleIndex)[i]];
		SVertexParticle wVP = {
			T2*vP.v,
			collideLevelSet.m_TransformNormalGridToWorld*vP.coneDir,
			vP.coneAngle
		};
		//rgVP2[i].tV = collideLevelSet.m_TransformObjToWorld*;
		//if (ParticleInLevelSet(wVP.v)) {
		T1*vP.tV = wVP.v;
		if (ParticleInLevelSet_Fast(wVP.v, vP.closestDist)) {
			cp.dist = vP.closestDist;
			vP.bIntersected = 1;
			cp.iNor=ComputeGradient_Fast(wVP.v);
			if (cp.iNor.LengthSq() < .5)
				continue;
			//if (IsNormalAdmissible(wVP.coneDir, wVP.coneAngle, (cp.iNor))) {
			vP.bAdmissible = 1;

			cp.iPos = m_TransformObjToWorld*wVP.v;
			cp.iNor = -cp.iNor;
			rgCP->PushBack(cp);
			/*} else {
				for (DWORD j=0, end=vP.rgFace.GetSize(); j<end; j++) {
					if (!collideLevelSet.m_rgIsFaceProcessed[vP.rgFace[j]->dwFace]) {
						collideLevelSet.m_rgIsFaceProcessed[vP.rgFace[j]->dwFace] = 1;
						collideLevelSet.m_rgToProcessFace.PushBack(vP.rgFace[j]);
					}
				}*/
			}//}
		/*else if (vP.closestDist < m_fCellSize) {
			for (DWORD j=0, end=vP.rgFace.GetSize(); j<end; j++) {
				if (!collideLevelSet.m_rgIsFaceProcessed[vP.rgFace[j]->dwFace]) {
					collideLevelSet.m_rgIsFaceProcessed[vP.rgFace[j]->dwFace] = 1;
					collideLevelSet.m_rgToProcessFace.PushBack(vP.rgFace[j]);
				}
			}
		}*/
	}
#ifdef PRINT
	g_Timer.EndTimer(1);
	g_Timer.DisplayTimer(1, "End intersection vertices");
#endif

#ifdef PRINT
	g_Timer.StartTimer(2);
#endif
	/* Edge-vertex containment */
	/* Compute the minimum penetration distance */
	/*for (DWORD i=0, end=m_pEdgeIndex->GetSize(); i<end; i++) {
		if ((*m_pEdgeIndex)[i] > m_rgEdge.GetSize())
			DebugBreak();

		SEdge * pEdge = &m_rgEdge[(*m_pEdgeIndex)[i]];
		if (pEdge->pVP1->bIntersected && pEdge->pVP2->bIntersected) continue;

		SEdge e = *pEdge;
		if (!pEdge->pVP1->bIntersected) e.pVP1 = pEdge->pVP2, e.pVP2 = pEdge->pVP1;

		if (HandleEdgeEdgeCollision(e, T1, rgCP, pEdge->pVP1->bIntersected + pEdge->pVP2->bIntersected, 1.f, collideLevelSet)) {
			//break;
		}
	}

	for (DWORD i=0, end=collideLevelSet.m_pEdgeIndex->GetSize(); i<end; i++) {
		if ((*collideLevelSet.m_pEdgeIndex)[i] > collideLevelSet.m_rgEdge.GetSize())
			DebugBreak();

		SEdge * pEdge = &collideLevelSet.m_rgEdge[(*collideLevelSet.m_pEdgeIndex)[i]];
		if (pEdge->pVP1->bIntersected && pEdge->pVP2->bIntersected) continue;
		
		SEdge e = *pEdge;
		if (!pEdge->pVP1->bIntersected) e.pVP1 = pEdge->pVP2, e.pVP2 = pEdge->pVP1;

		if (collideLevelSet.HandleEdgeEdgeCollision(e, T2, rgCP, pEdge->pVP1->bIntersected + pEdge->pVP2->bIntersected, -1.f, *this)) {
			//break;
		}
	}*/

#ifdef PRINT
	g_Timer.EndTimer(2);
	g_Timer.DisplayTimer(2, "End intersection edges");
#endif

#ifdef PRINT
	g_Timer.StartTimer(3);
#endif

	/* Face-face containment */
	/*for (DWORD i=0, end=m_rgFace.GetSize(); i<end; i++) {
		SFace & face = m_rgFace[i];
		HandleFace(collideLevelSet, STriangleF(m_TransformObjToWorld*face.rgBoundaryVert[0].v, 
			m_TransformObjToWorld*face.rgBoundaryVert[1].v, m_TransformObjToWorld*face.rgBoundaryVert[2].v), -1, rgCP);
	}*/
	/*
	float fDist;
	// Body 1 
	for (DWORD i=0, end=m_pVertexParticleIndex->GetSize(); i<end; i++) {
		SVertexParticle & vP = m_rgVertexParticle[(*m_pVertexParticleIndex)[i]];
		//if (!vP.bIntersected) {
			SVertexParticle wVP = {
				Vector3F(0,0,0),
				Vector3F(0,0,0),
				0
			};
			
			for (DWORD j=0, end2=vP.rgInsetV.GetSize(); j<end2; j++) {
				//if (vP.rgInsetV[j].vertDist < vP.closestDist)
				//	continue; 

				wVP.v = T1*vP.rgInsetV[j].v;
				if (collideLevelSet.ParticleInLevelSet_Fast(wVP.v, fDist)) {
					cp.dist = fDist;
					cp.iNor = -m_TransformNormalGridToWorld*vP.rgInsetV[j].nor;
					cp.iPos = collideLevelSet.m_TransformObjToWorld*wVP.v;
					rgCP->PushBack(cp);
				}
			}
	//	}
	}
	
	// Body 2 
	for (DWORD i=0, end=collideLevelSet.m_pVertexParticleIndex->GetSize(); i<end; i++) {
		SVertexParticle & vP = collideLevelSet.m_rgVertexParticle[(*collideLevelSet.m_pVertexParticleIndex)[i]];
		//if (!vP.bIntersected) {
			SVertexParticle wVP = {
				Vector3F(0,0,0),
				Vector3F(0,0,0),
				0
			};

			for (DWORD j=0, end2=vP.rgInsetV.GetSize(); j<end2; j++) {
				//if (vP.rgInsetV[j].vertDist < vP.closestDist)
				//	continue; 

				wVP.v = T2*vP.rgInsetV[j].v;
				if (ParticleInLevelSet_Fast(wVP.v, fDist)) {
					cp.dist = fDist;
					cp.iNor = collideLevelSet.m_TransformNormalGridToWorld*vP.rgInsetV[j].nor;
					cp.iPos = m_TransformObjToWorld*wVP.v;
					rgCP->PushBack(cp);
				}
			}
	//	}
	}*/

	// Body 1 
	/*m_bDrawTris = collideLevelSet.m_bDrawTris = 0;
	if (m_bDrawTris) m_pCGraphics->SetupTriangleDraw(Vector3F(0, 1.f, 0));
	for (DWORD i=0, end=m_rgToProcessFace.GetSize(); i<end; i++) {
		SFace & face = *m_rgToProcessFace[i];
		SVertexParticle wVP = {
			Vector3F(0,0,0),
			m_TransformNormalGridToWorld*face.nor,
			.9
		};
		
		if (!m_bUseDumbScheme) {
			for (DWORD j=0, end=face.rgInteriorVert.GetSize(); j<end; j++) {
				wVP.v = T1*face.rgInteriorVert[j].v;
				if (collideLevelSet.ParticleInLevelSet_Fast(wVP.v, fDist)) {
					cp.iNor=collideLevelSet.ComputeGradient_Fast(wVP.v);
					if (IsNormalAdmissible(wVP.coneDir, wVP.coneAngle, (cp.iNor)))  {
						cp.dist = fDist;
						cp.iPos = collideLevelSet.m_TransformObjToWorld*wVP.v;
						rgCP->PushBack(cp);
					}
				}
			}
		} else 
			for (DWORD i=0, end=face.rgBoundaryVert.GetSize(); i<end; i+=2) {
				Vector3F triNor = STriangleF(m_TransformObjToWorld*face.rgBoundaryVert[i].v,
					m_TransformObjToWorld*face.rgBoundaryVert[(i+1)%end].v, m_TransformObjToWorld*face.rgBoundaryVert[(i+2)%end].v).Normal();
				HandleFace(collideLevelSet, STriangleF(m_TransformObjToWorld*face.rgBoundaryVert[i].v,
					m_TransformObjToWorld*face.rgBoundaryVert[(i+1)%end].v, m_TransformObjToWorld*face.rgBoundaryVert[(i+2)%end].v), -1.f, rgCP);
			}
	}

	// Body 2 
	for (DWORD i=0, end=collideLevelSet.m_rgToProcessFace.GetSize(); i<end; i++) {
		SFace & face = *collideLevelSet.m_rgToProcessFace[i];
		SVertexParticle wVP = {
			Vector3F(0,0,0),
			collideLevelSet.m_TransformNormalGridToWorld*face.nor,
			.9
		};
		
		if (!m_bUseDumbScheme) {
			for (DWORD j=0, end=face.rgInteriorVert.GetSize(); j<end; j++) {
				wVP.v = T2*face.rgInteriorVert[j].v;
				if (ParticleInLevelSet_Fast(wVP.v, fDist)) {
					cp.iNor=ComputeGradient_Fast(wVP.v);
					if (IsNormalAdmissible(wVP.coneDir, wVP.coneAngle, (cp.iNor))) {
						cp.dist = fDist;
						cp.iPos = m_TransformObjToWorld*wVP.v;
						cp.iNor = -cp.iNor;
						rgCP->PushBack(cp);
					}
				}
			}
		} else {
			for (DWORD i=0, end=face.rgBoundaryVert.GetSize(); i<end; i+=2) {
				collideLevelSet.HandleFace(*this, STriangleF(collideLevelSet.m_TransformObjToWorld*face.rgBoundaryVert[i].v,
					collideLevelSet.m_TransformObjToWorld*face.rgBoundaryVert[(i+1)%end].v, collideLevelSet.m_TransformObjToWorld*face.rgBoundaryVert[(i+2)%end].v), 1.f, rgCP);
			}
		}
	}*/
	// (collideLevelSet.m_rgToProcessFace.GetSize() != 4)
	//	DebugBreak();

#ifdef PRINT
	g_Timer.EndTimer(3);
	g_Timer.DisplayTimer(3, "End intersection faces");
#endif

	/* Reset all the intersected vertices */ //optimize in future
	for (DWORD i=0, end=m_pVertexParticleIndex->GetSize(); i<end; i++) {
		SVertexParticle & vP = m_rgVertexParticle[(*m_pVertexParticleIndex)[i]];
		vP.bIntersected = 0;
		vP.bAdmissible = 0;
	}
	for (DWORD i=0, end=collideLevelSet.m_pVertexParticleIndex->GetSize(); i<end; i++) {
		SVertexParticle & vP = collideLevelSet.m_rgVertexParticle[(*collideLevelSet.m_pVertexParticleIndex)[i]];
		vP.bIntersected = 0;
		vP.bAdmissible = 0;
	}

	/* Reset all processed faces */
	bool * rgProcessedFace = m_rgIsFaceProcessed.GetData();
	for (DWORD i=0, end=m_rgIsFaceProcessed.GetSize(); i<end; i++) {
		rgProcessedFace[i] = 0;
	}
	m_rgToProcessFace.Resize(0);
	rgProcessedFace = collideLevelSet.m_rgIsFaceProcessed.GetData();
	for (DWORD i=0, end=collideLevelSet.m_rgIsFaceProcessed.GetSize(); i<end; i++) {
		rgProcessedFace[i] = 0;
	}
	collideLevelSet.m_rgToProcessFace.Resize(0);

	/*static DWORD dwTrueCounter = 0;
	dwTrueCounter++;
	DWORD dwDiff = rgCP->GetSize() - oldSize;
	if (dwTrueCounter > 100) {
		char buf[100];
		itoa(dwDiff, buf, 10);
		OutputDebugStringA(buf);
		OutputDebugStringA("\n");
		dwTrueCounter = 0;
	}*/

	#ifdef PRINT
		g_Timer.EndTimer(0);
		g_Timer.DisplayTimer(0, "End levelset collision\n");
		g_Timer.PresentTimer(100);
	#endif

	return rgCP->GetSize() - oldSize;
}

bool CLevelSet::HandleEdgeEdgeCollision(SEdge & edge, Matrix4x4F & T, CArray<SContactPoint> * rgCP, 
	DWORD dwType, float norFlip, CLevelSet & collideLevelSet)
{
	/* If both vertices are admissible, then there is no reason to consider the rest of the edge */
	if ((edge.pVP1->closestDist >= 0 && edge.pVP2->closestDist >= 0 && (edge.pVP1->closestDist + edge.pVP2->closestDist) > edge.edgeLength) ||
		(edge.pVP1->bAdmissible + edge.pVP2->bAdmissible == 2) ) 
		return 0;

	Vector3F v1(m_TransformObjToWorld*edge.pVP1->v);
	Vector3F v2(m_TransformObjToWorld*edge.pVP2->v);
	Vector3F coneDir = m_TransformNormalGridToWorld*edge.coneDir;
	Vector3F edgeDir(v2 - v1);
	float edgeLen = edgeDir.Length();
	int nVertPerEdge = floorf_ASM(edgeLen/m_fMinEdgeLen)-1;
	if (nVertPerEdge <= 0) return 0;
	float t = 1.f/(nVertPerEdge+1);

	/* Find the first interior point starting from v1 */
	DWORD dwVertV1 = 0;
	Vector3F & edgeDirV1 = edgeDir;
	float fClosestDistV1 = 0;
	SRay ray;
	SRayIntersectData intersect0, intersect1;
	ray.p = collideLevelSet.m_TransformWorldToObj*v1;
	ray.d = edgeDirV1.MulNormal(edgeDirV1, collideLevelSet.m_TransformWorldToObj);
	if (!collideLevelSet.m_OBBox.OBBoxIntersectRelativeW(ray, 1, &intersect0, &intersect1))
		return 0;
	intersect0.pos = collideLevelSet.m_TransformObjToWorld*intersect0.pos;
	intersect1.pos = collideLevelSet.m_TransformObjToWorld*intersect1.pos;

	Vector3F & v1Initial = intersect0.pos;
	if (!edge.pVP1->bIntersected)
		if (!MarchExteriorEdgeVertex(collideLevelSet, nVertPerEdge, edge.edgeLength, v1Initial, edgeDirV1, fClosestDistV1, v1, 1))
			return 0;
	// MarchInteriorEdgeVertex(collideLevelSet, nVertPerEdge, edge.edgeLength, v1Initial, edgeDirV1, fClosestDistV1, v1, 1)
	
	/* Find the first interior point starting from v2 */
	DWORD dwVertV2 = 0;
	Vector3F edgeDirV2(-edgeDir);
	float fClosestDistV2 = 0;
	Vector3F & v2Initial = intersect1.pos;
	if (!edge.pVP2->bIntersected)
		if (!MarchExteriorEdgeVertex(collideLevelSet, nVertPerEdge, edge.edgeLength, v2Initial, edgeDirV2, fClosestDistV2, v2, 1))
			return 0;

	/* Possibly compute a tighter bound on the intersection region */
	float fDist = 0;
	//v1 = v1 + (t*(float)dwVertV1)*edgeDirV1;
	//v2 = v2 + (t*(float)dwVertV2)*edgeDirV2;
	//if (Abs((int)dwVertV1 - (int)dwVertV2) <= 2) {
	Vector3F vOutside1(v1), vOutside2(v2);
	float stepSize = 1.*collideLevelSet.m_fCellSize;

	Vector3F stepDir(-edgeDir.Normalize());
	if (!edge.pVP1->bIntersected) {
		for (DWORD i=1; ; i++) {
			vOutside1 += stepSize*stepDir;
			if (!collideLevelSet.ParticleInLevelSet(vOutside1, fDist)) {
				break;
			}
			v1 = vOutside1;
		}
	}
		
	if (!edge.pVP2->bIntersected) {
		stepDir = -stepDir;
		for (DWORD i=1; ; i++) {
			vOutside2 += stepSize*stepDir;
			if (!collideLevelSet.ParticleInLevelSet(vOutside2, fDist)) {
				break;
			}
			v2 = vOutside2;
		}
	}

	return ComputeEdgeEdgeIntersection(collideLevelSet, nVertPerEdge, edge.pVP1->bAdmissible, edge.pVP2->bAdmissible, v1, v2, edgeDirV1, coneDir, edge.coneAngle, norFlip, rgCP);
}

/*

bool CLevelSet::HandleEdgeEdgeCollision(SEdge & edge, Matrix4x4F & T, CArray<SContactPoint> * rgCP, 
	DWORD dwType, float norFlip, CLevelSet & collideLevelSet)
{
	/* If both vertices are admissible, then there is no reason to consider the rest of the edge 
	if ((edge.pVP1->closestDist >= 0 && edge.pVP2->closestDist >= 0 && (edge.pVP1->closestDist + edge.pVP2->closestDist) > edge.edgeLength) ||
		(edge.pVP1->bAdmissible + edge.pVP2->bAdmissible == 2) ) 
		return 0;

	Vector3F v1(m_TransformObjToWorld*edge.pVP1->v);
	Vector3F v2(m_TransformObjToWorld*edge.pVP2->v);
	Vector3F coneDir = m_TransformNormalGridToWorld*edge.coneDir;
	Vector3F edgeDir(v2 - v1);
	float edgeLen = edgeDir.Length();
	int nVertPerEdge = floorf_ASM(edgeLen/m_fMinEdgeLen)-1;
	if (nVertPerEdge <= 0) return 0;
	float t = 1.f/(nVertPerEdge+1);

	/* Find the first interior point starting from v1 
	DWORD dwVertV1 = 0;
	Vector3F & edgeDirV1 = edgeDir;
	float fClosestDistV1 = 0;
	SRay ray;
	SRayIntersectData intersect0, intersect1;
	ray.p = collideLevelSet.m_TransformWorldToObj*v1;
	ray.d = edgeDirV1.MulNormal(edgeDirV1, collideLevelSet.m_TransformWorldToObj);
	if (!collideLevelSet.m_OBBox.OBBoxIntersectRelativeW(ray, 1, &intersect0, &intersect1))
		return 0;
	intersect0.pos = collideLevelSet.m_TransformObjToWorld*intersect0.pos;
	intersect1.pos = collideLevelSet.m_TransformObjToWorld*intersect1.pos;

	Vector3F & v1Initial = intersect0.pos;
	if (!edge.pVP1->bIntersected)
		if (!MarchExteriorEdgeVertex(collideLevelSet, nVertPerEdge, edge.edgeLength, v1Initial, edgeDirV1, fClosestDistV1, v1, 1))
			return 0;
	// MarchInteriorEdgeVertex(collideLevelSet, nVertPerEdge, edge.edgeLength, v1Initial, edgeDirV1, fClosestDistV1, v1, 1)
	
	/* Find the first interior point starting from v2 
	DWORD dwVertV2 = 0;
	Vector3F edgeDirV2(-edgeDir);
	float fClosestDistV2 = 0;
	Vector3F & v2Initial = intersect1.pos;
	if (!edge.pVP2->bIntersected)
		if (!MarchExteriorEdgeVertex(collideLevelSet, nVertPerEdge, edge.edgeLength, v2Initial, edgeDirV2, fClosestDistV2, v2, 1))
			return 0;

	return ComputeEdgeEdgeIntersection(collideLevelSet, nVertPerEdge, edge, v1, v2, edgeDirV1, coneDir, edge.coneAngle, norFlip, rgCP);
}
*/

bool CLevelSet::ComputeEdgeEdgeIntersection(CLevelSet & collideLevelSet, DWORD nVertPerEdge, bool bAdmissible1, bool bAdmisslbe2, Vector3F & v1, 
	Vector3F & v2, Vector3F & edgeDir, Vector3F & coneDir, float coneAngle, float norFlip, CArray<SContactPoint> * rgCP)
{
	/* If one is admissible, ... */ 
	if (bAdmissible1 + bAdmisslbe2 == 1) {
		Vector3F v, nor;
		if (bAdmissible1) v = v2, nor = collideLevelSet.ComputeGradient(v1);
		else v = v1, nor = collideLevelSet.ComputeGradient(v2);

		SContactPoint cp;
		cp.iNor = norFlip*nor;
		//cp.dist = 
		if (cp.iNor.LengthSq() < .5) return 0;
		
		cp.dwFaceIndex[0] = 1;

		cp.iPos = v;
		collideLevelSet.ParticleInLevelSet(cp.iPos, cp.dist);
		rgCP->PushBack(cp); 
		return 1;
	}

	/* Actual Edge-Edge stuff */
	Vector3F avgPos = .5f*(v1 + v2);
	Vector3F avgNor = collideLevelSet.ComputeGradient(avgPos);
	if (!IsNormalAdmissible(coneDir, coneAngle, avgNor)) return 0;
	
	Vector3F & n1 = collideLevelSet.ComputeGradient(v1);
	Vector3F & n2 = collideLevelSet.ComputeGradient(v2);

	Vector3F eLine(CrossXYZ(n1, n2).Normalize());
	/* If the avg nor does not lie in the span of n1, n2, then it is edge-face */
	if (abs(DotXYZ(eLine, avgNor)) > .001) {
		SContactPoint cp;
		cp.iNor = norFlip*avgNor;
		if (cp.iNor.LengthSq() < .5) return 0;

		cp.dwFaceIndex[0] = 1;

		cp.iPos = v1;
		collideLevelSet.ParticleInLevelSet(cp.iPos, cp.dist);
		rgCP->PushBack(cp);
		cp.iPos = v2;
		collideLevelSet.ParticleInLevelSet(cp.iPos, cp.dist);
		rgCP->PushBack(cp);

		return 1;
	} else {
		Vector3F iNor = norFlip*(CrossXYZ(edgeDir, eLine).Normalize());
		if (iNor.LengthSq() < .5) return 0;
		if (DotXYZ(iNor, norFlip*avgNor) < 0) 
			iNor = -iNor;

		SContactPoint cp;
		cp.iNor = iNor;

		/*cp.iPos = v1;
		rgCP->PushBack(cp);

		cp.iPos = v2;
		rgCP->PushBack(cp);*/

		cp.dwFaceIndex[0] = 2;

		cp.iPos = .5f*(v1+v2);
		collideLevelSet.ParticleInLevelSet(cp.iPos, cp.dist);
		rgCP->PushBack(cp);

		return 1;
	}
	return 0;

	
	/*SContactPoint cp;
	cp.dwFaceIndex[0] = 2;
	cp.iNor = norFlip*avgNor;

	//cp.iPos = .5f*(v1+v2);
	//rgCP->PushBack(cp);

	cp.iPos = v1;
	rgCP->PushBack(cp);

	cp.iPos = v2;
	rgCP->PushBack(cp);*/

	return 1;
}

float CLevelSet::ParticleInLevelSet_Cheap(Vector3F & particle, float & fDist)
{
	Vector3F gridPt(m_TransformWorldToObj*particle), cpt;
	fDist = 0;
	/* Check if the particle is in the grid */
	if (gridPt.x < m_GridMin.x || gridPt.y < m_GridMin.y || gridPt.z < m_GridMin.z || 
		gridPt.x > m_GridMax.x || gridPt.y > m_GridMax.y || gridPt.z > m_GridMax.z) return 0; 
	gridPt -= m_TransformObjToGrid;

	/* Check if the particle is in the polyhedron */
	int cellX = (int)floorf_ASM(gridPt.x/m_fCellSize);
	int cellY = (int)floorf_ASM(gridPt.y/m_fCellSize);
	int cellZ = (int)floorf_ASM(gridPt.z/m_fCellSize);
	if (cellX < 0 || cellY < 0 || cellZ < 0) 
		DebugBreak();
	if (cellX >= m_nCellsX || cellY >= m_nCellsY || cellZ >= m_nCellsZ) 
		DebugBreak();

	float t = ComputeSignedDistance(
		Vector3F(fracf(gridPt.x/m_fCellSize), fracf(gridPt.y/m_fCellSize), fracf(gridPt.z/m_fCellSize)),
		cellX, cellY, cellZ);

	fDist = t;
	return t < -1e-4;
}

bool CLevelSet::MarchExteriorEdgeVertex(CLevelSet & collideLevelSet, DWORD nVertPerEdge,
	float fEdgeLength, Vector3F & v1, Vector3F & edgeDir, float & fClosestDist, Vector3F & finalPt, bool bCmp)
{
	float t = 1.f/(nVertPerEdge+1);
	float stepSize = t*fEdgeLength;
	DWORD dwIncr = 0;
	SEdgeParticle wEP;
	do {
		int iAdvance = (int)floorf_ASM(Abs(fClosestDist)/stepSize);
		dwIncr += max(iAdvance, 1);
		if (dwIncr >= nVertPerEdge+1) {// > ?
			return 0;
		}

		wEP.v = v1 + (t*(float)dwIncr)*edgeDir;
	} while (collideLevelSet.ParticleInLevelSet(wEP.v, fClosestDist) != bCmp);
	finalPt = wEP.v;
	return 1;
}

bool CLevelSet::MarchInteriorEdgeVertex(CLevelSet & collideLevelSet, DWORD nVertPerEdge,
	float fEdgeLength, Vector3F & v1, Vector3F & edgeDir, float & fClosestDist, Vector3F & finalPt, bool bCmp)
{
	float t = 1.f/(nVertPerEdge+1);
	float stepSize = t*fEdgeLength;
	DWORD dwIncr = 0;
	SEdgeParticle wEP;
	do {
		int iAdvance = (int)floorf_ASM(Abs(fClosestDist)/stepSize);
		dwIncr += max(iAdvance, 1);
		if (dwIncr >= nVertPerEdge+1) {// > ?
			return 0;
		}

		finalPt = wEP.v;
		wEP.v = v1 + (t*(float)dwIncr)*edgeDir;
	} while (collideLevelSet.ParticleInLevelSet(wEP.v, fClosestDist) == 1);
	return 1;
}

bool CLevelSet::MarchToAdmissibleVertex(CLevelSet & collideLevelSet, DWORD nVertPerEdge,
	float fEdgeLength, Vector3F & v1, Vector3F & edgeDir, Vector3F & coneDir, float coneAngle)
{
	exit(0);
	float t = 1.f/(nVertPerEdge+1);
	float stepSize = t*fEdgeLength;
	DWORD dwIncr = 0;
	Vector3F nor(collideLevelSet.ComputeGradient(v1));
	while (!IsNormalAdmissible(coneDir, coneAngle, nor)) {
		int iAdvance = 1;
		dwIncr += iAdvance;
		if (dwIncr >= nVertPerEdge+1) {
			return 0;
		}

		v1 = v1 + (t*(float)dwIncr)*edgeDir;
		float fDist;
		if (!collideLevelSet.ParticleInLevelSet(v1, fDist))
			return 0;
		nor = collideLevelSet.ComputeGradient(v1);
	}

	return 1;
}

Vector3F CLevelSet::ComputeGradient(Vector3F & pt)
{
	Vector3F gridPt(TransformWorldToGrid(pt));
	int cellX = (int)floorf_ASM(gridPt.x/m_fCellSize);
	int cellY = (int)floorf_ASM(gridPt.y/m_fCellSize);
	int cellZ = (int)floorf_ASM(gridPt.z/m_fCellSize);
	Vector3F dxyz(fracf(gridPt.x/m_fCellSize), fracf(gridPt.y/m_fCellSize), fracf(gridPt.z/m_fCellSize));

	if (cellX < 0 || cellY < 0 || cellZ < 0) 
		DebugBreak();
	if (cellX >= m_nCellsX || cellY >= m_nCellsY || cellZ >= m_nCellsZ) 
		DebugBreak();

	/*m_rgLSVertex[(cellX)*m_nLSVertsY*m_nLSVertsX + (12)*m_nLSVertsX + (22)].dist;

	m_rgLSVertex[(cellX)*m_nLSVertsY*m_nLSVertsX + (12)*m_nLSVertsX + (22+1)].dist;
	m_rgLSVertex[(cellX)*m_nLSVertsY*m_nLSVertsX + (12)*m_nLSVertsX + (22-1)].dist;

	m_rgLSVertex[(cellX)*m_nLSVertsY*m_nLSVertsX + (12-1)*m_nLSVertsX + (22)].dist;
	m_rgLSVertex[(cellX)*m_nLSVertsY*m_nLSVertsX + (12+1)*m_nLSVertsX + (22)].dist;

	m_rgLSVertex[(cellX-1)*m_nLSVertsY*m_nLSVertsX + (12)*m_nLSVertsX + (22)].dist;
	m_rgLSVertex[(cellX+1)*m_nLSVertsY*m_nLSVertsX + (12)*m_nLSVertsX + (22)].dist;*/

	//cellX -= 2;
	//cellX = 22;
	//cellY = 12;
	//cellZ = 6;

	 Vector3F vec(Vector3F(
		ComputeSignedDistance(Vector3F(1.f, dxyz.y, dxyz.z), cellX, cellY, cellZ) - 
		ComputeSignedDistance(Vector3F(0.f, dxyz.y, dxyz.z), cellX, cellY, cellZ),

		ComputeSignedDistance(Vector3F(dxyz.x, 1.f, dxyz.z), cellX, cellY, cellZ) - 
		ComputeSignedDistance(Vector3F(dxyz.x, 0.f, dxyz.z), cellX, cellY, cellZ),

		ComputeSignedDistance(Vector3F(dxyz.x, dxyz.y, 1.f), cellX, cellY, cellZ) - 
		ComputeSignedDistance(Vector3F(dxyz.x, dxyz.y, 0.f), cellX, cellY, cellZ)));
	 return m_TransformNormalGridToWorld * vec.Normalize();
}


Vector3F CLevelSet::ComputeGradient_Fast(Vector3F & pt)
{
	Vector3F gridPt(pt - m_TransformObjToGrid);
	int cellX = (int)floorf_ASM(gridPt.x/m_fCellSize);
	int cellY = (int)floorf_ASM(gridPt.y/m_fCellSize);
	int cellZ = (int)floorf_ASM(gridPt.z/m_fCellSize);
	Vector3F dxyz(fracf(gridPt.x/m_fCellSize), fracf(gridPt.y/m_fCellSize), fracf(gridPt.z/m_fCellSize));

	if (cellX < 0 || cellY < 0 || cellZ < 0) 
		DebugBreak();
	if (cellX >= m_nCellsX || cellY >= m_nCellsY || cellZ >= m_nCellsZ) 
		DebugBreak();

	 Vector3F vec(Vector3F(
		ComputeSignedDistance(Vector3F(1.f, dxyz.y, dxyz.z), cellX, cellY, cellZ) - 
		ComputeSignedDistance(Vector3F(0.f, dxyz.y, dxyz.z), cellX, cellY, cellZ),

		ComputeSignedDistance(Vector3F(dxyz.x, 1.f, dxyz.z), cellX, cellY, cellZ) - 
		ComputeSignedDistance(Vector3F(dxyz.x, 0.f, dxyz.z), cellX, cellY, cellZ),

		ComputeSignedDistance(Vector3F(dxyz.x, dxyz.y, 1.f), cellX, cellY, cellZ) - 
		ComputeSignedDistance(Vector3F(dxyz.x, dxyz.y, 0.f), cellX, cellY, cellZ)));
	 return m_TransformNormalGridToWorld * vec.Normalize();
}

void CLevelSet::DrawLevelset(CCollisionGraphics * pInstance)
{
	/*Matrix4x4F rot;
	m_OBBox.RotationW(rot);
	pInstance->DrawBox(m_OBBox.GetCenterW(), m_OBBox.GetHalfWidthsW(), rot);
	return;*/

	int count=0;
	for (int k=0; k<m_nLSVertsZ; k++) {
		for (int j=0; j<m_nLSVertsY; j++) {
			for (int i=0; i<m_nLSVertsX; i++) {
				float dist = m_rgLSVertex[k*m_nLSVertsY*m_nLSVertsX + j*m_nLSVertsX + k].dist;
				Vector3F pos(Vector3F((float)i * m_fCellSize, (float)j * m_fCellSize, (float)k * m_fCellSize) + m_TransformObjToGrid);
			
				//int d =8;
				//if ((i > d && i < m_nLSVertsX - d) || (j > d && j < m_nLSVertsY - d) || (k > d && k < m_nLSVertsZ - d))
				//	continue;

				/*if (dist <= .04 && dist > -.6f*m_fCellSize) {				
					pos = m_TransformObjToWorld * pos;
					pInstance->DrawPoint(pos, .01f, Vector3F(0, 0, 1.f));
					//Vector3F dir(ComputeGradient(pos));
					//pInstance->DrawNormal(pos, dir, .01f, .2, Vector3F(0, 1, 0));
				//}*/
			  if (dist <= 0) {
					pos = m_TransformObjToWorld * pos;
					//pInstance->DrawPoint(pos, .01f, Vector3F(1.f, 0, 0));
					Matrix4x4F rT;
					rT.MTranslation(pos);
					pInstance->DrawBox(rT, Vector3F(m_fCellSize));
				}
			}
		}
	}
}

void CLevelSet::DrawAdmissibleZones(CCollisionGraphics * pInstance)
{
	SVertexParticle * pParticles = m_rgVertexParticle.GetData();
	for (DWORD i=0, end=m_rgVertexParticle.GetSize(); i<end; i++) {
		pInstance->DrawNormal(m_TransformObjToWorld*pParticles[i].v, m_TransformNormalGridToWorld*pParticles[i].coneDir, .02, .2, Vector3F(1.f));
	}
}

void CLevelSet::DestroyLevelSet()
{
	if (m_rgLSVertex) {
		delete[] m_rgLSVertex;
		m_rgLSVertex = NULL;
		
		m_rgVertexParticle.Clear();
		m_rgEdgeParticle.Clear();
		m_rgEdge.Clear();
		m_rgFace.Clear();
		m_rgToProcessFace.Clear();
		m_rgIsFaceProcessed.Clear();
		m_rgEdgePiece.Clear();

		m_rgDefaultVertexParticleIndex.Clear();
		m_rgDefaultEdgeIndex.Clear();
	}
}



void CLevelSet::HandleFace(CLevelSet & collideLevelSet, STriangleF & face, float norFlip, CArray<SContactPoint> * rgCP)
{
	float fClosestDist;
	if (!collideLevelSet.ParticleInLevelSet(face.Centroid(), fClosestDist)) return;
	float fCentroidRadius = face.CentroidRadius();
	if (fCentroidRadius < fClosestDist) return;

	float fTriSize = face.MaxEdgeLength();
	if (fTriSize <= 4.f*collideLevelSet.m_fCellSize) {
		SContactPoint cp;
		cp.iNor = norFlip*face.Normal();
		cp.dist = fClosestDist;

		cp.iPos = face.vPosW[0];
		if (collideLevelSet.ParticleInLevelSet(cp.iPos, fClosestDist)) rgCP->PushBack(cp);
		cp.iPos = face.vPosW[1];
		if (collideLevelSet.ParticleInLevelSet(cp.iPos, fClosestDist)) rgCP->PushBack(cp);
		cp.iPos = face.vPosW[2];
		if (collideLevelSet.ParticleInLevelSet(cp.iPos, fClosestDist)) rgCP->PushBack(cp);

		return;
	}

	STriangleF rgSubdividedTri[4];
	SubdivideTriangle(face, rgSubdividedTri);

	HandleFace(collideLevelSet, rgSubdividedTri[0], norFlip, rgCP);
	HandleFace(collideLevelSet, rgSubdividedTri[1], norFlip, rgCP);
	HandleFace(collideLevelSet, rgSubdividedTri[2], norFlip, rgCP);
	HandleFace(collideLevelSet, rgSubdividedTri[3], norFlip, rgCP);
}


};



/*static const float eps = 1e-4;
	float fPosDist = FLT_MAX, fPosPlaneDist = FLT_MAX;
	float fNegDist = FLT_MAX, fNegPlaneDist = -FLT_MAX;
	float fDist = 0;
	for (DWORD i=0, end=nVert/3; i<end; i++) {
		STriangleF tri(rgVert[i*3 + 0], rgVert[i*3 + 1], rgVert[i*3 + 2]);
		Vector3F cPt = ComputeClosestPoint(tri, pt);
		Vector3F dist(pt - cPt);
		float fDist = dist.Length();

		PlaneF p(rgVert[i*3 + 0], rgVert[i*3 + 1], rgVert[i*3 + 2]);
		float fPlaneDist = p.DistanceToPlane(pt);

		if (fPlaneDist >= 0 && fDist < fPosDist)		{fPosPlaneDist = fPlaneDist; fPosDist = fDist; }
		else if (fPlaneDist <= 0 && fDist < fNegDist)	{fNegPlaneDist = fPlaneDist; fNegDist = fDist; }

		//if (fPlaneDist >= eps && fPlaneDist < fPosPlaneDist)		{fPosPlaneDist = fPlaneDist; fPosDist = fDist; }
		//else if (fPlaneDist < eps && fPlaneDist > fNegPlaneDist)	{fNegPlaneDist = fPlaneDist; fNegDist = fDist; }
	}

	//return fPosPlaneDist < 1e6 ? fPosPlaneDist : fNegPlaneDist;

	if (abs(fPosDist - fNegDist) < 1e-3)
		return abs(fPosPlaneDist) > abs(fNegPlaneDist) ? fPosDist : -fNegDist;
	
	return fPosDist < fNegDist ? fPosDist : -fNegDist;
	*/

	/*
	DWORD nNor = 0;
	Vector3F angleWeightedNor(0,0,0), nextNor;
	DWORD dwNextEdge = dwInitialEdge;
	DWORD dwNextTri = dwInitialTri;

	do {
		Vector3F angleWeightedNor += ComputeAngleWeightedNormal();
	Vector3F preNor(SELECT_TRIANGLE_NORMAL(dwInitialTri));
	Vector3F nextNor(SELECT_TRIANGLE_NORMAL(dwInitialTri));
	//STriangleF & preTri = rgTri[dwInitialTri];
	bool bLooped = 0;
	do {
		nextNor = SELECT_TRIANGLE_NORMAL(dwNextTri);
		AverageNormals(nextNor, preNor, avgNor, nNor);

		DWORD dwPreTri = dwNextTri;
		dwNextTri = rgAdj[3*dwNextTri + dwNextEdge];

		/* Check the three edges 
		dwNextEdge = 0;
		while (rgAdj[3*dwNextTri+dwNextEdge] != dwPreTri) dwNextEdge++;
		dwNextEdge = (dwNextEdge + 1) % 3;
		
		preNor = nextNor;

		if (dwPreTri == dwInitialTri && bLooped) {
			break;
		} bLooped = 1;
	} while (1);
	if (nNor == 0) { /* Vertex on face 
		dir = nextNor;
		fAngle = .99f;
		return 1;
	}
	dir = avgNor = avgNor.Normalize();

	/* Compute the maximum divergence (fTheta) among the avgNor and the polygon normals 
	fAngle = 1.;
	dwNextTri = dwInitialTri;
	dwNextEdge = dwInitialEdge;
	do {
		/* Find the maximum of theta 
		Vector3F & n = SELECT_TRIANGLE_NORMAL(dwNextTri);
		float d = DotXYZ(n, avgNor);
		fAngle = min(fAngle, d);

		DWORD dwPreTri = dwNextTri;
		dwNextTri = rgAdj[3*dwNextTri + dwNextEdge];

		/* Check the three edges 
		dwNextEdge = 0;
		while (rgAdj[3*dwNextTri+dwNextEdge] != dwPreTri) dwNextEdge++;
		dwNextEdge = (dwNextEdge + 1) % 3;

	} while (dwNextTri != dwInitialTri);

	return nNor > 2 ? 1 : 1;
	*/


		//float fDist;
	//for (DWORD i=1; i<=nVertPerEdge; i++) {
	//	SEdgeParticle wEP(v1 + (t*(float)i)*edgeDir);
	//	if (collideLevelSet.ParticleInLevelSet(wEP.v, fDist)) {

	//		float stepSize = .5f*collideLevelSet.m_fCellSize;
	//		//Vector3F stepDir(edgeDir.Normalize());
	//		float maxLeft = (v1 - wEP.v).Length()+.1;
	//		float maxRight = (v2 - wEP.v).Length()+.1;
	//		/* Step forward */
	//		DWORD idx = 0;
	//		Vector3F vOutside1;
	//		for (; ; idx++) {
	//			vOutside1 = (wEP.v + ((float)idx * stepSize)*stepDir);
	//			if ((float)idx * stepSize > maxRight)
	//				return 0;

	//			if (!collideLevelSet.ParticleInLevelSet(vOutside1, fDist)) {
	//				idx--;
	//				break;
	//			}
	//		}
	//		Vector3F & n1 = collideLevelSet.ComputeGradient(vOutside1);

	//		/* Step backward */
	//		idx = 0;
	//		stepDir = -stepDir;
	//		Vector3F vOutside2;
	//		bool found = 0;
	//		for (; ; idx++) {
	//			vOutside2 = (wEP.v + ((float)idx * stepSize)*stepDir);
	//			if ((float)idx * stepSize > maxLeft)
	//				return 0;

	//			if (!collideLevelSet.ParticleInLevelSet(vOutside2, fDist)) {
	//				idx--;
	//				found=1;
	//				break;
	//			}
	//		}
	//		Vector3F & n2 = collideLevelSet.ComputeGradient(vOutside2);

	//		Vector3F eLine(CrossXYZ(n1, n2));
	//		//if (DotXYZ(eLine, edgeDir) > .9f) {
	//		//	return 0;
	//		//}
	//		Vector3F avgPos = .5f*(vOutside1 + vOutside2);
	//		Vector3F avgNor = collideLevelSet.ComputeGradient(avgPos);
	//		/* If the avg nor does not lie in the span of n1, n2, then it is edge-face */
	//		Vector3F eLineNorm(eLine.Normalize());
	//		float dd = abs(DotXYZ(eLineNorm, avgNor)); 
	//		if (dd > .001) {
	//			SContactPoint cp;
	//			for (DWORD j=1; j<=nVertPerEdge; j++) {
	//				Vector3F v(v1 + (t*(float)j)*edgeDir);
	//				if (!collideLevelSet.ParticleInLevelSet(v, fDist)) continue;
	//				cp.iPos = v;
	//				cp.iNor = collideLevelSet.ComputeGradient(v);
	//				if (cp.iNor.LengthSq() < .5 || !IsNormalAdmissible(coneDir, edge.coneAngle, cp.iNor)) continue;
	//				cp.iNor = norFlip*cp.iNor;
	//				rgCP->PushBack(cp);
	//			}
	//		
	//			return 1;
	//		} else {
	//			Vector3F iNor = CrossXYZ(edgeDir, eLine).Normalize();
	//			if (DotXYZ(iNor, wEP.v) > 0) iNor = -iNor;
	//			/* Check that the normal is contained in the admissible zone */
	//			float d = abs(DotXYZ(iNor, coneDir)) + 5e-3;
	//			//if (d > edge.coneAngle) {
	//				/* An edge is of dimension one fewer than a vertex */
	//				d = DotXYZ(iNor, edgeDir);
	//				if (DotXYZ(iNor, edgeDir) < 1e-3) {
	//					SContactPoint cp;
	//					cp.iPos = (.5f*(vOutside1 + vOutside2));
	//					cp.iNor = iNor;
	//					if (cp.iNor.LengthSq() < .5) return 0;
	//					rgCP->PushBack(cp);

	//					return 1;
	//				}
	//			//}
	//		}
	//		return 0;
	//	}
	//}
	//return 0;