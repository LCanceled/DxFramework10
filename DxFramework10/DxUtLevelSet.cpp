
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
#include <functional>

#define GET_TRIANGLE_NORMAL(_verts, _i) (STriangleF(_verts[3*_i + 0], _verts[3*_i + 1], _verts[3*_i + 2]).Normal())

namespace DxUt {

CLevelSet::CLevelSet(): m_bUseFaceSampling(0), m_bDrawTris(0)
{
	m_TransformWorldToObj.MIdenity();
	m_TransformObjToWorld.MIdenity();
	m_TransformNormalGridToWorld.MIdenity();

	m_ValidParticleDistance = 1e-4;
}

int CLevelSet::ComputeAdmissibleZone(UINT uiInitialTri, UINT uiInitialEdge,
	Vector3F * verts, UINT * pAdj, Vector3F & angleWeightedNor, Vector3F & edgeNor, float & coneCosAngle, CArray<Vector3F> * adjFaceNormals)
{
	UINT nNor = 0;
	angleWeightedNor = Vector3F(0,0,0);
	UINT uiNextEdge = uiInitialEdge;
	UINT uiNextTri = uiInitialTri;
	do {
		/* Compute the angle weighted normal */
		Vector3F & n = GET_TRIANGLE_NORMAL(verts, uiNextTri);
		int iPreEdge = (int)uiNextEdge-1 < 0 ? 2 : (int)uiNextEdge-1;
		Vector3F e1(Vector3F(verts[3*uiNextTri + iPreEdge]			- verts[3*uiNextTri + uiNextEdge]).Normalize());
		Vector3F e2(Vector3F(verts[3*uiNextTri + (uiNextEdge+1)%3] - verts[3*uiNextTri + uiNextEdge]).Normalize());
		float fAngle = acosf(DotXYZ(e1, e2));
		angleWeightedNor += fAngle*n;

		UINT uiPreTri = uiNextTri;
		uiNextTri = pAdj[3*uiNextTri + uiNextEdge];
		if (uiNextTri == -1)
			return -1;

		/* Check the three edges */
		uiNextEdge = 0;
		while (pAdj[3*uiNextTri+uiNextEdge] != uiPreTri) {
			uiNextEdge++;
			if (3*uiNextTri+uiNextEdge == -1) return -1;//goto errorCase;
			if (uiNextEdge > 2) {
				uiNextTri = uiInitialTri;
				return -1;
			}
		}
		uiNextEdge = (uiNextEdge + 1) % 3;

		nNor++;
	} while (uiNextTri != uiInitialTri);
	angleWeightedNor = angleWeightedNor.Normalize();
	
	/* Compute the edge normal */ {
		Vector3F n1 = GET_TRIANGLE_NORMAL(verts, uiInitialTri);
		UINT uiAdjacentTri = pAdj[3*uiInitialTri + uiInitialEdge];
		if (3*uiInitialTri + uiInitialEdge == -1) return -1;
		Vector3F n2 = GET_TRIANGLE_NORMAL(verts, uiAdjacentTri);
		edgeNor = (n1 + n2).Normalize();
	}

	/* Compute the minimum cos angle (coneCosAngle) between the angleWeightedNor and the polygon normals */
	coneCosAngle = 1.;
	uiNextTri = uiInitialTri;
	uiNextEdge = uiInitialEdge;
	do {
		/* Find the max of the angle */
		Vector3F & n = GET_TRIANGLE_NORMAL(verts, uiNextTri);
		if (adjFaceNormals) adjFaceNormals->PushBack(n);
		float d = DotXYZ(n, angleWeightedNor);
		coneCosAngle = min(coneCosAngle, d);

		UINT uiPreTri = uiNextTri;
		uiNextTri = pAdj[3*uiNextTri + uiNextEdge];
		if (uiNextTri == -1)
			return -1;

		/* Check the three edges */
		uiNextEdge = 0;
		while (pAdj[3*uiNextTri+uiNextEdge] != uiPreTri) {
			uiNextEdge++;
			if (3*uiNextTri+uiNextEdge == -1) return -1;
			if (uiNextEdge > 2) {
				uiNextTri = uiInitialTri;
				return -1;
			}
		}
		uiNextEdge = (uiNextEdge + 1) % 3;
		nNor++;
	} while (uiNextTri != uiInitialTri);

	return nNor;
}

void CLevelSet::ComputeNondegenerateVertices(UINT nVert, UINT nTri, Vector3F * verts, UINT * pAdj, CFinitePointGrid3F<UINT> & vertToIndex)
{
	m_VertexParticles.Reserve(nVert);
	SVertexParticle * pVP = m_VertexParticles.GetData();
	for (UINT i=0; i<nVert; i++) {
		CArray<Vector3F> adjFaceNormals;
		SVertexParticle vp = {verts[i], Vector3F(0,0,0), 0.f };
		UINT uiEntry = 0, uiIndex = 0;
		if (!m_VertexParticles.FindIfEquals(vp, uiEntry)) {
			Vector3F dummy;
			if (ComputeAdmissibleZone(i / 3, i % 3, verts, pAdj, vp.coneDir, dummy, vp.coneCosAngle, &adjFaceNormals) > 0) {
				uiIndex = m_VertexParticles.GetSize();
				vertToIndex.AddPoint(vp.v, uiIndex);
				
				vp.bIntersected = 0;
				vp.bAdmissible = 0;
				m_VertexParticles.PushBack(vp);
				m_VertexParticles.GetBack().adjFaceNormals.Copy(adjFaceNormals);
			}
			else if (!vertToIndex.PointInGrid(vp.v, &uiIndex)) {
				uiIndex = -1;
				vertToIndex.AddPoint(vp.v, uiIndex);
			}
		}
	}
}

void CLevelSet::AddEdgeVertices(Vector3F & v1, Vector3F & v2, Vector3F & v3,
	CFinitePointGrid3F<UINT> & vertToIndex, Vector3F & normal, Vector3F * adjTriVerts, UINT & uiIndex)
{
	Vector3F & tV1 = adjTriVerts[1] - adjTriVerts[0];
	Vector3F & tV2 = adjTriVerts[2] - adjTriVerts[0];
	Vector3F tNor(CrossXYZ(tV1, tV2).Normalize());
	// Do not add interior edges of coplanar triangles
	//if (m_bUseFaceSampling) {
		if (DotXYZ(normal, tNor) > .99f) {
			uiIndex = -1; return;
		}
	//}

	SEdgeParticle edge;
	uiIndex = -1;
	// The admissible zones of some particles could not computed correctly
	if (!vertToIndex.PointInGrid(v1, &uiIndex)) {
		DebugBreak();
		uiIndex = -1; return; 
	} 
	if (uiIndex == -1) return;
	edge.pVP1 = &m_VertexParticles[uiIndex];
	if (!vertToIndex.PointInGrid(v2, &uiIndex)) {
		DebugBreak();
		uiIndex = -1; return; 
	}
	if (uiIndex == -1) return;
	edge.pVP2 = &m_VertexParticles[uiIndex];
	edge.coneDir = (normal + tNor).Normalize();
	edge.coneCosAngle = DotXYZ(normal, edge.coneDir);
	edge.edgeLength = (edge.pVP1->v - edge.pVP2->v).Length();

	edge.adjTriangleVerts[0] = v3;
	UINT uiAdjTriVertIndex=0;
	while (1) {
		if (adjTriVerts[uiAdjTriVertIndex] != v1 && adjTriVerts[uiAdjTriVertIndex] != v2) break;
		uiAdjTriVertIndex++;
		Assert(uiAdjTriVertIndex < 3, "CLevelSet::AddEdgeVertices cannot find adjacent triangle vertex.");
	}
	edge.adjTriangleVerts[1] = adjTriVerts[uiAdjTriVertIndex];
	edge.adjFaceNormals[0] = normal;
	edge.adjFaceNormals[1] = tNor;

	// Compute orthogonal edge vectors
	Vector3F edgeDir(edge.pVP1->v - edge.pVP2->v);
	Vector3F dir(edge.adjTriangleVerts[0] - edge.pVP1->v);
	edge.adjOrthoEdgeVecs[0] = CrossXYZ(normal, edgeDir).Normalize();
	if (DotXYZ(edge.adjOrthoEdgeVecs[0], dir) < 0) edge.adjOrthoEdgeVecs[0] = -edge.adjOrthoEdgeVecs[0];

	dir = edge.adjTriangleVerts[1] - edge.pVP1->v;
	edge.adjOrthoEdgeVecs[1] = CrossXYZ(tNor, edgeDir).Normalize();
	if (DotXYZ(edge.adjOrthoEdgeVecs[1], dir) < 0) edge.adjOrthoEdgeVecs[1] = -edge.adjOrthoEdgeVecs[1];

	edge.bIntersected = 0;
	edge.bAdmissible = 0;
	
	if (edge.edgeLength > m_CellSize) {
		uiIndex = m_EdgeParticles.GetSize();
		m_EdgeParticles.PushBack(edge);
	} else uiIndex = -1;
}


/*void CLevelSet::ComputeInsetVertices(Vector3F & nor, CFinitePointGrid3F<UINT> & vertToIndex, CArray<SSegment3F> & edgeDisordereds)
{
	SPolygon3F poly;
	CArray<SSegment3F> rgEdgeOrdered;
	poly.ComputePolygon(edgeDisordereds);
	CArray<SSegment3F> & edges = poly.edges;

	Vector3F o(0,0,0), B1(0), B2(0);
	//Compute2DPlaneBasis(nor, B1, B2);
	CArray<SFaceParticle::SFaceVertex> & interiorVertices = m_FaceParticles.GetBack().interiorVertices;
	for (UINT i=0, end=edges.GetSize(); i<end; i++) {
		SSegment3F & edge1 = edges[i];
		SSegment3F & edge2 = edges[(i+1)%end];
		Vector3F & v1 = (edge1.e1 - edge1.e2).Normalize();
		Vector3F & v2 = (edge2.e2 - edge2.e1).Normalize();
		Vector3F & v = (v1 + v2).Normalize();

		float dist = .125;//m_fMaxFacePenetrationDepth/CrossXYZ(v, v1).Length();
		Vector3F & pt = edge1.e2 + abs(dist)*v;

		UINT uiIndex = 0;
		if (!vertToIndex.PointInGrid(edge1.e2, &uiIndex))
			DebugBreak();
		m_VertexParticles[uiIndex].rgInsetV.PushBack(SFaceVertex(pt, nor, (edge1.e2 - pt).Length()));
		//interiorVertices.PushBack(SFaceParticle::SFaceVertex(ProjectPointTo2DPlaneBasis(pt, o, B1, B2), pt, NULL));
		interiorVertices.PushBack(SFaceParticle::SFaceVertex(Vector2F(0), pt, NULL));
	}
}

void CLevelSet::ComputeFaceVertices(UINT uiTri, SVisitedTriangle * tris, UINT * pAdj,
	CFinitePointGrid3F<UINT> & vertToIndex, CFinitePointGrid3F<UINT> & bVisitedEdges, CArray<SSegment3F> & edges)
{
	SVisitedTriangle * pVTri = &tris[uiTri];
	pVTri->bVisited = 1;	
	Vector3F & nor = m_FaceParticles.GetBack().nor;
	//PlaneF plane(pVTri->tri.vPosW[0], pVTri->tri.vPosW[1], pVTri->tri.vPosW[2]);
	Vector3F o(0,0,0), B1(0), B2(0);
	//Compute2DPlaneBasis(nor, B1, B2);

	/* Add the exterior edges 
	for (UINT j=0; j<3; j++) {
		SVisitedTriangle * nextVTri = &tris[pAdj[3*uiTri + j]];
		Vector3F nextNor(nextVTri->tri.Normal());
		//if (DotXYZ(nor, nextNor) < .999f) {
			edges.PushBack(SSegment3F(pVTri->tri.vPosW[j], pVTri->tri.vPosW[(j+1)%3]));
		//}
	}

	
	for (UINT i=0; i<3; i++) {
		UINT uiIndex, dwDummy;
		SVisitedTriangle * nextVTri = &tris[pAdj[3*uiTri + i]];
		Vector3F nextNor(nextVTri->tri.Normal());
		//if (DotXYZ(nor, nextNor) < .999f) {
			Vector3F & pt = pVTri->tri.vPosW[i];
			vertToIndex.PointInGrid(pt, &uiIndex);

			SFaceParticle & f = m_FaceParticles.GetBack();
			//f.boundaryVertices.PushBack(SFaceParticle::SFaceVertex(ProjectPointTo2DPlaneBasis(
			//	m_VertexParticles[uiIndex].v, o, B1, B2), m_VertexParticles[uiIndex].v, &m_VertexParticles[uiIndex]));
			f.boundaryVertices.PushBack(SFaceParticle::SFaceVertex(Vector2F(0), pt, &m_VertexParticles[uiIndex]));
			m_VertexParticles[uiIndex].rgFace.PushBack(&f);
			f.v += pt;
			f.dwFace = m_FaceParticles.GetSize()-1;
		//} else if (!tris[pAdj[3*uiTri + i]].bVisited) 
		//	ComputeFaceVertices(pAdj[3*uiTri + i], tris, pAdj, vertToIndex, bVisitedEdges, edges);
	}
}*/

void CLevelSet::ComputeMeshParticles(UINT nVert, UINT nTri, Vector3F * verts, UINT * pAdj, char * szLevelSetFile)
{
	CAABBox aabb;
	aabb.ComputeAABBox(verts, nVert);

	/* Compute the vertex particles */
	const float eps = 1e-3;
	CFinitePointGrid3F<UINT> vertToIndex;
	Vector3F gridMinP = m_GridMin;
	vertToIndex.CreateGrid(m_nCellsX, m_nCellsY, m_nCellsZ, m_CellSize, gridMinP);
	ComputeNondegenerateVertices(nVert, nTri, verts, pAdj, vertToIndex);
	
	/* Compute the edge particles */
	CFinitePointGrid3F<UINT> edgeToIndex;
	edgeToIndex.CreateGrid(m_nCellsX, m_nCellsY, m_nCellsZ, m_CellSize, gridMinP);

	/* V + F - E = 2 approximately */
	UINT nEdges = (m_VertexParticles.GetSize() + nTri - 2);
	//m_EdgeParticle.Reserve(nVertPerEdge*nEdges);
	m_EdgeParticles.Reserve(nEdges);  
	for (UINT i=0, dwSize=nTri*3; i<dwSize; i+=3) {
		Vector3F e1Avg(.5f*(verts[i+0]+verts[i+1]));
		Vector3F e2Avg(.5f*(verts[i+1]+verts[i+2]));
		Vector3F e3Avg(.5f*(verts[i+2]+verts[i+0]));
		Vector3F v1(verts[i+1] - verts[i+0]);
		Vector3F v2(verts[i+2] - verts[i+0]);
		Vector3F normal(CrossXYZ(v1, v2).Normalize());

		UINT uiTmp = 0, uiIndex = 0;
		if (!edgeToIndex.PointInGrid(e1Avg, &uiTmp)) {
			if (pAdj[i+0] != -1) {
				AddEdgeVertices(verts[i+0], verts[i+1], verts[i+2], vertToIndex, normal, &verts[3*pAdj[i+0]], uiIndex);
			} else uiIndex = -1;
			edgeToIndex.AddPoint(e1Avg, uiIndex);
		}

		if (!edgeToIndex.PointInGrid(e2Avg, &uiTmp)) {
			if (pAdj[i+1] != -1) {
				AddEdgeVertices(verts[i+1], verts[i+2], verts[i+0], vertToIndex, normal, &verts[3*pAdj[i+1]], uiIndex);
			} else uiIndex = -1;
			edgeToIndex.AddPoint(e2Avg, uiIndex);
		}
		
		if (!edgeToIndex.PointInGrid(e3Avg, &uiTmp)) {
			if (pAdj[i+2] != -1) {
				AddEdgeVertices(verts[i+2], verts[i+0], verts[i+1], vertToIndex, normal, &verts[3*pAdj[i+2]], uiIndex);
			} else uiIndex = -1;
			edgeToIndex.AddPoint(e3Avg, uiIndex); 
		}
	}

	/* Compute the face vertices  */
	if (m_bUseFaceSampling) {
		m_FaceParticles.Resize(nTri);
		for (UINT i=0; i<nTri; i++) {
			m_FaceParticles[i].boundaryVertices[0] = verts[3*i+0];
			m_FaceParticles[i].boundaryVertices[1] = verts[3*i+1];
			m_FaceParticles[i].boundaryVertices[2] = verts[3*i+2];
		}
	}
	//edgeToIndex.Reset();
	/*m_FaceParticles.Reserve(nTri);
	CArray<Vector3F> rgFaceVert;
	CArray<SSegment3F> edges;
	for (UINT i=0; i<nTri; i++) {
		if (!tris[i].bVisited) {
			m_FaceParticles.PushBack();
			Vector3F & nor = tris[i].tri.Normal();
			m_FaceParticles.GetBack().nor = nor;
			edges.Resize(0);

			//m_FaceParticles.GetBack().boundaryVertices.PushBack(SFaceParticle::SFaceVertex(Vector2F(0), tris[i].tri.vPosW[0], NULL));
			//m_FaceParticles.GetBack().boundaryVertices.PushBack(SFaceParticle::SFaceVertex(Vector2F(0), tris[i].tri.vPosW[1], NULL));
			//m_FaceParticles.GetBack().boundaryVertices.PushBack(SFaceParticle::SFaceVertex(Vector2F(0), tris[i].tri.vPosW[2], NULL));
			//ComputeFaceVertices(i, tris, pAdj, vertToIndex, edgeToIndex, edges);
			//ComputeInsetVertices(nor, vertToIndex, edges);

			//m_FaceParticles.GetBack().v /= m_FaceParticles.GetBack().rgVP.GetSize();
			/*rgFaceVert.Resize(m_FaceParticles.GetBack().interiorVertices.GetSize());
			for (UINT j=0, end=rgFaceVert.GetSize(); j<end; j++)
				rgFaceVert[j] = m_FaceParticles.GetBack().interiorVertices[j].v;
			m_FaceParticles.GetBack().oBB.ComputeOBB(rgFaceVert.GetData(), rgFaceVert.GetSize(), COBBox::CVVertices);
		}
	}
	delete[] tris;
	rgFaceVert.Clear();

	m_IsFaceProcessed.Resize(m_FaceParticles.GetSize());
	for (UINT i=0, end=m_IsFaceProcessed.GetSize(); i<end; i++) {
		m_IsFaceProcessed[i] = 0;
	}
	m_ToProcessFaceParticles.Reserve(m_IsFaceProcessed.GetSize());
	*/

	m_OBBox.ComputeOBB(verts, nVert, COBBox::OBBComputeMethod::CVTriangles);
	m_AABBox.ComputeAABBox(verts, nVert);

	m_AABBox.MinPW() = m_GridMin;
	m_AABBox.MaxPW() = m_GridMax;
	
	Vector3F rotVecs[] = {		
		m_OBBox.GetRotVecW(0) = Vector3F(1,0,0),
		m_OBBox.GetRotVecW(1) = Vector3F(0,1,0),
		m_OBBox.GetRotVecW(2) = Vector3F(0,0,1) };
	m_OBBox.SetOBB(m_AABBox.CenterPointW(), m_AABBox.HalfWidthsW(), rotVecs);//


	//PostComputeMeshParticles(szLevelSetFile, verts, nVert, pAdj, vertToIndex, edgeToIndex);

	vertToIndex.DestroyGrid();
	edgeToIndex.DestroyGrid();

	/* Collision indices */
	m_DefaultVertexParticleIndices.Resize(m_VertexParticles.GetSize());
	for (UINT i=0, end=m_VertexParticles.GetSize(); i<end; i++) m_DefaultVertexParticleIndices[i] = i;
	m_pVertexParticleIndex = &m_DefaultVertexParticleIndices;
	
	m_DefaultEdgeIndices.Resize(m_EdgeParticles.GetSize());
	for (UINT i=0, end=m_EdgeParticles.GetSize(); i<end; i++) m_DefaultEdgeIndices[i] = i;
	m_pEdgeIndex = &m_DefaultEdgeIndices;
}

void CLevelSet::ComputeSDF(Vector3F * verts, UINT nVert, UINT * pAdj)
{
	/* Compute the triangles */
	STriangleFEx * tris = new STriangleFEx[nVert/3];
	int counter =0;
	for (UINT i=0, end=nVert/3; i<end; i++) {
		float fDummy = 0;
		tris[i].vPosW[0] = verts[3*i + 0];
		tris[i].vPosW[1] = verts[3*i + 1];
		tris[i].vPosW[2] = verts[3*i + 2];
		tris[i].nor = GET_TRIANGLE_NORMAL(verts, i);
		tris[i].bBadTriangle |= ComputeAdmissibleZone(i, 0, verts, pAdj, tris[i].vertNor[0], tris[i].edgeNor[0], fDummy, NULL) == -1;
		tris[i].bBadTriangle |= ComputeAdmissibleZone(i, 1, verts, pAdj, tris[i].vertNor[1], tris[i].edgeNor[1], fDummy, NULL) == -1;
		tris[i].bBadTriangle |= ComputeAdmissibleZone(i, 2, verts, pAdj, tris[i].vertNor[2], tris[i].edgeNor[2], fDummy, NULL) == -1;
		float triArea = tris[i].Area();
		tris[i].bBadTriangle |= triArea < 1e-5;
		if (triArea < 1e-5)
			int a=0;
		if (tris[i].bBadTriangle)
			int a=0;
	}

	/* Compute the signed distance field */
	const float eps = 1e-3;
	CAABBox aabb;
	m_GridBox.ComputeAABBox(verts, nVert);
	m_GridMin = m_GridBox.MinPW() - Vector3F(eps);
	m_GridMax = m_GridBox.MaxPW() + Vector3F(eps);
	m_GridBox.SetMinP(m_GridMin);
	m_GridBox.SetMaxP(m_GridMax);
	m_TransformObjToGrid = m_GridMin;
	m_nCellsX = (int)ceilf_ASM((m_GridMax.x - m_GridMin.x)/m_CellSize); m_nLSVertsX = m_nCellsX+1;
	m_nCellsY = (int)ceilf_ASM((m_GridMax.y - m_GridMin.y)/m_CellSize); m_nLSVertsY = m_nCellsY+1;
	m_nCellsZ = (int)ceilf_ASM((m_GridMax.z - m_GridMin.z)/m_CellSize); m_nLSVertsZ = m_nCellsZ+1;

	m_LSVertices = new SLevelSetVertex[m_nLSVertsX*m_nLSVertsY*m_nLSVertsZ];
	for (int k=0; k<m_nLSVertsZ; k++) {
		for (int j=0; j<m_nLSVertsY; j++) {
			for (int i=0; i<m_nLSVertsX; i++) {
				m_LSVertices[k*m_nLSVertsY*m_nLSVertsX + j*m_nLSVertsX + i].dist = ComputeSignedDistance(
					Vector3F((float)i * m_CellSize, (float)j * m_CellSize, (float)k * m_CellSize) + m_TransformObjToGrid, verts, nVert, tris);
			}
		}
	}

	delete[] tris;
}

float CLevelSet::ComputeSignedDistance(Vector3F & pt, Vector3F * verts, UINT nVert, STriangleFEx * tris)
{
	static const float eps = 1e-4;
	float fBestDist = FLT_MAX;
	UINT uiIndex = 0;
	UINT uiType = 0;
	Vector3F closestPt;
	for (UINT i=0, end=nVert/3; i<end; i++) {
		UINT type = 0;
		Vector3F cPt = ComputeClosestPoint(tris[i], pt, type);
		if (tris[i].bBadTriangle) type = 0x3;
		Vector3F v(pt - cPt);
		float dist = v.Length();

		if (dist < fBestDist) {
			fBestDist = dist;
			uiIndex = i;
			uiType = type;
			closestPt = cPt;
		}
	}

	STriangleFEx & tri = tris[uiIndex];
	Vector3F closestFeatureNor;
	Vector3F triNormal = tri.Normal();
	float area = tri.Area();
	/* Face closest feature */
	if ((uiType & 0x3) == 0)		closestFeatureNor = tri.nor;
	/* Edge closest feature */
	else if ((uiType & 0x3) == 1)	closestFeatureNor = tri.edgeNor[uiType >> 2];
	/* Vertex closest feature */
	else							closestFeatureNor = tri.vertNor[uiType >> 2];

	Vector3F dir(pt - closestPt);
	return DotXYZ(dir, closestFeatureNor) > 0 ? fBestDist : -fBestDist;
}

void CLevelSet::CreateLevelSet(CMesh * pMesh, char * szLevelSetFile, bool bLoadFromFile, float cellSize)
{
	UINT nTris = pMesh->GetNumTriangles();
	STriangleF * tris = pMesh->GetTriangles();
	UINT * pAdj = pMesh->GetAdjancey();

	CreateLevelSet(tris, nTris, pAdj, szLevelSetFile, bLoadFromFile, cellSize);
}

void CLevelSet::CreateLevelSet(STriangleF * tris, UINT nTri, UINT * pAdj, char * szLevelSetFile, bool bLoadFromFile, float cellSize)
{
	UINT nVert = 3*nTri;
	Vector3F * verts = new Vector3F[nVert];
	for (UINT i=0; i<3*nTri; i++) {verts[i] = tris[i/3].vPosW[i%3]; }
	if (bLoadFromFile) {
		if (!LoadLevelSet(szLevelSetFile))
			DxUtSendError("CLevelSet::CreateLevelSet level set could not be loaded.");
	} else {
		m_CellSize = cellSize;
		ComputeSDF(verts, nVert, pAdj);
		WriteLevelSet(szLevelSetFile);
	}

	ComputeMeshParticles(nVert, nVert/3, verts, pAdj, szLevelSetFile);

	delete[] verts;
}

bool CLevelSet::LoadLevelSet(char * szFile)
{
	char file[MAX_PATH];
	ChangeExtension(szFile, "lvset", file);

	std::ifstream stream(file, std::ios::binary);
	if (!stream) return 0;
	//Assert(stream, "CLevelSet::_LoadLevelSet invalid lvset file specified."); 

	stream.read((char*)&m_nLSVertsX, sizeof(UINT));
	stream.read((char*)&m_nLSVertsY, sizeof(UINT));
	stream.read((char*)&m_nLSVertsZ, sizeof(UINT));
	m_nCellsX = m_nLSVertsX-1;
	m_nCellsY = m_nLSVertsY-1;
	m_nCellsZ = m_nLSVertsZ-1;

	stream.read((char*)&m_GridMin.x, sizeof(float));
	stream.read((char*)&m_GridMin.y, sizeof(float));
	stream.read((char*)&m_GridMin.z, sizeof(float));

	stream.read((char*)&m_GridMax.x, sizeof(float));
	stream.read((char*)&m_GridMax.y, sizeof(float));
	stream.read((char*)&m_GridMax.z, sizeof(float));

	stream.read((char*)&m_CellSize, sizeof(float));

	m_GridBox.SetMinP(m_GridMin);
	m_GridBox.SetMaxP(m_GridMax);
	m_TransformObjToGrid = m_GridMin;

	m_LSVertices = new SLevelSetVertex[m_nLSVertsX*m_nLSVertsY*m_nLSVertsZ];
	for (int k=0; k<m_nLSVertsZ; k++) {
		for (int j=0; j<m_nLSVertsY; j++) {
			for (int i=0; i<m_nLSVertsX; i++) {
				stream.read((char*)&m_LSVertices[k*m_nLSVertsY*m_nLSVertsX + j*m_nLSVertsX + i].dist, sizeof(float)*1);
			}
		}
	}
	stream.close();

	return 1;
}

void CLevelSet::WriteLevelSet(char * szFile)
{
	char file[MAX_PATH];
	ChangeExtension(szFile, "lvset", file);

	std::ofstream stream(file, std::ios::out | std::ios::binary); stream.close();
	stream.open(file, std::ios::out | std::ios::binary);

	stream.write((char*)&m_nLSVertsX, sizeof(UINT));
	stream.write((char*)&m_nLSVertsY, sizeof(UINT));
	stream.write((char*)&m_nLSVertsZ, sizeof(UINT));
	m_nCellsX = m_nLSVertsX-1;
	m_nCellsY = m_nLSVertsY-1;
	m_nCellsZ = m_nLSVertsZ-1;

	stream.write((char*)&m_GridMin.x, sizeof(float));
	stream.write((char*)&m_GridMin.y, sizeof(float));
	stream.write((char*)&m_GridMin.z, sizeof(float));

	stream.write((char*)&m_GridMax.x, sizeof(float));
	stream.write((char*)&m_GridMax.y, sizeof(float));
	stream.write((char*)&m_GridMax.z, sizeof(float));

	stream.write((char*)&m_CellSize, sizeof(float));

	for (int k=0; k<m_nLSVertsZ; k++) {
		for (int j=0; j<m_nLSVertsY; j++) {
			for (int i=0; i<m_nLSVertsX; i++) {
				float dist = m_LSVertices[k*m_nLSVertsY*m_nLSVertsX + j*m_nLSVertsX + i].dist;
				stream.write((char*)&dist, sizeof(float)*1);
			}
		}
	}
	stream.close();
}

//#define PRINT

UINT CLevelSet::LevelSetCollision(CLevelSet & collideLevelSet, CArray<SContactPoint> * CPs)
{
	if (!m_OBBox.OBBoxIntersectW(collideLevelSet.m_OBBox)) {
		return 0;
	}
	//if (!m_AABBox.AABBoxIntersectW(collideLevelSet.m_AABBox)) return 0;

#ifdef PRINT
	CTimer::Get().StartTimer(0);
#endif
	UINT oldSize = CPs->GetSize();

	/* Vertex containment */
	/* Body 1 */
	SContactPoint cp;
	Matrix4x4F T1 = collideLevelSet.m_TransformWorldToObj*m_TransformObjToWorld;
	for (UINT i=0, end=m_pVertexParticleIndex->GetSize(); i<end; i++) {
		SVertexParticle & vP = m_VertexParticles[(*m_pVertexParticleIndex)[i]];
		Vector3F wVP = m_TransformObjToWorld*vP.v;
		if (collideLevelSet.ParticleInLevelSet(wVP, vP.closestDist)) {
			vP.bIntersected = 1;
			vP.bAdmissible = 0;
			cp.dist = vP.closestDist;
			cp.iNor=collideLevelSet.ComputeGradient(wVP);
			if (cp.iNor.LengthSq() < .5) 
				continue;

			//cp.iNor = Vector3F(0,-1,0);

			if (IsNormalAdmissible(m_TransformNormalGridToWorld*vP.coneDir, vP.coneCosAngle, cp.iNor)) {
				vP.bAdmissible = 1;
				cp.iPos = wVP;
				CPs->PushBack(cp);
			} 
		}
	}
	
	/* Body 2 */
	Matrix4x4F T2 = m_TransformWorldToObj*collideLevelSet.m_TransformObjToWorld;
	for (UINT i=0, end=collideLevelSet.m_pVertexParticleIndex->GetSize(); i<end; i++) {
		SVertexParticle & vP = collideLevelSet.m_VertexParticles[(*collideLevelSet.m_pVertexParticleIndex)[i]];
		Vector3F wVP = collideLevelSet.m_TransformObjToWorld*vP.v;
		if (ParticleInLevelSet(wVP, vP.closestDist)) {
			cp.dist = vP.closestDist;
			vP.bIntersected = 1;
			vP.bAdmissible = 0;
			cp.iNor=ComputeGradient(wVP);
			if (cp.iNor.LengthSq() < .5)
				continue;

			//cp.iNor = Vector3F(0,1,0);

			if (IsNormalAdmissible(collideLevelSet.m_TransformNormalGridToWorld*vP.coneDir, vP.coneCosAngle, (cp.iNor))) {
				vP.bAdmissible = 1;

				cp.iPos = wVP;
				cp.iNor = -cp.iNor;
				CPs->PushBack(cp);
			} 
		}
	}
#ifdef PRINT
	CTimer::Get().EndTimer(0, "End intersection vertices");
#endif

#ifdef PRINT
	CTimer::Get().StartTimer(1);
#endif

	auto func = [&] (const SEdgeParticle & e, const SEdgeParticle * pEdge, Matrix4x4F & T, float norFlip, CLevelSet & lhs, CLevelSet & rhs) {
		for (int i=0; i<2; i++) {
			SEdgeParticle eTest1 = e;
			Vector3F pts[2];
			Vector3F e1(eTest1.adjTriangleVerts[i] - pEdge->pVP1->v);
			Vector3F e2(eTest1.adjTriangleVerts[i] - pEdge->pVP2->v);
			
			pts[0] = (.2f/DotXYZ(eTest1.adjOrthoEdgeVecs[i], e1))*e1 + pEdge->pVP1->v;
			pts[1] = (.2f/DotXYZ(eTest1.adjOrthoEdgeVecs[i], e2))*e2 + pEdge->pVP2->v;
			//pts[0] = .2f*e1 + pEdge->pVP1->v;
			//pts[1] = .2f*e2 + pEdge->pVP2->v;

			SVertexParticle vp1;
			vp1.v = pts[0];
			vp1.bIntersected = 0;
			vp1.bAdmissible = 0;
			vp1.closestDist = 0;
		
			SVertexParticle vp2;
			vp2.v = pts[1];
			vp2.bIntersected = 0;
			vp2.bAdmissible = 0;
			vp2.closestDist = 0;
		
			eTest1.pVP1 = &vp1;
			eTest1.pVP2 = &vp2;
			eTest1.coneCosAngle = .95;
			eTest1.coneDir = eTest1.adjFaceNormals[i];

			if (lhs.HandleEdgeEdgeCollision(eTest1, T, CPs, eTest1.pVP1->bIntersected + eTest1.pVP2->bIntersected, norFlip, rhs)) {
			}
		}
	};

	/* Edge-vertex containment */
	/* Compute the minimum penetration distance */
	for (UINT i=0, end=m_pEdgeIndex->GetSize(); i<end; i++) {
		if ((*m_pEdgeIndex)[i] > m_EdgeParticles.GetSize())
			DebugBreak();

		SEdgeParticle * pEdge = &m_EdgeParticles[(*m_pEdgeIndex)[i]];
		if (pEdge->pVP1->bIntersected && pEdge->pVP2->bIntersected) continue;

		SEdgeParticle e = *pEdge;
		if (!pEdge->pVP1->bIntersected) e.pVP1 = pEdge->pVP2, e.pVP2 = pEdge->pVP1;

		if (!HandleEdgeEdgeCollision(e, T1, CPs, pEdge->pVP1->bIntersected + pEdge->pVP2->bIntersected, 1.f, collideLevelSet)) {
			if (!e.bIntersected || e.bAdmissible) continue;
			func(e, pEdge, T1, 1.f, *this, collideLevelSet);
		} 
	}

	for (UINT i=0, end=collideLevelSet.m_pEdgeIndex->GetSize(); i<end; i++) {
		if ((*collideLevelSet.m_pEdgeIndex)[i] > collideLevelSet.m_EdgeParticles.GetSize())
			DebugBreak();

		SEdgeParticle * pEdge = &collideLevelSet.m_EdgeParticles[(*collideLevelSet.m_pEdgeIndex)[i]];
		if (pEdge->pVP1->bIntersected && pEdge->pVP2->bIntersected) continue;
		
		SEdgeParticle e = *pEdge;
		if (!pEdge->pVP1->bIntersected) e.pVP1 = pEdge->pVP2, e.pVP2 = pEdge->pVP1;

		if (!collideLevelSet.HandleEdgeEdgeCollision(e, T2, CPs, pEdge->pVP1->bIntersected + pEdge->pVP2->bIntersected, -1.f, *this)) {
			if (!e.bIntersected || e.bAdmissible) continue;
			func(e, pEdge, T2, -1.f, collideLevelSet, *this);
		} 
	}

#ifdef PRINT
	CTimer::Get().EndTimer(1, "End intersection edges");
#endif

	/*
	if (m_bDrawTris) CCollisionGraphics::SetupTriangleDraw(Vector3F(0, 1.f, 0));
	for (UINT i=0, end=m_FaceParticles.GetSize(); i<end; i++) {
		SFaceParticle & face = m_FaceParticles[i];

		HandleFace(collideLevelSet, STriangleF(m_TransformObjToWorld*face.boundaryVertices[0],
			m_TransformObjToWorld*face.boundaryVertices[1], m_TransformObjToWorld*face.boundaryVertices[2]), -1.f, CPs);
	}

	// Body 2 
	for (UINT i=0, end=collideLevelSet.m_FaceParticles.GetSize(); i<end; i++) {
		SFaceParticle & face = collideLevelSet.m_FaceParticles[i];
		
		collideLevelSet.HandleFace(*this, STriangleF(collideLevelSet.m_TransformObjToWorld*face.boundaryVertices[0],
			collideLevelSet.m_TransformObjToWorld*face.boundaryVertices[1], collideLevelSet.m_TransformObjToWorld*face.boundaryVertices[2]), 1.f, CPs);
	}*/


	/* Reset all the intersected vertices */ //optimize in future
	for (UINT i=0, end=m_pVertexParticleIndex->GetSize(); i<end; i++) {
		SVertexParticle & vP = m_VertexParticles[(*m_pVertexParticleIndex)[i]];
		vP.bIntersected = 0;
		vP.bAdmissible = 0;
	}
	for (UINT i=0, end=collideLevelSet.m_pVertexParticleIndex->GetSize(); i<end; i++) {
		SVertexParticle & vP = collideLevelSet.m_VertexParticles[(*collideLevelSet.m_pVertexParticleIndex)[i]];
		vP.bIntersected = 0;
		vP.bAdmissible = 0;
	}

	/* Reset all processed faces */
	bool * rgProcessedFace = m_IsFaceProcessed.GetData();
	for (UINT i=0, end=m_IsFaceProcessed.GetSize(); i<end; i++) {
		rgProcessedFace[i] = 0;
	}
	m_ToProcessFaceParticles.Resize(0);
	rgProcessedFace = collideLevelSet.m_IsFaceProcessed.GetData();
	for (UINT i=0, end=collideLevelSet.m_IsFaceProcessed.GetSize(); i<end; i++) {
		rgProcessedFace[i] = 0;
	}
	collideLevelSet.m_ToProcessFaceParticles.Resize(0);

	#ifdef PRINT
		CTimer::Get().PresentTimers(10, 1);
	#endif

	return CPs->GetSize() - oldSize;
}

bool CLevelSet::HandleEdgeEdgeCollision(SEdgeParticle & edge, Matrix4x4F & T, CArray<SContactPoint> * CPs, 
	UINT uiType, float norFlip, CLevelSet & collideLevelSet)
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
	if (edgeLen < m_CellSize) return 0;

	/* Intersect against the bounding box of the other object */ 
	Vector3F & edgeDirV1 = edgeDir;
	float closestDistV1 = 0;
	SRay ray;
	SRayIntersectData intersect0, intersect1;
	ray.p = collideLevelSet.m_TransformWorldToObj*v1 - collideLevelSet.m_OBBox.GetCenterL();
	ray.d = edgeDirV1.MulNormal(edgeDirV1, collideLevelSet.m_TransformWorldToObj);
	
	Vector3F v1Final(v1), v2Final(v2);
	intersect0.pos = v1;
	intersect1.pos = v2;
	/*if (!collideLevelSet.m_OBBox.OBBoxIntersectRelativeW(ray, 1, &intersect0, &intersect1))
		return 0;
	intersect0.pos = collideLevelSet.m_TransformObjToWorld*(intersect0.pos + collideLevelSet.m_OBBox.GetCenterL());
	intersect1.pos = collideLevelSet.m_TransformObjToWorld*(intersect1.pos + collideLevelSet.m_OBBox.GetCenterL());

	/* Find the first interior point starting from v1 
	float cellSize = m_CellSize;
	Vector3F v1Final(v1), v2Final(v2);
	Vector3F edgeD(v2-v1);
	Vector3F edgeNord(edgeD.Normalize());
	intersect0.pos += .004*edgeNord; 
	intersect1.pos -= .004*edgeNord; */
	//std::function<bool(Vector3F, Vector3F, float, float)> binSearch = 
	//	[&collideLevelSet, &v1Final, &v2Final, cellSize, &binSearch, &edgeD](Vector3F v1, Vector3F v2, float closestDist1, float closestDist2)->bool {
	
	/*auto binSearch = [&collideLevelSet, &v1Final, &v2Final, cellSize, &edgeD] (Vector3F v1, Vector3F v2) {
		while (1) {
			Vector3F dir(v2-v1);
			float dist = dir.Length();
			if (dist < cellSize) return 0;
			dir /= dist;

			float closestDist;
			Vector3F vAvg(.5f*(v1+v2));
			bool b = collideLevelSet.ParticleInLevelSet(vAvg, closestDist);
			closestDist = Abs(closestDist);
			if (b) {v1Final = v2Final = vAvg; return 1; }
			Vector3F vMid1(vAvg-(closestDist + cellSize)*dir);
			Vector3F vMid2(vAvg+(closestDist + cellSize)*dir);

			float closestDist1, closestDist2;
			bool b1 = collideLevelSet.ParticleInLevelSet(vMid1, closestDist1);
			bool b2 = collideLevelSet.ParticleInLevelSet(vMid2, closestDist2);
			float d = (vMid2-vMid1).Length();
			float close = closestDist1+closestDist2;
	
			if (b1) {v1Final = v2Final = vMid1; return 1; }
			else if (b2) {v1Final = v2Final = vMid2; return 1; }
			else if (d < (closestDist1+closestDist2)) return 0;
			else {
				if (closestDist1 < closestDist2) {
					v2 = vAvg;
				} else {
					v1 = vAvg;
				} //
			}
		}
		return 0;
	};
	if (!binSearch(intersect0.pos, intersect1.pos)) return 0;//*/

	/*
	auto newtonSearch = [&collideLevelSet, &v1Final, &v2Final, cellSize, &edgeD] (Vector3F v1, Vector3F v2) {
		while (1) {
			Vector3F dir(v2-v1);
			float dist = dir.Length();
			if (DotXYZ(dir, edgeD) < 0)
				return 0;
			if (dist < cellSize) return 0;
			dir /= dist;

			float closestDist;
			bool b = collideLevelSet.ParticleInLevelSet(v1, closestDist);
			if (b) {v1Final = v2Final = v1; return 1; }

			collideLevelSet.ParticleInLevelSet(v1, closestDist);
			Vector3F nor = collideLevelSet.ComputeGradient(v1);
			if (DotXYZ(nor, dir) > 0) return 0;

			float offset = closestDist/DotXYZ(nor, dir);
			offset = Abs(offset) + cellSize;

			v1 = v1 + offset*dir;
		}
		return 0;
	};
	if (!newtonSearch(intersect0.pos, intersect1.pos)) return 0;//*/
	

	/* Find the first interior point starting from v1 */
	Vector3F edgeDirV1Nor(edgeDirV1/edge.edgeLength);
	float t1 = (intersect0.pos - v1).Length();
	closestDistV1 = 0;
	if (!edge.pVP1->bIntersected)
		if (!MarchExteriorEdgeVertex(v1, edgeDirV1Nor, edge.edgeLength, t1, coneDir, edge.coneCosAngle, collideLevelSet, closestDistV1, v1Final, edge.bIntersected))
			return 0;
	
	// Find the first interior point starting from v2 
	Vector3F edgeDirV2(-edgeDir);
	Vector3F edgeDirV2Nor(edgeDirV2/edge.edgeLength);
	float closestDistV2 = 0;
	float t2 = (intersect1.pos - v2).Length();
	if (!edge.pVP2->bIntersected)
		if (!MarchExteriorEdgeVertex(v2, edgeDirV2Nor, edge.edgeLength, t2, coneDir, edge.coneCosAngle, collideLevelSet, closestDistV2, v2Final, edge.bIntersected))
			return 0;
			//*/

	//v2Final = v1Final;
	edge.bAdmissible = 1;
	
	/*SContactPoint cp;
	cp.iNor = Vector3F(0,1,0);

	cp.uiFaceIndex[0] = 2;

	cp.iPos = .5f*(v1+v2);
	collideLevelSet.ParticleInLevelSet(cp.iPos, cp.dist);
	CPs->PushBack(cp);

	if ((edgeDir+v1+v2+cp.iPos).Length()+cp.dist > 100000)
		exit(0);

	return 1;*/


	return ComputeEdgeEdgeIntersection(collideLevelSet, v1Final, v2Final, edgeDirV1, coneDir, edge.coneCosAngle, norFlip, CPs);
}

bool CLevelSet::ComputeEdgeEdgeIntersection(CLevelSet & collideLevelSet, Vector3F & v1, 
	Vector3F & v2, Vector3F & edgeDir, Vector3F & coneDir, float coneCosAngle, float norFlip, CArray<SContactPoint> * CPs)
{
	Vector3F avgPos = .5f*(v1 + v2); // TODO: don't do this. march from both sides instead

	/*
		Vector3F v2Final(v2);
	Vector3F edgeDirV2(-edgeDir);
	Vector3F edgeDirV2Nor(edgeDirV2/edge.edgeLength);
	float closestDistV2 = 0;
	float t2 = (intersect1.pos - v2).Length();
	if (!edge.pVP2->bIntersected)
		if (!MarchExteriorEdgeVertex(v2, edgeDirV2Nor, edge.edgeLength, t2, coneDir, edge.coneCosAngle, collideLevelSet, closestDistV2, v2Final, edge.bIntersected))
			*/

	Vector3F avgNor = collideLevelSet.ComputeGradient(avgPos);
	//if (!IsNormalAdmissible(coneDir, coneCosAngle, avgNor)) {
	//	return 0;
	//}
	
	Vector3F & n1 = collideLevelSet.ComputeGradient(v1);
	Vector3F & n2 = collideLevelSet.ComputeGradient(v2);

	Vector3F eLine(CrossXYZ(n1, n2).Normalize());
	/* If the avg nor does not lie in the span of n1, n2, then it is edge-face */
	if (abs(DotXYZ(eLine, avgNor)) > .001 || 1) {
		SContactPoint cp;
		cp.iNor = norFlip*avgNor;
		if (cp.iNor.LengthSq() < .5) return 0;

		cp.uiFaceIndex[0] = 1;

		cp.iPos = v1;
		collideLevelSet.ParticleInLevelSet(cp.iPos, cp.dist);
		CPs->PushBack(cp);
		cp.iPos = v2;
		collideLevelSet.ParticleInLevelSet(cp.iPos, cp.dist);
		CPs->PushBack(cp);

		return 1;
	} else {
		Vector3F iNor = norFlip*(CrossXYZ(edgeDir, eLine).Normalize());
		if (iNor.LengthSq() < .5) return 0;
		if (DotXYZ(iNor, norFlip*avgNor) < 0) 
			iNor = -iNor;

		SContactPoint cp;
		cp.iNor = iNor;

		/*cp.iPos = v1;
		CPs->PushBack(cp);

		cp.iPos = v2;
		CPs->PushBack(cp);*/

		cp.uiFaceIndex[0] = 2;

		cp.iPos = .5f*(v1+v2);
		collideLevelSet.ParticleInLevelSet(cp.iPos, cp.dist);
		CPs->PushBack(cp);

		return 1;
	}
	return 0;
}

bool CLevelSet::MarchExteriorEdgeVertex(Vector3F & vStart, Vector3F & edgeDir, float edgeLen, float t, 
	Vector3F & coneDir, float cosConeAngle, CLevelSet & collideLevelSet, float & closestDist, Vector3F & finalPt, bool & bIntersected)
{
	bool b1;
	Vector3F _finalPt;
	do {
		t += max(Abs(closestDist), collideLevelSet.m_CellSize);
		if (t > edgeLen) return 0;
		_finalPt = vStart + t * edgeDir;

		b1 = collideLevelSet.ParticleInLevelSet(_finalPt, closestDist);
		if (b1) 
			bIntersected = 1;
	} while (!b1 || 
		!IsNormalAdmissible(coneDir, cosConeAngle, collideLevelSet.ComputeGradient(_finalPt)));

	finalPt = _finalPt;
	return 1;
}

bool CLevelSet::MarchInteriorEdgeVertex(Vector3F & vStart, Vector3F & edgeDir, float edgeLen, float t, 
	Vector3F & coneDir, float cosConeAngle, CLevelSet & collideLevelSet, float & closestDist, Vector3F & finalPt, bool & bIntersected)
{
	bool b1;
	Vector3F _finalPt;
	Vector3F prePt;
	do {
		prePt = vStart + t * edgeDir;
		t += max(Abs(closestDist), .5f*collideLevelSet.m_CellSize);
		if (t > edgeLen) return 0;
		_finalPt = vStart + t * edgeDir;

		b1 = collideLevelSet.ParticleInLevelSet(_finalPt, closestDist);
		if (b1) 
			bIntersected = 1;
	} while (b1 && IsNormalAdmissible(coneDir, cosConeAngle, collideLevelSet.ComputeGradient(_finalPt)));

	finalPt = prePt;
	return 1;
}

//http://gilscvblog.wordpress.com/2013/08/23/bag-of-words-models-for-visual-categorization/
float CLevelSet::ComputeSignedDistance(Vector3F & dxyz, int cellX, int cellY, int cellZ)
{
	int CZ0 = cellZ*m_nLSVertsY*m_nLSVertsX;
	int CZ1 = (cellZ+1)*m_nLSVertsY*m_nLSVertsX;
	int CY0 = cellY*m_nLSVertsX;
	int CY1 = (cellY+1)*m_nLSVertsX;

	return (float)InterpolateTrilinear(dxyz.x, dxyz.y, dxyz.z,
			m_LSVertices[CZ0 + CY0 + cellX+0].dist,
			m_LSVertices[CZ0 + CY0 + cellX+1].dist,
			m_LSVertices[CZ0 + CY1 + cellX+0].dist,
			m_LSVertices[CZ0 + CY1 + cellX+1].dist,
		
			m_LSVertices[CZ1 + CY0 + cellX+0].dist,
			m_LSVertices[CZ1 + CY0 + cellX+1].dist,
			m_LSVertices[CZ1 + CY1 + cellX+0].dist,
			m_LSVertices[CZ1 + CY1 + cellX+1].dist);
}

bool CLevelSet::ParticleInLevelSet(Vector3F & particle, float & dist)
{
	Vector3F gridPt(m_TransformWorldToObj*particle), cpt;
	/* Check if the particle is in the grid */
	if (gridPt.x < m_GridMin.x || gridPt.y < m_GridMin.y || gridPt.z < m_GridMin.z || 
		gridPt.x > m_GridMax.x || gridPt.y > m_GridMax.y || gridPt.z > m_GridMax.z)  {
			dist = fastSqrt(m_GridBox.ClosestPointSq(gridPt, cpt));
			return 0; 
	}

	gridPt -= m_TransformObjToGrid;
	/* Check if the particle is in the polyhedron */
	int cellX = (int)floorf_ASM(gridPt.x/m_CellSize);
	int cellY = (int)floorf_ASM(gridPt.y/m_CellSize);
	int cellZ = (int)floorf_ASM(gridPt.z/m_CellSize);
	if (cellX < 0 || cellY < 0 || cellZ < 0) 
		DebugBreak();
	if (cellX >= m_nCellsX || cellY >= m_nCellsY || cellZ >= m_nCellsZ) 
		DebugBreak();

	float t = ComputeSignedDistance(
		Vector3F(fracf(gridPt.x/m_CellSize), fracf(gridPt.y/m_CellSize), fracf(gridPt.z/m_CellSize)),
		cellX, cellY, cellZ);

	dist = t;
	return t < 1e-3;//-m_ValidParticleDistance;
}

Vector3F CLevelSet::ComputeGradient(Vector3F & pt)
{
	Vector3F gridPt(TransformWorldToGrid(pt));
	int cellX = (int)floorf_ASM(gridPt.x/m_CellSize);
	int cellY = (int)floorf_ASM(gridPt.y/m_CellSize);
	int cellZ = (int)floorf_ASM(gridPt.z/m_CellSize);
	Vector3F dxyz(fracf(gridPt.x/m_CellSize), fracf(gridPt.y/m_CellSize), fracf(gridPt.z/m_CellSize));

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

void CLevelSet::HandleFace(CLevelSet & collideLevelSet, STriangleF & face, float norFlip, CArray<SContactPoint> * CPs)
{
	float closestDist;
	const float cosConeAngle = 1.f;
	Vector3F ctd(face.Centroid());
	collideLevelSet.ParticleInLevelSet(face.Centroid(), closestDist);
	float centroidRadius = face.CentroidRadius();
	
	if (centroidRadius < closestDist && closestDist > 0) {
		if (m_bDrawTris) CCollisionGraphics::DrawTriangle(face.vPosW[0], face.vPosW[1], face.vPosW[2], &Vector3F(1.f, 0, 0));
		return;
	}

	float triSize = face.MaxEdgeLength();
	if (triSize <= 2*m_CellSize) {
		SContactPoint cp;
		Vector3F nor = face.Normal();
		cp.iNor = norFlip*nor;
		int bFound = 0;

		cp.iPos = face.vPosW[0];
		if (collideLevelSet.ParticleInLevelSet(cp.iPos, cp.dist) && IsNormalAdmissible(collideLevelSet.ComputeGradient(cp.iPos), cosConeAngle, nor)) {
			CPs->PushBack(cp); bFound=1; }
		cp.iPos = face.vPosW[1];
		if (collideLevelSet.ParticleInLevelSet(cp.iPos, cp.dist) && IsNormalAdmissible(collideLevelSet.ComputeGradient(cp.iPos), cosConeAngle, nor)) {
			CPs->PushBack(cp); bFound=1; }
		cp.iPos = face.vPosW[2];
		if (collideLevelSet.ParticleInLevelSet(cp.iPos, cp.dist) && IsNormalAdmissible(collideLevelSet.ComputeGradient(cp.iPos), cosConeAngle, nor)) {
			this;
			CPs->PushBack(cp); bFound=1; }

		if (bFound) {
			if (m_bDrawTris) CCollisionGraphics::DrawTriangle(face.vPosW[0], face.vPosW[1], face.vPosW[2], &Vector3F(0, 1.f, 0)); 
		}
		else {
			if (m_bDrawTris) CCollisionGraphics::DrawTriangle(face.vPosW[0], face.vPosW[1], face.vPosW[2], &Vector3F(1.f, 0, 0)); 
		}
		return;
	}

	STriangleF subdividedTris[4];
	SubdivideTriangle(face, subdividedTris);

	HandleFace(collideLevelSet, subdividedTris[0], norFlip, CPs);
	HandleFace(collideLevelSet, subdividedTris[1], norFlip, CPs);
	HandleFace(collideLevelSet, subdividedTris[2], norFlip, CPs);
	HandleFace(collideLevelSet, subdividedTris[3], norFlip, CPs);
}

void CLevelSet::DrawLevelset()
{
	/*Matrix4x4F rot;
	m_OBBox.RotationW(rot);
	pInstance->DrawBox(m_OBBox.GetCenterW(), m_OBBox.GetHalfWidthsW(), rot);
	return;*/

	int count=0;
	for (int k=0; k<m_nLSVertsZ; k++) {
		for (int j=0; j<m_nLSVertsY; j++) {
			for (int i=0; i<m_nLSVertsX; i++) {
				float dist = m_LSVertices[k*m_nLSVertsY*m_nLSVertsX + j*m_nLSVertsX + i].dist;
				Vector3F pos(Vector3F((float)i * m_CellSize, (float)j * m_CellSize, (float)k * m_CellSize) + m_TransformObjToGrid);
			
				//int d =8;
				//if ((i > d && i < m_nLSVertsX - d) || (j > d && j < m_nLSVertsY - d) || (k > d && k < m_nLSVertsZ - d))
				//	continue;

				/*if (dist <= .04 && dist > -.6f*m_CellSize) {				
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
					CCollisionGraphics::DrawBox(rT, Vector3F(m_CellSize));
				}
			}
		}
	}
}

void CLevelSet::DrawAdmissibleZones()
{
	SVertexParticle * pParticles = m_VertexParticles.GetData();
	for (UINT i=0, end=m_VertexParticles.GetSize(); i<end; i++) {
		CCollisionGraphics::DrawNormal(m_TransformObjToWorld*pParticles[i].v, m_TransformNormalGridToWorld*pParticles[i].coneDir, .02, .2, Vector3F(1.f));
	}
}

void CLevelSet::DestroyLevelSet()
{
	if (m_LSVertices) {
		delete[] m_LSVertices;
		m_LSVertices = NULL;
		
		m_VertexParticles.Clear();
		//m_EdgeParticle.Clear();
		m_EdgeParticles.Clear();
		m_FaceParticles.Clear();
		m_ToProcessFaceParticles.Clear();
		m_IsFaceProcessed.Clear();

		m_DefaultVertexParticleIndices.Clear();
		m_DefaultEdgeIndices.Clear();
	}
}


};
