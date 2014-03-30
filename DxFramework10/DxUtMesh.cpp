
#include "DxUtMesh.h"
#include "DxUtVertex.h"
#include "DxUtMeshPool.h"

namespace DxUt {

CMesh::CMesh():m_pMesh(0), m_nSubsets(0)
{
}
/*
void CMesh::CreateMesh(DWORD nTri, DWORD nVert, DWORD dwOptions, 
	const D3D10_INPUT_ELEMENT_DESC * aDesc, DWORD cDesc, DWORD nSubsets, char * szName)
{
	Assert(!m_pMesh, "CMesh::CreateMesh mesh must be destroyed before creating a new one.");

	m_Name = szName;
	if (CMeshPool::GetResource(szName, (ID3DX10MeshEx**)&m_pMesh)) {
		ReleaseX(m_pMesh);
	}

	if (FAILED(D3DX10CreateMesh(g_pD3DDevice, aDesc, cDesc, aDesc->SemanticName, nVert, nTri, dwOptions, &m_pMesh))) {
		DxUtSendError("CMesh::CreateMesh ID3DX10Mesh could not be created."); 
	}

	m_nSubsets = nSubsets;
	m_Materials = new SMaterial[nSubsets];
	m_SRViews = new ID3D10ShaderResourceView*[nSubsets];

	m_Materials = new SMaterial[nSubsets];
	m_SRViews = new ID3D10ShaderResourceView*[nSubsets];
		
	for (DWORD i=0; i<1; i++) {
		m_Materials[i].amb = D3DXCOLOR(.4f, .4f, .4f, 1.f);
		m_Materials[i].dif = D3DXCOLOR(.7f, .7f, .7f, 1.f);
		m_Materials[i].spe = D3DXCOLOR(.4f, .4f, .4f, 1.f);
		m_Materials[i].pow = 60.f;

		if (FAILED(D3DX10CreateShaderResourceViewFromFileA(g_pD3DDevice, "White.dds", 0, 0, &m_SRViews[i], 0))) {
			DxUtSendErrorEx("CMesh::LoadMeshFromFile could not load mesh texture.", "White.dds");
		}
	}
	for (DWORD i=1; i<nSubsets; i++) {
		m_Materials[i].amb = D3DXCOLOR(.4f, .4f, .4f, 1.f);
		m_Materials[i].dif = D3DXCOLOR(.7f, .7f, .7f, 1.f);
		m_Materials[i].spe = D3DXCOLOR(.4f, .4f, .4f, 1.f);
		m_Materials[i].pow = 60.f;
	}

	CMeshPool::PutResource(szName, (ID3DX10MeshEx*)m_pMesh);
}*/

void CMesh::LoadMeshFromFile(char * szMeshFile, DWORD dwOptions, Vector3F & scale)
{
	Assert(!m_pMesh, "CMesh::CreateMesh mesh must be destroyed before creating a new one.");

	HANDLE hFile = CreateFileA(szMeshFile, GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
	if (!hFile) DxUtSendErrorEx("CMesh::LoadMeshFromFile could not open mesh file.", szMeshFile);
	DWORD nFaces, nVert, num=0;
	ReadFile(hFile, &nFaces, sizeof(DWORD), &num, 0);
	ReadFile(hFile, &nVert, sizeof(DWORD), &num, 0);
	D3D10_INPUT_ELEMENT_DESC const * aDesc = GetVertexElementDescPNT();
	m_dwStride = sizeof(SVertexPNT);


	m_Name = szMeshFile;
	if (!CMeshPool::GetResource((char*)szMeshFile, this)) {
		if (FAILED(D3DX10CreateMesh(g_pD3DDevice, aDesc, 3, aDesc->SemanticName, nVert, nFaces, dwOptions, &m_pMesh))) {
			DxUtSendErrorEx("CMesh::LoadMeshFromFile could not be loaded from file.", szMeshFile); 
		}

		//Set the vertex data
		m_nVertices = nVert;
		m_Vertices = new Vector3F[nVert];
		SVertexPNT * vert = new SVertexPNT[nVert];
		ReadFile(hFile, vert, nVert*sizeof(SVertexPNT), &num, 0);
		for (DWORD i=0; i<nVert; i++) {
			vert[i].pos = scale*vert[i].pos;
			m_Vertices[i] = scale*vert[i].pos;
		}
		m_pMesh->SetVertexData(0, vert);

		//Set the index data
		DWORD * indi = (DWORD*)vert;
		ReadFile(hFile, indi, 3*nFaces*sizeof(DWORD), &num, 0);
		m_pMesh->SetIndexData(indi, 3*nFaces);

		// Extract the triangles
		m_nFaces = nFaces;
		m_Tris = new STriangleF[nFaces];
		for (int i=0; i<3*nFaces; i++) {
			m_Tris[i].vPosW[0] = m_Vertices[indi[3*i+0]];
			m_Tris[i].vPosW[1] = m_Vertices[indi[3*i+2]];
			m_Tris[i].vPosW[2] = m_Vertices[indi[3*i+2]];
		}

		//Set the attribute data
		UINT * atri = (UINT*)vert;
		ReadFile(hFile, atri, nFaces*sizeof(UINT), &num, 0);
		m_pMesh->SetAttributeData(atri);

		// Extract the adj info
		m_Adj = new DWORD[3*nFaces];
		ReadFile(hFile, m_Adj, 3*nFaces*sizeof(DWORD), &num, 0);

		delete[] vert;
		vert = 0;

		m_pMesh->GenerateAdjacencyAndPointReps(0.001f);
		m_pMesh->Optimize(D3DX10_MESHOPT_ATTR_SORT|D3DX10_MESHOPT_VERTEX_CACHE,0,0);
		m_pMesh->CommitToDevice();

		CMeshPool::PutResource(szMeshFile, this);
	} else {
		SetFilePointer(hFile, nVert*sizeof(SVertexPNT)+3*nFaces*sizeof(DWORD)+nFaces*sizeof(UINT)+3*nFaces*sizeof(DWORD), NULL, FILE_CURRENT);
	}

	ReadFile(hFile, &m_nSubsets, sizeof(DWORD), &num, 0);
	m_Materials = new SMaterial[m_nSubsets];
	m_SRViews = new ID3D10ShaderResourceView*[m_nSubsets];

	for (DWORD i=0, len=0; i<m_nSubsets; i++) {
		ReadFile(hFile, &m_Materials[i], sizeof(SMaterial), &num, 0);
		ReadFile(hFile, &len, sizeof(DWORD), &num, 0);
		if (len) {
			CHAR texFileName[128] = {0};					//It is assumed that the name of a file is small.
			ReadFile(hFile, &texFileName, len, &num, 0);
			
			if (FAILED(D3DX10CreateShaderResourceViewFromFileA(g_pD3DDevice, texFileName, 0, 0, &m_SRViews[i], 0))) {
				DxUtSendErrorEx("CMesh::LoadMeshFromFile could not load mesh texture.", texFileName);
			}

			TextureCreationHook(texFileName, i);
		}
		else {
			if (FAILED(D3DX10CreateShaderResourceViewFromFileA(g_pD3DDevice,  "White.dds", 0, 0, &m_SRViews[i], 0))) {
				DxUtSendErrorEx("CMesh::LoadMeshFromFile could not load mesh texture.",  "White.dds");
			}
		}
	}
	if (!m_nSubsets) {
		m_Materials = new SMaterial[1];
		m_Materials[0].amb = D3DXCOLOR(.4f, .4f, .4f, 1.f);
		m_Materials[0].dif = D3DXCOLOR(.7f, .7f, .7f, 1.f);
		m_Materials[0].spe = D3DXCOLOR(.4f, .4f, .4f, 1.f);
		m_Materials[0].pow = 60.f;
		m_SRViews = new ID3D10ShaderResourceView*[1];

		if (FAILED(D3DX10CreateShaderResourceViewFromFileA(g_pD3DDevice, "White.dds", 0, 0, &m_SRViews[0], 0))) {
			DxUtSendErrorEx("CMesh::LoadMeshFromFile could not load mesh texture.", "White.dds");
		}
		m_nSubsets++;
	}
}

void CMesh::ShallowCopy(CMesh * pMesh)
{
	this->m_Name = pMesh->m_Name;
	this->m_pMesh = pMesh->m_pMesh;
	this->m_dwStride = pMesh->m_dwStride;
	this->m_nSubsets = pMesh->m_nSubsets;
	this->m_nVertices = pMesh->m_nVertices;
	this->m_nFaces = pMesh->m_nFaces;
	this->m_Materials = pMesh->m_Materials;
	this->m_SRViews = pMesh->m_SRViews;
	this->m_Vertices = pMesh->m_Vertices;
	this->m_Tris = pMesh->m_Tris;
	this->m_Adj = pMesh->m_Adj;
}

void CMesh::Destroy()
{
	Assert(m_pMesh != NULL, "CMesh::DestroyMesh cannot destroy a mesh that was never created.");

	if (m_Materials) {
		delete[] m_Materials;
		m_Materials = NULL;

		for (DWORD i=0; i<m_nSubsets; i++) ReleaseX(m_SRViews[i]);
		delete[] m_SRViews;
		m_SRViews = NULL;
	}

	m_nSubsets = 0;
}


};











/*void CMesh::SaveMeshToFile(char * szMeshFile, SUPPORTEDMESHTYPES smt, float adjEps)
{
	char file[MAX_PATH];
	InsertDirectory(g_szFileDir, szMeshFile, file);  
	
	WORD len = strlen(file);
	file[len-1] = 't';
	file[len+0] = 'x';
	file[len+1] = 't';
	file[len+2] = '\0';
	HANDLE hFile = CreateFileA(file, GENERIC_WRITE, FILE_SHARE_READ, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
	if (!hFile) {
		DxUtSendErrorEx("SaveMeshToFile could not create file.", file);
	}

	//Faces and vertices
	DWORD num=0;
	DWORD nFaces = m_pMesh->GetNumFaces();
	WriteFile(hFile, &nFaces, sizeof(DWORD), &num, 0);
	DWORD nVert = m_pMesh->GetNumVertices();
	WriteFile(hFile, &nVert, sizeof(DWORD), &num, 0);

	//Vertex buffer
	LPDIRECT3DVERTEXBUFFER9 vB;
	m_pMesh->GetVertexBuffer(&vB);

	SVertexPNT * vert;
	vB->Lock(0, 0, (void**)&vert, 0);
	WriteFile(hFile, (void*)vert, nVert*sizeof(SVertexPNT), &num, 0);
	vB->Unlock();

	//Index buffer
	LPDIRECT3DINDEXBUFFER9 iB;
	m_pMesh->GetIndexBuffer(&iB);

	DWORD * index;
	iB->Lock(0, 0, (void**)&index, 0);
	WriteFile(hFile, (void*)index, 3*nFaces*sizeof(DWORD), &num, 0);
	iB->Unlock();
	
	//Attributes
	DWORD * atri;
	m_pMesh->LockAttributeBuffer(0, (DWORD**)&atri);
	WriteFile(hFile, atri, nFaces*sizeof(DWORD), &num, 0);
	m_pMesh->UnlockAttributeBuffer();

	//Number of materials
	WriteFile(hFile, &numMaterial, sizeof(DWORD), &num, 0);

	D3DXMATERIAL * d3dMat = (D3DXMATERIAL*)materialBuf->GetBufferPointer();
	for (DWORD i=0; i<numMaterial; i++) {
		Material mat;
		mat.amb = d3dMat[i].MatD3D.Ambient;
		mat.dif = d3dMat[i].MatD3D.Diffuse;
		mat.spe = d3dMat[i].MatD3D.Specular;
		mat.pow = d3dMat[i].MatD3D.Power;
		WriteFile(hFile, &mat, sizeof(Material), &num, 0);

		if (d3dMat[i].pTextureFilename) {
			DWORD len = strlen(d3dMat[i].pTextureFilename)+1;
			WriteFile(hFile, &len, sizeof(DWORD), &num, 0);  

			CHAR * texFile = new CHAR[len+1];
			memcpy(texFile+1, d3dMat[i].pTextureFilename, len-1);
			texFile[0] = '/';
			texFile[len] = '\0';
			WriteFile(hFile, texFile, len, &num, 0);  

			delete[] texFile;
		}
		else {
			DWORD d=0;
			WriteFile(hFile, &d, sizeof(DWORD), &num, 0);  
		}
	}

	adj->Release();
	materialBuf->Release();
	m_pMesh->Release();

	CloseHandle(hFile);

	/*hFile = CreateFileA(szMeshFile, GENERIC_READ, 0, NULL, OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
	if (!hFile) return false;

	nFaces, nVert, num=0;
	DWORD a, b;
	ReadFile(hFile, &a, sizeof(DWORD), &num, 0);
	ReadFile(hFile, &b, sizeof(DWORD), &num, 0);

	return true;
	}
*/