
#include "DxUtMesh.h"
#include "DxUtVertex.h"

namespace DxUt {

CMesh::CMesh():m_pMesh(0), m_nSubsets(0)
{
}

void CMesh::CreateMesh(DWORD nTri, DWORD nVert, DWORD dwOptions, 
	const D3D10_INPUT_ELEMENT_DESC * aDesc, DWORD cDesc, DWORD nSubsets)
{
	Assert(!m_pMesh, "CMesh::CreateMesh mesh must be destroyed before creating a new one.");
	
	if (FAILED(D3DX10CreateMesh(g_pD3DDevice, aDesc, cDesc, aDesc->SemanticName, nVert, nTri, dwOptions, &m_pMesh))) {
		DxUtSendError("CMesh::CreateMesh ID3DX10Mesh could not be created."); 
	}

	m_nSubsets = nSubsets;
	m_rgMat = new SMaterial[nSubsets];
	m_rgSRView = new ID3D10ShaderResourceView*[nSubsets];

	m_rgMat = new SMaterial[nSubsets];
	m_rgSRView = new ID3D10ShaderResourceView*[nSubsets];
		
	for (DWORD i=0; i<1; i++) {
		m_rgMat[i].amb = D3DXCOLOR(.4f, .4f, .4f, 1.f);
		m_rgMat[i].dif = D3DXCOLOR(.7f, .7f, .7f, 1.f);
		m_rgMat[i].spe = D3DXCOLOR(.4f, .4f, .4f, 1.f);
		m_rgMat[i].pow = 60.f;

		CHAR texFile[MAX_PATH];
		InsertDirectory(g_szFileDir, "White.dds", texFile);

		if (FAILED(D3DX10CreateShaderResourceViewFromFileA(g_pD3DDevice, texFile, 0, 0, &m_rgSRView[i], 0))) {
			DxUtSendErrorEx("CMesh::LoadMeshFromFile could not load mesh texture.", texFile);
		}
	}
	for (DWORD i=1; i<nSubsets; i++) {
		m_rgMat[i].amb = D3DXCOLOR(.4f, .4f, .4f, 1.f);
		m_rgMat[i].dif = D3DXCOLOR(.7f, .7f, .7f, 1.f);
		m_rgMat[i].spe = D3DXCOLOR(.4f, .4f, .4f, 1.f);
		m_rgMat[i].pow = 60.f;
	}
}

void CMesh::LoadMeshFromFile(char * szMeshFile, DWORD dwOptions, Vector3F & scale)
{
	Assert(!m_pMesh, "CMesh::CreateMesh mesh must be destroyed before creating a new one.");

	char file[MAX_PATH];
	InsertDirectoryEx(g_szFileDir, szMeshFile, file);  

	HANDLE hFile = CreateFileA(file, GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
	if (!hFile) DxUtSendErrorEx("CMesh::LoadMeshFromFile could not open mesh file.", file);

	DWORD nFaces, nVert, num=0;
	ReadFile(hFile, &nFaces, sizeof(DWORD), &num, 0);
	ReadFile(hFile, &nVert, sizeof(DWORD), &num, 0);
	D3D10_INPUT_ELEMENT_DESC const * aDesc = GetVertexElementDescPNT();
	m_dwStride = sizeof(SVertexPNT);

	if (FAILED(D3DX10CreateMesh(g_pD3DDevice, aDesc, 3, aDesc->SemanticName, nVert, nFaces, dwOptions, &m_pMesh))) {
		DxUtSendErrorEx("CMesh::LoadMeshFromFile could not be loaded from file.", file); 
	}

	//Set the vertex data
	SVertexPNT * vert = new SVertexPNT[nVert];
	ReadFile(hFile, vert, nVert*sizeof(SVertexPNT), &num, 0);
	for (DWORD i=0; i<nVert; i++) {
		vert[i].pos = scale*vert[i].pos;
		/*if (vert[i].pos == Vector3F(-0.37572321, 0.0066642496, 0.18663974)) {
			vert[i].pos += Vector3F(0, -.5f, 0);
		}
		if (vert[i].pos == Vector3F(-0.39442098, 0.0026622498, 0.0077272495)) {
			vert[i].pos += Vector3F(0, -.5f, 0);
		}
		if (vert[i].pos == Vector3F(-0.39615372, 0.0076462496, 0.088365242)) {
			vert[i].pos += Vector3F(0, -.5f, 0);
		}*/

	}
	m_pMesh->SetVertexData(0, vert);

	//Set the index data
	DWORD * indi = (DWORD*)vert;
	ReadFile(hFile, indi, 3*nFaces*sizeof(DWORD), &num, 0);
	m_pMesh->SetIndexData(indi, 3*nFaces);

	//Set the attribute data
	UINT * atri = (UINT*)vert;
	ReadFile(hFile, atri, nFaces*sizeof(UINT), &num, 0);
    m_pMesh->SetAttributeData(atri);

	delete[] vert;
	vert = 0;

	m_pMesh->GenerateAdjacencyAndPointReps(0.001f);
	m_pMesh->Optimize(D3DX10_MESHOPT_ATTR_SORT|D3DX10_MESHOPT_VERTEX_CACHE,0,0);
	m_pMesh->CommitToDevice();

	ReadFile(hFile, &m_nSubsets, sizeof(DWORD), &num, 0);
	m_rgMat = new SMaterial[m_nSubsets];
	m_rgSRView = new ID3D10ShaderResourceView*[m_nSubsets];

	for (DWORD i=0, len=0; i<m_nSubsets; i++) {
		ReadFile(hFile, &m_rgMat[i], sizeof(SMaterial), &num, 0);
		ReadFile(hFile, &len, sizeof(DWORD), &num, 0);
		if (len) {
			CHAR texFileName[64] = {0};					//It is assumed that the name of a file is small.
			ReadFile(hFile, &texFileName, len, &num, 0);

			CHAR texFile[MAX_PATH];
			InsertDirectory(g_szFileDir, texFileName, texFile);

			if (FAILED(D3DX10CreateShaderResourceViewFromFileA(g_pD3DDevice, texFile, 0, 0, &m_rgSRView[i], 0))) {
				DxUtSendErrorEx("CMesh::LoadMeshFromFile could not load mesh texture.", texFile);
			}

			TextureCreationHook(texFile, i);
		}
		else {
			CHAR texFile[MAX_PATH];
			InsertDirectory(g_szFileDir, "/White.dds", texFile);


			if (FAILED(D3DX10CreateShaderResourceViewFromFileA(g_pD3DDevice, texFile, 0, 0, &m_rgSRView[i], 0))) {
				DxUtSendErrorEx("CMesh::LoadMeshFromFile could not load mesh texture.", texFile);
			}
		}
	}
	if (!m_nSubsets) {
		m_rgMat = new SMaterial[1];
		m_rgMat[0].amb = D3DXCOLOR(.4f, .4f, .4f, 1.f);
		m_rgMat[0].dif = D3DXCOLOR(.7f, .7f, .7f, 1.f);
		m_rgMat[0].spe = D3DXCOLOR(.4f, .4f, .4f, 1.f);
		m_rgMat[0].pow = 60.f;
		m_rgSRView = new ID3D10ShaderResourceView*[1];

		CHAR texFile[MAX_PATH];
		InsertDirectory(g_szFileDir, "White.dds", texFile);

		if (FAILED(D3DX10CreateShaderResourceViewFromFileA(g_pD3DDevice, texFile, 0, 0, &m_rgSRView[0], 0))) {
			DxUtSendErrorEx("CMesh::LoadMeshFromFile could not load mesh texture.", texFile);
		}
		m_nSubsets++;
	}
}

void CMesh::DestroyMesh()
{
	Assert(m_pMesh != NULL, "CMesh::DestroyMesh cannot destroy a mesh that was never created.");

	ReleaseX(m_pMesh);
	
	if (m_rgMat) {
		delete[] m_rgMat;
		m_rgMat = NULL;

		for (DWORD i=0; i<m_nSubsets; i++) ReleaseX(m_rgSRView[i]);
		delete[] m_rgSRView;
		m_rgSRView = NULL;
	}

	m_nSubsets = 0;
}

void ExtractVerticesFromMesh(ID3DX10Mesh * pMesh, Vector3F * rgVert, DWORD dwStride)
{
	ID3DX10MeshBuffer * vBuf;
	pMesh->GetVertexBuffer(0, &vBuf);

	SIZE_T vBufSize = vBuf->GetSize();
	UINT nVert = pMesh->GetVertexCount();

	void * vertex = NULL;
	vBuf->Map(&vertex, &vBufSize);
	for (UINT i=0; i<nVert; i++) {
		rgVert[i] = *((Vector3F*)((CHAR*)(vertex) + i*dwStride));
	}
	vBuf->Unmap();

	ReleaseX(vBuf);
}

void ExtractVertexTriangleListFromMesh(ID3DX10Mesh * pMesh, Vector3F * rgVert, DWORD dwStride)
{
	ID3DX10MeshBuffer * vBuf;
	pMesh->GetVertexBuffer(0, &vBuf);

	ID3DX10MeshBuffer * iBuf;
	pMesh->GetIndexBuffer(&iBuf);

	SIZE_T vBufSize = vBuf->GetSize();
	SIZE_T iBufSize = iBuf->GetSize();
	UINT nVert = 3*pMesh->GetFaceCount(); 

	void * vertex = NULL;
	if (pMesh->GetFlags() &  D3DX10_MESH_32_BIT) {
		DWORD * index = NULL;
		vBuf->Map(&vertex, &vBufSize);
		iBuf->Map((void**)&index, &iBufSize);
		for (UINT i=0; i<nVert; i++) {
			rgVert[i] = *((Vector3F*)((CHAR*)vertex + dwStride*index[i]));
		}
	} else {
		WORD * index = NULL;
		vBuf->Map(&vertex, &vBufSize);
		iBuf->Map((void**)&index, &iBufSize);
		for (UINT i=0; i<nVert; i++) {
			rgVert[i] = *((Vector3F*)((CHAR*)vertex + dwStride*index[i]));
		}
	}
	iBuf->Unmap();
	vBuf->Unmap();

	ReleaseX(iBuf);
	ReleaseX(vBuf);
}

//pMesh must have adjancey information
void ExtractAdjanceyFromMesh(ID3DX10Mesh * pMesh, DWORD * rgAdj) 
{
	ID3DX10MeshBuffer * aBuf;
	if (FAILED(pMesh->GetAdjacencyBuffer(&aBuf))) {
		DxUtSendError("ExtractAdjanceyFromMesh must have a mesh with adjancey information.");
	}

	SIZE_T aBufSize = aBuf->GetSize();
	UINT nFaces = pMesh->GetFaceCount();
	
	if ((pMesh->GetFlags() & D3DX10_MESH_32_BIT)) {
		DWORD * pAdj = NULL;
		aBuf->Map((void**)&pAdj, &aBufSize);
		for (UINT i=0; i<3*nFaces; i++) rgAdj[i] = pAdj[i];
	} else {
		WORD * pAdj = NULL;
		aBuf->Map((void**)&pAdj, &aBufSize);
		for (UINT i=0; i<3*nFaces; i++) rgAdj[i] = pAdj[i];
	}
	aBuf->Unmap();

	ReleaseX(aBuf);

}


};











/*void CMesh::SaveMeshToFile(char * szMeshFile, SUPPORTEDMESHTYPES smt, FLOAT adjEps)
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