
#include "DxUtMesh.h"
#include "DxUtVertex.h"
#include "DxUtMeshPool.h"
#include <fstream>
#include <sstream> 
#include <string> 

namespace DxUt {

void ExtractAdjanceyFromMesh(ID3DX10Mesh * pMesh, UINT * pAdjOut) 
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
		for (UINT i=0; i<3*nFaces; i++) pAdjOut[i] = pAdj[i];
	} else {
		WORD * pAdj = NULL;
		aBuf->Map((void**)&pAdj, &aBufSize);
		for (UINT i=0; i<3*nFaces; i++) pAdjOut[i] = pAdj[i];
	}
	aBuf->Unmap();

	ReleaseX(aBuf);

}

CMesh::CMesh():m_pMesh(0), m_nSubsets(0)
{
}
/*
void CMesh::CreateMesh(UINT nTri, UINT nVert, UINT uiOptions, 
	const D3D10_INPUT_ELEMENT_DESC * aDesc, UINT cDesc, UINT nSubsets, char * szName)
{
	Assert(!m_pMesh, "CMesh::CreateMesh mesh must be destroyed before creating a new one.");

	m_Name = szName;
	if (CMeshPool::GetResource(szName, (ID3DX10MeshEx**)&m_pMesh)) {
		ReleaseX(m_pMesh);
	}

	if (FAILED(D3DX10CreateMesh(g_pD3DDevice, aDesc, cDesc, aDesc->SemanticName, nVert, nTri, uiOptions, &m_pMesh))) {
		DxUtSendError("CMesh::CreateMesh ID3DX10Mesh could not be created."); 
	}

	m_nSubsets = nSubsets;
	m_Materials = new SMaterial[nSubsets];
	m_SRViews = new ID3D10ShaderResourceView*[nSubsets];

	m_Materials = new SMaterial[nSubsets];
	m_SRViews = new ID3D10ShaderResourceView*[nSubsets];
		
	for (UINT i=0; i<1; i++) {
		m_Materials[i].amb = D3DXCOLOR(.4f, .4f, .4f, 1.f);
		m_Materials[i].dif = D3DXCOLOR(.7f, .7f, .7f, 1.f);
		m_Materials[i].spe = D3DXCOLOR(.4f, .4f, .4f, 1.f);
		m_Materials[i].pow = 60.f;

		if (FAILED(D3DX10CreateShaderResourceViewFromFileA(g_pD3DDevice, "White.dds", 0, 0, &m_SRViews[i], 0))) {
			DxUtSendErrorEx("CMesh::LoadMeshFromFile could not load mesh texture.", "White.dds");
		}
	}
	for (UINT i=1; i<nSubsets; i++) {
		m_Materials[i].amb = D3DXCOLOR(.4f, .4f, .4f, 1.f);
		m_Materials[i].dif = D3DXCOLOR(.7f, .7f, .7f, 1.f);
		m_Materials[i].spe = D3DXCOLOR(.4f, .4f, .4f, 1.f);
		m_Materials[i].pow = 60.f;
	}

	CMeshPool::PutResource(szName, (ID3DX10MeshEx*)m_pMesh);
}*/

void CMesh::LoadMeshFromFile(char * szMeshFile, UINT uiOptions, const Vector3F & scale)
{
	char * ptr = strstr(szMeshFile, ".txt");
	if (ptr != NULL) {LoadMeshFromFileTXT(szMeshFile, uiOptions, scale); return; }
	ptr = strstr(szMeshFile, ".off");
	if (ptr != NULL) {LoadMeshFromFileOFF(szMeshFile, uiOptions, scale); return; }
	ptr = strstr(szMeshFile, ".obj");
	if (ptr != NULL) {LoadMeshFromFileObj(szMeshFile, uiOptions, scale); return; }
	else {
		DxUtSendError("CMesh::LoadMeshFromFile must specify a valid extension (.txt, .off, .obj)");
	}
}

void CMesh::LoadMeshFromFileOFF(char * szMeshFile, UINT uiOptions, const Vector3F & scale)
{
	Assert(!m_pMesh, "CMesh::LoadMeshFromFile mesh must be destroyed before creating a new one.");

	std::fstream stream(szMeshFile);
	if (!stream) DxUtSendErrorEx("CMesh::LoadMeshFromFile could not open mesh file.", szMeshFile);

	std::string dummy;
	UINT nFaces, nVert, nEdges;
	DWORD dwNum=0;
	stream >> dummy;
	stream >> nVert;
	stream >> nFaces;
	stream >> nEdges;
	D3D10_INPUT_ELEMENT_DESC const * aDesc = GetVertexElementDescPNT();
	m_uiStride = sizeof(SVertexPNT);

	m_Name = szMeshFile;
	if (!CMeshPool::GetResource((char*)szMeshFile, this)) {
		if (FAILED(D3DX10CreateMesh(g_pD3DDevice, aDesc, 3, aDesc->SemanticName, nVert, nFaces, uiOptions, &m_pMesh))) {
			DxUtSendErrorEx("CMesh::LoadMeshFromFile could not be loaded from file.", szMeshFile); 
		}

		//Set the vertex data positions
		m_nVertices = nVert;
		m_Vertices = new Vector3F[nVert];
		SVertexPNT * vert = new SVertexPNT[nVert];
		for (int i=0; i<nVert; i++) {
			stream >> vert[i].pos.x;  
			stream >> vert[i].pos.z;  
			stream >> vert[i].pos.y;  
			vert[i].pos.z = -vert[i].pos.z; 
			
			vert[i].pos = scale*vert[i].pos;
			m_Vertices[i] = vert[i].pos;
		}
		
		//Set the index data
		UINT * indi = new UINT[3*nFaces];
		for (int i=0; i<nFaces; i++) {
			DWORD n;
			stream >> n;
			Assert(n==3, "CMesh::LoadMeshFromFileOFF cannot load off meshes with non-triangular faces.");
			stream >> indi[3*i+0];
			stream >> indi[3*i+1];
			stream >> indi[3*i+2];
		}
		m_pMesh->SetIndexData(indi, 3*nFaces);

		// Extract the triangles
		m_nFaces = nFaces;
		m_Tris = new STriangleF[nFaces];
		for (UINT i=0; i<nFaces; i++) {
			m_Tris[i].vPosW[0] = m_Vertices[indi[3*i+0]];
			m_Tris[i].vPosW[1] = m_Vertices[indi[3*i+1]];
			m_Tris[i].vPosW[2] = m_Vertices[indi[3*i+2]];
			
			Vector3F nor(m_Tris[i].Normal());
			vert[indi[3*i+0]].nor = nor;
			vert[indi[3*i+1]].nor = nor;
			vert[indi[3*i+2]].nor = nor;
		}
		m_pMesh->SetVertexData(0, vert);

		//Set the attribute data
		UINT * atri = (UINT*)vert;
		memset(atri, 0, nFaces*sizeof(UINT));
		m_pMesh->SetAttributeData(atri);

		delete[] vert;
		delete[] indi;
		vert = 0;

		// Compute adjancey
		m_pMesh->GenerateAdjacencyAndPointReps(.001);
		m_Adj = new UINT[3*nFaces];
		ExtractAdjanceyFromMesh(m_pMesh, m_Adj);
		
		m_pMesh->Optimize(D3DX10_MESHOPT_ATTR_SORT|D3DX10_MESHOPT_VERTEX_CACHE,0,0);
		m_pMesh->CommitToDevice();

		CMeshPool::PutResource(szMeshFile, this);
	} 

	m_Materials = new SMaterial[1];
	m_Materials[0].amb = D3DXCOLOR(.4f, .4f, .4f, 1.f);
	m_Materials[0].dif = D3DXCOLOR(.7f, .7f, .7f, 1.f);
	m_Materials[0].spe = D3DXCOLOR(.4f, .4f, .4f, 1.f);
	m_Materials[0].pow = 60.f;
	m_SRViews = new ID3D10ShaderResourceView*[1];

	if (FAILED(D3DX10CreateShaderResourceViewFromFileA(g_pD3DDevice, "White.dds", 0, 0, &m_SRViews[0], 0))) {
		DxUtSendErrorEx("CMesh::LoadMeshFromFile could not load mesh texture.", "White.dds");
	}
	m_nSubsets=1;
}

void CMesh::LoadMeshFromFileObj(char * szMeshFile, UINT uiOptions, const Vector3F & scale)
{
	Assert(!m_pMesh, "CMesh::LoadMeshFromFile mesh must be destroyed before creating a new one.");

	UINT nFaces, nVert, nEdges;
	D3D10_INPUT_ELEMENT_DESC const * aDesc = GetVertexElementDescPNT();
	m_uiStride = sizeof(SVertexPNT);

	auto GetNextNumF = [] (char * str, int & index) {
		while (str[index] != ' ') index++;
		index++;
		return atof(str+index);
	};
	auto GetNextNumI = [] (char * str, int & index) {
		while (str[index] != ' ') index++;
		index++;
		return atoi(str+index);
	};

	m_Name = szMeshFile;
	if (!CMeshPool::GetResource((char*)szMeshFile, this)) {
	
		CArray<Vector3F, 1, 1> objVerts; 
		CArray<int[3], 1, 1> objIndices;
		std::ifstream istream(szMeshFile);
		istream.seekg (0, istream.end);
		int fileLen = istream.tellg();
		istream.seekg (0, istream.beg);
		char * data = new char[fileLen+1];
		istream.read(data, fileLen);
		int fileIndex = 0;	
        while(fileIndex < fileLen) {
			char * line = data+fileIndex;
			if (line[0] == '#' || line[0] == '\n') {}
			else if (line[0] == 'v' && line[1] == ' ') {
				Vector3F v;
				v.x = GetNextNumF(data, fileIndex);
				v.y = GetNextNumF(data, fileIndex);
				v.z = GetNextNumF(data, fileIndex);
				objVerts.PushBack(v);
			}
            else if(line[0] == 'f'){
				int indices[3];
				indices[0] = GetNextNumI(data, fileIndex);
				indices[1] = GetNextNumI(data, fileIndex);
				indices[2] = GetNextNumI(data, fileIndex);
				objIndices.PushBack(indices);
            }

			while (fileIndex < fileLen && data[fileIndex] != '\n') fileIndex++;
			fileIndex++;
        }
		delete[] data;

		nVert = objVerts.GetSize();
		nFaces = objIndices.GetSize();

		// Get index data
		UINT * indi = new UINT[3*nFaces];
		for (int i=0; i<nFaces; i++) {
			indi[3*i+0] = objIndices[i][0]-1;
			indi[3*i+1] = objIndices[i][1]-1;
			indi[3*i+2] = objIndices[i][2]-1;
		}

		// Get vertices and triangles
		nVert = 3*nFaces;
		m_nVertices = nVert;
		m_nFaces = nFaces;
		m_Vertices = new Vector3F[nVert];
		SVertexPNT * vert = new SVertexPNT[nVert];
		m_Tris = new STriangleF[nFaces];
		for (int i=0; i<nFaces; i++) {
			vert[3*i+0].pos = scale*objVerts[indi[3*i+0]];
			vert[3*i+1].pos = scale*objVerts[indi[3*i+1]];
			vert[3*i+2].pos = scale*objVerts[indi[3*i+2]];

			m_Vertices[3*i+0] = vert[3*i+0].pos;
			m_Vertices[3*i+1] = vert[3*i+1].pos;
			m_Vertices[3*i+2] = vert[3*i+2].pos;

			m_Tris[i].vPosW[0] = m_Vertices[3*i+0];
			m_Tris[i].vPosW[1] = m_Vertices[3*i+1];
			m_Tris[i].vPosW[2] = m_Vertices[3*i+2];
			
			Vector3F nor(m_Tris[i].Normal());
			vert[3*i+0].nor = nor;
			vert[3*i+1].nor = nor;
			vert[3*i+2].nor = nor;

			indi[3*i+0] = 3*i+0;
			indi[3*i+1] = 3*i+1;
			indi[3*i+2] = 3*i+2;
		}


		/*//Set the vertex data positions
		m_nVertices = nVert;
		m_Vertices = new Vector3F[nVert];
		SVertexPNT * vert = new SVertexPNT[nVert];
		for (int i=0; i<nVert; i++) {
			vert[i].pos = scale*objVerts[i];
			m_Vertices[i] = vert[i].pos;
		}

		//Set the index data
		UINT * indi = new UINT[3*nFaces];
		for (int i=0; i<nFaces; i++) {
			indi[3*i+0] = objIndices[i][0]-1;
			indi[3*i+1] = objIndices[i][1]-1;
			indi[3*i+2] = objIndices[i][2]-1;
		}

		// Extract the triangles
		m_nFaces = nFaces;
		m_Tris = new STriangleF[nFaces];
		for (UINT i=0; i<nFaces; i++) {
			m_Tris[i].vPosW[0] = m_Vertices[indi[3*i+0]];
			m_Tris[i].vPosW[1] = m_Vertices[indi[3*i+1]];
			m_Tris[i].vPosW[2] = m_Vertices[indi[3*i+2]];
			
			Vector3F nor(m_Tris[i].Normal());
			vert[indi[3*i+0]].nor = nor;
			vert[indi[3*i+1]].nor = nor;
			vert[indi[3*i+2]].nor = nor;
		}*/

		if (FAILED(D3DX10CreateMesh(g_pD3DDevice, aDesc, 3, aDesc->SemanticName, nVert, nFaces, uiOptions, &m_pMesh))) {
			DxUtSendErrorEx("CMesh::LoadMeshFromFile could not be loaded from file.", szMeshFile); 
		}

		m_pMesh->SetIndexData(indi, 3*nFaces);
		m_pMesh->SetVertexData(0, vert);

		//Set the attribute data
		UINT * atri = (UINT*)vert;
		memset(atri, 0, nFaces*sizeof(UINT));
		m_pMesh->SetAttributeData(atri);

		delete[] vert;
		delete[] indi;
		vert = 0;

		// Compute adjancey
		m_pMesh->GenerateAdjacencyAndPointReps(.001);
		m_Adj = new UINT[3*nFaces];
		ExtractAdjanceyFromMesh(m_pMesh, m_Adj);
		
		m_pMesh->Optimize(D3DX10_MESHOPT_ATTR_SORT|D3DX10_MESHOPT_VERTEX_CACHE,0,0);
		m_pMesh->CommitToDevice();

		CMeshPool::PutResource(szMeshFile, this);
	} 

	m_Materials = new SMaterial[1];
	m_Materials[0].amb = D3DXCOLOR(.4f, .4f, .4f, 1.f);
	m_Materials[0].dif = D3DXCOLOR(.7f, .7f, .7f, 1.f);
	m_Materials[0].spe = D3DXCOLOR(.4f, .4f, .4f, 1.f);
	m_Materials[0].pow = 60.f;
	m_SRViews = new ID3D10ShaderResourceView*[1];

	if (FAILED(D3DX10CreateShaderResourceViewFromFileA(g_pD3DDevice, "White.dds", 0, 0, &m_SRViews[0], 0))) {
		DxUtSendErrorEx("CMesh::LoadMeshFromFile could not load mesh texture.", "White.dds");
	}
	m_nSubsets=1;
}

void CMesh::LoadMeshFromFileTXT(char * szMeshFile, UINT uiOptions, const Vector3F & scale)
{
	Assert(!m_pMesh, "CMesh::LoadMeshFromFile mesh must be destroyed before creating a new one.");

	std::ifstream stream(szMeshFile, std::ifstream::binary);
	if (!stream) DxUtSendErrorEx("CMesh::LoadMeshFromFile could not open mesh file.", szMeshFile);

	UINT nFaces, nVert;
	DWORD dwNum=0;
	stream.read((char*)&nFaces, sizeof(UINT));
	stream.read((char*)&nVert, sizeof(UINT));
	D3D10_INPUT_ELEMENT_DESC const * aDesc = GetVertexElementDescPNT();
	m_uiStride = sizeof(SVertexPNT);

	m_Name = szMeshFile;
	if (!CMeshPool::GetResource((char*)szMeshFile, this)) {
		if (FAILED(D3DX10CreateMesh(g_pD3DDevice, aDesc, 3, aDesc->SemanticName, nVert, nFaces, uiOptions, &m_pMesh))) {
			DxUtSendErrorEx("CMesh::LoadMeshFromFile could not be loaded from file.", szMeshFile); 
		}

		//Set the vertex data
		m_nVertices = nVert;
		m_Vertices = new Vector3F[nVert];
		SVertexPNT * vert = new SVertexPNT[nVert];
		stream.read((char*)vert, nVert*sizeof(SVertexPNT));
		for (UINT i=0; i<nVert; i++) {
			vert[i].pos = scale*vert[i].pos;
			m_Vertices[i] = vert[i].pos;
		}
		m_pMesh->SetVertexData(0, vert);

		//Set the index data
		UINT * indi = new UINT[3*nFaces];
		stream.read((char*)indi, 3*nFaces*sizeof(UINT));
		m_pMesh->SetIndexData(indi, 3*nFaces);

		// Extract the triangles
		m_nFaces = nFaces;
		m_Tris = new STriangleF[nFaces];
		for (UINT i=0; i<nFaces; i++) {
			m_Tris[i].vPosW[0] = m_Vertices[indi[3*i+0]];
			m_Tris[i].vPosW[1] = m_Vertices[indi[3*i+1]];
			m_Tris[i].vPosW[2] = m_Vertices[indi[3*i+2]];
		}

		//Set the attribute data
		UINT * atri = (UINT*)vert;
		stream.read((char*)atri, nFaces*sizeof(UINT));
		m_pMesh->SetAttributeData(atri);

		// Extract the adj info
		m_Adj = new UINT[3*nFaces];
		stream.read((char*)m_Adj, 3*nFaces*sizeof(UINT));

		delete[] vert;
		delete[] indi;

		m_pMesh->GenerateAdjacencyAndPointReps(0.001f);
		m_pMesh->Optimize(D3DX10_MESHOPT_ATTR_SORT|D3DX10_MESHOPT_VERTEX_CACHE,0,0);
		m_pMesh->CommitToDevice();

		CMeshPool::PutResource(szMeshFile, this);
	} else {
		stream.seekg(2*sizeof(UINT)+nVert*sizeof(SVertexPNT)+3*nFaces*sizeof(UINT)+nFaces*sizeof(UINT)+3*nFaces*sizeof(UINT));
	}

	stream.read((char*)&m_nSubsets, sizeof(UINT));
	m_Materials = new SMaterial[m_nSubsets];
	m_SRViews = new ID3D10ShaderResourceView*[m_nSubsets];

	for (UINT i=0, len=0; i<m_nSubsets; i++) {
		stream.read((char*)&m_Materials[i], sizeof(SMaterial));
		stream.read((char*)&len, sizeof(UINT));
		if (len) {
			CHAR texFileName[128] = {0};					//It is assumed that the name of a file is small.
			stream.read((char*)&texFileName, len);
			
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

void CMesh::ShallowCopy(CMesh & rhs)
{
	this->m_Name = rhs.m_Name;
	this->m_pMesh = rhs.m_pMesh;
	this->m_uiStride = rhs.m_uiStride;
	this->m_nSubsets = rhs.m_nSubsets;
	this->m_nVertices = rhs.m_nVertices;
	this->m_nFaces = rhs.m_nFaces;
	this->m_Materials = rhs.m_Materials;
	this->m_SRViews = rhs.m_SRViews;
	this->m_Vertices = rhs.m_Vertices;
	this->m_Tris = rhs.m_Tris;
	this->m_Adj = rhs.m_Adj;
}

void CMesh::Destroy()
{
	Assert(m_pMesh != NULL, "CMesh::DestroyMesh cannot destroy a mesh that was never created.");

	if (m_Materials) {
		delete[] m_Materials;
		m_Materials = NULL;

		for (UINT i=0; i<m_nSubsets; i++) ReleaseX(m_SRViews[i]);
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
	UINT dwNum=0;
	UINT nFaces = m_pMesh->GetNumFaces();
	WriteFile(hFile, &nFaces, sizeof(UINT), &dwNum, 0);
	UINT nVert = m_pMesh->GetNumVertices();
	WriteFile(hFile, &nVert, sizeof(UINT), &dwNum, 0);

	//Vertex buffer
	LPDIRECT3DVERTEXBUFFER9 vB;
	m_pMesh->GetVertexBuffer(&vB);

	SVertexPNT * vert;
	vB->Lock(0, 0, (void**)&vert, 0);
	WriteFile(hFile, (void*)vert, nVert*sizeof(SVertexPNT), &dwNum, 0);
	vB->Unlock();

	//Index buffer
	LPDIRECT3DINDEXBUFFER9 iB;
	m_pMesh->GetIndexBuffer(&iB);

	UINT * index;
	iB->Lock(0, 0, (void**)&index, 0);
	WriteFile(hFile, (void*)index, 3*nFaces*sizeof(UINT), &dwNum, 0);
	iB->Unlock();
	
	//Attributes
	UINT * atri;
	m_pMesh->LockAttributeBuffer(0, (UINT**)&atri);
	WriteFile(hFile, atri, nFaces*sizeof(UINT), &dwNum, 0);
	m_pMesh->UnlockAttributeBuffer();

	//Number of materials
	WriteFile(hFile, &numMaterial, sizeof(UINT), &dwNum, 0);

	D3DXMATERIAL * d3dMat = (D3DXMATERIAL*)materialBuf->GetBufferPointer();
	for (UINT i=0; i<numMaterial; i++) {
		Material mat;
		mat.amb = d3dMat[i].MatD3D.Ambient;
		mat.dif = d3dMat[i].MatD3D.Diffuse;
		mat.spe = d3dMat[i].MatD3D.Specular;
		mat.pow = d3dMat[i].MatD3D.Power;
		WriteFile(hFile, &mat, sizeof(Material), &dwNum, 0);

		if (d3dMat[i].pTextureFilename) {
			UINT len = strlen(d3dMat[i].pTextureFilename)+1;
			WriteFile(hFile, &len, sizeof(UINT), &dwNum, 0);  

			CHAR * texFile = new CHAR[len+1];
			memcpy(texFile+1, d3dMat[i].pTextureFilename, len-1);
			texFile[0] = '/';
			texFile[len] = '\0';
			WriteFile(hFile, texFile, len, &dwNum, 0);  

			delete[] texFile;
		}
		else {
			UINT d=0;
			WriteFile(hFile, &d, sizeof(UINT), &dwNum, 0);  
		}
	}

	adj->Release();
	materialBuf->Release();
	m_pMesh->Release();

	CloseHandle(hFile);

	/*hFile = CreateFileA(szMeshFile, GENERIC_READ, 0, NULL, OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
	if (!hFile) return false;

	nFaces, nVert, dwNum=0;
	UINT a, b;
	ReadFile(hFile, &a, sizeof(UINT), &dwNum, 0);
	ReadFile(hFile, &b, sizeof(UINT), &dwNum, 0);

	return true;
	}
*/