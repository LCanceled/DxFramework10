
#include <fstream>
#include <algorithm>
#include "DxUtClouds.h"

//Various parameters for the shading computation of a cloud
#define CLOUD_SOLID_ANGLE	.09f
#define CLOUD_ALBEDO		.95f
#define CLOUD_EXTINCTION	80.f
#define CLOUD_EVALUATED		0.544309905374f
//#define CLOUD_EVALUATED	1.f*CLOUD_ALBEDO*CLOUD_EXTINCTION*CLOUD_SOLID_ANGLE/(4.f*D3DX_PI);

//The max radius a cloud can have
#define CLOUD_MAX_RADIUS	1.0e6f

//The size in texels of the impostor texture
#define CLOUD_IMPOSTER_SIZE 512

FLOAT AvgSample(float * grid, INT cc, INT cPos, INT size)
{
	INT yStart = cPos/cc - size/2;	yStart = max(0, yStart);
	INT yEnd = yStart+size;			yEnd = min(cc, yEnd);
	INT xStart = cPos%cc - size/2;	xStart = max(0, xStart);
	INT xEnd = xStart+size;			xEnd = min(cc, xEnd);

	FLOAT avg = 0.f;
	FLOAT nSamples = 0;
	for (INT i=yStart; i<yEnd; i++) {
		for (INT j=xStart; j<xEnd; j++) { 
			avg += grid[i*cc+j];
			nSamples++;
		}
	}

	return avg/nSamples;
}

namespace DxUt {

//The illumination texture and viewport size for ComputeMultipleForwardScattering
CloudSystem::CloudSystem()//:m_dwViewportSize(32)
{
}

void CloudSystem::InitCloudSystem(DWORD nClouds, FLOAT fParticleSize, CHAR * szCloudSplatTex)
{
	m_nClouds = nClouds;

	//Create the clouds using the first one to share resources
	m_rgCloud = new Cloud[nClouds];
	m_rgCloud[0].InitCloud(fParticleSize, szCloudSplatTex);

	//Create impostor resources
	D3D10_TEXTURE2D_DESC texDesc;
	texDesc.Width     = CLOUD_IMPOSTER_SIZE;
	texDesc.Height    = CLOUD_IMPOSTER_SIZE;
	texDesc.MipLevels = 0;
	texDesc.ArraySize = 1;
	texDesc.Format    = DXGI_FORMAT_R8G8B8A8_UNORM;
	texDesc.SampleDesc.Count   = 1;  
	texDesc.SampleDesc.Quality = 0;  
	texDesc.Usage          = D3D10_USAGE_DEFAULT;
	texDesc.BindFlags      = D3D10_BIND_RENDER_TARGET | D3D10_BIND_SHADER_RESOURCE;
	texDesc.CPUAccessFlags = 0; 
	texDesc.MiscFlags      = D3D10_RESOURCE_MISC_GENERATE_MIPS;

	Cloud * p = m_rgCloud+1;
	for (DWORD i=1; i<nClouds; i++, p++) {
		ID3D10Texture2D * impostorTex;
		if (FAILED(g_pD3DDevice->CreateTexture2D(&texDesc, 0, &impostorTex))) {
			DxUtSendError("InitCloud could not create imposter render target texture.");
		}

		if (FAILED(g_pD3DDevice->CreateRenderTargetView(impostorTex, 0, &p->m_pImposterRTView))) {
			DxUtSendError("InitCloud could not create imposter render target view.");
		}

		if (FAILED(g_pD3DDevice->CreateShaderResourceView(impostorTex, 0, &p->m_pImposterSRView))) {
			DxUtSendError("InitCloud could not create imposter render target view.");
		}

		ReleaseX(impostorTex);
	}

	p = m_rgCloud+1;
	Cloud * pStart = m_rgCloud;
	for (DWORD i=1; i<nClouds; i++, p++) {
		p->m_pVertexBuffer =				pStart->m_pVertexBuffer;
		p->m_pCloudSplatSRView =			pStart->m_pCloudSplatSRView;
		p->m_pIlluminationTex =				pStart->m_pIlluminationTex;
		p->m_pIlluminationReadBackTex =		pStart->m_pIlluminationReadBackTex;
		p->m_pIlluminationRTView =			pStart->m_pIlluminationRTView;

		p->m_dwViewportSize =				pStart->m_dwViewportSize;
		p->m_Viewport =						pStart->m_Viewport;

		p->m_CloudEffect.GetEffect() =		pStart->m_CloudEffect.GetEffect();
		p->m_CloudEffect.eVertexLayout =	pStart->m_CloudEffect.eVertexLayout;
		p->m_CloudEffect.eTech =			pStart->m_CloudEffect.eTech;
		p->m_CloudEffect.eWVP =				pStart->m_CloudEffect.eWVP;
		p->m_CloudEffect.eColor =			pStart->m_CloudEffect.eColor;
		p->m_CloudEffect.eTexOffsets =		pStart->m_CloudEffect.eTexOffsets;
		p->m_CloudEffect.eTexture =			pStart->m_CloudEffect.eTexture;
		p->m_fParticleSize =				pStart->m_fParticleSize;
	}
	m_nActiveClouds = 0;

	m_rgSortedCloud = new Cloud*[nClouds];
	for (DWORD i=0; i<m_nClouds; i++) 
		m_rgSortedCloud[i] = m_rgCloud+i;
}

void CloudSystem::AddCloud(CHAR * szCloudfile, FLOAT fCellDimensions, DWORD nCells, 
		Vector3F & cloudPos, Vector3F & sunDir, Vector3F & lightColor)
{
	if (m_nActiveClouds == m_nClouds)
		DxUtSendError("AddCloud will exceed cloud buffer capacity.");

	m_rgCloud[m_nActiveClouds++].CreateCloudFromFile(szCloudfile, fCellDimensions, nCells, cloudPos, sunDir, lightColor);
}

void CloudSystem::AddCloudFast(CHAR * szCloudfile, FLOAT fCellDimensions, DWORD nCells, 
	Vector3F & cloudPos, Vector3F & sunDir, Vector3F & lightColor)
{
	if (m_nActiveClouds == m_nClouds)
		DxUtSendError("AddCloud will exceed cloud buffer capacity.");

	m_rgCloud[m_nActiveClouds++].CreateCloudFastFromFile(szCloudfile, fCellDimensions, nCells, cloudPos, sunDir, lightColor);
}

int QSortCloudComp(const void *arg1, const void *arg2)
{
	return ((Cloud*)*((LONG*)arg1))->m_fDistToCam < ((Cloud*)*((LONG*)arg2))->m_fDistToCam;
}

void CloudSystem::RenderCloudSystem(Vector3F & camPos, Matrix4x4F & view, Matrix4x4F & proj, DWORD nCloudsToUpdate)
{
	for (DWORD i=0; i<m_nActiveClouds; i++)
		m_rgSortedCloud[i]->m_fDistToCam = (camPos - m_rgSortedCloud[i]->m_CloudPos).Length();

	for (DWORD i=0; i<m_nActiveClouds; i++) {
		DWORD k = i;
		FLOAT gd = 0;
		for (DWORD j=i; j<m_nActiveClouds; j++) {
			if (m_rgSortedCloud[j]->m_fDistToCam > gd) {
				k = j;
				gd = m_rgSortedCloud[j]->m_fDistToCam;
			}
		}
		Cloud * p = m_rgSortedCloud[i];
		m_rgSortedCloud[i] = m_rgSortedCloud[k];
		m_rgSortedCloud[k] = p;
	}

	//qsort is not always accurate ?
	//qsort(m_rgSortedCloud, m_nActiveClouds, sizeof(LONG), QSortCloudComp);

	for (int i=m_nActiveClouds-1; i>=0 && nCloudsToUpdate > 0; i--) {
		if (m_rgSortedCloud[i]->UpdateImpostor(camPos)) {
			m_rgSortedCloud[i]->BuildImpostor(camPos);
			nCloudsToUpdate--;
		}
	}

	Matrix4x4F viewProj(view*proj);
	for (DWORD i=0; i<m_nActiveClouds; i++) {
		if (m_rgSortedCloud[i]->InCloudRadius(camPos)) 
			m_rgSortedCloud[i]->RenderCloudParticles(camPos, viewProj);
		else
			m_rgSortedCloud[i]->RenderImpostor(camPos, viewProj);
	}
}

void CloudSystem::Destroy()
{
	for (DWORD i=1; i<m_nActiveClouds; i++) {
		ReleaseX(m_rgCloud[i].m_pImposterRTView);	
		ReleaseX(m_rgCloud[i].m_pImposterSRView);	
		delete[] m_rgCloud[i].m_rgCloudParticles;
		m_rgCloud[i].m_rgCloudParticles = NULL;
	}
	m_rgCloud[0].Destroy();

	delete[] m_rgCloud;
	m_rgCloud = NULL;
	delete[] m_rgSortedCloud;
	m_rgSortedCloud = NULL;
}

Vector2F TexCoordOffsets[16] = {
	Vector2F(0.0f, 0.0f),
	Vector2F(.25f, 0.0f),
	Vector2F(.50f, 0.0f),
	Vector2F(.75f, 0.0f),
	Vector2F(0.0f, .25f),
	Vector2F(.25f, .25f),
	Vector2F(.50f, .25f),
	Vector2F(.75f, .25f),
	Vector2F(0.0f, .50f),
	Vector2F(.25f, .50f),
	Vector2F(.50f, .50f),
	Vector2F(.75f, .50f),
	Vector2F(0.0f, .75f),
	Vector2F(.25f, .75f),
	Vector2F(.50f, .75f),
	Vector2F(.75f, .75f)
};

Cloud::Cloud():m_dwViewportSize(32)
{
}

void Cloud::InitCloud(FLOAT fParticleSize, CHAR * szCloudSplatTex)
{
	//Create vertex buffer
	SVertexPS vert[1] = {
		Vector3F(0.f, 0.f, 0.f), (WORD)fParticleSize
	};
	m_fParticleSize = fParticleSize;

	D3D10_BUFFER_DESC bd;
	bd.ByteWidth = sizeof(SVertexPS);
	bd.Usage = D3D10_USAGE_DEFAULT;
	bd.BindFlags = D3D10_BIND_VERTEX_BUFFER;
	bd.CPUAccessFlags = 0;
	bd.MiscFlags = 0;
	D3D10_SUBRESOURCE_DATA data;
	data.pSysMem = vert;
	if (FAILED(g_pD3DDevice->CreateBuffer(&bd, &data, &m_pVertexBuffer))) {
		DxUtSendError("InitCloud could not create the vertex buffer.");
	}

	//Create cloud splat render target view
	char file[MAX_PATH];
	InsertDirectory(g_szFileDir, szCloudSplatTex, file);

	if (FAILED(D3DX10CreateShaderResourceViewFromFileA(g_pD3DDevice, file, 0, 0, &m_pCloudSplatSRView, 0))) {
		DxUtSendErrorEx("InitCloud could not load the cloud splat texture.", file);
	}

	//Create the the resources for ComputeMultipleForwardScattering
	D3D10_TEXTURE2D_DESC texDesc;
	texDesc.Width     = m_dwViewportSize;
	texDesc.Height    = m_dwViewportSize;
	texDesc.MipLevels = 0;
	texDesc.ArraySize = 1;
	texDesc.Format    = DXGI_FORMAT_R32_FLOAT;
	texDesc.SampleDesc.Count   = 1;  
	texDesc.SampleDesc.Quality = 0;  
	texDesc.Usage          = D3D10_USAGE_DEFAULT;
	texDesc.BindFlags      = D3D10_BIND_RENDER_TARGET;
	texDesc.CPUAccessFlags = 0; 
	texDesc.MiscFlags      = 0;

	if (FAILED(g_pD3DDevice->CreateTexture2D(&texDesc, 0, &m_pIlluminationTex))) {
		DxUtSendError("InitCloud could not create illumination texture.");
	}

	texDesc.Width     = m_dwViewportSize;
	texDesc.Height    = m_dwViewportSize;
	texDesc.MipLevels = 0;
	texDesc.ArraySize = 1;
	texDesc.Format    = DXGI_FORMAT_R32_FLOAT;
	texDesc.SampleDesc.Count   = 1;  
	texDesc.SampleDesc.Quality = 0;  
	texDesc.Usage          = D3D10_USAGE_STAGING;
	texDesc.BindFlags      = 0;
	texDesc.CPUAccessFlags = D3D10_CPU_ACCESS_READ; 
	texDesc.MiscFlags      = 0;

	if (FAILED(g_pD3DDevice->CreateTexture2D(&texDesc, 0, &m_pIlluminationReadBackTex))) {
		DxUtSendError("InitCloud could not create illumination render target texture.");
	}

	if (FAILED(g_pD3DDevice->CreateRenderTargetView(m_pIlluminationTex, 0, &m_pIlluminationRTView))) {
		DxUtSendError("InitCloud could not create illumination render target view.");
	}

	//Create impostor resources
	texDesc.Width     = CLOUD_IMPOSTER_SIZE;
	texDesc.Height    = CLOUD_IMPOSTER_SIZE;
	texDesc.MipLevels = 0;
	texDesc.ArraySize = 1;
	texDesc.Format    = DXGI_FORMAT_R8G8B8A8_UNORM;
	texDesc.SampleDesc.Count   = 1;  
	texDesc.SampleDesc.Quality = 0;  
	texDesc.Usage          = D3D10_USAGE_DEFAULT;
	texDesc.BindFlags      = D3D10_BIND_RENDER_TARGET | D3D10_BIND_SHADER_RESOURCE;
	texDesc.CPUAccessFlags = 0; 
	texDesc.MiscFlags      = D3D10_RESOURCE_MISC_GENERATE_MIPS;

	ID3D10Texture2D * impostorTex;
	if (FAILED(g_pD3DDevice->CreateTexture2D(&texDesc, 0, &impostorTex))) {
		DxUtSendError("InitCloud could not create imposter render target texture.");
	}

	if (FAILED(g_pD3DDevice->CreateRenderTargetView(impostorTex, 0, &m_pImposterRTView))) {
		DxUtSendError("InitCloud could not create imposter render target view.");
	}

	if (FAILED(g_pD3DDevice->CreateShaderResourceView(impostorTex, 0, &m_pImposterSRView))) {
		DxUtSendError("InitCloud could not create imposter render target view.");
	}
	ReleaseX(impostorTex);

	//Create view port
	m_Viewport.TopLeftX = 0;
	m_Viewport.TopLeftY = 0;
	m_Viewport.Width	= m_dwViewportSize;
	m_Viewport.Height	= m_dwViewportSize;
	m_Viewport.MinDepth	= 0.f;
	m_Viewport.MaxDepth	= 1.f;

	//Create effect
	m_CloudEffect.CreateEffect("/Cloud.fx");
	m_CloudEffect.eTech		= m_CloudEffect->GetTechniqueByName("Render");
	m_CloudEffect.eWVP		= m_CloudEffect->GetVariableByName("g_WVP")->AsMatrix();
	m_CloudEffect.eColor	= m_CloudEffect->GetVariableByName("g_Color");
	m_CloudEffect.eTexOffsets = m_CloudEffect->GetVariableByName("g_TexOffsets");
	m_CloudEffect.eTexture	= m_CloudEffect->GetVariableByName("g_Tex")->AsShaderResource();
	m_CloudEffect.eTexture->SetResource(m_pCloudSplatSRView);

	D3D10_INPUT_ELEMENT_DESC desc[] = {
		{"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D10_INPUT_PER_VERTEX_DATA, 0},
		{"SIZE", 0, DXGI_FORMAT_R32_FLOAT, 0, 12, D3D10_INPUT_PER_VERTEX_DATA, 0}
	};
	CreateInputLayout(m_CloudEffect.eTech, desc, 2, 0, m_CloudEffect.eVertexLayout);
}

void Cloud::CreateCloudFromFile(CHAR * szCloudfile, FLOAT scale, DWORD nCells, 
		Vector3F & cloudPos, Vector3F & sunDir, Vector3F & lightColor)
{
	INT cc = (nCells%2) + nCells;
	m_nCells = cc;

	m_CloudPos = cloudPos;
	m_SunDir = sunDir.Normalize();
	m_LightColor = lightColor;
	m_LastCamPos = cloudPos;

	CHAR file[MAX_PATH];
	InsertDirectory(g_szFileDir, szCloudfile, file);

	std::fstream stream(file);
	if (!stream) DxUtSendErrorEx("CreateCloudFromFile could not open the cloud file.", file);

	stream >> m_fCloudRadius;
	m_fCloudRadius *= scale;
	m_fCellDimen = 2.f*(m_fCloudRadius/cc)*scale;

	stream >> m_nCloudParticles;
	m_rgCloudParticles = new CloudParticle[m_nCloudParticles];

	for (DWORD i=0; i<m_nCloudParticles; i++) {
		srand((UINT)time(0) + i);

		Vector3F pos;
		stream >> pos.x; 
		stream >> pos.z;
		stream >> pos.y;

		m_rgCloudParticles[i].pos = scale*pos;
		m_rgCloudParticles[i].type = rand()%16;
	}

	ComputeMultipleForwardScattering();
}

void Cloud::CreateCloudFastFromFile(CHAR * szCloudfile, FLOAT scale, DWORD nCells, 
		Vector3F & cloudPos, Vector3F & sunDir, Vector3F & lightColor)
{
	INT cc = (nCells%2) + nCells;
	m_nCells = cc;

	m_CloudPos = cloudPos;
	m_SunDir = sunDir.Normalize();
	m_LightColor = lightColor;
	m_LastCamPos = cloudPos;

	CHAR file[MAX_PATH];
	InsertDirectory(g_szFileDir, szCloudfile, file);

	std::fstream stream(file);
	if (!stream) DxUtSendErrorEx("CreateCloudFromFile could not open the cloud file.", file);

	float halfRowSize = 0;
	stream >> halfRowSize; 
	float cd = 2.f*(halfRowSize/cc);
	m_fCellDimen = cd;

	stream >> m_nCloudParticles;
	m_rgCloudParticles = new CloudParticle[m_nCloudParticles];
	
	DWORD gridSize = cc*cc*cc;
	CellVoxel * grid = new CellVoxel[gridSize];
	ZeroMemory(grid, gridSize*sizeof(CellVoxel));

	Vector3F forward(m_SunDir);
	Vector3F up(0.f, -1.f, 0);
	Vector3F right(CrossXYZ(up, forward).Normalize());
	up = CrossXYZ(forward, right);
	Matrix4x4F rot(
		right.x,	right.y,	right.z,	0,
		up.x,		up.y,		up.z,		0,
		forward.x,	forward.y,	forward.z,	0,
		0,			0,			0,			1.f);

	DWORD cPIndex = 0;
	for (DWORD i=0; i<m_nCloudParticles; i++) {
		srand((UINT)time(0) + i);

		Vector3F pos;
		stream >> pos.x; 
		stream >> pos.z;
		stream >> pos.y;

		Vector3F tPos(rot*pos);
		INT xi = (INT)floorf((tPos.x + halfRowSize)/cd);
		if (xi < 0 || xi > cc) continue;
		INT yi = (INT)floorf((tPos.y + halfRowSize)/cd);
		if (yi < 0 || yi > cc) continue;
		INT zi = (INT)floorf((tPos.z + halfRowSize)/cd); 
		if (zi < 0 || zi > cc) continue;
		if (grid[zi*cc*cc+yi*cc+xi].cPIndex)
			continue;

		m_rgCloudParticles[cPIndex].pos = scale*pos;
		m_rgCloudParticles[cPIndex].type = rand() % 16;
		grid[zi*cc*cc+yi*cc+xi].cPIndex = ++cPIndex;
	}
	m_fCellDimen *= scale;
	m_fCloudRadius = scale*halfRowSize;
	m_nCloudParticles = cPIndex;

	INT colorGridSize = cc*cc;
	float * colorGrid = new float[colorGridSize];
	for (INT i=0; i<colorGridSize; i++)
		colorGrid[i] = 1.f;

	Vector3F sunPos(-m_fCloudRadius*m_SunDir);
	
	FLOAT mul = m_dwViewportSize/(2.f*m_fCloudRadius);
	for (DWORD i=0; i<gridSize; i++) {
		if (!grid[i].cPIndex)
			continue;
		cPIndex = grid[i].cPIndex-1;

		//Fake ComputeMultipleForwardScattering
		INT nPixels = (INT)(mul*sqrt((sunPos - m_rgCloudParticles[cPIndex].pos).LengthSq()*CLOUD_SOLID_ANGLE));
		nPixels = max(nPixels, 1);

		float avg = AvgSample(colorGrid, cc, i%colorGridSize, nPixels);
		float color = CLOUD_EVALUATED*avg;
		m_rgCloudParticles[cPIndex].color = Vector3F(
			color*m_LightColor.x,
			color*m_LightColor.y, 
			color*m_LightColor.z);

		colorGrid[i%colorGridSize] = 1.f*color;
	}
	delete[] grid;
	grid = NULL;

	delete[] colorGrid;
	colorGrid = NULL;

	/*
	It should be noted that this function does not gurantee that
	every particle in the cloud file is copied to m_rgCloudParticles.
	*/
}

void Cloud::ComputeMultipleForwardScattering()
{
	Vector3F sunPos(-(m_fCloudRadius+m_fParticleSize)*m_SunDir);

	D3DXMATRIX view;
	D3DXMatrixLookAtLH(&view, (D3DXVECTOR3*)&sunPos, &D3DXVECTOR3(0, 0, 0), &D3DXVECTOR3(0, 1.f, 0));

	D3DXMATRIX proj;
	D3DXMatrixOrthoLH(&proj, 2.f*(m_fCloudRadius+m_fParticleSize),
		2.f*(m_fCloudRadius+m_fParticleSize), .1f, 2.f*(m_fCloudRadius+m_fParticleSize));
	D3DXMATRIX viewProj(view*proj);

	SortCloudParticlesFrontToBack(sunPos);

	g_pD3DDevice->IASetInputLayout(m_CloudEffect.eVertexLayout);
	g_pD3DDevice->IASetPrimitiveTopology(D3D10_PRIMITIVE_TOPOLOGY_POINTLIST);
	UINT stride = sizeof(SVertexPS);
	UINT offsets = 0;
	g_pD3DDevice->IASetVertexBuffers(0, 1, &m_pVertexBuffer, &stride, &offsets);

	ID3D10RenderTargetView* renderTargets[1] = {m_pIlluminationRTView};
	g_pD3DDevice->OMSetRenderTargets(1, renderTargets, 0);
	g_pD3DDevice->RSSetViewports(1, &m_Viewport);
	FLOAT clearColor[] = {1.f};
	g_pD3DDevice->ClearRenderTargetView(m_pIlluminationRTView, clearColor);

	for (DWORD i=0; i<m_nCloudParticles; i++) {
		Vector3F centerTexCoord;
		D3DXVec3TransformCoord((D3DXVECTOR3*)&centerTexCoord, (D3DXVECTOR3*)&m_rgCloudParticles[i].pos, &viewProj);
		INT pixelCentX = (DWORD)((.5f*(1.f+centerTexCoord.x))*m_dwViewportSize);
		INT pixelCentY = (DWORD)((.5f*(1.f-centerTexCoord.y))*m_dwViewportSize);
		INT nPixels = (INT)(sqrt(m_rgCloudParticles[i].dist*CLOUD_SOLID_ANGLE)*m_dwViewportSize/(2.f*m_fCloudRadius));
		nPixels = max(nPixels, 1);
		if ((pixelCentX + nPixels/2) >= ((INT)m_dwViewportSize)) 
			nPixels = 2*(m_dwViewportSize - pixelCentX);
		if ((pixelCentY + nPixels/2) >= ((INT)m_dwViewportSize)) 
			nPixels = 2*(m_dwViewportSize - pixelCentY);

		INT x = pixelCentX-nPixels/2; x = max(x, 0);
		INT y = pixelCentY-nPixels/2; y = max(y, 0);

		D3D10_BOX box = {x, y, 0, x+nPixels, y+nPixels, 1};
		g_pD3DDevice->CopySubresourceRegion(m_pIlluminationReadBackTex, 0, 0, 0, 0, m_pIlluminationTex, 0, &box);

		D3D10_MAPPED_TEXTURE2D map;
		m_pIlluminationReadBackTex->Map(0, D3D10_MAP_READ, 0, &map);
		FLOAT * ar = (FLOAT*)map.pData, avg = 0;
		for (INT row=0; row<nPixels; row++) {
			INT rowStart = row*map.RowPitch/4;
			for (INT col=0; col<nPixels; col++) {
				avg += ar[rowStart + col];
			}
		}
		m_pIlluminationReadBackTex->Unmap(0);
		avg /= nPixels*nPixels;

		float color = avg*CLOUD_EVALUATED;
		m_rgCloudParticles[i].color.x = color*m_LightColor.x;
		m_rgCloudParticles[i].color.y = color*m_LightColor.y;
		m_rgCloudParticles[i].color.z = color*m_LightColor.z;
		//m_rgCloudParticles[i].color.a = 1.f - exp(-CLOUD_EXTINCTION);

		D3DXCOLOR particleColor;
		particleColor.r = 1.5f*m_rgCloudParticles[i].color.x;
		particleColor.g = 1.5f*m_rgCloudParticles[i].color.y;
		particleColor.b = 1.5f*m_rgCloudParticles[i].color.z;
		particleColor.a = 1.f - exp(-CLOUD_EXTINCTION);

		Vector3F & pos = m_rgCloudParticles[i].pos;
		Vector3F w(Vector3F(-sunPos + pos).Normalize());
		//This not exactly the correct way to calculate v, but it's good enough.
		Vector3F v(Vector3F(-(w.y+w.z)/w.x, 1.f, 1.f).Normalize());
		Vector3F u(CrossXYZ(v,w));
		D3DXMATRIX world(u.x, v.x, w.x, pos.x,
			u.y, v.y, w.y, pos.y,
			u.z, v.z, w.z, pos.z,
			0,   0,   0,   1.0f);
		D3DXMatrixTranspose(&world, &world);

		m_CloudEffect.eWVP->SetMatrix((float*)(&(world*viewProj)));
		m_CloudEffect.eColor->SetRawValue(&particleColor, 0, sizeof(D3DXCOLOR));
		m_CloudEffect.eTexOffsets->SetRawValue(&TexCoordOffsets[m_rgCloudParticles[i].type], 0, sizeof(Vector2F));
		m_CloudEffect.eTech->GetPassByIndex(0)->Apply(0);

		g_pD3DDevice->Draw(1, 0);
	}

	ResetRenderTargetAndView();
}

INT QSortCloudParticlesCompBTF(const void * arg1, const void * arg2)
{
	return ((Cloud::CloudParticle*)arg1)->dist <= ((Cloud::CloudParticle*)arg2)->dist;
}

void Cloud::SortCloudParticlesBackToFront(Vector3F & camPos)
{
	for (DWORD i=0; i<m_nCloudParticles; i++) 
		m_rgCloudParticles[i].dist = (m_rgCloudParticles[i].pos - camPos).LengthSq();

	for (DWORD i=0; i<m_nCloudParticles; i++) {
		DWORD k = i;
		FLOAT gd = 0;
		for (DWORD j=i; j<m_nCloudParticles; j++) {
			if (m_rgCloudParticles[j].dist > gd) {
				k = j;
				gd = m_rgCloudParticles[j].dist;
			}
		}
		CloudParticle p	= m_rgCloudParticles[i];
		m_rgCloudParticles[i] = m_rgCloudParticles[k];
		m_rgCloudParticles[k] = p;
	}

	//qsort is not sorting correctly
	//qsort(m_rgCloudParticles, m_nCloudParticles, sizeof(CloudParticle), QSortCloudParticlesCompBTF);
}

INT QSortCloudParticlesCompFTB(const void * arg1, const void * arg2)
{
	return ((Cloud::CloudParticle*)arg1)->dist > ((Cloud::CloudParticle*)arg2)->dist;
}

void Cloud::SortCloudParticlesFrontToBack(Vector3F & sunPos)
{
	for (DWORD i=0; i<m_nCloudParticles; i++) 
		m_rgCloudParticles[i].dist = (sunPos - m_rgCloudParticles[i].pos).LengthSq();

	for (DWORD i=0; i<m_nCloudParticles; i++) {
		DWORD k = i;
		FLOAT gd = CLOUD_MAX_RADIUS;
		for (DWORD j=i; j<m_nCloudParticles; j++) {
			if (m_rgCloudParticles[j].dist < gd) {
				k = j;
				gd = m_rgCloudParticles[j].dist;
			}
		}
		CloudParticle p	= m_rgCloudParticles[i];
		m_rgCloudParticles[i] = m_rgCloudParticles[k];
		m_rgCloudParticles[k] = p;
	}

	//qsort(m_rgCloudParticles, m_nCloudParticles, sizeof(CloudParticle), QSortCloudParticlesCompFTB);
}

/*
The impostor is updated when the angle between the last camera position
and the new one is greater than arccos(.99 + alpha). The alpha factor is
computed as |(a dot (0, 1, 0))|*.009f = |a.y|*.009f. Alpha is necessary
because the camera moves a greater distance around the cloud when its 
position relative to the cloud is close to the line through the poles of it.
The constant factors .99f and .009f are fixed for now, but it maybe be 
necessary to let the user change these values at any given time.
*/
BOOL Cloud::UpdateImpostor(Vector3F & camPos)
{
	Vector3F a((camPos - m_CloudPos).Normalize());
	Vector3F b((m_LastCamPos - m_CloudPos).Normalize());
	if (DotXYZ(a, b) < (.99f + abs(a.y)*.009f))
		return 1;
	
	return 0;
}

void Cloud::BuildImpostor(Vector3F & camPos)
{
	ID3D10RenderTargetView* renT[1] = {m_pImposterRTView};
	g_pD3DDevice->OMSetRenderTargets(1, renT, 0);

	D3D10_VIEWPORT viewPort;
	viewPort.TopLeftX = 0;
	viewPort.TopLeftY = 0;
	viewPort.Width	= CLOUD_IMPOSTER_SIZE;
	viewPort.Height	= CLOUD_IMPOSTER_SIZE;
	viewPort.MinDepth	= 0.f;
	viewPort.MaxDepth	= 1.f;
	g_pD3DDevice->RSSetViewports(1, &viewPort);

	float color[] = {0.f, 0.f, 0.f, 0.f};
	g_pD3DDevice->ClearRenderTargetView(m_pImposterRTView, color);

	D3DXMATRIX view;
	Vector3F vec((camPos - m_CloudPos).Normalize());
	float dist = (m_fCloudRadius+2.f*m_fParticleSize)/tanf(D3DX_PI/4.);
	D3DXMatrixLookAtLH(&view, (D3DXVECTOR3*)&(dist*vec+m_CloudPos), (D3DXVECTOR3*)&m_CloudPos, &D3DXVECTOR3(0, 1.f, 0));

	D3DXMATRIX proj;
	D3DXMatrixPerspectiveFovLH(&proj, D3DX_PI/2., 1.f, 1.f, m_fParticleSize+m_fCloudRadius+dist);

	RenderCloudParticles(camPos, *((Matrix4x4F*)((float*)(view*proj))) );

	g_pD3DDevice->GenerateMips(m_pImposterSRView);
	ResetRenderTargetAndView();

	m_LastCamPos = camPos;
}

void Cloud::RenderImpostor(Vector3F & camPos, Matrix4x4F & viewProj)
{
	g_pD3DDevice->IASetInputLayout(m_CloudEffect.eVertexLayout);
	g_pD3DDevice->IASetPrimitiveTopology(D3D10_PRIMITIVE_TOPOLOGY_POINTLIST);
	UINT stride = sizeof(SVertexPS);
	UINT offsets = 0;
	g_pD3DDevice->IASetVertexBuffers(0, 1, &m_pVertexBuffer, &stride, &offsets);

	Matrix4x4F scale;
	float size = .5f*(m_fCloudRadius+2.f*m_fParticleSize);
	scale.MScaling(Vector3F(size, size, 0.f));

	Vector3F & pos = m_CloudPos;
	//Normally the current position of the camera is used to align a impostor.
	//However, if the last camera position is used instead, there is generally less popping. 
	Vector3F w((pos - m_LastCamPos).Normalize());
	Vector3F v(0.f, 1.f, 0.f);
	v = CrossXYZ(v, w).Normalize();
	Vector3F u(CrossXYZ(w, v));
	Matrix4x4F world(
		v.x, u.x, w.x, pos.x,
		v.y, u.y, w.y, pos.y,
		v.z, u.z, w.z, pos.z,
		0,   0,   0,   1.0f);

	m_CloudEffect.eWVP->SetMatrix((float*)(&(scale*world*viewProj)));
	m_CloudEffect.eTexture->SetResource(m_pImposterSRView);
	m_CloudEffect.eTech->GetPassByIndex(1)->Apply(0);

	g_pD3DDevice->Draw(1, 0);
}

void Cloud::RenderCloudParticles(Vector3F & camPos, Matrix4x4F & viewProj)
{
	SortCloudParticlesBackToFront(camPos - m_CloudPos);

	g_pD3DDevice->IASetInputLayout(m_CloudEffect.eVertexLayout);
	g_pD3DDevice->IASetPrimitiveTopology(D3D10_PRIMITIVE_TOPOLOGY_POINTLIST);
	UINT stride = sizeof(SVertexPS);
	UINT offsets = 0;
	g_pD3DDevice->IASetVertexBuffers(0, 1, &m_pVertexBuffer, &stride, &offsets);
	m_CloudEffect.eTexture->SetResource(m_pCloudSplatSRView);

	for (DWORD i=0; i<m_nCloudParticles; i++) {
		Vector3F & pos = m_rgCloudParticles[i].pos + m_CloudPos;
		Vector3F w((pos - camPos).Normalize());
		Vector3F v(0.f, 1.f, 0.f);
		v = CrossXYZ(v, w).Normalize();
		Vector3F u(CrossXYZ(w, v));
		Matrix4x4F world(
			v.x, u.x, w.x, pos.x,
			v.y, u.y, w.y, pos.y,
			v.z, u.z, w.z, pos.z,
			0,   0,   0,   1.0f);

		m_CloudEffect.eWVP->SetMatrix((float*)(&(world*viewProj)));

		float rCosAlpha = DotXYZ(m_SunDir, w);
		float phase = (.75f * (1.f + (rCosAlpha*rCosAlpha)));
		if (phase < 1.f) phase = 1.f;
		phase = 1.f;
		D3DXCOLOR c;
		c.r = .5f + m_rgCloudParticles[i].color.x*phase;
		c.g = .5f + m_rgCloudParticles[i].color.y*phase;
		c.b = .5f + m_rgCloudParticles[i].color.z*phase;
		c.a = 1.f;

		m_CloudEffect.eColor->SetRawValue(&c, 0, sizeof(D3DXCOLOR));
		m_CloudEffect.eTexOffsets->SetRawValue(&TexCoordOffsets[m_rgCloudParticles[i].type], 0, sizeof(Vector2F));
		m_CloudEffect.eTech->GetPassByIndex(0)->Apply(0);

		g_pD3DDevice->Draw(1, 0);
	}
}

void Cloud::RenderCloud(Vector3F & camPos, Matrix4x4F & viewProj)
{
	if (UpdateImpostor(camPos)) {
		BuildImpostor(camPos);
	}

	m_fDistToCam = (camPos - m_CloudPos).Length();
	if (InCloudRadius(camPos)) 
		RenderCloudParticles(camPos, viewProj);
	else 
		RenderImpostor(camPos, viewProj);
}

void Cloud::Destroy()
{
	//Resources
	ReleaseX(m_pVertexBuffer);
	ReleaseX(m_pCloudSplatSRView);
	ReleaseX(m_pIlluminationTex);
	ReleaseX(m_pIlluminationReadBackTex);
	ReleaseX(m_pIlluminationRTView);

	m_CloudEffect.DestroyEffect();

	delete[] m_rgCloudParticles;
}





};
