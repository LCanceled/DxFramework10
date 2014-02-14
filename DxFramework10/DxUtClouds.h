
#ifndef DXUTCLOUDS_H
#define DXUTCLOUDS_H

#include "DxUtInclude.h"
#include "DxUtEffect.h"

namespace DxUt {

class Cloud;

class CloudSystem {
private:
	DWORD m_nClouds;			//The total number of clouds
	DWORD m_nActiveClouds;		//The number of clouds active
	Cloud * m_rgCloud;			//The clouds in the system
	FLOAT * m_rgDistance;		//Distance from camera to a cloud
	Cloud ** m_rgSortedCloud;	//The clouds sorted in increasing distance from the camera
public:
	CloudSystem();
	~CloudSystem() {}

	//Initializes the rendering resources
	//fParticleSize is the dimensions of a cloud particle sprite
	//szCloudSplatTex is the splat texture for an individual cloud particle
	void InitCloudSystem(DWORD nClouds, FLOAT fParticleSize, CHAR * szCloudSplatTex);

	//Adds a cloud from a file with multiple forward scattering
	//fCellDimensions is the length of a cell in which a cloud sprite will be drawn
	//nCells is the number of cells in a row,col for a given cloud
	void AddCloud(CHAR * szCloudfile, FLOAT fCellDimensions, DWORD nCells, 
		Vector3F & cloudPos, Vector3F & sunDir, Vector3F & lightColor);

	//Adds a cloud from a file with simplified lighting
	//fCellDimensions is the length of a cell in which a cloud sprite will be drawn
	//nCells is the number of cells in a row,col for a given cloud
	void AddCloudFast(CHAR * szCloudfile, FLOAT fCellDimensions, DWORD nCells, 
		Vector3F & cloudPos, Vector3F & sunDir, Vector3F & lightColor);

	//nCloudsToUpdate is the number of clouds to update on each draw call
	void RenderCloudSystem(Vector3F & camPos, Matrix4x4F & view, Matrix4x4F & proj, DWORD nCloudsToUpdate=10);

	void Destroy();
};

class Cloud {
private:
	friend class CloudSystem;

	//Resources
	ID3D10Buffer * m_pVertexBuffer;
	ID3D10ShaderResourceView * m_pCloudSplatSRView;
	ID3D10Texture2D * m_pIlluminationTex;
	ID3D10Texture2D * m_pIlluminationReadBackTex;
	ID3D10RenderTargetView * m_pIlluminationRTView;

	ID3D10RenderTargetView * m_pImposterRTView;
	ID3D10ShaderResourceView * m_pImposterSRView;

	//Viewport
	DWORD m_dwViewportSize;
	D3D10_VIEWPORT m_Viewport;

	//Cloud structures
	struct CloudsEffect : public CEffect {
		ID3D10InputLayout * eVertexLayout;

		ID3D10EffectTechnique * eTech;
		ID3D10EffectMatrixVariable * eWVP;
		ID3D10EffectVariable * eColor;
		ID3D10EffectVariable * eTexOffsets;
		ID3D10EffectShaderResourceVariable * eTexture;
	} m_CloudEffect;

	struct SVertexPS {
		Vector3F pos;
		FLOAT size;
	};

	//Cloud
	FLOAT m_fCellDimen;		//Dimensions of a cell
	DWORD m_nCells;			//Number of cells in a row, col

	Vector3F m_CloudPos;	//Position of the cloud
	FLOAT m_fCloudRadius;	//Radius of the cloud
	FLOAT m_fParticleSize;	//Size of a cloud particle width, height

	Vector3F m_SunDir;		//Direction of the sun vector
	Vector3F m_LightColor;	//Color of light shinning on the cloud
	Vector3F m_LastCamPos;	//Position of where the camera was during the last imposter update
	FLOAT m_fDistToCam;		//Distance from the cloud position to the camera

	struct CellVoxel {
		DWORD cPIndex;		//Index into the cloud particle array
	};

	struct CloudParticle {
		Vector3F pos;		//Pos of a particle
		FLOAT dist;			//Dist from particle to camera
		Vector3F color;		//Color of the particle
		DWORD type;			//The type of sprite to be associated
	} * m_rgCloudParticles;
	DWORD m_nCloudParticles;

	void ComputeMultipleForwardScattering();
	void SortCloudParticlesBackToFront(Vector3F & camPos);
	void SortCloudParticlesFrontToBack(Vector3F & sunPos);
	BOOL UpdateImpostor(Vector3F & camPos);
	void BuildImpostor(Vector3F & camPos);
	void RenderImpostor(Vector3F & camPos, Matrix4x4F & viewProj);
	void RenderCloudParticles(Vector3F & camPos, Matrix4x4F & viewProj);
	BOOL InCloudRadius(Vector3F & camPos);
	friend INT QSortCloudComp(const void * arg1, const void * arg2);
	friend INT QSortCloudParticlesCompBTF(const void * arg1, const void * arg2);
	friend INT QSortCloudParticlesCompFTB(const void * arg1, const void * arg2);
public:
	Cloud();
	~Cloud() {}

	//Initializes the rendering resources
	//fParticleSize is the dimensions of a cloud particle sprite
	//szCloudSplatTex is the splat texture for an individual cloud particle
	void InitCloud(FLOAT fParticleSize, CHAR * szCloudSplatTex);

	//Creates a cloud from a file with multiple forward scattering
	//fCellDimensions is the length of a cell in which a cloud sprite will be drawn
	//nCells is the number of cells in a row,col for a given cloud
	void CreateCloudFromFile(CHAR * szCloudfile, FLOAT scale, DWORD nCells, 
		Vector3F & cloudPos, Vector3F & sunDir, Vector3F & lightColor);

	//Creates a cloud from a file with simplified lighting
	//fCellDimensions is the length of a cell in which a cloud sprite will be drawn
	//nCells is the number of cells in a row,col for a given cloud
	void CreateCloudFastFromFile(CHAR * szCloudfile, FLOAT scale, DWORD nCells, 
		Vector3F & cloudPos, Vector3F & sunDir, Vector3F & lightColor);

	void RenderCloud(Vector3F & camPos, Matrix4x4F & viewProj);

	Vector3F & GetCloudPos() {return m_CloudPos;}

	void Destroy();
};

inline BOOL Cloud::InCloudRadius(Vector3F & camPos) 
{
	return m_fDistToCam < 2.f*m_fCloudRadius;
}


};


#endif




