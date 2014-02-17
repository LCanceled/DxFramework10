
#ifndef DXUTFX_H
#define DXUTFX_H

#include "DxUtInclude.h"
#include "DxUtEffect.h"

namespace DxUt {

class CPNPhongFx : public CEffect {
public:
	ID3D10InputLayout * eVertexLayout;

	ID3D10EffectTechnique * eTech;
	ID3D10EffectMatrixVariable * eWVP;
	ID3D10EffectMatrixVariable * eWorld;
	ID3D10EffectVariable * eMaterial;
	ID3D10EffectVariable * eLight;
	ID3D10EffectVariable * eCamPos;
};

class CPTPhongFx : public CEffect {
public:
	ID3D10InputLayout * eVertexLayout;

	ID3D10EffectTechnique * eTech;
	ID3D10EffectMatrixVariable * eWVP;
	ID3D10EffectMatrixVariable * eWorld;
	ID3D10EffectShaderResourceVariable * eTexture;
	ID3D10EffectScalarVariable * eTextureTile;
};


class CPNTPhongFx : public CEffect {
public:
	ID3D10InputLayout * eVertexLayout;

	ID3D10EffectTechnique * eTech;
	ID3D10EffectMatrixVariable * eWVP;
	ID3D10EffectMatrixVariable * eWorld;
	ID3D10EffectVariable * eMaterial;
	ID3D10EffectVariable * eLight;
	ID3D10EffectVariable * eCamPos;
	ID3D10EffectShaderResourceVariable * eTexture;
	ID3D10EffectScalarVariable * eTextureTile;
};

class CNormalMappingFx : public CEffect {
public:
	ID3D10EffectTechnique * eTech;
	ID3D10EffectMatrixVariable * eWVP;
	ID3D10EffectMatrixVariable * eWorld;
	ID3D10EffectVariable * eMaterial;
	ID3D10EffectVariable * eLight;
	ID3D10EffectVariable * eCamPos;
	ID3D10EffectShaderResourceVariable * eTexture;
	ID3D10EffectShaderResourceVariable * eNorTexture;
	ID3D10EffectVectorVariable * eTextureTile;
};

class CParallaxMappingFx : public CEffect {
public:
	ID3D10InputLayout * eVertexLayout;

	ID3D10EffectTechnique * eTech;
	ID3D10EffectMatrixVariable * eWVP;
	ID3D10EffectMatrixVariable * eWorld;
	ID3D10EffectVariable * eMaterial;
	ID3D10EffectVariable * eLight;
	ID3D10EffectVariable * eCamPos;
	ID3D10EffectShaderResourceVariable * eTexture;
	ID3D10EffectShaderResourceVariable * eNHTexture;
	ID3D10EffectScalarVariable * eTextureTile;
	ID3D10EffectScalarVariable * eHeightScale;


	ID3D10EffectVectorVariable * eMinSamples;
	ID3D10EffectVectorVariable * eMaxSamples;
	ID3D10EffectVectorVariable * eLODDistanceMin;
	ID3D10EffectVectorVariable * eLODDistanceMax;
};

class CGaussianBlurFx : public CEffect {
public:
	ID3D10EffectTechnique * eTech;
	ID3D10EffectVectorVariable * eTexWidth;
	ID3D10EffectVectorVariable * eTexHeight;
	ID3D10EffectShaderResourceVariable * eTexture;

	ID3D10EffectVectorVariable * eBlurSizeX;
	ID3D10EffectVectorVariable * eBlurSizeY;
	ID3D10EffectVectorVariable * eSigma;
};

struct CFractalFx : public DxUt::CEffect {
public:
	ID3D10EffectTechnique * eTech;
	ID3D10EffectVectorVariable * eScale;
	ID3D10EffectVectorVariable * eTrans;
	ID3D10EffectVectorVariable * eNumIter;

	ID3D10EffectVectorVariable * eGrad1;
	ID3D10EffectVectorVariable * eGrad2;
	ID3D10EffectVectorVariable * eColorFactor;
};


};

#endif
