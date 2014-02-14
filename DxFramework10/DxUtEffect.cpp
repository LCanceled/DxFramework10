
#include "DxUtEffect.h"

namespace DxUt {

void CEffect::CreateEffect(CHAR * szFxFile)
{
	char file[MAX_PATH];
	InsertDirectoryEx(g_szFileDir, szFxFile, file);

	DWORD flags = D3D10_SHADER_ENABLE_STRICTNESS;
#if defined (DEBUG) | (_DEBUG)
	flags |= D3D10_CREATE_DEVICE_DEBUG;
#endif
	ID3D10Blob * errors = NULL;
	D3DX10CreateEffectFromFileA(file, NULL, NULL, "fx_4_0", flags, 0,
		g_pD3DDevice, NULL, NULL, &m_pEffect, &errors, NULL);
	if (errors) {
		MessageBoxA(0, (char*)errors->GetBufferPointer(), 0, 0);
		_DestroyProcess(); }
	else if (!m_pEffect) {
		DxUtSendErrorEx("CreateEffect could not create ID3DX10Effect from file.", file); 
	}

	ReleaseX(errors);
}

void CEffect::CreateEffect(HMODULE hMod, LPCWSTR fxResource)
{
	DWORD flags = D3D10_SHADER_ENABLE_STRICTNESS;
#if defined (DEBUG) | (_DEBUG)
	flags |= D3D10_CREATE_DEVICE_DEBUG;
#endif
	ID3D10Blob * errors = NULL;
	D3DX10CreateEffectFromResourceW(hMod, fxResource, NULL, NULL, NULL,
		"fx_4_0", flags, 0, g_pD3DDevice, NULL, NULL, &m_pEffect, &errors, NULL);
	if (errors) {
		MessageBoxA(0, (char*)errors->GetBufferPointer(), 0, 0);
		_DestroyProcess(); }
	else if (!m_pEffect) {
		DxUtSendErrorEx("CreateEffect could not create ID3DX10Effect from resource.", (CHAR*)fxResource); 
	}

	ReleaseX(errors);
}

void CreateInputLayout(ID3D10EffectTechnique *& pTec, CONST D3D10_INPUT_ELEMENT_DESC * rgDesc,
	DWORD nEl, DWORD dwPass, ID3D10InputLayout *& lay)
{
	D3D10_PASS_DESC pDsc;
	pTec->GetPassByIndex(dwPass)->GetDesc(&pDsc);

	if(FAILED(g_pD3DDevice->CreateInputLayout(rgDesc, nEl, pDsc.pIAInputSignature, pDsc.IAInputSignatureSize, &lay))) {
		DxUtSendError("CreateInputLayout could not create ID3D10InputLayout.");
	}
}

void ExtractValuesFromImageFile(CHAR * szImageFile, void ** ppData,
	DWORD dwStride, DWORD * pdwImageWidth, DWORD * pdwImageHeight)
{
	CHAR file[MAX_PATH];
	InsertDirectory(g_szFileDir, szImageFile, file);

	D3DX10_IMAGE_INFO imageInfo;
	D3DX10GetImageInfoFromFileA(file, 0, &imageInfo, 0);
	DWORD width = imageInfo.Width; 
	if (pdwImageWidth) *pdwImageWidth = width;
	DWORD height = imageInfo.Height;
	if (pdwImageHeight) *pdwImageHeight = height;

	D3DX10_IMAGE_LOAD_INFO loadInfo;
	loadInfo.Width  = width;
	loadInfo.Height = height;
	loadInfo.Depth  = imageInfo.Depth;
	loadInfo.FirstMipLevel = 0;
	loadInfo.MipLevels = imageInfo.MipLevels;
	loadInfo.Usage = D3D10_USAGE_STAGING;
	loadInfo.BindFlags = 0;
	loadInfo.CpuAccessFlags = D3D10_CPU_ACCESS_READ;
	loadInfo.MiscFlags = 0;
	loadInfo.Format = imageInfo.Format;
	loadInfo.Filter = D3DX10_FILTER_NONE;
	loadInfo.MipFilter = D3DX10_FILTER_NONE;
	loadInfo.pSrcInfo  = 0;

	ID3D10Texture2D * tex;
	if (FAILED(D3DX10CreateTextureFromFileA(g_pD3DDevice, file, &loadInfo, 0, (ID3D10Resource**)&tex, NULL))) {
		DxUtSendErrorEx("ExtractValuesFromImageFile could not load the image file.", file);
	}

	*ppData = new CHAR[dwStride*width*height];
	D3D10_MAPPED_TEXTURE2D map;
	tex->Map(0, D3D10_MAP_READ, 0, &map);
	BYTE * ar = (BYTE*)map.pData;
	for (DWORD i=0; i<height; i++) {
		DWORD rowStart1 = i*dwStride*width;
		DWORD rowStart2 = i*dwStride*map.RowPitch/4;
		for (DWORD j=0; j<dwStride*width; j++) {
			((BYTE*)(*ppData))[rowStart1 + j] = ar[rowStart2 + j];
		}
	}
	tex->Unmap(0);
	ReleaseX(tex);
}


};

			//memcpy(((BYTE*)(*ppData) + j*4), ((BYTE*)map.pData + j*4), dwStride);

				/**ppData = new BYTE[width*height*dwStride];
	D3D10_MAPPED_TEXTURE2D map;
	tex->Map(0, D3D10_MAP_READ, 0, &map);
	BYTE * ar = (BYTE*)map.pData;
	for (DWORD i=0; i<height; i++) {
		DWORD rowStart1 = i*dwStride*width;
		DWORD rowStart2 = i*dwStride*map.RowPitch/4;
		for (DWORD j=0; j<width*dwStride; j++) {
			((BYTE*)(*ppData))[rowStart1 + j] = ar[rowStart2 + j];
		}
		float * ask = (float*)(*ppData);
		ask[0];
		ask[1];
		int a=0;
	}*/