
#include "DxUtEffect.h"
#include "DxUtEffectPool.h"

namespace DxUt {

void CEffect::CreateEffect(CHAR * szFxFile)
{
	UINT flags = D3D10_SHADER_ENABLE_STRICTNESS;
#if defined (DEBUG) | (_DEBUG)
	flags |= D3D10_CREATE_DEVICE_DEBUG;
#endif
	ID3D10Blob * errors = NULL;
	D3DX10CreateEffectFromFileA(szFxFile, NULL, NULL, "fx_4_0", flags, 0,
		g_pD3DDevice, NULL, NULL, &m_pEffect, &errors, NULL);
	if (errors) {
		MessageBoxA(0, (char*)errors->GetBufferPointer(), 0, 0);
		_DestroyProcess(); }
	else if (!m_pEffect) {
		DxUtSendErrorEx("CreateEffect could not create ID3DX10Effect from file.", szFxFile); 
	}

	ReleaseX(errors);
}

void CEffect::Destroy()
{
	ReleaseX(m_pEffect);
}

void CreateInputLayout(ID3D10EffectTechnique *& pTec, CONST D3D10_INPUT_ELEMENT_DESC * rgDesc,
	UINT nEl, UINT uiPass, ID3D10InputLayout *& lay)
{
	D3D10_PASS_DESC pDsc;
	pTec->GetPassByIndex(uiPass)->GetDesc(&pDsc);

	if(FAILED(g_pD3DDevice->CreateInputLayout(rgDesc, nEl, pDsc.pIAInputSignature, pDsc.IAInputSignatureSize, &lay))) {
		DxUtSendError("CreateInputLayout could not create ID3D10InputLayout.");
	}
}


};

	