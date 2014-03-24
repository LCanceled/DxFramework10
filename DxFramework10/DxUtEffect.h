
#ifndef DXUTEFFECT_H
#define DXUTEFFECT_H

#include "DxUtInclude.h"
#include "DxUtMaterial.h"
#include "DxUtLight.h"

namespace DxUt {

class CEffect {
protected:
	ID3D10Effect * m_pEffect;
public:
	CEffect():m_pEffect(0) {}
	virtual ~CEffect() {}

	void CreateEffect(CHAR * szFxFile);
	void CreateEffect(HMODULE hMod, LPCWSTR fxResource);

	ID3D10Effect *& GetEffect() {return m_pEffect;}
	ID3D10Effect *& operator->() {return m_pEffect;}

	void Destroy() {ReleaseX(m_pEffect);}
};

void CreateInputLayout(ID3D10EffectTechnique *& pTec, CONST D3D10_INPUT_ELEMENT_DESC * rgDesc,
	DWORD nElements, DWORD dwPass, ID3D10InputLayout *& pLayout);

};


#endif
