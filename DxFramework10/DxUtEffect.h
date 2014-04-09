
#ifndef DXUTEFFECT_H
#define DXUTEFFECT_H

#include "DxUtInclude.h"
#include "DxUtMaterial.h"
#include "DxUtLight.h"
#include "DxUtResourcePool.h"

namespace DxUt {

class CEffect : public CPooledResource<CEffect> {
protected:
	ID3D10Effect * m_pEffect;
public:
	CEffect():m_pEffect(0) {}
	virtual ~CEffect() {}

	virtual void CreateEffect(CHAR * szFxFile)=0;

	ID3D10Effect *& GetEffect() {return m_pEffect;}
	ID3D10Effect *& operator->() {return m_pEffect;}

	virtual void ShallowCopy(CEffect & rhs) =0{
		this->m_pEffect = rhs.m_pEffect;
	}

	virtual void Destroy();
};

void CreateInputLayout(ID3D10EffectTechnique *& pTec, CONST D3D10_INPUT_ELEMENT_DESC * rgDesc,
	UINT nElements, UINT uiPass, ID3D10InputLayout *& pLayout);

};


#endif
