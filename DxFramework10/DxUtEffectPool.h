
#ifndef DXUTEFFECTPOOL_H
#define DXUTEFFECTPOOL_H

#include "DxUtArray.h"
#include "DxUtShaders.h"

#define MAX_EFFECT_NAME_LEN MAX_PATH

namespace DxUt {

	//Make this a generic class
/* Cannot make this class a singelton because of DirectX */
class CEffectPool {
private:
	struct SEffectElement {
		char szEffectName[MAX_EFFECT_NAME_LEN];
		CEffect * pEffect;

		bool operator==(SEffectElement & e) {
			return strcmp(szEffectName, e.szEffectName) == 0;
		}
	};
	static CArray<SEffectElement, 8> m_rgElement;

	void DestroyEffectPool() {
		for (DWORD i=0, end=m_rgElement.GetSize(); i<end; i++) {
			m_rgElement[i].pEffect->DestroyEffect();
			delete m_rgElement[i].pEffect;
		}
		m_rgElement.Clear();
	}

	CEffectPool() {}
	//~CEffectPool() {}

	friend class CD3DApp;
public:
	/* Not a realtime function */
	//pointers will get murdered 
	template <typename TEffect>
	static bool GetEffect(char * szFile, TEffect ** pEffect) {
		Assert(strlen(szFile)+1 < MAX_EFFECT_NAME_LEN, "CEffect::GetEffect effect name is too long.");

		DWORD dwIdx = 0;
		SEffectElement el;
		strcpy(el.szEffectName, szFile);
		if (m_rgElement.FindIfEquals(el, dwIdx)) {
			*pEffect = (TEffect*)m_rgElement[dwIdx].pEffect;
			return 1;
		} else {
			TEffect * pNewEffect = new TEffect;
			pNewEffect->CreateEffect(szFile);
			el.pEffect = pNewEffect;

			m_rgElement.PushBack(el);
			*pEffect = pNewEffect;
			return 0;
		}
	}
};


};


#endif