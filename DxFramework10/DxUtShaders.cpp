
#include "DxUtShaders.h"
#include "DxUtVertex.h"
#include "DxUtEffectPool.h"

namespace DxUt {

void CPNTPhongFx::CreateEffect(CHAR * szFxFile)
{
	char * szFileName = szFxFile ? szFxFile : "PNT_Phong.fx";
	if (!CEffectPool::GetResource(szFileName, this)) {
		CEffect::CreateEffect(szFileName);

		CPNTPhongFx & fx = *this;
		fx.eTech		= fx->GetTechniqueByIndex(0);
		fx.eWVP			= fx->GetVariableByName("g_WVP")->AsMatrix();
		fx.eWorld		= fx->GetVariableByName("g_World")->AsMatrix();
		fx.eMaterial	= fx->GetVariableByName("g_Mat");
		fx.eLight		= fx->GetVariableByName("g_Light");
		fx.eCamPos		= fx->GetVariableByName("g_CamPos");
		fx.eTexture		= fx->GetVariableByName("g_Tex")->AsShaderResource();

		DxUt::CreateInputLayout(fx.eTech, DxUt::GetVertexElementDescPNT(), 3, 0, fx.eVertexLayout);

		CEffectPool::PutResource(szFileName, this);
	}
}

void CPNTPhongFx::ShallowCopy(CEffect & rhs)
{
	CEffect::ShallowCopy(rhs);
	CPNTPhongFx & rhsPNT = *((CPNTPhongFx*)&rhs);
	this->eVertexLayout = rhsPNT.eVertexLayout;
	this->eTech = rhsPNT.eTech;
	this->eWVP = rhsPNT.eWVP;
	this->eWorld = rhsPNT.eWorld;
	this->eMaterial = rhsPNT.eMaterial;
	this->eLight = rhsPNT.eLight;
	this->eCamPos = rhsPNT.eCamPos;
	this->eTexture = rhsPNT.eTexture;
}


};