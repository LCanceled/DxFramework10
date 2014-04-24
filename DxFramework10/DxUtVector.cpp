
#include "DxUtVector.h"
#include "DxUtMatrix.h"

namespace DxUt {

inline float fastSqrt(const float x)
{
	__m128 r0; 
	r0 = _mm_load_ss(&x);
	r0 = _mm_sqrt_ss(r0);

	float flt;
	_mm_store_ss(&flt, r0);

	return flt;
}

double g_fVectorEqualityEps = 1e-4;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////     Vector2F    ///////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Vector2F Vector2F::Normalize() const
{
	double mag = (double)sqrtf(x*x + y*y);
	if (mag < 1.e-8)
		return Vector2F(x*1.e-8, y*1.e-8);

	float mul = (float)(1.0/mag);
	return Vector2F(x*mul, y*mul);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////     Vector3F    ///////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Vector3F Vector3F::Normalize() const
{
	double mag = (double)sqrtf(x*x + y*y + z*z); 
	if (mag == 0) return Vector3F(0, 0, 0);
	//Assert(mag != 0, "Vector3F::Normalize vector of magnitude 0 was requested to be normalized.");
	
	mag = 1.0/mag;
	return Vector3F(x*mag, y*mag, z*mag);
}
/*
Vector3F Vector3F::NormalizeSq()
{
	double mag = x*x + y*y + z*z; 
	Assert(mag != 0, "Vector3F::Normalize vector of magnitude 0 was requested to be normalized.");
	
	mag = 1.0/mag;
	return Vector3F(x*x*mag, y*y*mag, z*z*mag);
}
*/
Matrix4x4F Vector3F::SkewMatrix3x3F()
{
	return Matrix4x4F(0, z, -y, 0, -z, 0, x, 0, y, -x, 0, 0, 0, 0, 0, 0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////     Vector4F    ///////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Vector4F Vector4F::Normalize() const
{
	double mag = (double)sqrtf(x*x + y*y + z*z + w*w);
	if (mag < 1.e-8)
		return Vector4F(x*1.e-8, y*1.e-8, z*1.e-8, w*1.e-8);

	float mul = (float)(1.0/mag); 
	return Vector4F(x*mul, y*mul, z*mul, w*mul);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////     VectorNF    ///////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void VectorNF::CreateVector(UINT nElements)
{
	Assert(!c, "VectorNF::CreateVector must destroy the vector before creating a new one.");
	
	c = new float[nElements];
	m_nElements = nElements;
}

float VectorNF::Length() 
{
	double d = 0;
	for (UINT i=0; i<m_nElements; i++)
		d += c[i] * c[i];
	return (float)fastSqrt(d);
}

float VectorNF::LengthSq() 
{
	double d = 0;
	for (UINT i=0; i<m_nElements; i++)
		d += c[i] * c[i];
	return (float)d;
}

void VectorNF::DestroyVector()
{
	Assert(c, "VectorNF::DestroyVector cannot destroy a vector that has not been created.");
	
	delete[] c;
	c = NULL;
	m_nElements = 0;
}


};



