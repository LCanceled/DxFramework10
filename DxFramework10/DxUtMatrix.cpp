
#include "DxUtMatrix.h"
#include "DxUtVector.h"
#include "DxUtTriangle.h"

namespace DxUt {

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////   Matrix4x4F    ///////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Matrix4x4F Matrix4x4F::operator+(const Matrix4x4F & A)
{
	return Matrix4x4F(
		m[0][0] + A.m[0][0], m[1][0] + A.m[1][0], m[2][0] + A.m[2][0], m[3][0] + A.m[3][0],
		m[0][1] + A.m[0][1], m[1][1] + A.m[1][1], m[2][1] + A.m[2][1], m[3][1] + A.m[3][1],
		m[0][2] + A.m[0][2], m[1][2] + A.m[1][2], m[2][2] + A.m[2][2], m[3][2] + A.m[3][2],
		m[0][3] + A.m[0][3], m[1][3] + A.m[1][3], m[2][3] + A.m[2][3], m[3][3] + A.m[3][3]);
}

Matrix4x4F Matrix4x4F::operator-(const Matrix4x4F & A)
{
	return Matrix4x4F(
		m[0][0] - A.m[0][0], m[1][0] - A.m[1][0], m[2][0] - A.m[2][0], m[3][0] - A.m[3][0],
		m[0][1] - A.m[0][1], m[1][1] - A.m[1][1], m[2][1] - A.m[2][1], m[3][1] - A.m[3][1],
		m[0][2] - A.m[0][2], m[1][2] - A.m[1][2], m[2][2] - A.m[2][2], m[3][2] - A.m[3][2],
		m[0][3] - A.m[0][3], m[1][3] - A.m[1][3], m[2][3] - A.m[2][3], m[3][3] - A.m[3][3]);
}

Matrix4x4F Matrix4x4F::operator*(const Matrix4x4F & A)
{
	return Matrix4x4F(
		m[0][0] * A.m[0][0] + m[0][1] * A.m[1][0] + m[0][2] * A.m[2][0] + m[0][3] * A.m[3][0],
		m[1][0] * A.m[0][0] + m[1][1] * A.m[1][0] + m[1][2] * A.m[2][0] + m[1][3] * A.m[3][0],
		m[2][0] * A.m[0][0] + m[2][1] * A.m[1][0] + m[2][2] * A.m[2][0] + m[2][3] * A.m[3][0],
		m[3][0] * A.m[0][0] + m[3][1] * A.m[1][0] + m[3][2] * A.m[2][0] + m[3][3] * A.m[3][0],

		m[0][0] * A.m[0][1] + m[0][1] * A.m[1][1] + m[0][2] * A.m[2][1] + m[0][3] * A.m[3][1],
		m[1][0] * A.m[0][1] + m[1][1] * A.m[1][1] + m[1][2] * A.m[2][1] + m[1][3] * A.m[3][1],
		m[2][0] * A.m[0][1] + m[2][1] * A.m[1][1] + m[2][2] * A.m[2][1] + m[2][3] * A.m[3][1],
		m[3][0] * A.m[0][1] + m[3][1] * A.m[1][1] + m[3][2] * A.m[2][1] + m[3][3] * A.m[3][1],

		m[0][0] * A.m[0][2] + m[0][1] * A.m[1][2] + m[0][2] * A.m[2][2] + m[0][3] * A.m[3][2],
		m[1][0] * A.m[0][2] + m[1][1] * A.m[1][2] + m[1][2] * A.m[2][2] + m[1][3] * A.m[3][2],
		m[2][0] * A.m[0][2] + m[2][1] * A.m[1][2] + m[2][2] * A.m[2][2] + m[2][3] * A.m[3][2],
		m[3][0] * A.m[0][2] + m[3][1] * A.m[1][2] + m[3][2] * A.m[2][2] + m[3][3] * A.m[3][2],

		m[0][0] * A.m[0][3] + m[0][1] * A.m[1][3] + m[0][2] * A.m[2][3] + m[0][3] * A.m[3][3],
		m[1][0] * A.m[0][3] + m[1][1] * A.m[1][3] + m[1][2] * A.m[2][3] + m[1][3] * A.m[3][3],
		m[2][0] * A.m[0][3] + m[2][1] * A.m[1][3] + m[2][2] * A.m[2][3] + m[2][3] * A.m[3][3],
		m[3][0] * A.m[0][3] + m[3][1] * A.m[1][3] + m[3][2] * A.m[2][3] + m[3][3] * A.m[3][3]);
}

Matrix4x4F operator*(float flt, const Matrix4x4F & A)
{
	return Matrix4x4F(
		A.m[0][0] * flt, A.m[1][0] * flt, A.m[2][0] * flt, A.m[3][0] * flt,
		A.m[0][1] * flt, A.m[1][1] * flt, A.m[2][1] * flt, A.m[3][1] * flt,
		A.m[0][2] * flt, A.m[1][2] * flt, A.m[2][2] * flt, A.m[3][2] * flt,
		A.m[0][3] * flt, A.m[1][3] * flt, A.m[2][3] * flt, A.m[3][3] * flt);
}

Matrix4x4F Matrix4x4F::operator*(float flt)
{
	return Matrix4x4F(
		m[0][0] * flt, m[1][0] * flt, m[2][0] * flt, m[3][0] * flt,
		m[0][1] * flt, m[1][1] * flt, m[2][1] * flt, m[3][1] * flt,
		m[0][2] * flt, m[1][2] * flt, m[2][2] * flt, m[3][2] * flt,
		m[0][3] * flt, m[1][3] * flt, m[2][3] * flt, m[3][3] * flt);
}

Matrix4x4F & Matrix4x4F::operator+=(const Matrix4x4F & A)
{
	m[0][0] += A.m[0][0], m[1][0] += A.m[1][0], m[2][0] += A.m[2][0], m[3][0] += A.m[3][0];
	m[0][1] += A.m[0][1], m[1][1] += A.m[1][1], m[2][1] += A.m[2][1], m[3][1] += A.m[3][1];
	m[0][2] += A.m[0][2], m[1][2] += A.m[1][2], m[2][2] += A.m[2][2], m[3][2] += A.m[3][2];
	m[0][3] += A.m[0][3], m[1][3] += A.m[1][3], m[2][3] += A.m[2][3], m[3][3] += A.m[3][3];
	return *this;
}

Matrix4x4F & Matrix4x4F::operator-=(const Matrix4x4F & A)
{
	m[0][0] -= A.m[0][0], m[1][0] -= A.m[1][0], m[2][0] -= A.m[2][0], m[3][0] -= A.m[3][0];
	m[0][1] -= A.m[0][1], m[1][1] -= A.m[1][1], m[2][1] -= A.m[2][1], m[3][1] -= A.m[3][1];
	m[0][2] -= A.m[0][2], m[1][2] -= A.m[1][2], m[2][2] -= A.m[2][2], m[3][2] -= A.m[3][2];
	m[0][3] -= A.m[0][3], m[1][3] -= A.m[1][3], m[2][3] -= A.m[2][3], m[3][3] -= A.m[3][3];
	return *this;
}

Matrix4x4F & Matrix4x4F::operator*=(const Matrix4x4F & A)
{
	Matrix4x4F B(*this);
	m[0][0] = B.m[0][0] * A.m[0][0] + B.m[0][1] * A.m[1][0] + B.m[0][2] * A.m[2][0] + B.m[0][3] * A.m[3][0];
	m[1][0] = B.m[1][0] * A.m[0][0] + B.m[1][1] * A.m[1][0] + B.m[1][2] * A.m[2][0] + B.m[1][3] * A.m[3][0];
	m[2][0] = B.m[2][0] * A.m[0][0] + B.m[2][1] * A.m[1][0] + B.m[2][2] * A.m[2][0] + B.m[2][3] * A.m[3][0];
	m[3][0] = B.m[3][0] * A.m[0][0] + B.m[3][1] * A.m[1][0] + B.m[3][2] * A.m[2][0] + B.m[3][3] * A.m[3][0];

	m[0][1] = B.m[0][0] * A.m[0][1] + B.m[0][1] * A.m[1][1] + B.m[0][2] * A.m[2][1] + B.m[0][3] * A.m[3][1];
	m[1][1] = B.m[1][0] * A.m[0][1] + B.m[1][1] * A.m[1][1] + B.m[1][2] * A.m[2][1] + B.m[1][3] * A.m[3][1];
	m[2][1] = B.m[2][0] * A.m[0][1] + B.m[2][1] * A.m[1][1] + B.m[2][2] * A.m[2][1] + B.m[2][3] * A.m[3][1];
	m[3][1] = B.m[3][0] * A.m[0][1] + B.m[3][1] * A.m[1][1] + B.m[3][2] * A.m[2][1] + B.m[3][3] * A.m[3][1];

	m[0][2] = B.m[0][0] * A.m[0][2] + B.m[0][1] * A.m[1][2] + B.m[0][2] * A.m[2][2] + B.m[0][3] * A.m[3][2];
	m[1][2] = B.m[1][0] * A.m[0][2] + B.m[1][1] * A.m[1][2] + B.m[1][2] * A.m[2][2] + B.m[1][3] * A.m[3][2];
	m[2][2] = B.m[2][0] * A.m[0][2] + B.m[2][1] * A.m[1][2] + B.m[2][2] * A.m[2][2] + B.m[2][3] * A.m[3][2];
	m[3][2] = B.m[3][0] * A.m[0][2] + B.m[3][1] * A.m[1][2] + B.m[3][2] * A.m[2][2] + B.m[3][3] * A.m[3][2];

	m[0][3] = B.m[0][0] * A.m[0][3] + B.m[0][1] * A.m[1][3] + B.m[0][2] * A.m[2][3] + B.m[0][3] * A.m[3][3];
	m[1][3] = B.m[1][0] * A.m[0][3] + B.m[1][1] * A.m[1][3] + B.m[1][2] * A.m[2][3] + B.m[1][3] * A.m[3][3];
	m[2][3] = B.m[2][0] * A.m[0][3] + B.m[2][1] * A.m[1][3] + B.m[2][2] * A.m[2][3] + B.m[2][3] * A.m[3][3];
	m[3][3] = B.m[3][0] * A.m[0][3] + B.m[3][1] * A.m[1][3] + B.m[3][2] * A.m[2][3] + B.m[3][3] * A.m[3][3];
	return *this;
}

Matrix4x4F & Matrix4x4F::operator*=(float flt)
{
	m[0][0] *= flt, m[1][0] *= flt, m[2][0] *= flt, m[3][0] *= flt;
	m[0][1] *= flt, m[1][1] *= flt, m[2][1] *= flt, m[3][1] *= flt;
	m[0][2] *= flt, m[1][2] *= flt, m[2][2] *= flt, m[3][2] *= flt;
	m[0][3] *= flt, m[1][3] *= flt, m[2][3] *= flt, m[3][3] *= flt;
	return *this;
}

Matrix4x4F Matrix4x4F::Inverse() const
{
	Matrix4x4F A;
	float a00 = m[0][0]; float a01 = m[0][1]; float a02 = m[0][2]; float a03 = m[0][3];
	float a10 = m[1][0]; float a11 = m[1][1]; float a12 = m[1][2]; float a13 = m[1][3];
	float a20 = m[2][0]; float a21 = m[2][1]; float a22 = m[2][2]; float a23 = m[2][3];
	float a30 = m[3][0]; float a31 = m[3][1]; float a32 = m[3][2]; float a33 = m[3][3];

	A.m[0][0] =      a11*a22*a33 - a11*a23*a32 - a21*a12*a33 + a21*a13*a32 + a31*a12*a23 - a31*a13*a22;
	A.m[0][1] =    - a01*a22*a33 + a01*a23*a32 + a21*a02*a33 - a21*a03*a32 - a31*a02*a23 + a31*a03*a22;
	A.m[0][2] =      a01*a12*a33 - a01*a13*a32 - a11*a02*a33 + a11*a03*a32 + a31*a02*a13 - a31*a03*a12;
	A.m[0][3] =    - a01*a12*a23 + a01*a13*a22 + a11*a02*a23 - a11*a03*a22 - a21*a02*a13 + a21*a03*a12;
	A.m[1][0] =    - a10*a22*a33 + a10*a23*a32 + a20*a12*a33 - a20*a13*a32 - a30*a12*a23 + a30*a13*a22;
	A.m[1][1] =      a00*a22*a33 - a00*a23*a32 - a20*a02*a33 + a20*a03*a32 + a30*a02*a23 - a30*a03*a22;
	A.m[1][2] =    - a00*a12*a33 + a00*a13*a32 + a10*a02*a33 - a10*a03*a32 - a30*a02*a13 + a30*a03*a12;
	A.m[1][3] =      a00*a12*a23 - a00*a13*a22 - a10*a02*a23 + a10*a03*a22 + a20*a02*a13 - a20*a03*a12;
	A.m[2][0] =      a10*a21*a33 - a10*a23*a31 - a20*a11*a33 + a20*a13*a31 + a30*a11*a23 - a30*a13*a21;
	A.m[2][1] =    - a00*a21*a33 + a00*a23*a31 + a20*a01*a33 - a20*a03*a31 - a30*a01*a23 + a30*a03*a21;
	A.m[2][2] =      a00*a11*a33 - a00*a13*a31 - a10*a01*a33 + a10*a03*a31 + a30*a01*a13 - a30*a03*a11;
	A.m[2][3] =    - a00*a11*a23 + a00*a13*a21 + a10*a01*a23 - a10*a03*a21 - a20*a01*a13 + a20*a03*a11;
	A.m[3][0] =    - a10*a21*a32 + a10*a22*a31 + a20*a11*a32 - a20*a12*a31 - a30*a11*a22 + a30*a12*a21;
	A.m[3][1] =      a00*a21*a32 - a00*a22*a31 - a20*a01*a32 + a20*a02*a31 + a30*a01*a22 - a30*a02*a21;
	A.m[3][2] =    - a00*a11*a32 + a00*a12*a31 + a10*a01*a32 - a10*a02*a31 - a30*a01*a12 + a30*a02*a11;
	A.m[3][3] =      a00*a11*a22 - a00*a12*a21 - a10*a01*a22 + a10*a02*a21 + a20*a01*a12 - a20*a02*a11;

	float d = a00*A.m[0][0] + a10*A.m[0][1] +  a20*A.m[0][2] + a30*A.m[0][3];
	if (!d) d += 1e-8f;

	float div = 1.f/d;
	A.m[0][0] *= div; A.m[0][1] *= div; A.m[0][2] *= div; A.m[0][3] *= div;
	A.m[1][0] *= div; A.m[1][1] *= div; A.m[1][2] *= div; A.m[1][3] *= div;
	A.m[2][0] *= div; A.m[2][1] *= div; A.m[2][2] *= div; A.m[2][3] *= div;
	A.m[3][0] *= div; A.m[3][1] *= div; A.m[3][2] *= div; A.m[3][3] *= div;
	return A;

}

Matrix4x4F Matrix4x4F::Transpose() const
{
	return Matrix4x4F(
		m[0][0], m[0][1], m[0][2], m[0][3],
		m[1][0], m[1][1], m[1][2], m[1][3],
		m[2][0], m[2][1], m[2][2], m[2][3],
		m[3][0], m[3][1], m[3][2], m[3][3]);
}

Matrix4x4F Matrix4x4F::InverseTranspose() const
{
	Matrix4x4F A(Inverse());

	float t = 0;
	t = A.m[0][1]; A.m[0][1] = A.m[1][0]; A.m[1][0] = t;
	t = A.m[0][2]; A.m[0][2] = A.m[2][0]; A.m[2][0] = t;
	t = A.m[0][3]; A.m[0][3] = A.m[3][0]; A.m[3][0] = t;
	t = A.m[1][2]; A.m[1][2] = A.m[2][1]; A.m[2][1] = t;
	t = A.m[1][3]; A.m[1][3] = A.m[3][1]; A.m[3][1] = t;
	t = A.m[2][3]; A.m[2][3] = A.m[3][2]; A.m[3][2] = t;
	return A;
}

Matrix4x4F & Matrix4x4F::MInverse()
{
	float a00 = m[0][0]; float a01 = m[0][1]; float a02 = m[0][2]; float a03 = m[0][3];
	float a10 = m[1][0]; float a11 = m[1][1]; float a12 = m[1][2]; float a13 = m[1][3];
	float a20 = m[2][0]; float a21 = m[2][1]; float a22 = m[2][2]; float a23 = m[2][3];
	float a30 = m[3][0]; float a31 = m[3][1]; float a32 = m[3][2]; float a33 = m[3][3];

	m[0][0] =      a11*a22*a33 - a11*a23*a32 - a21*a12*a33 + a21*a13*a32 + a31*a12*a23 - a31*a13*a22;
	m[0][1] =    - a01*a22*a33 + a01*a23*a32 + a21*a02*a33 - a21*a03*a32 - a31*a02*a23 + a31*a03*a22;
	m[0][2] =      a01*a12*a33 - a01*a13*a32 - a11*a02*a33 + a11*a03*a32 + a31*a02*a13 - a31*a03*a12;
	m[0][3] =    - a01*a12*a23 + a01*a13*a22 + a11*a02*a23 - a11*a03*a22 - a21*a02*a13 + a21*a03*a12;
	m[1][0] =    - a10*a22*a33 + a10*a23*a32 + a20*a12*a33 - a20*a13*a32 - a30*a12*a23 + a30*a13*a22;
	m[1][1] =      a00*a22*a33 - a00*a23*a32 - a20*a02*a33 + a20*a03*a32 + a30*a02*a23 - a30*a03*a22;
	m[1][2] =    - a00*a12*a33 + a00*a13*a32 + a10*a02*a33 - a10*a03*a32 - a30*a02*a13 + a30*a03*a12;
	m[1][3] =      a00*a12*a23 - a00*a13*a22 - a10*a02*a23 + a10*a03*a22 + a20*a02*a13 - a20*a03*a12;
	m[2][0] =      a10*a21*a33 - a10*a23*a31 - a20*a11*a33 + a20*a13*a31 + a30*a11*a23 - a30*a13*a21;
	m[2][1] =    - a00*a21*a33 + a00*a23*a31 + a20*a01*a33 - a20*a03*a31 - a30*a01*a23 + a30*a03*a21;
	m[2][2] =      a00*a11*a33 - a00*a13*a31 - a10*a01*a33 + a10*a03*a31 + a30*a01*a13 - a30*a03*a11;
	m[2][3] =    - a00*a11*a23 + a00*a13*a21 + a10*a01*a23 - a10*a03*a21 - a20*a01*a13 + a20*a03*a11;
	m[3][0] =    - a10*a21*a32 + a10*a22*a31 + a20*a11*a32 - a20*a12*a31 - a30*a11*a22 + a30*a12*a21;
	m[3][1] =      a00*a21*a32 - a00*a22*a31 - a20*a01*a32 + a20*a02*a31 + a30*a01*a22 - a30*a02*a21;
	m[3][2] =    - a00*a11*a32 + a00*a12*a31 + a10*a01*a32 - a10*a02*a31 - a30*a01*a12 + a30*a02*a11;
	m[3][3] =      a00*a11*a22 - a00*a12*a21 - a10*a01*a22 + a10*a02*a21 + a20*a01*a12 - a20*a02*a11;

	float d = a00*m[0][0] + a10*m[0][1] +  a20*m[0][2] + a30*m[0][3];
	if (!d) d += 1e-8f;

	float div = 1.f/d;
	m[0][0] *= div; m[0][1] *= div; m[0][2] *= div; m[0][3] *= div;
	m[1][0] *= div; m[1][1] *= div; m[1][2] *= div; m[1][3] *= div;
	m[2][0] *= div; m[2][1] *= div; m[2][2] *= div; m[2][3] *= div;
	m[3][0] *= div; m[3][1] *= div; m[3][2] *= div; m[3][3] *= div;
	return *this;
}

Matrix4x4F & Matrix4x4F::MTranspose()
{
	float t = 0;
	t = m[0][1]; m[0][1] = m[1][0]; m[1][0] = t;
	t = m[0][2]; m[0][2] = m[2][0]; m[2][0] = t;
	t = m[0][3]; m[0][3] = m[3][0]; m[3][0] = t;
	t = m[1][2]; m[1][2] = m[2][1]; m[2][1] = t;
	t = m[1][3]; m[1][3] = m[3][1]; m[3][1] = t;
	t = m[2][3]; m[2][3] = m[3][2]; m[3][2] = t;
	return *this;
}

Matrix4x4F & Matrix4x4F::MInverseTranspose()
{
	MInverse();

	float t = 0;
	t = m[0][1]; m[0][1] = m[1][0]; m[1][0] = t;
	t = m[0][2]; m[0][2] = m[2][0]; m[2][0] = t;
	t = m[0][3]; m[0][3] = m[3][0]; m[3][0] = t;
	t = m[1][2]; m[1][2] = m[2][1]; m[2][1] = t;
	t = m[1][3]; m[1][3] = m[3][1]; m[3][1] = t;
	t = m[2][3]; m[2][3] = m[3][2]; m[3][2] = t;
	return *this;
}

Matrix4x4F & Matrix4x4F::MScaling(const Vector3F & scl)
{
	m[0][0] = scl.x, m[1][0] = 0,	  m[2][0] = 0,		m[3][0] = 0;
	m[0][1] = 0,	 m[1][1] = scl.y, m[2][1] = 0,		m[3][1] = 0;
	m[0][2] = 0,	 m[1][2] = 0,	  m[2][2] = scl.z,	m[3][2] = 0;
	m[0][3] = 0,	 m[1][3] = 0,	  m[2][3] = 0,		m[3][3] = 1;
	return *this;
}

Matrix4x4F & Matrix4x4F::MScaling(float x, float y, float z)
{
	m[0][0] = x,	m[1][0] = 0,	m[2][0] = 0,	m[3][0] = 0;
	m[0][1] = 0,	m[1][1] = y,	m[2][1] = 0,	m[3][1] = 0;
	m[0][2] = 0,	m[1][2] = 0,	m[2][2] = z,	m[3][2] = 0;
	m[0][3] = 0,	m[1][3] = 0,	m[2][3] = 0,	m[3][3] = 1;
	return *this;
}

Matrix4x4F & Matrix4x4F::MTranslation(Vector3F trans)
{
	m[0][0] = 1,		m[1][0] = 0,		m[2][0] = 0,		m[3][0] = 0;
	m[0][1] = 0,		m[1][1] = 1,		m[2][1] = 0,		m[3][1] = 0;
	m[0][2] = 0,		m[1][2] = 0,		m[2][2] = 1,		m[3][2] = 0;
	m[0][3] = trans.x,	m[1][3] = trans.y,	m[2][3] = trans.z,	m[3][3] = 1;
	return *this;
}

Matrix4x4F & Matrix4x4F::MTranslation(float x, float y, float z)
{
	m[0][0] = 1,	m[1][0] = 0,	m[2][0] = 0,	m[3][0] = 0;
	m[0][1] = 0,	m[1][1] = 1,	m[2][1] = 0,	m[3][1] = 0;
	m[0][2] = 0,	m[1][2] = 0,	m[2][2] = 1,	m[3][2] = 0;
	m[0][3] = x,	m[1][3] = y,	m[2][3] = z,	m[3][3] = 1;
	return *this;
}

Matrix4x4F & Matrix4x4F::MRotationXLH(float fTheta)
{ 
	m[0][0] = 1,	m[1][0] = 0,			m[2][0] = 0,			m[3][0] = 0;
	m[0][1] = 0,	m[1][1] = cosf(fTheta),	m[2][1] = sinf(fTheta),	m[3][1] = 0;
	m[0][2] = 0,	m[1][2] = -sinf(fTheta),	m[2][2] = cosf(fTheta),	m[3][2] = 0;
	m[0][3] = 0,	m[1][3] = 0,			m[2][3] = 0,			m[3][3] = 1;
	return *this;
}

Matrix4x4F & Matrix4x4F::MRotationYLH(float fTheta)
{
	m[0][0] = cosf(fTheta),	m[1][0] = 0,	m[2][0] = -sinf(fTheta),	m[3][0] = 0;
	m[0][1] = 0,			m[1][1] = 1,	m[2][1] = 0,			m[3][1] = 0;
	m[0][2] = sinf(fTheta),	m[1][2] = 0,	m[2][2] = cosf(fTheta),	m[3][2] = 0;
	m[0][3] = 0,			m[1][3] = 0,	m[2][3] = 0,			m[3][3] = 1;
	return *this;
}

Matrix4x4F & Matrix4x4F::MRotationZLH(float fTheta)
{
	m[0][0] = cosf(fTheta),	m[1][0] = sinf(fTheta),	m[2][0] = 0,	m[3][0] = 0;
	m[0][1] = -sinf(fTheta),	m[1][1] = cosf(fTheta),	m[2][1] = 0,	m[3][1] = 0;
	m[0][2] = 0,			m[1][2] = 0,			m[2][2] = 1,	m[3][2] = 0;
	m[0][3] = 0,			m[1][3] = 0,			m[2][3] = 0,	m[3][3] = 1;
	return *this;
}

Matrix4x4F & Matrix4x4F::MRotationAxisLH(const Vector3F & v, float fTheta)
{
	Vector3F vn(v);
	vn.Normalize();
	float c = cosf(fTheta);
	float s = sinf(fTheta);
	float t = 1.f - cosf(fTheta);

	m[0][0] = t*vn.x*vn.x + c,		m[1][0] = t*vn.x*vn.y - s*vn.z,	m[2][0] = t*vn.x*vn.z + s*vn.y,	m[3][0] = 0;
	m[0][1] = t*vn.x*vn.y + s*vn.z, m[1][1] = t*vn.y*vn.y + c,		m[2][1] = t*vn.y*vn.z - s*vn.x,	m[3][1] = 0;
	m[0][2] = t*vn.x*vn.z - s*vn.y,	m[1][2] = t*vn.y*vn.z + s*vn.x,	m[2][2] = t*vn.z*vn.z + c,		m[3][2] = 0;
	m[0][3] = 0,					m[1][3] = 0,					m[2][3] = 0,					m[3][3] = 1;
	return *this;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////Functions for Matrix4x4F/////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Finds the covariance and the mean of a list of vertices in R^3
void CovarianceVertices3x3F(Vector3F * pVert, UINT nVert, Matrix4x4F & cov, Vector3F & mean)
{
	Vector3F avg(0, 0, 0);
	for (UINT i=0; i<nVert; i++)
		avg += pVert[i];
	mean = (avg /= (float)nVert);

	float m02=0, m01=0, m12=0, m00=0, m11=0, m22=0;
	for (UINT i=0; i<nVert; i++) {
		Vector3F pos(pVert[i] - avg);

		m02 = pos.z*pos.x;
		m01 = pos.y*pos.x;
		m12 = pos.z*pos.y;
		m00 = pos.x*pos.x;
		m11 = pos.y*pos.y;
		m22 = pos.z*pos.z;
	}
    
	float mul = 1.f/(float)nVert;
	cov.MIdenity();
	cov.m[0][0] = mul*m00;
	cov.m[1][1] = mul*m11;
	cov.m[2][2] = mul*m22;
	cov.m[2][0] = cov.m[0][2] = mul*m02;
	cov.m[1][0] = cov.m[0][1] = mul*m01;
	cov.m[2][1] = cov.m[1][2] = mul*m12;
}

//Finds the covariance and the mean of a list of triangles in R^3 specified by their vertices
void CovarianceTriangles3x3F(Vector3F * pVert, UINT nVert, Matrix4x4F & cov, Vector3F & mean)
{
	if (nVert % 3) DxUtSendError("Covariance3x3F nVert must be divisible by 3.");

	UINT nTri = nVert/3;
	double sArea = 0;								//Surface area of all the triangles
	Vector3F tCtd(0, 0, 0);							//Triangle centroid or mean of the triangles
	double m02=0, m01=0, m12=0, m00=0, m11=0, m22=0;
	for (UINT i=0; i<nTri; i++) {
		STriangleF tri(pVert[3*i+0],  pVert[3*i+1],  pVert[3*i+2]);

		float area = tri.Area();		sArea += area;
		Vector3F ctd(tri.Centroid());	tCtd += area*ctd;

		Vector3F * v = tri.vPosW;
		m02 += area*(9.f*ctd.x*ctd.z +
			v[0].x*v[0].z + 
			v[1].x*v[1].z +
			v[2].x*v[2].z)/12.f;

		m01 += area*(9.f*ctd.x*ctd.y +
			v[0].x*v[0].y + 
			v[1].x*v[1].y +
			v[2].x*v[2].y)/12.f;

		m12 += area*(9.f*ctd.y*ctd.z +
			v[0].y*v[0].z + 
			v[1].y*v[1].z +
			v[2].y*v[2].z)/12.f;

		m00 += area*(9.f*ctd.x*ctd.x +
			v[0].x*v[0].x + 
			v[1].x*v[1].x +
			v[2].x*v[2].x)/12.f;

		m11 += area*(9.f*ctd.y*ctd.y +
			v[0].y*v[0].y + 
			v[1].y*v[1].y +
			v[2].y*v[2].y)/12.f;

		m22 += area*(9.f*ctd.z*ctd.z +
			v[0].z*v[0].z + 
			v[1].z*v[1].z +
			v[2].z*v[2].z)/12.f;
	}

	float mul = 1.f/sArea;
	mean = mul*tCtd;

	cov.MIdenity();
	cov.m[0][0] = (m00 - mul*tCtd.x*tCtd.x);
	cov.m[1][1] = (m11 - mul*tCtd.y*tCtd.y);
	cov.m[2][2] = (m22 - mul*tCtd.z*tCtd.z);
	cov.m[2][0] = cov.m[0][2] = (m02 - mul*tCtd.x*tCtd.z);
	cov.m[1][0] = cov.m[0][1] = (m01 - mul*tCtd.x*tCtd.y);
	cov.m[2][1] = cov.m[1][2] = (m12 - mul*tCtd.y*tCtd.z);
}

//Finds the covariance and the mean of a list of triangles in R^3
void CovarianceTriangles3x3F(STriangleF * pTri, UINT nTri, Matrix4x4F & cov, Vector3F & mean)
{
	float sArea = 0;								//Surface area of all the triangles
	Vector3F tCtd(0, 0, 0);							//Triangle centroid or mean of the triangles
	float m02=0, m01=0, m12=0, m00=0, m11=0, m22=0;
	for (UINT i=0, j=0; i<nTri; i++) {
		float area = pTri[i].Area();		sArea += area;
		Vector3F ctd(pTri[i].Centroid());	tCtd += area*ctd;

		Vector3F * v = pTri[i].vPosW;
		m02 += area*(9.f*ctd.x*ctd.z +
			v[0].x*v[0].z + 
			v[1].x*v[1].z +
			v[2].x*v[2].z)/12.f;

		m01 += area*(9.f*ctd.x*ctd.y +
			v[0].x*v[0].y + 
			v[1].x*v[1].y +
			v[2].x*v[2].y)/12.f;

		m12 += area*(9.f*ctd.y*ctd.z +
			v[0].y*v[0].z + 
			v[1].y*v[1].z +
			v[2].y*v[2].z)/12.f;

		m00 += area*(9.f*ctd.x*ctd.x +
			v[0].x*v[0].x + 
			v[1].x*v[1].x +
			v[2].x*v[2].x)/12.f;

		m11 += area*(9.f*ctd.y*ctd.y +
			v[0].y*v[0].y + 
			v[1].y*v[1].y +
			v[2].y*v[2].y)/12.f;

		m22 += area*(9.f*ctd.z*ctd.z +
			v[0].z*v[0].z + 
			v[1].z*v[1].z +
			v[2].z*v[2].z)/12.f;
	}
	float mul = 1.f/sArea;
	mean = mul*tCtd;

	cov.MIdenity();
	cov.m[0][0] = (m00 - mul*tCtd.x*tCtd.x);
	cov.m[1][1] = (m11 - mul*tCtd.y*tCtd.y);
	cov.m[2][2] = (m22 - mul*tCtd.z*tCtd.z);
	cov.m[2][0] = cov.m[0][2] = (m02 - mul*tCtd.x*tCtd.z);
	cov.m[1][0] = cov.m[0][1] = (m01 - mul*tCtd.x*tCtd.y);
	cov.m[2][1] = cov.m[1][2] = (m12 - mul*tCtd.y*tCtd.z);
}

void JacobiRotation3x3F(Matrix4x4F & A, WORD p, WORD q, DOUBLE & c, DOUBLE & s)
{
	if (abs(A.m[p][q]) > .000001) {
		double t = 0;
		double r = (A.m[q][q] - A.m[p][p])/(2.0*A.m[p][q]);

        if (r >= 0.0)
			t = 1.0/(r + sqrt(1.0 + r*r));
        else t = -1.0/(-r + sqrt(1.0 + r*r));

		c = 1.0/sqrt(1.0 + t*t);
		s = t*c;
    }
	else {
		c = 1.0;
		s = 0.0;
    }
}

void JacobiTransformation3x3F(Matrix4x4F & A, Matrix4x4F & eiM, Vector3F & eiVal, UINT maxIter)
{
	Matrix4x4F J, _A(A);
	eiM.MIdenity();

	double c=0, s=0;
    for (WORD i=0; i<maxIter; i++) {
		//Up triangle
		double up=0.0;
        for (WORD j=0; j<3; j++) {
            for (WORD k=j+1; k<3; k++) {
                up += _A.m[j][k];
            }
        }
        if (abs(up) <= .000001) 
			break;

		for (WORD p=0; p<3; p++) {
			for (WORD q=p+1; q<3; q++) {
				JacobiRotation3x3F(_A, p, q, c, s);

				J.MIdenity();
				J.m[p][p] = (float)c;	 J.m[p][q] = (float)s;
				J.m[q][p] = (float)(-s); J.m[q][q] = (float)c;

				eiM = eiM*J;
				_A = (J.Transpose()*_A)*J;
			}
		}
	}
	eiVal.x = _A.m[0][0];
	eiVal.y = _A.m[1][1];
	eiVal.z = _A.m[2][2];
}

//The column with the largest eiVal will be put in eiVec
void MaxEigenVectors3x3F(Matrix4x4F & eiM, Vector3F & eiVal, Vector3F & eiVec)
{
	WORD col = 0;
	float best = eiVal.x;
	if (eiVal.y > best) {best = eiVal.y; col = 1;}
	if (eiVal.z > best) {best = eiVal.z; col = 2;}

	eiVec.x = eiM.m[0][col];
	eiVec.y = eiM.m[1][col];
	eiVec.z = eiM.m[2][col];
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////       MatrixNxNF     ///////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MatrixNxNF::CreateMatrix(UINT nRows)
{
	Assert(!c, "MatrixNxNF::CreateMatrix must destroy the matrix before creating a new one.");

	c = new float[nRows*nRows];
	m_nRows = nRows;
}

MatrixNxNF & MatrixNxNF::MIdenity()
{
	for (UINT i=0; i<m_nRows; i++) {
		UINT col = i*m_nRows;
		for (UINT j=0; j<m_nRows; j++) {
			c[col+j] = i==j;
		}
	}
	return *this;
}

MatrixNxNF & MatrixNxNF::operator=(MatrixNxNF & A)
{
	Assert(m_nRows == A.m_nRows, "MatrixNxNF::operator= can not assign matrices of different sizes.");

	memcpy(this, &A, sizeof(float)*m_nRows);
	return *this;
}

MatrixNxNF & MatrixNxNF::operator+=(MatrixNxNF & A)
{
	Assert(m_nRows == A.m_nRows, "MatrixNxNF::operator+= can not add matrices of different sizes.");

	for (UINT i=0; i<m_nRows; i++) {
		UINT col = i*m_nRows;
		for (UINT j=0; j<m_nRows; j++) {
			c[col+j] += A.c[col+j];
		}
	}
	return *this;
}

MatrixNxNF & MatrixNxNF::operator-=(MatrixNxNF & A)
{
	Assert(m_nRows == A.m_nRows, "MatrixNxNF::operator-= can not subtract matrices of different sizes.");

	for (UINT i=0; i<m_nRows; i++) {
		UINT col = i*m_nRows;
		for (UINT j=0; j<m_nRows; j++) {
			c[col+j] -= A.c[col+j];
		}
	}
	return *this;
}

MatrixNxNF & MatrixNxNF::operator*=(float flt)
{
	float mul = flt;
	for (UINT i=0; i<m_nRows; i++) {
		UINT col = i*m_nRows;
		for (UINT j=0; j<m_nRows; j++) {
			c[col+j] *= mul;
		}
	}
	return *this;
}

void MatrixNxNF::DestroyMatrix()
{
	Assert(c, "MatrixNxNF::DestroyMatrix cannot destroy a matrix that has not been created.");

	delete[] c;
	c = NULL;
	m_nRows = 0;
}



};
