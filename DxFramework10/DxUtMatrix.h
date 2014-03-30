
#ifndef DXUTMATRTIX_H
#define DXUTMATRIX_H

#include <d3dx10.h>

#include "DxUtError.h"

#include "DxUtVector.h"

namespace DxUt {

//Matrix4x4F assumes that its columns are vectors
//not its rows. All its member functions and all
//other functions and class assume this except for 
//the camera class which assumes row vectors. Note
//also that D3D10 uses a left handed coordinate
//system in which matrices are row vector form.

class Vector3F;
struct STriangleF;

class Matrix4x4F  {
public:
	float m[4][4];
public:
	Matrix4x4F() {}
	//Matrix4x4F(Matrix4x4F & copy) {
	//	memcpy(m, copy.m, sizeof(float)*16);}
	Matrix4x4F(float m11, float m21, float m31, float m41,
		float m12, float m22, float m32, float m42,
		float m13, float m23, float m33, float m43,
		float m14, float m24, float m34, float m44);
	Matrix4x4F(float diag);
	Matrix4x4F(float * _c) {memcpy(m, _c, sizeof(float)*16);}
	//Matrix4x4F(float _c[4][4]) {memcpy(m, _c, sizeof(float)*16);}
	//~Matrix4x4F() {}

	Matrix4x4F operator+() {return *this;}
	Matrix4x4F operator-() {return (-1.f)*(*this);}

	Matrix4x4F operator+(Matrix4x4F & A);
	Matrix4x4F operator-(Matrix4x4F & A);
	Matrix4x4F operator*(Matrix4x4F & A);
	friend Matrix4x4F operator*(float flt, Matrix4x4F & A);
	Matrix4x4F operator*(float flt);

	Matrix4x4F & operator=(Matrix4x4F & A);
	Matrix4x4F & operator+=(Matrix4x4F & A);
	Matrix4x4F & operator-=(Matrix4x4F & A);
	Matrix4x4F & operator*=(Matrix4x4F & A);
	Matrix4x4F & operator*=(float flt);

	Matrix4x4F Inverse();
	Matrix4x4F Transpose();
	Matrix4x4F InverseTranspose();

	/* Each operation applies to myself */
	Matrix4x4F & MZero();
	Matrix4x4F & MIdenity();
	Matrix4x4F & MInverse();
	Matrix4x4F & MTranspose();
	Matrix4x4F & MInverseTranspose();
	Matrix4x4F & MScaling(Vector3F & scl);
	Matrix4x4F & MScaling(float x, float y, float z);
	Matrix4x4F & MTranslation(Vector3F trans);
	Matrix4x4F & MTranslation(float x, float y, float z);
	Matrix4x4F & MRotationXLH(float fTheta);
	Matrix4x4F & MRotationYLH(float fTheta);
	Matrix4x4F & MRotationZLH(float fTheta);
	Matrix4x4F & MRotationAxisLH(Vector3F & v, float fTheta);

	Vector3F GetColumnVec3F(UINT c) {return Vector3F(m[0][c], m[1][c], m[2][c]); }
	Vector3F GetRowVec3F(UINT c) {return Vector3F(m[c][0], m[c][1], m[c][2]); }
};

inline Matrix4x4F::Matrix4x4F(
	float m11, float m21, float m31, float m41,
	float m12, float m22, float m32, float m42,
	float m13, float m23, float m33, float m43,
	float m14, float m24, float m34, float m44)
{
	m[0][0] = m11, m[1][0] = m21, m[2][0] = m31, m[3][0] = m41;
	m[0][1] = m12, m[1][1] = m22, m[2][1] = m32, m[3][1] = m42;
	m[0][2] = m13, m[1][2] = m23, m[2][2] = m33, m[3][2] = m43;
	m[0][3] = m14, m[1][3] = m24, m[2][3] = m34, m[3][3] = m44;
}

inline Matrix4x4F::Matrix4x4F(float diag)
{
	m[0][0] = diag, m[1][0] = 0, m[2][0] = 0, m[3][0] = 0;
	m[0][1] = 0, m[1][1] = diag, m[2][1] = 0, m[3][1] = 0;
	m[0][2] = 0, m[1][2] = 0, m[2][2] = diag, m[3][2] = 0;
	m[0][3] = 0, m[1][3] = 0, m[2][3] = 0, m[3][3] = diag;
}

inline Matrix4x4F & Matrix4x4F::operator=(Matrix4x4F & A)
{
	memcpy(this, &A, sizeof(float)*4*4);
	return *this;
}

inline Matrix4x4F & Matrix4x4F::MZero()
{
	m[0][0] = m[1][0] = m[2][0] = m[3][0] = 0;
	m[0][1] = m[1][1] = m[2][1] = m[3][1] = 0;
	m[0][2] = m[1][2] = m[2][2] = m[3][2] = 0;
	m[0][3] = m[1][3] = m[2][3] = m[3][3] = 0;
	return *this;
}

inline Matrix4x4F & Matrix4x4F::MIdenity()
{
	m[0][0] = 1.f; m[1][0] = m[2][0] = m[3][0] = 0;
	m[1][1] = 1.f; m[0][1] = m[2][1] = m[3][1] = 0;
	m[2][2] = 1.f; m[0][2] = m[1][2] = m[3][2] = 0;
	m[3][3] = 1.f; m[0][3] = m[1][3] = m[2][3] = 0;
	return *this;
}

inline Matrix4x4F _Idenity4x4F()
{
	return Matrix4x4F(
		1.f, 0, 0, 0,
		0, 1.f, 0, 0,
		0, 0, 1.f, 0,
		0, 0, 0, 1.f);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////Functions for Matrix4x4F/////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
//The matrices in the functions are always in the form meaning
//  c11 c12 c13 0
//  c21 c22 c23 0
//	c31 c32 c33 0
//  0   0   0   1
//that the functions treat the matrices as if they were 3x3 ones.
/////////////////////////////////////////////////////////////////

//Finds the covariance and the mean of a list of vertices in R^3
void CovarianceVertices3x3F(Vector3F * pVert, UINT nVert, Matrix4x4F & cov, Vector3F & mean);

//Finds the covariance and the mean of a list of triangles in R^3
void CovarianceTriangles3x3F(STriangleF * pTri, UINT nTri, Matrix4x4F & cov, Vector3F & mean);

//Finds the covariance and the mean of a list of triangles in R^3 specified by their vertices
void CovarianceTriangles3x3F(Vector3F * pVert, UINT nVert, Matrix4x4F & cov, Vector3F & mean);

void JacobiTransformation3x3F(Matrix4x4F & A, Matrix4x4F & eiM, Vector3F & eiVal, UINT maxIter=40);
//The column with the largest eiVal will be put in eiVec
void MaxEigenVectors3x3F(Matrix4x4F & eiM, Vector3F & eiVal, Vector3F & eiVec);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////// MatrixNxNF /////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class MatrixNxNF  {
public:
	float * c;
private:
	UINT m_nRows;
public:
	MatrixNxNF():c(0), m_nRows(0) {}
	//~MatrixNxNF() {}

	void CreateMatrix(UINT nRows);
	UINT Size() {return m_nRows;}

	MatrixNxNF & operator=(MatrixNxNF & A);
	MatrixNxNF & operator+=(MatrixNxNF & A);
	MatrixNxNF & operator-=(MatrixNxNF & A);
	MatrixNxNF & operator*=(float flt);

	MatrixNxNF & MIdenity();

	void DestroyMatrix();
};


};


#endif
