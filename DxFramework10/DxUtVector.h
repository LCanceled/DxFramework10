
#ifndef DXUTVECTOR_H
#define DXUTVECTOR_H

#include <d3dx10.h>

#include "DxUtError.h"
#include <iostream>

namespace DxUt {

#define DotXY(a, b)    ((a.x*b.x + a.y*b.y))
#define DotXYZ(a, b)   ((a.x*b.x + a.y*b.y + a.z*b.z))
#define CrossXY(a, b)  ((a.x*b.y - b.x*a.y))
#define CrossXYZ(a, b) (Vector3F(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x))

extern double g_fVectorEqualityEps;

class Vector2F  {
public:
	union {
		struct {
			float x;
			float y;
		};
		float c[2];
	};
public:
	Vector2F() {}
	Vector2F(float _s):x(_s),y(_s) {}
	Vector2F(float _x, float _y):x(_x), y(_y) {}
	//Vector2F(float * _c) {memcpy(c, _c, sizeof(float)*2);}
	//~Vector2F() {}

	Vector2F operator+() {return *this;}
	Vector2F operator-() {return Vector2F(-x, -y);}

	Vector2F operator+(const Vector2F & v) const {return Vector2F(x+v.x, y+v.y);}
	Vector2F operator-(const Vector2F & v) const {return Vector2F(x-v.x, y-v.y);}
	friend Vector2F operator*(float flt, const Vector2F & v) {return Vector2F(v.x*flt, v.y*flt);}
	//Vector2F operator*(float flt) {return Vector2F(x*flt, y*flt);}
	Vector2F operator*(const Vector2F & v) const {return Vector2F(x*v.x, y*v.y);}
	Vector2F operator/(const Vector2F & v) const {return Vector2F(x/v.x, y/v.y);}
	Vector2F operator/(float flt) {float d = 1.f/flt; return Vector2F(x*d, y*d);}

	Vector2F & operator=(const Vector2F & v) {x = v.x, y = v.y; return *this;}
	Vector2F & operator+=(const Vector2F & v) {x += v.x, y += v.y; return *this;}
	Vector2F & operator-=(const Vector2F & v) {x -= v.x, y -= v.y; return *this;}
	Vector2F & operator*=(float flt) {x *= flt, y *= flt; return *this;}
	Vector2F & operator/=(float flt) {float d = 1.f/flt; x *= d, y *= d; return *this;}

	//void operator()(float _x, float _y, float _z) {x = _x, y = _y;}
	//void operator()(float * _c) {memcpy(c, _c, sizeof(float)*2);}

	bool operator==(const Vector2F & v) const {
		return abs(v.x - x) < g_fVectorEqualityEps && abs(v.y - y) < g_fVectorEqualityEps;
	}
	bool operator!=(const Vector2F & v) const {
		return !(abs(v.x - x) < g_fVectorEqualityEps && abs(v.y - y) < g_fVectorEqualityEps);
	}
	friend bool operator<(const Vector2F & v1, const Vector2F & v2) {
		return ((Vector2F)v1).LengthSq() < ((Vector2F)v2).LengthSq();
	}

	friend std::ostream & operator << (std::ostream & out, const Vector2F & pt) {
		out << pt.x << ' ' << pt.y;
		return out;
	}

	float Length() {return sqrtf(x*x + y*y);}
	float LengthSq() {return x*x + y*y;}
	Vector2F Normalize();
};

class Matrix4x4F;

class Vector3F {
public:
	union {
		struct {
			float x;
			float y;
			float z;
		};
		float c[3];
	};
public:
	Vector3F() {}
	Vector3F(float _s):x(_s),y(_s),z(_s) {}
	//Vector3F(Vector3F & copy) {memcpy(c, copy.c, sizeof(float)*3);}
	Vector3F(float _x, float _y, float _z):x(_x), y(_y), z(_z) {}
	//Vector3F(float * _c) {memcpy(c, _c, sizeof(float)*3);}
	//~Vector3F() {}

	Vector3F operator+() const {return *this;}
	Vector3F operator-() const {return Vector3F(-x, -y, -z);}

	Vector3F operator+(const Vector3F & v) const {return Vector3F(x+v.x, y+v.y, z+v.z);}
	friend Vector3F operator-(const Vector3F & v1, const Vector3F & v2) {return Vector3F(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z); }//Vector3F(x-v.x, y-v.y, z-v.z);}
	friend Vector3F operator*(float flt, const Vector3F & v) {return Vector3F(v.x*flt, v.y*flt, v.z*flt);}
	//Vector3F operator*(float flt) {return Vector3F(x*flt, y*flt, z*flt);}
	Vector3F operator*(const Vector3F & v) const {return Vector3F(x*v.x, y*v.y, z*v.z);}
	Vector3F operator/(const Vector3F & v) const {return Vector3F(x/v.x, y/v.y, z/v.z);}
	Vector3F operator/(float flt) {float d = 1.f/flt; return Vector3F(x*d, y*d, z*d);}

	Vector3F & operator=(const Vector3F & v) {x = v.x, y = v.y, z = v.z; return *this;}
	Vector3F & operator+=(const Vector3F & v) {x += v.x, y += v.y, z += v.z; return *this;}
	Vector3F & operator-=(const Vector3F & v) {x -= v.x, y -= v.y, z -= v.z; return *this;}
	Vector3F & operator*=(float flt) {x *= flt, y *= flt, z *= flt; return *this;}
	Vector3F & operator/=(float flt) {float d = 1.f/flt; x *= d, y *= d, z *= d; return *this;}

	//void operator()(float _x, float _y, float _z) {x = _x, y = _y, z = _z;}
	//void operator()(float * _c) {memcpy(c, _c, sizeof(float)*3);}

	//v1^v2 should always be enclosed in parentheses (v1^v2)
	//Vector3F operator^(Vector3F & v) {return Vector3F(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x);}
	float Length() {return sqrtf(x*x + y*y + z*z);}
	float LengthSq() {return x*x + y*y + z*z;}

	Vector3F Normalize();
	//Vector3F NormalizeSq();
	Matrix4x4F SkewMatrix3x3F();
	friend Vector3F operator*(const Matrix4x4F & A, const Vector3F & v);
	friend Vector3F operator*(const Vector3F & v, const Matrix4x4F & A);

	/* The vector is assumed to be a normal in this case */
	Vector3F MulNormal(const Matrix4x4F & A, const Vector3F & v);
	Vector3F MulNormal(const Vector3F & v, const Matrix4x4F & A);

	bool operator==(const Vector3F & v) const {
		return abs(v.x - x) < g_fVectorEqualityEps && abs(v.y - y) < g_fVectorEqualityEps && abs(v.z - z) < g_fVectorEqualityEps;
	}
	bool operator!=(const Vector3F & v) const {
		return !(abs(v.x - x) < g_fVectorEqualityEps && abs(v.y - y) < g_fVectorEqualityEps && abs(v.z - z) < g_fVectorEqualityEps);
	}
	//friend bool operator<(const Vector3F & v1, const Vector3F & v2) {
	//	return 0;//((Vector3F)v1).LengthSq() < ((Vector3F)v2).LengthSq();
	//}

	friend std::ostream & operator << (std::ostream & out, const Vector3F & pt) {
		out << pt.x << ' ' << pt.y << ' ' << pt.z;
		return out;
	}

	float operator[](int iIndex) {return c[iIndex];}
};

__forceinline Vector3F operator*(const Matrix4x4F & A, const Vector3F & v)
{
	float * m = (float*)&A;
	return Vector3F(
		m[0]*v.x + m[1]*v.y + m[2]*v.z + m[3],
		m[4]*v.x + m[5]*v.y + m[6]*v.z + m[7],
		m[8]*v.x + m[9]*v.y + m[10]*v.z + m[11]);
}

__forceinline Vector3F operator*(const Vector3F & v, const Matrix4x4F & A)
{
	float * m = (float*)&A;
	return Vector3F(
		v.x*m[0] + v.y*m[1] + v.z*m[2] + m[3],
		v.x*m[4] + v.y*m[5] + v.z*m[6] + m[7],
		v.x*m[8] + v.y*m[9] + v.z*m[10] + m[11]);
}

__forceinline Vector3F Vector3F::MulNormal(const Matrix4x4F & A, const Vector3F & v)
{
	float * m = (float*)&A;
	return Vector3F(
		m[0]*v.x + m[1]*v.y + m[2]*v.z,
		m[4]*v.x + m[5]*v.y + m[6]*v.z,
		m[8]*v.x + m[9]*v.y + m[10]*v.z);
}

__forceinline Vector3F Vector3F::MulNormal(const Vector3F & v, const Matrix4x4F & A)
{
	float * m = (float*)&A;
	return Vector3F(
		v.x*m[0] + v.y*m[1] + v.z*m[2],
		v.x*m[4] + v.y*m[5] + v.z*m[6],
		v.x*m[8] + v.y*m[9] + v.z*m[10]);
}
/*
class Vector3D {
public:
	union {
		struct {
			double x;
			double y;
			double z;
		};
		double c[3];
	};
public:
	Vector3D() {}
	Vector3D(double _s):x(_s),y(_s),z(_s) {}
	//Vector3D(Vector3D & copy) {memcpy(c, copy.c, sizeof(float)*3);}
	Vector3D(double _x, double _y, double _z):x(_x), y(_y), z(_z) {}
	Vector3D(Vector3F & v) {x = v.x, y = v.y, z = v.z; }
	//Vector3D(float * _c) {memcpy(c, _c, sizeof(float)*3);}
	//~Vector3D() {}

	Vector3D operator+() {return *this;}
	Vector3D operator-() {return Vector3D(-x, -y, -z);}

	Vector3D operator+(Vector3D & v) {return Vector3D(x+v.x, y+v.y, z+v.z);}
	Vector3D operator-(Vector3D & v) {return Vector3D(x-v.x, y-v.y, z-v.z);}
	friend Vector3D operator*(double flt, Vector3D & v) {return Vector3D(v.x*flt, v.y*flt, v.z*flt);}
	//Vector3D operator*(float flt) {return Vector3D(x*flt, y*flt, z*flt);}
	Vector3D operator*(Vector3D & v) {return Vector3D(x*v.x, y*v.y, z*v.z);}
	Vector3D operator/(Vector3D & v) {return Vector3D(x/v.x, y/v.y, z/v.z);}
	Vector3D operator/(double flt) {double d = 1.f/flt; return Vector3D(x*d, y*d, z*d);}

	Vector3D & operator=(Vector3D & v) {x = v.x, y = v.y, z = v.z; return *this;}
	Vector3D & operator+=(Vector3D & v) {x += v.x, y += v.y, z += v.z; return *this;}
	Vector3D & operator-=(Vector3D & v) {x -= v.x, y -= v.y, z -= v.z; return *this;}
	Vector3D & operator*=(double flt) {x *= flt, y *= flt, z *= flt; return *this;}
	Vector3D & operator/=(double flt) {double d = 1.f/flt; x *= d, y *= d, z *= d; return *this;}

	//void operator()(float _x, float _y, float _z) {x = _x, y = _y, z = _z;}
	//void operator()(float * _c) {memcpy(c, _c, sizeof(float)*3);}

	//v1^v2 should always be enclosed in parentheses (v1^v2)
	//Vector3D operator^(Vector3D & v) {return Vector3D(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x);}
	float Length() {return sqrtf(x*x + y*y + z*z);}
	float LengthSq() {return x*x + y*y + z*z;}

	float operator[](int iIndex) {return c[iIndex];}
};*/

class Vector4F  {
public:
	union {
		struct {
			float x;
			float y;
			float z;
			float w;
		};
		float c[4];
	};
public:
	Vector4F() {}
	Vector4F(float _s):x(_s),y(_s),z(_s),w(_s) {}
	//Vector4F(Vector4F & copy) {memcpy(c, copy.c, sizeof(float)*4);}
	Vector4F(float _x, float _y, float _z, float _w):x(_x), y(_y), z(_z), w(_w) {}
	//Vector4F(float * _c) {memcpy(c, _c, sizeof(float)*4);}
	//~Vector4F() {}

	Vector4F operator+() const {return *this;}
	Vector4F operator-() const {return Vector4F(-x, -y, -z, -w);}

	Vector4F operator+(const Vector4F & v) const {return Vector4F(x+v.x, y+v.y, z+v.z, w+v.w);}
	Vector4F operator-(const Vector4F & v) const {return Vector4F(x-v.x, y-v.y, z-v.z, w-v.w);}
	friend Vector4F operator*(float flt, const Vector4F & v) {return Vector4F(v.x*flt, v.y*flt, v.z*flt, v.w*flt);}
	//Vector4F operator*(float flt) {return Vector4F(x*flt, y*flt, z*flt, w*flt);}
	Vector4F operator*(const Vector4F & v) const {return Vector4F(x*v.x, y*v.y, z*v.z, w*v.w);}
	Vector4F operator/(const Vector4F & v) const {return Vector4F(x/v.x, y/v.y, z/v.z, w/v.w);}
	Vector4F operator/(float flt) {float d = 1.f/flt; return Vector4F(x*d, y*d, z*d, w*d);}

	Vector4F & operator=(const Vector4F & v) {x = v.x, y = v.y, z = v.z, w = v.w; return *this;}
	Vector4F & operator+=(const Vector4F & v) {x += v.x, y += v.y, z += v.z, w += v.w; return *this;}
	Vector4F & operator-=(const Vector4F & v) {x -= v.x, y -= v.y, z -= v.z, w -= v.w; return *this;}
	Vector4F & operator*=(float flt) {x *= flt, y *= flt, z *= flt, w *= flt; return *this;}
	Vector4F & operator/=(float flt) {float d = 1.f/flt; x *= d, y *= d, z *= d, w *= d; return *this;}

	//void operator()(float _x, float _y, float _z, float _w) {x = _x, y = _y, z = _z, w = _w;}
	//void operator()(float * _c) {memcpy(c, _c, sizeof(float)*4);}

	bool operator==(const Vector4F & v) const {
		return abs(v.x - x) < g_fVectorEqualityEps && abs(v.y - y) < g_fVectorEqualityEps 
			&& abs(v.z - z) < g_fVectorEqualityEps  && abs(v.w - w) < g_fVectorEqualityEps;
	}
	friend bool operator<(const Vector4F & v1, const Vector4F & v2) {
		return ((Vector4F)v1).LengthSq() < ((Vector4F)v2).LengthSq();
	}

	float Length() {return sqrtf(x*x + y*y + z*z + w*w);}
	float LengthSq() {return x*x + y*y + z*z + w*w;}

	Vector4F Normalize();
	friend Vector4F operator*(const Matrix4x4F & A, const Vector4F & v);
	friend Vector4F operator*(const Vector4F & v, const Matrix4x4F & A);
};
/*
inline Vector4F operator*(Matrix4x4F & A, Vector4F & v)
{
	return Vector4F(
		A.m[0][0]*v.x + A.m[0][1]*v.y + A.m[0][2]*v.z + A.m[0][3]*v.w,
		A.m[1][0]*v.x + A.m[1][1]*v.y + A.m[1][2]*v.z + A.m[1][3]*v.w,
		A.m[2][0]*v.x + A.m[2][1]*v.y + A.m[2][2]*v.z + A.m[2][3]*v.w,
		A.m[3][0]*v.x + A.m[3][1]*v.y + A.m[3][2]*v.z + A.m[3][3]*v.w);
}

inline Vector4F operator*(Vector4F & v, Matrix4x4F & A)
{
	return Vector4F(
		v.x*A.m[0][0] + v.y*A.m[1][0] + v.z*A.m[2][0] + v.w*A.m[3][0],
		v.x*A.m[0][1] + v.y*A.m[1][1] + v.z*A.m[2][1] + v.w*A.m[3][1],
		v.x*A.m[0][2] + v.y*A.m[1][2] + v.z*A.m[2][2] + v.w*A.m[3][2],
		v.x*A.m[0][3] + v.y*A.m[1][3] + v.z*A.m[2][3] + v.w*A.m[3][3]);
}*/

class VectorNF  {
public:
	float * c;
private:
	UINT m_nElements;
public:
	VectorNF():c(0), m_nElements(0) {}
	//~VectorNF() {}

	void CreateVector(UINT nElements);
	UINT Size() {return m_nElements; }

	VectorNF & operator=(VectorNF & v);
	VectorNF & operator+=(VectorNF & v);
	VectorNF & operator-=(VectorNF & v);
	friend VectorNF & operator*=(VectorNF & v, float flt);
	VectorNF & operator/=(float flt);

	void Set(float flt);
	float Length();
	float LengthSq();

	void DestroyVector();
};

inline VectorNF & VectorNF::operator=(VectorNF & v) 
{
	Assert(m_nElements == v.m_nElements, "VectorNF::operator= can not have vectors of different sizes.");

	memcpy(this->c, v.c, sizeof(float)*m_nElements);
	return *this;
}

inline VectorNF & VectorNF::operator+=(VectorNF & v) 
{
	Assert(m_nElements == v.m_nElements, "VectorNF::operator+= can not add vectors of different sizes.");

	for (UINT i=0; i<m_nElements; i++)
		c[i] += v.c[i];
	return *this;
}

inline VectorNF & VectorNF::operator-=(VectorNF & v) 
{
	Assert(m_nElements == v.m_nElements, "VectorNF::operator-= can not subtract vectors of different sizes.");

	for (UINT i=0; i<m_nElements; i++)
		c[i] -= v.c[i];
	return *this;
}

inline VectorNF & operator*=(VectorNF & v, float flt)
{
	float * c = v.c;
	for (UINT i=0, vSize = v.m_nElements; i<vSize; i++)
		c[i] *= flt;
	return v;
}

inline VectorNF & VectorNF::operator/=(float flt) 
{
	float mul = 1.f/flt;
	for (UINT i=0; i<m_nElements; i++)
		c[i] *= mul;
	return *this;
}

inline void VectorNF::Set(float flt)
{
	for (UINT i=0; i<m_nElements; i++) c[i] = flt;
}


};


#endif
