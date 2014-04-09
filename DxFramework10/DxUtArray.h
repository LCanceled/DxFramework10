
#ifndef DXUTARRAY_H
#define DXUTARRAY_H

#include <windows.h>
#include "DxUtError.h"
#include <vector>

namespace DxUt {

/* For the an array of size n, the new capacity of the array is computed by uiGrowLinear + n*uiGrowPower for PushBack */
template <typename T, UINT uiGrowLinear=1, UINT uiGrowPower=0>
class CArray {
private:
	UINT m_uiCapacity;
	UINT m_uiSize;
	T * m_Entries;

	void p_ResizeArray(__int64 liCapacity);
public:
	CArray():m_uiCapacity(0), m_uiSize(0), m_Entries(0)  {
		Assert(uiGrowLinear, "CArray::CArray a nonzero growCoefficient must be given."); }
	CArray(CArray & cpy) {Assert(0, "CArray::CArray a copy constructor is not defined"); }
	~CArray() {Clear(); }

	/* Increase the capacity of the array by n */
	void Reserve(UINT n);
	/* Increase or decrease the size of the array to n */
	void Resize(UINT n);
	void PushBack(const T & element); 
	void PushBack(void);
	/* Shifts the elements one over, creating an empty element at 0 */
	void PushBackShift();
	/* Decrements the size of the array by one */
	/* The capacity of the array remains the same */
	void PopBackSize();

	/* Clears the array and shares the memory of the array with pMem. The capacity */
	/* of the array remains fixed at nBytes. Allocations beyond result in an error. */
	//void ReserveSharedMemory(void * pMem, UINT nBytes=-1);

	/* Reverse array */
	void Reverse();

	void PushForward(T & element) {
		DebugBreak();
		//m_Entries.insert(m_Entries.begin(), element); 
	}

	/* Standard linear search through the array */
	bool FindIfEquals(T & element, UINT & uiEntry);

	UINT GetSize() {return m_uiSize;}
	UINT GetCapacity() {return m_uiCapacity;}
	T * GetData() {return m_Entries;}
	T & GetBack() {return m_Entries[m_uiSize-1];}

	void SetSize(UINT uiSize);

	//T & operator[](UINT uiI) {return m_Entries[uiI]; }
	T & operator[](UINT uiI) {
		Assert(uiI >= 0 && uiI < m_uiSize, "CArray::operator[] index out of range.");
		return m_Entries[uiI]; 
	}
	CArray & operator=(CArray & array) {
		DxUtSendError("CArray::operator= not supported.");
	}

	void Copy(CArray & toCopy);

	void Clear();
};
/*
template <typename T, UINT uiGrowLinear, UINT uiGrowPower>
inline void CArray<T, uiGrowLinear, uiGrowPower>::ReserveSharedMemory(void * pMem, UINT nBytes)
{
	DebugBreak();
	Clear();

	m_Entries = pMem;
	m_uiCapacity = nBytes;
	m_uiSize = 0;
}*/
/*
template <typename T, UINT uiGrowLinear, UINT uiGrowPower>
inline void CArray<T, uiGrowLinear, uiGrowPower>::Reverse()
{
	for (UINT i=0, end=m_uiSize/2; i<end; i++) {
		m_Entries.reverse
	}
}*/

template <typename T, UINT uiGrowLinear, UINT uiGrowPower>
inline void CArray<T, uiGrowLinear, uiGrowPower>::Reserve(UINT n)
{
	if (m_uiCapacity < n)
		p_ResizeArray(n);
}

template <typename T, UINT uiGrowLinear, UINT uiGrowPower>
inline void CArray<T, uiGrowLinear, uiGrowPower>::Resize(UINT n)
{
	if ((int)n - (int)m_uiCapacity > 0)
		p_ResizeArray(n);
	m_uiSize = n;
}

template <typename T, UINT uiGrowLinear, UINT uiGrowPower>
inline void CArray<T, uiGrowLinear, uiGrowPower>::PushBack(const T & element) 
{
	if (m_uiSize >= m_uiCapacity) {
		p_ResizeArray(uiGrowLinear + m_uiSize*(uiGrowPower + 1));
	}

	memcpy(&m_Entries[m_uiSize], &element, sizeof(T)); 
	m_uiSize++; 
}

template <typename T, UINT uiGrowLinear, UINT uiGrowPower>
inline void CArray<T, uiGrowLinear, uiGrowPower>::PushBack(void)
{
	if (m_uiSize >= m_uiCapacity) { 
		p_ResizeArray(uiGrowLinear + m_uiSize*(uiGrowPower + 1));
	}

	m_uiSize++; 
}

//Does not erases the last element and decremets the size
template <typename T, UINT uiGrowLinear, UINT uiGrowPower>
inline void CArray<T, uiGrowLinear, uiGrowPower>::PopBackSize() 
{
	Assert(m_uiSize, "CArray::PopBackSize cannot pop back when array is of size 0."); 

	m_uiSize--;
}

/*template <typename T, UINT uiGrowLinear, UINT uiGrowPower>
inline void CArray<T, uiGrowLinear, uiGrowPower>::PushBackShift()
{
	std::vector<T>::iterator iter = m_Entries.begin();
	m_Entries.insert(iter, T());
}*/

template <typename T, UINT uiGrowLinear, UINT uiGrowPower>
inline bool CArray<T, uiGrowLinear, uiGrowPower>::FindIfEquals(T & element, UINT & uiEntry) 
{
	for (UINT i=0; i<m_uiSize; i++) {
		if (m_Entries[i] == element) {
			uiEntry = i;
			return 1;
		}
	}
	return 0;
}

template <typename T, UINT uiGrowLinear, UINT uiGrowPower>
inline void CArray<T, uiGrowLinear, uiGrowPower>::SetSize(UINT uiSize)
{
	Assert(dwSize <= m_uiCapacity, "CArray::SetSize dwSize is greater than capacity.");

	m_uiSize = dwSize;
}

template <typename T, UINT uiGrowLinear, UINT uiGrowPower>
inline void CArray<T, uiGrowLinear, uiGrowPower>::p_ResizeArray(__int64 liCapacity)
{
	T * ar = new T[(UINT)liCapacity];
	Assert(ar, "CArray::p_ResizeArray out of memory.");

	memcpy(ar, m_Entries, min(m_uiSize, (UINT)liCapacity)*sizeof(T));
	if (m_Entries) delete[] m_Entries;

	m_Entries = ar;
	m_uiCapacity = (UINT)liCapacity;
}

template <typename T, UINT uiGrowLinear, UINT uiGrowPower>
inline void CArray<T, uiGrowLinear, uiGrowPower>::Copy(CArray & toCopy) 
{
	Assert(this->m_Entries != toCopy.m_Entries, "CArray::Copy cannot copy oneself.");

	Clear();
	Reserve(toCopy.m_uiCapacity);
	Resize(toCopy.m_uiSize);
	memcpy(m_Entries, toCopy.m_Entries, sizeof(T)*m_uiSize);
}

template <typename T, UINT uiGrowLinear, UINT uiGrowPower>
inline void CArray<T, uiGrowLinear, uiGrowPower>::Clear() 
{
	if (m_Entries) {
		delete[] m_Entries; m_Entries = 0;
		m_uiSize = 0;
		m_uiCapacity = 0;
	}
}


};


#endif

