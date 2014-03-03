
#ifndef DXUTARRAY_H
#define DXUTARRAY_H

#include <windows.h>
#include "DxUtError.h"
#include <vector>

namespace DxUt {

/* For the an array of size n, the new capacity of the array is computed by dwGrowLinear + n*dwGrowPower for PushBack */
template <typename T, DWORD dwGrowLinear=1, DWORD dwGrowPower=0>
class CArray {
private:
	DWORD m_dwCapacity;
	DWORD m_dwSize;
	T * m_Entries;

	void p_ResizeArray(DWORD64 capacity);
public:
	CArray():m_dwCapacity(0), m_dwSize(0), m_Entries(0)  {
		Assert(dwGrowLinear, "CArray::CArray a nonzero growCoefficient must be given."); }
	CArray(CArray & cpy) {Assert(0, "CArray::CArray a copy constructor is not defined"); }
	~CArray() {Clear(); }

	/* Increase the capacity of the array by n */
	void Reserve(DWORD n);
	/* Increase or decrease the size of the array to n */
	void Resize(DWORD n);
	void PushBack(const T & element); 
	void PushBack(void);
	/* Shifts the elements one over, creating an empty element at 0 */
	void PushBackShift();
	/* Decrements the size of the array by one */
	/* The capacity of the array remains the same */
	void PopBackSize();

	/* Clears the array and shares the memory of the array with pMem. The capacity */
	/* of the array remains fixed at nBytes. Allocations beyond result in an error. */
	//void ReserveSharedMemory(void * pMem, DWORD nBytes=-1);

	/* Reverse array */
	void Reverse();

	void PushForward(T & element) {
		DebugBreak();
		//m_Entries.insert(m_Entries.begin(), element); 
	}

	/* Standard linear search through the array */
	bool FindIfEquals(T & element, DWORD & dwEntry);

	DWORD GetSize() {return m_dwSize;}
	DWORD GetCapacity() {return m_dwCapacity;}
	T * GetData() {return m_Entries;}
	T & GetBack() {return m_Entries[m_dwSize-1];}

	void SetSize(DWORD dwSize);

	//T & operator[](DWORD dwI) {return m_Entries[dwI]; }
	T & operator[](DWORD dwI) {
		Assert(dwI >= 0 && dwI < m_dwSize, "CArray::operator[] index out of range.");
		return m_Entries[dwI]; 
	}
	CArray & operator=(CArray & array) {
		DxUtSendError("CArray::operator= not supported.");
	}

	void Copy(CArray & toCopy);

	void Clear();
};
/*
template <typename T, DWORD dwGrowLinear, DWORD dwGrowPower>
inline void CArray<T, dwGrowLinear, dwGrowPower>::ReserveSharedMemory(void * pMem, DWORD nBytes)
{
	DebugBreak();
	Clear();

	m_Entries = pMem;
	m_dwCapacity = nBytes;
	m_dwSize = 0;
}*/
/*
template <typename T, DWORD dwGrowLinear, DWORD dwGrowPower>
inline void CArray<T, dwGrowLinear, dwGrowPower>::Reverse()
{
	for (DWORD i=0, end=m_dwSize/2; i<end; i++) {
		m_Entries.reverse
	}
}*/

template <typename T, DWORD dwGrowLinear, DWORD dwGrowPower>
inline void CArray<T, dwGrowLinear, dwGrowPower>::Reserve(DWORD n)
{
	if (m_dwCapacity < n)
		p_ResizeArray(n);
}

template <typename T, DWORD dwGrowLinear, DWORD dwGrowPower>
inline void CArray<T, dwGrowLinear, dwGrowPower>::Resize(DWORD n)
{
	if ((int)n - (int)m_dwCapacity > 0)
		p_ResizeArray(n);
	m_dwSize = n;
}

template <typename T, DWORD dwGrowLinear, DWORD dwGrowPower>
inline void CArray<T, dwGrowLinear, dwGrowPower>::PushBack(const T & element) 
{
	if (m_dwSize >= m_dwCapacity) {
		p_ResizeArray(dwGrowLinear + m_dwSize*(dwGrowPower + 1));
	}

	memcpy(&m_Entries[m_dwSize], &element, sizeof(T)); 
	m_dwSize++; 
}

template <typename T, DWORD dwGrowLinear, DWORD dwGrowPower>
inline void CArray<T, dwGrowLinear, dwGrowPower>::PushBack(void)
{
	if (m_dwSize >= m_dwCapacity) { 
		p_ResizeArray(dwGrowLinear + m_dwSize*(dwGrowPower + 1));
	}

	m_dwSize++; 
}

//Does not erases the last element and decremets the size
template <typename T, DWORD dwGrowLinear, DWORD dwGrowPower>
inline void CArray<T, dwGrowLinear, dwGrowPower>::PopBackSize() 
{
	Assert(m_dwSize, "CArray::PopBackSize cannot pop back when array is of size 0."); 

	m_dwSize--;
}

/*template <typename T, DWORD dwGrowLinear, DWORD dwGrowPower>
inline void CArray<T, dwGrowLinear, dwGrowPower>::PushBackShift()
{
	std::vector<T>::iterator iter = m_Entries.begin();
	m_Entries.insert(iter, T());
}*/

template <typename T, DWORD dwGrowLinear, DWORD dwGrowPower>
inline bool CArray<T, dwGrowLinear, dwGrowPower>::FindIfEquals(T & element, DWORD & dwEntry) 
{
	for (DWORD i=0; i<m_dwSize; i++) {
		if (m_Entries[i] == element) {
			dwEntry = i;
			return 1;
		}
	}
	return 0;
}

template <typename T, DWORD dwGrowLinear, DWORD dwGrowPower>
inline void CArray<T, dwGrowLinear, dwGrowPower>::SetSize(DWORD dwSize)
{
	Assert(dwSize <= m_dwCapacity, "CArray::SetSize dwSize is greater than capacity.");

	m_dwSize = dwSize;
}

template <typename T, DWORD dwGrowLinear, DWORD dwGrowPower>
inline void CArray<T, dwGrowLinear, dwGrowPower>::p_ResizeArray(DWORD64 dwCapacity)
{
	T * ar = new T[(DWORD)dwCapacity];
	Assert(ar, "CArray::p_ResizeArray out of memory.");

	memcpy(ar, m_Entries, min(m_dwSize, (DWORD)dwCapacity)*sizeof(T));
	if (m_Entries) delete[] m_Entries;

	m_Entries = ar;
	m_dwCapacity = dwCapacity;
}

template <typename T, DWORD dwGrowLinear, DWORD dwGrowPower>
inline void CArray<T, dwGrowLinear, dwGrowPower>::Copy(CArray & toCopy) 
{
	Assert(this->m_Entries != toCopy.m_Entries, "CArray::Copy cannot copy oneself.");

	Clear();
	Reserve(toCopy.m_dwCapacity);
	Resize(toCopy.m_dwSize);
	memcpy(m_Entries, toCopy.m_Entries, sizeof(T)*m_dwSize);
}

template <typename T, DWORD dwGrowLinear, DWORD dwGrowPower>
inline void CArray<T, dwGrowLinear, dwGrowPower>::Clear() 
{
	if (m_Entries) {
		delete[] m_Entries; m_Entries = 0;
		m_dwSize = 0;
		m_dwCapacity = 0;
	}
}


};


#endif

