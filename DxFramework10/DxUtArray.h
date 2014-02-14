
#ifndef DXUTARRAY_H
#define DXUTARRAY_H

#include <windows.h>
#include "DxUtError.h"
#include <vector>

#define STDVECTOR_MODE

namespace DxUt {

/* For the an array of size n, the new capacity of the array is computed by dwGrowLinear + n*dwGrowPower for PushBack */
template <typename T, DWORD dwGrowLinear=1, DWORD dwGrowPower=0>
class CArray {
private:
	DWORD m_dwCapacity : 31;
	bool m_bSharedMemory : 1;
	DWORD m_dwSize;
#ifndef STDVECTOR_MODE
	std::vector<T> m_rgEntry;
#else
	T * m_rgEntry;
#endif

	void p_ResizeArray(DWORD64 capacity);
public:
	CArray():m_dwCapacity(0), m_bSharedMemory(0), m_dwSize(0), m_rgEntry(0)  {
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
	void ReserveSharedMemory(void * pMem, DWORD nBytes=-1);

	/* Reverse array */
	void Reverse();

	void PushForward(T & element) {
		DebugBreak();
		//m_rgEntry.insert(m_rgEntry.begin(), element); 
	}

	/* Standard linear search through the array */
	bool FindIfEquals(T & element, DWORD & dwEntry) {
#ifndef STDVECTOR_MODE
		for (DWORD i=0, end=m_rgEntry.size(); i<end; i++) {
#else
		for (DWORD i=0; i<m_dwSize; i++) {
#endif
			if (m_rgEntry[i] == element) {
				dwEntry = i;
				return 1;
			}
		}
		return 0;
	}

#ifndef STDVECTOR_MODE
	DWORD GetSize() {return m_rgEntry.size(); }
	DWORD GetCapacity() {return m_rgEntry.capacity(); }
	T * GetData() {return m_rgEntry.data(); }
	T & GetBack() {return m_rgEntry.back(); }
#else
	DWORD GetSize() {return m_dwSize;}
	DWORD GetCapacity() {return m_dwCapacity;}
	T * GetData() {return m_rgEntry;}
	T & GetBack() {return m_rgEntry[m_dwSize-1];}
#endif
	void SetSize(DWORD dwSize);

	//T & operator[](DWORD dwI) {return m_rgEntry[dwI]; }
	T & operator[](DWORD dwI) {
		Assert(dwI >= 0 && dwI < m_dwSize, "Array::operator[] index out of range.");
		return m_rgEntry[dwI]; 
	}

	void Clear();
};

template <typename T, DWORD dwGrowLinear, DWORD dwGrowPower>
inline void CArray<T, dwGrowLinear, dwGrowPower>::ReserveSharedMemory(void * pMem, DWORD nBytes)
{
	DebugBreak();
	Clear();

	m_bSharedMemory = 1;
	m_rgEntry = pMem;
	m_dwCapacity = nBytes;
	m_dwSize = 0;
}
/*
template <typename T, DWORD dwGrowLinear, DWORD dwGrowPower>
inline void CArray<T, dwGrowLinear, dwGrowPower>::Reverse()
{
	for (DWORD i=0, end=m_dwSize/2; i<end; i++) {
		m_rgEntry.reverse
	}
}*/

template <typename T, DWORD dwGrowLinear, DWORD dwGrowPower>
inline void CArray<T, dwGrowLinear, dwGrowPower>::Reserve(DWORD n)
{
#ifndef STDVECTOR_MODE
	m_rgEntry.reserve(n);
#else
	if (m_dwCapacity < n)
		p_ResizeArray(n);
#endif
}

template <typename T, DWORD dwGrowLinear, DWORD dwGrowPower>
inline void CArray<T, dwGrowLinear, dwGrowPower>::Resize(DWORD n)
{
#ifndef STDVECTOR_MODE
	m_rgEntry.resize(n);
	m_dwSize = m_rgEntry.size();
#else
	if ((int)n - (int)m_dwCapacity > 0)
		p_ResizeArray(n);
	m_dwSize = n;
#endif
}

template <typename T, DWORD dwGrowLinear, DWORD dwGrowPower>
inline void CArray<T, dwGrowLinear, dwGrowPower>::PushBack(const T & element) 
{
#ifndef STDVECTOR_MODE
	m_rgEntry.push_back(element);
#else
	if (m_dwSize >= m_dwCapacity) {
		p_ResizeArray(dwGrowLinear + m_dwSize*(dwGrowPower + 1));
	}

	memcpy(&m_rgEntry[m_dwSize], &element, sizeof(T)); 
	m_dwSize++; 
#endif
}

template <typename T, DWORD dwGrowLinear, DWORD dwGrowPower>
inline void CArray<T, dwGrowLinear, dwGrowPower>::PushBack(void)
{
#ifndef STDVECTOR_MODE
	m_rgEntry.push_back(T());
#else
	if (m_dwSize >= m_dwCapacity) { 
		p_ResizeArray(dwGrowLinear + m_dwSize*(dwGrowPower + 1));
	}

	m_dwSize++; 
#endif
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
	std::vector<T>::iterator iter = m_rgEntry.begin();
	m_rgEntry.insert(iter, T());
}*/

template <typename T, DWORD dwGrowLinear, DWORD dwGrowPower>
inline void CArray<T, dwGrowLinear, dwGrowPower>::SetSize(DWORD dwSize)
{
	Assert(dwSize <= m_dwCapacity, "CArray::SetSize dwSize is greater than capacity.");

	m_dwSize = dwSize;
}

template <typename T, DWORD dwGrowLinear, DWORD dwGrowPower>
inline void CArray<T, dwGrowLinear, dwGrowPower>::p_ResizeArray(DWORD64 dwCapacity)
{
	Assert(!m_bSharedMemory, "CArray::p_ResizeArray cannot resize a shared memory array.");

	T * rg = new T[(DWORD)dwCapacity];
	Assert(rg, "CArray::p_ResizeArray out of memory.");

	memcpy(rg, m_rgEntry, min(m_dwSize, (DWORD)dwCapacity)*sizeof(T));
	if (m_rgEntry) delete[] m_rgEntry;

	m_rgEntry = rg;
	m_dwCapacity = dwCapacity;
}

template <typename T, DWORD dwGrowLinear, DWORD dwGrowPower>
inline void CArray<T, dwGrowLinear, dwGrowPower>::Clear() 
{
#ifndef STDVECTOR_MODE
	m_rgEntry.clear();
#else
	if (m_rgEntry) {
		if (!m_bSharedMemory) {delete[] m_rgEntry; m_rgEntry = 0; }
		m_dwSize = 0;
		m_dwCapacity = 0;
		m_bSharedMemory = 0;
	}
#endif
}


};


#endif

