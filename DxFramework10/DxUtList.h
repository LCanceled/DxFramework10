
#ifndef DXUTLIST_H
#define DXUTLIST_H

#include <windows.h>
#include "DxUtError.h"

namespace DxUt {

template <typename T>
class List {
	struct Node {
		Node():pDscdt(0), pChild(0) {}
		~Node() {}

		T data; 
		Node * pDscdt;
		Node * pChild;
	};

	struct IndexNode {
		DWORD index;
		Node * node;
	};

	Node * m_pListBegin;
	Node * m_pListEnd;
	IndexNode m_pCurrNode;
	DWORD m_NumIdenities;

	Node * FindNode(DWORD index);
public:
	List():m_pListBegin(0), m_pListEnd(0), m_NumIdenities(0) {
		m_pListBegin = m_pListEnd = new Node; m_pCurrNode.node = NULL; }
	~List() {Clear(); delete m_pListEnd; }

	void PushBack(T & element);
	void PushBack(VOID);
	void PushFront(T & element);
	void PushFront(VOID);
	void InsertBefore(DWORD index, T & element);
	void InsertAfter(DWORD index, T & element);
	void PopBack() {Erase(m_NumIdenities-1); }
	void PopFront() {Erase(0); }
	void Erase(DWORD index);
	bool InList(T & element, DWORD * index=0);
	bool InList(bool(*equal)(T, T), T & element, DWORD * index=0);

	DWORD & Size() {return m_NumIdenities;}
	T & GetLast() {return m_pCurrNode.node->data;}
	T & GetNext();
	T & operator[](DWORD index);

	void Clear();
};

template <typename T>
inline void List<T>::PushBack(T & element) 
{
	//if (m_NumIdenities != 0) {
		m_pListEnd->pChild = new Node;
		m_pListEnd->pChild->data = element;
		m_pListEnd = m_pListEnd->pChild;
	//}
	/*else {
		m_pListEnd = new Node;
		m_pListBegin = m_pListEnd;
		m_pListEnd->data = element;
	}*/

	m_pCurrNode.index = m_NumIdenities++;
	m_pCurrNode.node = m_pListEnd;
}

template <typename T>
inline void List<T>::PushBack(VOID)
{
	//if (m_NumIdenities != 0) {
		m_pListEnd->pChild = new Node;
		m_pListEnd = m_pListEnd->pChild;
	//}
	/*else {
		m_pListEnd = new Node;
		m_pListBegin = m_pListEnd;
	}*/

	m_pCurrNode.index = m_NumIdenities++;
	m_pCurrNode.node = m_pListEnd;
}

template <typename T>
inline void List<T>::PushFront(T & element) 
{
	//if (m_NumIdenities != 0) {
		Node * dscdt = m_pListBegin->pDscdt;
		m_pListBegin->pDscdt = new Node;
		m_pListBegin->pDscdt->data = element;

		m_pListBegin->pDscdt->pChild = m_pListBegin;
		m_pListBegin = m_pListBegin->pDscdt;
	//}
	/*else {
		m_pListBegin = new Node;
		m_pListEnd = m_pListBegin;
		m_pListBegin->data = element;
	}*/

	m_pCurrNode.index = 0;
	m_pCurrNode.node = m_pListBegin;
	m_NumIdenities++; 
}

template <typename T>
inline void List<T>::PushFront(VOID)
{
	if (m_NumIdenities != 0) {
		m_pListBegin->pDscdt = new Node;

		m_pListBegin->pDscdt->pChild = m_pListBegin;
		m_pListBegin = m_pListBegin->pDscdt;
	}
	else {
		m_pListBegin = new Node;
		m_pListEnd = m_pListBegin;
	}

	m_pCurrNode.index = 0;
	m_pCurrNode.node = m_pListBegin;
	m_NumIdenities++;
}

template <typename T>
inline void List<T>::InsertBefore(DWORD index, T & element)
{
	if (index >= m_NumIdenities) {
		DxUtSendError("Index for list InsertBefore() is out of range."); }

	if (index == 0) 
		PushFront(element);
	else {
		Node * node = 0;
		node = FindNode(index);
		Node * ptr = node->pDscdt;

		ptr->pChild = new Node;
		ptr->pChild->data = element;
		ptr->pChild->pChild = node;
		ptr->pChild->pDscdt = node->pDscdt;
		node->pDscdt = ptr->pChild;

		m_pCurrNode.index = index;
		m_pCurrNode.node = ptr->pChild;
		m_NumIdenities++;
	}
}

template <typename T>
inline void List<T>::InsertAfter(DWORD index, T element)
{
	if (index >= m_NumIdenities) {
		DxUtSendError("Index for list InsertAfter() is out of range."); }

	if (index == (m_NumIdenities-1)) 
		PushBack(element);
	else {
		Node * node = 0;
		node = FindNode(index);
		Node * ptr = node->pChild;

		ptr->pDscdt = new Node;
		ptr->pDscdt->data = element;
		ptr->pDscdt->pDscdt = node;
		ptr->pDscdt->pChild = node->pChild;
		node->pChild = ptr->pDscdt;

		m_pCurrNode.index = index+1;
		m_pCurrNode.node = ptr->pDscdt;
		m_NumIdenities++;
	}
}

template <typename T>
inline void List<T>::Erase(DWORD index) 
{
	if (index >= m_NumIdenities) {
		DxUtSendError("Index for list Erase() is out of range"); }

	if (index == 0 && m_NumIdenities == 1) {
		delete m_pListBegin;
		m_pListBegin = NULL; 
		m_pListEnd = NULL;
		m_pCurrNode.index = 0;
		m_pCurrNode.node = m_pListBegin; }
	else if (index == 0) {
		m_pListBegin = m_pListBegin->pChild;
		delete m_pListBegin->pDscdt; 
		m_pListBegin->pDscdt = NULL; 
		m_pCurrNode.index = 0;
		m_pCurrNode.node = m_pListBegin; 
	}
	else if (index == (m_NumIdenities-1)) {
		m_pListEnd = m_pListEnd->pDscdt;
		delete m_pListEnd->pChild; 
		m_pListEnd->pChild = NULL; 
		m_pCurrNode.index = m_NumIdenities-1;
		m_pCurrNode.node = m_pListEnd; 
	}
	else {
		Node * node = 0;
		node = FindNode(index);
		Node * ptr1 = node->pDscdt;
		Node * ptr2 = node->pChild;
		ptr1->pChild = node->pChild;
		ptr2->pDscdt = node->pDscdt;
		m_pCurrNode.index = index;
		m_pCurrNode.node = ptr2;

		delete node;
		node = NULL;
	}

	m_NumIdenities--;
}

template <typename T>
inline bool List<T>::InList(T element, DWORD * index)
{
	Node * node = m_pListBegin;
	for (DWORD i=0; i<m_NumIdenities; i++) {
		if (node->data == element) {
			if (index) *index = i;
			return true;
		}
		node = node->pChild;
	}
	return false;
}

template <typename T>
inline bool List<T>::InList(bool(*equal)(T, T), T element, DWORD * index=0)
{
	Node * node = m_pListBegin;
	for (DWORD i=0; i<m_NumIdenities; i++) {
		if (equal(node->data, element)) {
			if (index) *index = i;
			return true;
		}
		node = node->pChild;
	}
	return false;
}

template <typename T>
inline T & List<T>::GetNext()
{
#ifdef DEBUG | _DEBUG
	if (m_pCurrNode.index == m_NumIdenities=1) {
		DxUtSendError("GetNext() is out of range."); }
#endif

	m_pCurrNode.index++;
	return (m_pCurrNode.node = m_pCurrNode.node->pChild)->data;
}

template <typename T>
inline T & List<T>::operator[](DWORD index) 
{
#ifdef DEBUG | _DEBUG
	if (index >= m_NumIdenities) {
		DxUtSendError("Index for list operator[] is out of range."); }
#endif

	return FindNode(index)->data;
}

template <typename T>
inline List<T> & List<T>::operator=(List<T> & ref)
{
	m_pListBegin = new Node;
	m_pListBegin->data = ref.m_pListBegin->data;
	m_pListEnd = m_pListBegin;

	for (DWORD i=1; i<ref.m_NumIdenities; i++) {
		m_pListEnd->pChild = new Node;
		m_pListEnd->pChild->data = ref[i];
		m_pListEnd->pChild->pDscdt = m_pListEnd;
		m_pListEnd = m_pListEnd->pChild;
	}

	m_pCurrNode.index = 0;
	m_pCurrNode.node = m_pListBegin;
	m_NumIdenities = ref.m_NumIdenities;

	return *this;
}

template <typename T>
template <typename T2>
inline List<T> & List<T>::operator=(List<T2> & ref)
{
	m_pListBegin = new Node;
	m_pListBegin->data = ref[0];
	m_pListEnd = m_pListBegin;

	for (DWORD i=1; i<ref.Size(); i++) {
		m_pListEnd->pChild = new Node;
		m_pListEnd->pChild->data = ref[i];
		m_pListEnd->pChild->pDscdt = pDscdt;
		m_pListEnd = m_pListEnd->pChild;
	}

	m_pCurrNode.index = 0;
	m_pCurrNode.node = m_pListBegin;
	m_NumIdenities = ref.Size();

	return *this;
}

template <typename T>
inline typename List<T>::Node * List<T>::FindNode(DWORD index)
{
	Node * node=m_pCurrNode;
	DWORD startingIndex = m_pCurrNode.index;
	DWORD dwDistance = labs(m_pCurrNode.index - index);
	if (dwDistance > index) {node = m_pListBegin; startingIndex = 0; }
	else if (dwDistance > (m_NumIdenities - index)) {node = m_pListEnd; startingIndex = m_NumIdenities-1; }
	
	int dir = (index - startingIndex) > 0 ? 1 : -1, offset = (dir + 1)/2;
	for (int i=startingIndex; i!=index; i+=dir) {
		node = (Node*)node->pDscdt + offset;
	}

	return node;
}


template <typename T>
inline void List<T>::Clear() 
{
	if (m_pListBegin) {
		for (DWORD i=0; i<(m_NumIdenities-1); i++) {
			m_pListBegin = m_pListBegin->pChild;
			delete m_pListBegin->pDscdt;
			m_pListBegin->pDscdt = NULL;
		}
		delete m_pListBegin;
		m_pListBegin = NULL;
		m_pListEnd = NULL;
		m_pCurrNode.index = 0;
		m_pCurrNode.node = NULL;
		m_NumIdenities = 0;
	}
}


};


#endif



/*
	if (dCr < index && dCr < (m_NumIdenities - index)) {
		node = m_pCurrNode.node;
		if (m_pCurrNode.index < index) {
			for (DWORD i=m_pCurrNode.index; i<index; i++)
				node = node->pChild;
		}
		else {
			for (DWORD i=m_pCurrNode.index; i>index; i--)
				node = node->pDscdt;
		}
	}
	else if (index < (m_NumIdenities - index)) {
		node = m_pListBegin;
		for (DWORD i=0; i<index; i++)
			node = node->pChild;
	}
	else {
		node = m_pListEnd;
		for (DWORD i=(m_NumIdenities-1); i>index; i--)
			node = node->pDscdt;
	}
	*/