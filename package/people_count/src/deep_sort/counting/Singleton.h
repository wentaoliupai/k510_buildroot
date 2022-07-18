#pragma once
#include <string>
#include <thread>
#include <mutex>
#include "noncopyable.h"

template <class T>class CSingleton : private CNonCopyable
{
public:
	static T* getOrCreateInstance(bool vMultiThread = false);
	static void destroy();

protected:
	CSingleton(void)   {}
	virtual ~CSingleton(void)  {}

private:
	static T* volatile  mg_pInstance;
	static std::mutex  m_Muxtex;
};

template <class T>
T* CSingleton<T>::getOrCreateInstance(bool vMultiThread)
{
	if (NULL == mg_pInstance)
	{
		if (vMultiThread)
			m_Muxtex.lock();

		if (!mg_pInstance)   //double checked locking
		{
			mg_pInstance = new T;
			atexit(destroy);
		}

		if (vMultiThread)
			m_Muxtex.unlock();
	}
	return mg_pInstance;
}

template <class T>
void CSingleton<T>::destroy()
{
	if (mg_pInstance)
	{
		m_Muxtex.lock();
		if (mg_pInstance)
		{
			delete mg_pInstance;
			mg_pInstance = NULL;  //this line is important when getInstance() is called after destroy()
		}
		m_Muxtex.unlock();
	}
}

template <class T>
T* volatile CSingleton<T>::mg_pInstance = NULL;
template <class T> std::mutex  CSingleton<T>::m_Muxtex;

