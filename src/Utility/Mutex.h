#pragma once

#include "../Common.h"
#ifdef _WIN32
#include <windows.h>
#else
#include <pthread.h>
#include <errno.h>
#endif

namespace SLR{

class ScopedMutexLock;

class Mutex{
	friend class ScopedMutexLock;

protected:
#ifdef _WIN32
	// in win32 use critical sections, cause they're fast and simple
	CRITICAL_SECTION _cs;
#else
	// in linux/pthread, use pthread_mutex..
	pthread_mutex_t _mutex;
#endif
	
	// copy constructor and assignment are disallowed
	Mutex(const Mutex&){};					
	Mutex& operator=(const Mutex&){return *this;}

public:
	Mutex()
	{
#ifdef _WIN32
		InitializeCriticalSection(&_cs);	
#else
		pthread_mutex_init(&_mutex, NULL);
#endif
	}

	virtual ~Mutex()
	{
#ifdef _WIN32
		DeleteCriticalSection(&_cs);
#else
		pthread_mutex_destroy(&_mutex);
#endif
	}

	inline bool TryLock()
	{
#ifdef _WIN32
		return (TryEnterCriticalSection(&_cs)==TRUE);
#else
		return (pthread_mutex_trylock(&_mutex) != EBUSY);
#endif
	}

	inline void Lock(){lock();}

	inline void lock()
	{
#ifdef _WIN32
		EnterCriticalSection(&_cs);
#else
		pthread_mutex_lock(&_mutex);
#endif
	}

	inline void Unlock(){unlock();}

	inline void unlock()
	{
#ifdef _WIN32
		LeaveCriticalSection(&_cs);
#else
		pthread_mutex_unlock(&_mutex);
#endif
	}
};

class ScopedMutexLock;

template<typename T>
class Mutexed
{
	friend ScopedMutexLock;
public:
	Mutexed(const T init):val(init){};
	Mutexed(){};
  Mutexed(const Mutexed<T> & c)
  {
    val = c.AtomicCopy();
  }
  Mutexed<T>& operator=(const Mutexed<T>& c)
  {
    AtomicWrite(c.AtomicCopy());
    return *this;
  }
	operator T()
	{
		return val;
	}

	const T* operator->() const
	{
		return &val;
	}

	T* operator->() 
	{
		return &val;
	}

	T& operator*() const
	{
		return val;
	}

	T& operator*() 
	{
		return val;
	}
	
	T val;
	
	void lock() const{_m.lock();}
	void unlock() const {_m.unlock();}

	T AtomicCopy() const
	{
		lock();
		T ret(val);
		unlock();
		return ret;
	}

	void AtomicWrite(const T& v)
	{
		lock();
		val = v;
		unlock();
	}

protected:
	mutable Mutex _m;
};

class ScopedMutexLock
{
protected:
	Mutex* _m;

	// default, copy constructor and assignment are disallowed
	ScopedMutexLock(){};
	ScopedMutexLock(const ScopedMutexLock&){};					
	ScopedMutexLock& operator=(const ScopedMutexLock&){return *this;}

public:
	ScopedMutexLock(Mutex& mutex)
	{
		_m = &mutex;
		_m->lock();		
	}

	template<typename T>
	ScopedMutexLock(const Mutexed<T>& mutexed)
	{
		_m = &mutexed._m;
		_m->lock();		
	}

	~ScopedMutexLock()
	{
		_m->unlock();
	}
};

template<typename T>
class MutexedSPtr : public shared_ptr<T>
{
protected:
	Mutex _m;
public:
	MutexedSPtr(){};
	MutexedSPtr(T* a):shared_ptr<T>(a){};
	MutexedSPtr(const shared_ptr<T>& a){shared_ptr<T>::operator=(a);};
	MutexedSPtr operator=(const shared_ptr<T>& a){return shared_ptr<T>::operator=(a);};
	void lock(){_m.lock();}
	void unlock(){_m.unlock();}
};

} //namespace FLR
