// Fixed-span circular FIFO buffer
// 2003-2018 sergei lupashin
// License: BSD-3-clause
#pragma once

#ifdef _MSC_VER //  visual studio
#pragma warning(push)
#pragma warning(disable : 4244)
#endif

#include <assert.h>
#include <stdlib.h>

template<class T>
class FixedQueue{
public:
  FixedQueue(unsigned int span, T failret=T())
    :_span(span),_failret(failret)
  {
    assert(span>=1);
    _data = new T[_span+1];
    assert(_data!=NULL);
    reset();
  }

  FixedQueue(const FixedQueue& b)
	{
    _span = b._span;
		_data = new T[_span+1];
		_failret = b._failret;
		reset();
		if(b.empty()) return;
		/*for(unsigned int i=b.n_meas();i>0;i--)
		{
			push(b.at(i-1));
		}*/
		for(unsigned int i=0;i<b.n_meas();i++)
		{
			push(b.at(i));
		}


  }

  virtual ~FixedQueue()
	{
    delete [] _data;
  }

  void operator=(const FixedQueue& b)
	{
		delete [] _data;
		
		_span = b._span;
		_data = new T[_span+1];
		_failret = b._failret;
		reset();
		if(b.empty()) return;
		for(int i=(int)b.n_meas()-1;i>=0;i--)
		{
			push(b.at(i));
		}
	}

  void push(const T& val)
	{
		assert(_end <= _span);
    _data[_end] = val;
    _end = (_end+1)%(_span+1);
    if(_begin==_end)
      _begin = (_begin+1)%(_span+1);
  }

  inline unsigned int n_meas() const
	{
    if(_begin==_end)    return 0;
    if(_end<_begin)     return (_span+1)-(_begin-_end);
    else              return _end-_begin;
  }

  inline bool empty() const{  return _begin==_end; }
  inline bool full() const {  return n_meas()==_span; }
  inline void reset(){  _begin = _end = 0; }

  // newest pushed data (bottom)
  T newest() const
	{
    if(_begin==_end) return _failret;
    return _data[(_end + _span)%(_span+1)];
  }

  // oldest pushed data (top)
  T oldest() const
	{
    if(_begin==_end) return _failret;
    return _data[_begin];
  }

	inline T pop_newest()
	{
		if(_begin==_end) return _failret;
		_end = (_end-1+_span+1)%(_span+1);
		return _data[_end];
	}

	inline T pop_oldest()
	{
		if(_begin==_end) return _failret;
		unsigned int b = _begin;
		_begin = (_begin+1)%(_span+1);
		return _data[b];
	}
	
	inline T pop_oldest(unsigned int cnt)
	{
		// TODO: this could be a lot more efficient
		T ret;
		for(unsigned int i=0;i<cnt;i++)
		{
			ret = pop_oldest();
		}
		return ret;
	}

  // newest pushed data (bottom)
  T& newest()
	{
    if(_begin==_end) return _failret;
    return _data[(_end + _span)%(_span+1)];
  }

  // oldest pushed data (top)
  T& oldest()
	{
    if(_begin==_end) return _failret;
    return _data[_begin];
  }

  // 0 is the beginning (oldest) data
  T& operator[](unsigned int i)
	{
    if((i+1)>n_meas())
      return _failret;
    i = (i+_begin)%(_span+1);
		assert(i <= _span);
    return _data[i];
  }

	const T& operator[](unsigned int i) const 
	{
		if((i+1)>n_meas())
      return _failret;
    i = (i+_begin)%(_span+1);
		assert(i <= _span);
    return _data[i];
	}

	const T& at(unsigned int i) const
	{
		if((i+1)>n_meas())
      return _failret;
    i = (i+_begin)%(_span+1);
		assert(i <= _span);
    return _data[i];
	}

	T& at(unsigned int i) 
	{
		if((i+1)>n_meas())
      return _failret;
    i = (i+_begin)%(_span+1);
		assert(i <= _span);
    return _data[i];
	}

protected:
  unsigned int _begin, _end;
  unsigned int _span;
  T *_data;
  T _failret;
};

#ifdef _MSC_VER //  visual studio
#pragma warning(pop)
#endif
