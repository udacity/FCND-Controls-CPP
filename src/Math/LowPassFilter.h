#pragma once

template<typename T, typename T_TimeConstant=double> 
class LowPassFilter
{
public:
	LowPassFilter(T_TimeConstant timeConstant=1, T initialVal=1)
	{
		_timeConstant = timeConstant;
		_val = initialVal;
	}

	T Read() const
	{
		return _val;
	}

	T Update(T meas, double dt)
	{
		T_TimeConstant alpha = dt / (_timeConstant + dt);
		_val = (T) (_val * (1.0-alpha) + meas*alpha);
		return _val;
	}

  void Reset(T meas)
  {
    _val = meas;
  }

	void SetTau(T_TimeConstant tau)
	{
		_timeConstant = tau;
	}

protected:
	T _val;
	T_TimeConstant _timeConstant;
};