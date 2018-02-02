// Angle utilities, super-lightweight-robotics library
// License: BSD-3-clause

#pragma once
#include "Constants.h"

// normalize angle to -pi to pi
inline double AngleNormD(double angle)
{
	angle = fmod(angle, (2.0*M_PI));

	if (angle <= -M_PI)
	{
	   angle += (2.0*M_PI);
	}
	else if (angle > M_PI)
	{
	   angle -= (2.0*M_PI);
	}

	return angle;
}

// normalize angle to -pi to pi
inline float AngleNormF(float angle)
{
	angle = fmod(angle, (2.0f*(float)M_PI));

	if (angle <= -(float)M_PI)
	{
	   angle += (2.0f*(float)M_PI);
	}
	else if (angle > (float)M_PI)
	{
	   angle -= (2.0f*(float)M_PI);
	}

	return angle;
}