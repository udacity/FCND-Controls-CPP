#pragma once

#include "../Common.h"

/*
http://studios.cla.umn.edu/tutorials/cameraterms.pdf
*/
#ifdef _WIN32
#define  WIN32_LEAN_AND_MEAN
#include <GL/gl.h>
#pragma comment (lib, "opengl32.lib")
#pragma comment (lib, "glu32.lib")
#endif

#include "../Utility/Mutex.h"
#include "../Utility/Timer.h"
#include "../Math/LowPassFilter.h"

class Camera {
public:
  SLR::Mutex _mutex;
	V3D _lookat, _pos, _up;
	LowPassFilter<V3D> _lookatFiltered, _posFiltered, _upFiltered;

	Timer lastUserInteractionTimer;

public:
	Camera(V3D pos, V3D lookat);
	void PanGlobal (V3D delta);

	// positive = "dolly in" = closer to subject
	// negative = "dolly out" = further away from subject
	void DollyIn(double delta);

	void SetYaw(double angleRad);
	void YawAboutCenter(double angleRad);
	void TiltAboutCenter(double angleRad);
	void SetView(double aspect, bool filtered=true);

	void PanLeft(double delta);
	void PanUp(double delta);

	void DrawLookAtMarker(double size);
	void TranslateViaLookAt(V3D newLookAt, bool showTargetBall=true);
	void SetLookAt(V3D newLookAt);
  void SetUp(V3D up) { _up = up; }

	void Update(double dt);
	V3D FilteredLookAt() const{return _lookatFiltered.Read();}
	V3D FilteredUp() const{return _upFiltered.Read();}
	V3D FilteredPos() const{return _posFiltered.Read();}

	void Reset();

	V3D Pos() const 	{		return _pos;	}
	V3D Front() const	{		return (_lookat-_pos).norm();	}
};
