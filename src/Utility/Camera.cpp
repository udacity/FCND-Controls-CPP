#include "Camera.h"
#include "../Math/MathUtils.h"

#ifndef __APPLE__
#include <GL/gl.h>
#include <GL/glut.h>
#include <GL/glu.h>
#include <GL/freeglut.h>
#else
#include <GLUT/glut.h>
#endif

#define PI 3.1415926535897932384626433832795
#define PIdiv180 (PI/180.0)

using SLR::ScopedMutexLock;

Camera::Camera(V3D pos, V3D lookat)
{
	//Init with standard OGL values:
	_lookat = lookat;
	_pos = pos;
	_up = V3D (0.0, 0.0, 1.0);

	Reset();	

	_lookatFiltered.Reset(_lookat);
	_posFiltered.Reset(_pos);
	_upFiltered.Reset(_up);
}

void Camera::Reset()
{
	double tau=.1;
	_lookatFiltered.SetTau(tau);
	_posFiltered.SetTau(tau);
	_upFiltered.SetTau(tau);
}

void Camera::PanGlobal (V3D delta)
{
  ScopedMutexLock sml(_mutex);
	_lookat += delta;
	_pos += delta;
	lastUserInteractionTimer.Reset();
}

// positive == closer to subject
void Camera::DollyIn(double delta)
{
  ScopedMutexLock sml(_mutex);
	double r = _pos.dist(_lookat);
	r -= delta;
	r = CONSTRAIN(r,.1,50);
	_pos = _lookat + (_pos-_lookat).norm()*r;
	lastUserInteractionTimer.Reset();
}

void Camera::SetYaw(double angleRads)
{
  ScopedMutexLock sml(_mutex);
	double r = _pos.distXY(_lookat);
	_pos.x = _lookat.x + cos(angleRads)*r;
	_pos.y = _lookat.y + sin(angleRads)*r;
}

void Camera::YawAboutCenter(double angleRads)
{
  ScopedMutexLock sml(_mutex);
	double curAngle = atan2(_pos.y-_lookat.y,_pos.x-_lookat.x);
	double r = _pos.distXY(_lookat);
	curAngle += angleRads;

	_pos.x = _lookat.x + cos(curAngle)*r;
	_pos.y = _lookat.y + sin(curAngle)*r;
	lastUserInteractionTimer.Reset();
}

void Camera::TiltAboutCenter(double angleRads)
{
  ScopedMutexLock sml(_mutex);
	double r = (_pos-_lookat).mag();
	double curYaw = atan2(_pos.y-_lookat.y,_pos.x-_lookat.x);
	double curPitch = atan2(_pos.z-_lookat.z,_pos.distXY(_lookat));

	curPitch += angleRads;
	curPitch = CONSTRAIN(curPitch,-M_PI/2.0+.001,M_PI/2.0-.001);

	_pos.x = _lookat.x + r*cos(curYaw)*cos(curPitch);
	_pos.y = _lookat.y + r*sin(curYaw)*cos(curPitch);
	_pos.z = _lookat.z + r*sin(curPitch);	
	lastUserInteractionTimer.Reset();
}

void Camera::PanLeft(double delta)
{
  ScopedMutexLock sml(_mutex);
	V3D left = -((_lookat-_pos).cross(_up).norm());
	_lookat += left*delta;
	_pos += left*delta;
	lastUserInteractionTimer.Reset();
}

void Camera::PanUp(double delta)
{
  ScopedMutexLock sml(_mutex);
	V3D right = ((_lookat-_pos).cross(_up).norm());
	V3D realUp = right.cross(_lookat-_pos).norm();
	_lookat += realUp*delta;
	_pos += realUp*delta;
	lastUserInteractionTimer.Reset();
}

void Camera::TranslateViaLookAt(V3D la, bool showTargetBall)
{
  ScopedMutexLock sml(_mutex);
	_pos = _pos - _lookat + la;
	_lookat = la;
	if(showTargetBall)
	{
		lastUserInteractionTimer.Reset();
	}
}

void Camera::SetLookAt(V3D la)
{
  ScopedMutexLock sml(_mutex);
	_lookat = la;
}

void Camera::DrawLookAtMarker(double size)
{
	_mutex.lock();
	V3D la = _lookat;
	_mutex.unlock();

	glPushMatrix();
	glTranslated(la.x,la.y,la.z);

	GLUquadricObj* q = gluNewQuadric();
	gluSphere(q,size,16,6);
	gluDeleteQuadric(q);

	glPopMatrix();
}

void Camera::SetView( double aspect, bool filtered)
{
	_mutex.lock();
	V3D lookat = filtered?_lookatFiltered.Read():_lookat;
	V3D pos = filtered?_posFiltered.Read():_pos;
	V3D up = filtered?_upFiltered.Read():_up;
	_mutex.unlock();

	glMatrixMode(GL_PROJECTION); 
	glLoadIdentity(); 
	gluPerspective(50.0, aspect, .05, 200.0); 
	
	glMatrixMode(GL_MODELVIEW); 
  glPushMatrix();
	glLoadIdentity(); 

	//as we know the up vector, we can easily use gluLookAt:
	gluLookAt(pos.x,pos.y,pos.z,lookat.x,lookat.y,lookat.z,up.x,up.y,up.z);
}

void Camera::Update(double dt)
{
	_lookatFiltered.Update(_lookat,dt);
	_posFiltered.Update(_pos,dt);
	_upFiltered.Update(_up,dt);
}
