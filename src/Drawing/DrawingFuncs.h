#pragma once

#ifndef __APPLE__
#include <GL/gl.h>
#include <GL/glut.h>
#include <GL/glu.h>
#include <GL/freeglut.h>
#else
#include <GLUT/glut.h>
#endif

#include "Math/Quaternion.h"


void GLCube(V3F center, V3F dims, int cnt=3);
void GLRectangle(V3F center, V3F normal, V3F up, float width, float height, int numX, int numY);
void DrawX3D(V3D markingColor, V3D bodyColor=V3D(.2,.2,.2), double alpha=1, bool solidPart=true, bool transPart=true, GLUquadricObj *glQuadric=NULL);
void DrawQuarterX3D(bool front, V3D markingColor, V3D bodyColor, double alpha, GLUquadricObj *glQuadric, float armLength);
void GLCross(const V3F& center, const V3F& dims, bool gl_begin_line=true);
void DrawQuarterX3D_TransparentPart(double alpha, GLUquadricObj *glQuadric, float armLength);
void DrawStrokeText(const char* str, float x, float y, float z, float lineWidth, float scaleX=1, float scaleY=1);

using SLR::Quaternion;

namespace SLR {

 // Utility class for helping with OpenGL drawing 

class OpenGLDrawer
{
public:
  OpenGLDrawer();
  ~OpenGLDrawer();

  void PushLighting();
  void PopLighting();
  void SetLighting(bool enable);

  void DrawQuadrotor2(V3F pos, Quaternion<float> att, V3F color, V3F centerOffset, float centerScale, float armLength);

  void DrawArrow(double len, double r1, double r2, double arrowLen);
  void DrawArrow(V3D from, V3D to, V3D color);

  GLUquadricObj* Quadric() { return _glQuadric; };

  V3D cameraPos;

protected:
  GLUquadricObj* _glQuadric;
};

} // namespace SLR
