#include "../Common.h"
#include "DrawingFuncs.h"

using SLR::Quaternion;

using SLR::OpenGLDrawer;

// up is defined as the vector from the center to the top edge of the rectangle
// normal is the vector from the center in the direction of the front face
void GLRectangle(V3F center, V3F normal, V3F up, float width, float height, int numX, int numY)
{
	// ccw rectangle!
	glBegin(GL_QUADS);

	normal = normal.norm();
	up = up.norm();
	V3F left = normal.cross(up).norm();
	float dx = width/(float)numX;
	float dy = height/(float)numY;
	V3F bottomRight = center - up*height/2.f - left*width/2.f;
	for(int x=0;x<numX;x++)
	{
		for(int y=0;y<numY;y++)
		{
			V3F a = bottomRight + left*dx*(float)x + up*dy*(float)y;
			V3F b = a + up*dy;
			V3F c = b + left*dx;
			V3F d = c - up*dy;

			glNormal3f(normal.x,normal.y,normal.z);
			glVertex3f(a.x,a.y,a.z);
			glVertex3f(b.x,b.y,b.z);
			glVertex3f(c.x,c.y,c.z);
			glVertex3f(d.x,d.y,d.z);
		}
	}
	glEnd();

}

void GLCube(V3F center, V3F dims, int cnt)
{
	GLRectangle(center + V3F(0,0,1)*dims/2.f,V3F(0,0,1),V3F(1,0,0),dims.y,dims.x,cnt,cnt);		// top
	GLRectangle(center - V3F(0,0,1)*dims/2.f,V3F(0,0,-1),V3F(1,0,0),dims.y,dims.x,cnt,cnt);		// bottom
	
	GLRectangle(center + V3F(1,0,0)*dims/2.f,V3F(1,0,0),V3F(0,0,1),dims.y,dims.z,cnt,cnt);		// front
	GLRectangle(center - V3F(1,0,0)*dims/2.f,V3F(-1,0,0),V3F(0,0,1),dims.y,dims.z,cnt,cnt);		// back

	GLRectangle(center + V3F(0,1,0)*dims/2.f,V3F(0,1,0),V3F(0,0,1),dims.x,dims.z,cnt,cnt);		// left
	GLRectangle(center - V3F(0,1,0)*dims/2.f,V3F(0,-1,0),V3F(0,0,1),dims.x,dims.z,cnt,cnt);		// right
}

void GLCross(const V3F& center, const V3F& dims, bool gl_begin_line)
{
  if(gl_begin_line)
  {
    glBegin(GL_LINES);
  }

  glVertex3f(center.x,center.y,center.z+dims.z/2.f);
  glVertex3f(center.x,center.y,center.z-dims.z/2.f);
  glVertex3f(center.x,center.y+dims.y/2.f,center.z);
  glVertex3f(center.x,center.y-dims.y/2.f,center.z);
  glVertex3f(center.x+dims.x/2.f,center.y,center.z);
  glVertex3f(center.x-dims.x/2.f,center.y,center.z);

  if(gl_begin_line)
  {
    glEnd();
  }

}


const float propR = .1f;   // .1 for hummingbird
const float motorR = .02f; // .03 for hummingbird

void DrawQuarterX3D(bool front, V3D markingColor, V3D bodyColor, double alpha, GLUquadricObj *glQuadric, float armL)
{	
	if(front)
	{
		glColor4d(markingColor[0],markingColor[1],markingColor[2],alpha);
	}
	else
	{
		glColor4d(bodyColor[0],bodyColor[1],bodyColor[2],alpha);
	}
	// arm
	GLCube(V3F(armL/2.f,0,0),V3F(armL,.03f,.005f));
	
	glPushMatrix();
	glTranslated(armL,0,.005);
	
	// shaft + nut
	glPushMatrix();
	glColor4d(bodyColor[0],bodyColor[1],bodyColor[2],alpha);
	glTranslatef(0,0,.005f);
	gluCylinder(glQuadric,.003,.003,.015,8,1);
	glTranslatef(0,0,.015f);
	gluDisk(glQuadric,0,.003,8,1);
	glPopMatrix();

	glPushMatrix();
	glTranslated(0,0,-.01);

	float mcolor[] = { 1.f, 1.f, 1.f, 1.0f };
	mcolor[3] = (float)alpha;
	float noSpecular[] = {0,0,0,1};
	glColor4d(0,0,.8,alpha);

	glMaterialfv(GL_FRONT, GL_SPECULAR, mcolor);
	glMaterialf(GL_FRONT,GL_SHININESS,1.5f);

	gluCylinder(glQuadric, motorR, motorR, .02,16,2);	// motor

	glMaterialfv(GL_FRONT, GL_SPECULAR, noSpecular);
	glMaterialf(GL_FRONT,GL_SHININESS,0);
	glPopMatrix();

	glColor4d(.1,.1,.1,alpha);
	// bottom cap
	glPushMatrix();
	glTranslatef(0,0,-.01f);
	glRotatef(180,1,0,0);
	gluDisk(glQuadric,0, motorR,16,2);
	glPopMatrix();

	// top cap
	glTranslatef(0,0,.01f);
	gluDisk(glQuadric,0, motorR,16,2);
	
	glPopMatrix();
}

void DrawQuarterX3D_TransparentPart(double alpha, GLUquadricObj *glQuadric, float armL)
{
	glPushMatrix();
	glTranslated(armL,0,.005);

	// top cap
	glTranslatef(0,0,.01f);
	
	// props!
	glColor4d(.4,.4,.4,.4*alpha);
	glTranslated(0,0,.005);

	glDisable(GL_CULL_FACE);
	glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);

  glEnable(GL_POLYGON_OFFSET_FILL);
  glPolygonOffset(1.f, 1.f);
	gluDisk(glQuadric,0, propR,24,3);
	
  glDisable(GL_POLYGON_OFFSET_FILL);
	//glRotatef(180,1,0,0);
	//gluDisk(q,0,.1,24,3);

	// line highlight  

	glColor4d(.4,.4,.4,alpha);
	glBegin(GL_LINES);
	for(int i=0;i<24;i++)
	{
		double angle = (double)i/24.0 * M_PI * 2.0;
		double angle2 = (double)(i+1)/24.0 * M_PI * 2.0;
		double r = propR+.001;
		glVertex3d(r*sin(angle),r*cos(angle),0);
		glVertex3d(r*sin(angle2),r*cos(angle2),0);
	}
	glEnd();
	/*glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
	glLineWidth(1);
	glColor4d(0,0,0,alpha);
	gluDisk(q,0,.1,24,2);*/
	//glPolygonMode(GL_FRONT,GL_FILL);

	glEnable(GL_CULL_FACE);
	glPopMatrix();
}

void DrawStrokeText(const char* str, float x, float y, float z, float lineWidth, float scaleX, float scaleY)
{
  char *c;
  glPushMatrix();
  glTranslatef(x, y, z);
  glScalef(0.0006f*scaleX, 0.0008f*scaleY, z);

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_LINE_SMOOTH);
  glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
  glEnable(GL_LINE_SMOOTH);
  glLineWidth(lineWidth);
  for (c = (char*)str; *c != 0; c++)
  {
    glutStrokeCharacter(GLUT_STROKE_ROMAN, *c);
  }
  glLineWidth(1.0);
  glPopMatrix();
}

void DrawX3D(V3D markingColor, V3D bodyColor, double alpha, bool solidPart, bool transPart, GLUquadricObj *glQuadric)
{	
  const float armL = .17f;    // .17 for hummingbird

	bool cleanup = (glQuadric==NULL);
	if(glQuadric==NULL)
	{
		glQuadric = gluNewQuadric();
	}
	if(solidPart)
	{
		// center -- a box.
		glPushMatrix();
		glRotatef(45,0,0,1);
		glColor4d(bodyColor[0],bodyColor[1],bodyColor[2],alpha);
		GLCube(V3F(),V3F(.07f,.07f,.04f));
		glPopMatrix();

		DrawQuarterX3D(true, markingColor,bodyColor*.8,alpha,glQuadric, armL);
		glRotatef(90,0,0,1);
    DrawQuarterX3D(true, markingColor, bodyColor*.8, alpha, glQuadric, armL);
		glRotatef(90,0,0,1);
		DrawQuarterX3D(false,V3D(),bodyColor*.8,alpha,glQuadric, armL);
		glRotatef(90,0,0,1);
		DrawQuarterX3D(false,V3D(),bodyColor*.8,alpha,glQuadric, armL);
	}
	if(transPart)
	{
		DrawQuarterX3D_TransparentPart(alpha,glQuadric, armL);
		glRotatef(90,0,0,1);
		DrawQuarterX3D_TransparentPart(alpha,glQuadric, armL);
		glRotatef(90,0,0,1);
		DrawQuarterX3D_TransparentPart(alpha,glQuadric, armL);
		glRotatef(90,0,0,1);
		DrawQuarterX3D_TransparentPart(alpha,glQuadric, armL);
	}
	if(cleanup)
	{
		gluDeleteQuadric(glQuadric);
	}
}


void OpenGLDrawer::DrawQuadrotor2(V3F pos, Quaternion<float> att, V3F color, V3F centerOffset, float centerScale, float armL)
{
  glPushMatrix();

  // STRANGE: have to enable GL_SMOOTH here for it to work. something is switching the shade model back to flat.
  glShadeModel(GL_SMOOTH);

  glTranslated(pos[0], pos[1], pos[2]);

  V3D ypr = att.ToEulerYPR();

  glRotated(ypr[0] / M_PI * 180.0, 0, 0, 1);
  glRotated(ypr[1] / M_PI * 180.0, 0, 1, 0);
  glRotated(ypr[2] / M_PI * 180.0, 1, 0, 0);

  glRotatef(45, 0, 0, 1);
  glRotatef(180, 1, 0, 0);

  V3D bodyColor = V3D(.7, .7, 1);

  bool cleanup = (_glQuadric == NULL);
  if (_glQuadric == NULL)
  {
    _glQuadric = gluNewQuadric();
  }
  float alpha = 1;
  if (1)
  {
    // center -- a box.
    glPushMatrix();
    glRotatef(45, 0, 0, 1);
    glTranslated(centerOffset.x, centerOffset.y, centerOffset.z);
    glScalef(centerScale, centerScale, centerScale);
    
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(1.f, 1.f);
    glPolygonMode(GL_FRONT, GL_FILL);
    glColor4d(bodyColor[0], bodyColor[1], bodyColor[2], alpha);
    GLCube(V3F(), V3F(.07f, .07f, .04f));
    glDisable(GL_POLYGON_OFFSET_FILL);

    glPolygonMode(GL_FRONT, GL_LINE);
    glColor4d(.1,.1,.1, alpha);
    GLCube(V3F(), V3F(.07f, .07f, .04f),1);
    glPopMatrix();
    glPolygonMode(GL_FRONT, GL_FILL);

    DrawQuarterX3D(true, color, bodyColor*.8, alpha, _glQuadric, armL);
    glRotatef(90, 0, 0, 1);
    DrawQuarterX3D(true, color, bodyColor*.8, alpha, _glQuadric, armL);
    glRotatef(90, 0, 0, 1);
    DrawQuarterX3D(false, V3D(), bodyColor*.8, alpha, _glQuadric, armL);
    glRotatef(90, 0, 0, 1);
    DrawQuarterX3D(false, V3D(), bodyColor*.8, alpha, _glQuadric, armL);
  }
  if (1)
  {
    DrawQuarterX3D_TransparentPart(alpha, _glQuadric, armL);
    glRotatef(90, 0, 0, 1);
    DrawQuarterX3D_TransparentPart(alpha, _glQuadric, armL);
    glRotatef(90, 0, 0, 1);
    DrawQuarterX3D_TransparentPart(alpha, _glQuadric, armL);
    glRotatef(90, 0, 0, 1);
    DrawQuarterX3D_TransparentPart(alpha, _glQuadric, armL);
  }
  if (cleanup)
  {
    gluDeleteQuadric(_glQuadric);
  }

  glPolygonMode(GL_FRONT, GL_FILL);

  glPopMatrix();
}

void glCircle(float radius, int numSegments)
{
	glBegin(GL_LINE_LOOP);
	for(int i=0;i<numSegments;i++)
	{
		glVertex3f(radius*sinf(i*F_PI*2.f/(float)numSegments),radius*cosf(i*F_PI*2.f/ (float)numSegments),0);
	}
	glEnd();
}

#ifdef _MSC_VER //  visual studio
#pragma warning(disable:4100) //supress unused arguments warning
#endif


void OpenGLDrawer::DrawArrow(double len, double r1, double r2, double arrowLen)
{
  glPushMatrix();
	glRotated(180,1,0,0);
	gluDisk(_glQuadric,0,r1,12,5);
	glRotated(180,1,0,0);
	gluCylinder(_glQuadric,r1,r1,len,10,10);
	glTranslated(0,0,len);
	glRotated(180,1,0,0);
	gluDisk(_glQuadric,0,r2,12,5);
	glRotated(180,1,0,0);
	gluCylinder(_glQuadric,r2,0,arrowLen,12,5);
	glPopMatrix();
}

void OpenGLDrawer::DrawArrow(V3D from, V3D to, V3D color)
{
  V3D vec = (to-from).norm();
  double yaw = atan2(vec.y,vec.x);
  double pitch = -atan2(vec.z,vec.magXY());

  glPushMatrix();

  glTranslated(from.x, from.y, from.z);
	glRotated(yaw/M_PI*180.0,0,0,1);
	glRotated(pitch/M_PI*180.0,0,1,0);
  glRotated(90,0,1,0);

  glColor4d(color[0],color[1],color[2],1);

  double len = (to-from).mag();
  
  DrawArrow(len,.015,.035,MAX(MIN(.1,len/4),.2));

  glPopMatrix();
}



OpenGLDrawer::OpenGLDrawer()
{
	_glQuadric = gluNewQuadric();
}

OpenGLDrawer::~OpenGLDrawer()
{
	gluDeleteQuadric(_glQuadric);
}

void OpenGLDrawer::PushLighting()
{
	glPushAttrib(GL_ENABLE_BIT);
}

void OpenGLDrawer::PopLighting()
{
	glPopAttrib();
}

void OpenGLDrawer::SetLighting(bool enable)
{
	if(enable)
	{
		glEnable(GL_LIGHTING);
		glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);
	}
	else
	{
    glDisable(GL_LIGHT1);
		glDisable(GL_LIGHT0);
		glDisable(GL_LIGHTING);
	}
}

