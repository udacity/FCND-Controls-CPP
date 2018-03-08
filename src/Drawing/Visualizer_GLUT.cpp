#include "Common.h"
#include "Visualizer_GLUT.h"

#include <math.h>
#include "Math/MathUtils.h"
#include "Utility/StringUtils.h"
#include <limits>

#include "Drawing/DrawingFuncs.h"

#include "Simulation/QuadDynamics.h"
#include "Drawing/ColorUtils.h"
#include "GraphManager.h"

#ifndef __APPLE__
#include <GL/gl.h>
#include <GL/glut.h>
#include <GL/glu.h>
#include <GL/freeglut.h>
#else
#include <GLUT/glut.h>
#endif

#define GRAPH_SCALE  0.4f

using namespace SLR;

void LoadScenario(string scenarioFile);
Visualizer_GLUT* _g_viz = NULL;

///////////////////////////////////////////////
// GLUT CALLBACKS FOR MAIN OPENGL WINDOW

void _g_OnMouseClick(int button, int state, int x, int y)
{
  if (_g_viz != NULL)
  {
    _g_viz->OnMouseClick(button, state, x, y);
  }
}

void _g_OnResize(int width, int height)
{
  if (_g_viz != NULL)
  {
    _g_viz->OnResize(width,height);
  }
}

void _g_OnDisplay()
{
  if (_g_viz != NULL)
  {	
    _g_viz->Paint();
  }
}

void _g_OnMouseMove(int x, int y)
{
  if (_g_viz != NULL)
  {
    _g_viz->OnMouseMove(x,y);
  }
}

void _g_OnWindowClose()
{
  if (_g_viz != NULL)
  {
    _g_viz->_exiting = true;
  }
}

void _g_OnExit()
{
  exit(0);
}

bool _g_keySpecialStates[246];
bool _g_keyStates[256];

void _g_OnKeyPressed(unsigned char key, int x, int y) {
  _g_keyStates[key] = true; // Set the state of the current key to pressed  
}

void _g_OnKeyUp(unsigned char key, int x, int y) {
  _g_keyStates[key] = false; // Set the state of the current key to not pressed  
}

void _g_OnSpecialKeyPressed(int key, int x, int y) {
  _g_keySpecialStates[key] = true; // Set the state of the current key to pressed  
}

void _g_OnSpecialKeyUp(int key, int x, int y) {
  _g_keySpecialStates[key] = false; // Set the state of the current key to not pressed  
}


Visualizer_GLUT::Visualizer_GLUT(int *argcp, char **argv)
: _camera(V3D(-5,-1,-5),V3D(0,0,-1)), _draw_dt_ms(.1f)
{
  _camera.SetUp(V3D(0,0,-1));
  glutInit(argcp, argv);
	glQuadric = gluNewQuadric();

	_volumeCallList = 0;

	_cameraTrackingMode = "Independent";
	
	_objectSelected = false;
  _g_viz = this;
  _mouseLeftDown = _mouseRightDown = false;

  initializeGL(argcp, argv);
  
  Reset();
}

Visualizer_GLUT::~Visualizer_GLUT()
{
  glutMotionFunc(NULL);
  glutPassiveMotionFunc(NULL);
  glutMouseFunc(NULL);
  glutReshapeFunc(NULL);

  glutKeyboardFunc(NULL);
  glutKeyboardUpFunc(NULL);
  glutSpecialFunc(NULL);
  glutSpecialUpFunc(NULL);

  Sleep(100);
  gluDeleteQuadric(glQuadric);
  _g_viz = NULL;
}

bool Visualizer_GLUT::IsKeyDown(uint8_t key)
{
  return _g_keyStates[key];
}

bool Visualizer_GLUT::IsSpecialKeyDown(int specialKey)
{
  return _g_keySpecialStates[specialKey];
}

void Visualizer_GLUT::Reset()
{
  showPropCommands = false;
  showRefTrajectory = showActualTrajectory = false;
  paused = false;

	// default settings here..
	_drawVolumeBoundaries = true;
	// default background colors
	_bgColorBottomLeft = _bgColorBottomRight = V3F(.8f,.8f,.85f);			
	_bgColorTopLeft = _bgColorTopRight = V3F(.6f,.8f,.9f);

	_refLoc = V3D(3, .9, .3);

	_camera.Reset();

	_doubleClickTimer = Timer::InvalidTimer();

	GLuint tmp = _volumeCallList;
	_volumeCallList = MakeVolumeCallList();
	if(tmp!=_volumeCallList)
	{
		glDeleteLists(tmp, 1);
	}
}

void Visualizer_GLUT::Update()
{
  glutPostWindowRedisplay(_glutWindowNum);
  if (_exiting) return;
}

void Visualizer_GLUT::initializeGL(int *argcp, char **argv)
{
  glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB | GLUT_DEPTH);
  glutInitWindowPosition(200, 200);
  glutInitWindowSize(800, 600);
  _glutWindowNum = glutCreateWindow("Simulator!");

  glutMotionFunc(&_g_OnMouseMove);
  glutPassiveMotionFunc(NULL);
  glutMouseFunc(&_g_OnMouseClick);
  glutReshapeFunc(&_g_OnResize);
  glutDisplayFunc(&_g_OnDisplay);
  _exiting = false;
  glutWMCloseFunc(&_g_OnWindowClose);

  glutWMCloseFunc(&_g_OnExit);
  
  // MAC GLUT implementation doesn't have this
#ifndef __APPLE__
  glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION);
#endif

  glutKeyboardFunc(_g_OnKeyPressed);
  glutKeyboardUpFunc(_g_OnKeyUp);
  glutSpecialFunc(_g_OnSpecialKeyPressed);
  glutSpecialUpFunc(_g_OnSpecialKeyUp);

	glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);  
	glEnable(GL_BLEND);
	glEnable(GL_DEPTH_TEST);
	glShadeModel( GL_SMOOTH );
	
	glHint (GL_LINE_SMOOTH_HINT, GL_NICEST);	// Use The Good Calculations
	glEnable (GL_LINE_SMOOTH);

	glHint (GL_PERSPECTIVE_CORRECTION_HINT,GL_NICEST);
	
	// this supposedly makes rendering much slower, but gives much better results
	// for certain lighting situations... disabled...
	//glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);
	
	glClearColor(0.1f, 0.0f, 0.4f, 1.0f);					// blueish Background

	_glDraw.reset(new SLR::OpenGLDrawer());

	Reset();
}

void SetupLights(shared_ptr<SLR::OpenGLDrawer> glDraw)
{
	glDraw->SetLighting(true);
	
	// Create light components
	float ambientLight[] = { 0.2f, 0.2f, 0.2f, 1.0f };
	float diffuseLight[] = { 0.4f, 0.4f, 0.4f, 1.0f };
	//float specularLight[] = { 0.5f, 0.5f, 0.5f, 1.0f };
	float specularLight[] = { .7f, .7f, .7f, 1.0f };
	float position[] = { 0, 10, -6.f, 1.0f };

	// Assign created components to GL_LIGHT0
	glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLight);
	glLightfv(GL_LIGHT0, GL_SPECULAR, specularLight);
	glLightfv(GL_LIGHT0, GL_POSITION, position);


  // Create light components
	float pos2[] = { 10.f, 0, -6.f, 1.0f };

	// Assign created components to GL_LIGHT0
	glLightfv(GL_LIGHT1, GL_AMBIENT, ambientLight);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuseLight);
	glLightfv(GL_LIGHT1, GL_SPECULAR, specularLight);
	glLightfv(GL_LIGHT1, GL_POSITION, pos2);
}

void Visualizer_GLUT::DrawBackground()
{
	glMatrixMode(GL_PROJECTION); 
	glPushMatrix();
	glMatrixMode(GL_MODELVIEW); 
	glPushMatrix();

	glMatrixMode(GL_PROJECTION); 
	glLoadIdentity(); 
	gluOrtho2D(0,1,0,1);
	glMatrixMode(GL_MODELVIEW); 
	glLoadIdentity(); 

	// draw background
	glBegin(GL_QUADS);
	glColor4f(_bgColorBottomLeft[0],_bgColorBottomLeft[1],_bgColorBottomLeft[2],1);
	glVertex2f(0,0);	
	glColor4f(_bgColorBottomRight[0],_bgColorBottomRight[1],_bgColorBottomRight[2],1);
	glVertex2f(1,0);
	// top
	glColor4f(_bgColorTopRight[0],_bgColorTopRight[1],_bgColorTopRight[2],1);
	glVertex2f(1,1);	
	glColor4f(_bgColorTopLeft[0],_bgColorTopLeft[1],_bgColorTopLeft[2],1);
	glVertex2f(0,1);
	glEnd();
	
	glMatrixMode(GL_MODELVIEW); 
	glPopMatrix();
	glMatrixMode(GL_PROJECTION); 
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW); 
	
	// have to clear the depth bit... otherwise nothing gets painted afterwards
	glClear(GL_DEPTH_BUFFER_BIT);
}



void Visualizer_GLUT::DrawCoordinateReference()
{
	glPushMatrix();
  //V3D lookat = _camera.FilteredLookAt();
  //V3D up = _camera.FilteredUp();
  //V3D pos = _camera.FilteredPos();

  //V3D left = -((lookat-pos).cross(up).norm());
  //V3D realUp = (-left).cross(lookat-pos).norm();
	//V3D rosePos = pos + (lookat-pos).norm()*1.0 - left*.3 - realUp * .3;
	//V3D rosePos = pos + (lookat-pos).norm()*_refLoc.x + left*_refLoc.y + realUp * _refLoc.z;
  V3D rosePos(-2, -2, -.5f);
	
	//SetLighting(false);
	glTranslated(rosePos.x,rosePos.y,rosePos.z);
	//glColor4d(1,0,0,1);
	_glDraw->DrawArrow(V3D(),V3D(.3,0,0),V3D(1,0,0));
	_glDraw->DrawArrow(V3D(),V3D(0,.3,0),V3D(0,1,0));
	_glDraw->DrawArrow(V3D(),V3D(0,0,.3),V3D(0,0,1));
	glPopMatrix();
}

SLR::LineD Visualizer_GLUT::ScreenToPickVector(double x, double y)
{
	float winY = (float)(viewport[3] - y);			// Subtract The Current Mouse Y Coordinate From The Screen Height.
	float winX = (float)(x);

	V3D ray;
  gluUnProject( winX, winY, .5, modelMatrix, projMatrix, viewport, &ray.x, &ray.y, &ray.z);

  return SLR::LineD(_camera.FilteredPos(),_camera.FilteredPos()+(ray-_camera.FilteredPos()).norm()*50);
}

void Visualizer_GLUT::DrawTrajectories(shared_ptr<QuadDynamics> quad)
{
  if (quad)
  {
    if (quad->controller && showRefTrajectory)
    {
      VisualizeTrajectory(quad->controller->trajectory, true, V3F(0, 1, 1), 1.f, V3F(.1f, .2f, 1), quad->color, quad->controller->_trajectoryOffset);
    }
  }

  if (quad && quad->_followed_traj  && showActualTrajectory)
  {
    VisualizeTrajectory(*(quad->_followed_traj).get(), false, quad->color, 1.f, V3F(), V3F(), V3F(), 1);
  }
}

void Visualizer_GLUT::Draw(shared_ptr<QuadDynamics> quad)
{
  SetupLights(_glDraw);
  // enable color tracking
  //glDisable(GL_COLOR_MATERIAL);
  // set material properties which will be assigned by glColor
  glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);

  if (quad)
  {
    VisualizeQuadCopter(quad);
  }

}

void Visualizer_GLUT::Paint()
{
  _draw_dt_ms = (float)_lastDraw.Seconds() * 1000.f;
  _lastDraw.Reset();
  
  glutSetWindow(_glutWindowNum);
  
	Timer t;

	_camera.Update(_timeSinceLastPaint.Seconds());
	_timeSinceLastPaint.Reset();

	_glDraw->cameraPos = _camera.FilteredPos();

	glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);  
	glEnable(GL_BLEND);
	glEnable(GL_DEPTH_TEST);
	glShadeModel( GL_SMOOTH );
	
  // use the niceeeest lines
	glHint (GL_LINE_SMOOTH_HINT, GL_NICEST);	
	glEnable (GL_LINE_SMOOTH);
  
  int width = glutGet(GLUT_WINDOW_WIDTH);
  int height = glutGet(GLUT_WINDOW_HEIGHT);
    
	glViewport(0,0, width, height);

  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();


	DrawBackground();
	
	_camera.SetView((double)width/(double)height);

	glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
	//glPolygonMode(GL_FRONT_AND_BACK,GL_LINE); // wireframe mode!
	glEnable(GL_CULL_FACE);

  if ((_arrowBegin - _arrowEnd).mag() > 0.001)
  {
    V3F tmp = (_arrowBegin - _arrowEnd).norm()*.7f;
    _glDraw->DrawArrow(_arrowBegin+tmp, _arrowEnd+tmp, V3D(1, 0, 0));
  }

	// retrieve modelview/projection/viewport info to calculate 2D coords from 3D coords
	glGetDoublev(GL_MODELVIEW_MATRIX,modelMatrix);
	glGetDoublev(GL_PROJECTION_MATRIX,projMatrix);
	glGetIntegerv(GL_VIEWPORT,viewport);

  SetupLights(_glDraw);

  

	// enable color tracking
	glEnable(GL_COLOR_MATERIAL);
	// set material properties which will be assigned by glColor
	glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);

	DrawCoordinateReference();
	
	if(_drawVolumeBoundaries)
	{
		glCallList(_volumeCallList);
	}

  for (unsigned i = 0; i < quads.size(); i++)
  {
    Draw(quads[i]);
  }
  for (unsigned i = 0; i < quads.size(); i++)
  {
    DrawTrajectories(quads[i]);
  }

  // disable lights and fancy 3d effects for 2d drawing
	_glDraw->SetLighting(false);

  glShadeModel(GL_FLAT);
  glDisable(GL_CULL_FACE);
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_LIGHTING);

  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);

  // Draw graph if there is a graph
  if (graph)
  {
    glPushMatrix();
    // Move the graph to the bottom right corner and scale it down
    glTranslatef(1.f-GRAPH_SCALE,-(1.f-GRAPH_SCALE),0);
    glScalef(GRAPH_SCALE*.95f,GRAPH_SCALE*.95f, 1);

    graph->Paint(); // draw the graph

    glPopMatrix();
  }

  // Display the "paused" text if necessary
  if (paused)
  {
    glColor3f(1,0,0);
    DrawStrokeText("Paused", -1+0.1f, 1-0.2f, 0, 3.f);
  }

  //glutSwapBuffers();
  
  _last_draw_time_ms = (float)t.Seconds()*1000.f;

  if (_delayedScenarioLoader != "")
  {
    LoadScenario(_delayedScenarioLoader);
    _delayedScenarioLoader = "";
  }
}

void Visualizer_GLUT::VisualizeQuadCopter(shared_ptr<QuadDynamics> quad)
{
  glLineWidth(1);
  glEnable(GL_LINE_SMOOTH);
  _glDraw->DrawQuadrotor2(quad->Position(), quad->Attitude(), quad->color, V3F(quad->cx,quad->cy,0), quad->M/0.5f, quad->L);

  if (showPropCommands)
  {
    V3D pos = quad->Position();
    V3D fl = quad->GetArmLength() / sqrtf(2) * quad->Attitude().Rotate_BtoI(V3F(1, 1, 0)); 
    V3D fr = quad->GetArmLength() / sqrtf(2) * quad->Attitude().Rotate_BtoI(V3F(1, -1, 0));
    V3D down = fl.cross(fr).norm();
    const float maxThrust = 4.5f;
    
		VehicleCommand cmd = quad->GetCommands();
    _glDraw->DrawArrow(pos + fl, pos + fl + down*cmd.desiredThrustsN[0]/maxThrust, FalseColorRGB(cmd.desiredThrustsN[0]/maxThrust));			// front left
    _glDraw->DrawArrow(pos + fr, pos + fr + down* cmd.desiredThrustsN[1] / maxThrust, FalseColorRGB(cmd.desiredThrustsN[1] / maxThrust));	// front right
    _glDraw->DrawArrow(pos - fr, pos- fr + down* cmd.desiredThrustsN[2] / maxThrust, FalseColorRGB(cmd.desiredThrustsN[2] / maxThrust));		// rear left
    _glDraw->DrawArrow(pos - fl, pos - fl + down* cmd.desiredThrustsN[3] / maxThrust, FalseColorRGB(cmd.desiredThrustsN[3] / maxThrust));	// front right
  }
}

void Visualizer_GLUT::VisualizeTrajectory(const Trajectory& traj, bool drawPoints, V3F color, float alpha, V3F pointColor, V3F curPointColor, V3F offset, int style)
{
  // Draw the desired trajectory line
  if (style == 0)
  {
    _glDraw->SetLighting(false);
    glDisable(GL_LINE_SMOOTH);
    glLineWidth(1.5);
    glColor4d(color[0], color[1], color[2], alpha);
    glBegin(GL_LINE_STRIP);
    for (unsigned int i = 0; i < traj.traj.n_meas(); i++)
    {
      glVertex3fv((traj.traj[i].position + offset).getArray());
    }
    glEnd();
  }
  else if (style == 1)
  {
    _glDraw->SetLighting(false);
    glEnable(GL_LINE_SMOOTH);
    glLineWidth(1);
    glColor4d(color[0], color[1], color[2], alpha);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glDisable(GL_CULL_FACE);
    glBegin(GL_QUADS);
    for (unsigned int i = 1; i < traj.traj.n_meas(); i++)
    {
      V3F p = (traj.traj[i].position + offset);
      V3F l = traj.traj[i].attitude.Rotate_BtoI(V3F(0, 1, 0)) * 0.1f;
      glVertex3fv((p+l).getArray());
      glVertex3fv((p-l).getArray());
        
      p = (traj.traj[i-1].position + offset);
      l = traj.traj[i-1].attitude.Rotate_BtoI(V3F(0, 1, 0)) * 0.1f;
      glVertex3fv((p - l).getArray());
      glVertex3fv((p + l).getArray());
    }
    glEnd();
      
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    glColor4d(color[0], color[1], color[2], .1f);
    glBegin(GL_QUADS);
    for (unsigned int i = 1; i < traj.traj.n_meas(); i++)
    {
      V3F p = (traj.traj[i].position + offset);
      V3F l = traj.traj[i].attitude.Rotate_BtoI(V3F(0, 1, 0)) * 0.1f;
      glVertex3fv((p + l).getArray());
      glVertex3fv((p - l).getArray());

      p = (traj.traj[i - 1].position + offset);
      l = traj.traj[i - 1].attitude.Rotate_BtoI(V3F(0, 1, 0)) * 0.1f;
      glVertex3fv((p - l).getArray());
      glVertex3fv((p + l).getArray());
    }
    glEnd();
  }

  if (drawPoints)
  {
    // Draw the desired trajectory points as spheres
    for (unsigned int i = 0; i < traj.traj.n_meas(); i++)
    {
      V3F pos = traj.traj[i].position + offset;
      float r = 0.01f;
      // Draw the current trajectory point in a different colour
      if (i == (unsigned)traj.GetCurTrajectoryPoint())
      {
        glColor4f(curPointColor[0], curPointColor[1], curPointColor[2], 1);
        r = 0.03f;
      }
      else
      {
        glColor4f(pointColor[0], pointColor[1], pointColor[2], 1);
      }
      glPushMatrix();
      glTranslatef(pos.x, pos.y, pos.z);
      gluSphere(_glDraw->Quadric(), r, 6, 6);
      glPopMatrix();
    }
  }
}

void Visualizer_GLUT::OnResize(int,int)
{
  Paint();
}

void Visualizer_GLUT::OnMouseClick(int button, int state, int x, int y)
{
  lastPosX = x;
  lastPosY = y;

  if (button == GLUT_LEFT_BUTTON)
  {
    _mouseLeftDown = (state == GLUT_DOWN);
  }
  if (button == GLUT_RIGHT_BUTTON)
  {
    _mouseRightDown = (state == GLUT_DOWN);
  }
  if (button == GLUT_MIDDLE_BUTTON)
  {
    _mouseMiddleDown = (state == GLUT_DOWN);
  }

  if (_mouseLeftDown && _doubleClickTimer.ElapsedSeconds() < .2f)
  {
    OnMouseDoubleLClick(x,y);
  }

  _doubleClickTimer.Reset();
}

void Visualizer_GLUT::OnMouseDoubleLClick(int x, int y)
{
  SLR::LineD line = ScreenToPickVector(x, y);
  int minDistIndex = -1;
  double minDist = 1000.0;

  for (unsigned i = 0; i < quads.size(); i++)
  {
    if (line.Dist(quads[i]->Position()).mag() < minDist)
    {
      minDist = line.Dist(quads[i]->Position()).mag();
      minDistIndex = i;
    }
  }

  if (minDist < .5)
  {
    _camera.SetLookAt(quads[minDistIndex]->Position());
  }
}

void Visualizer_GLUT::OnMouseMove(int x, int y)
{
	int dx = x - lastPosX;
	int dy = y - lastPosY;

  if (_mouseLeftDown && (IsKeyDown('x') || IsKeyDown('X')))
	{
		_camera.PanLeft(-dx/20.0);
		_camera.PanUp(-dy/20.0);
	}
  else if(_mouseLeftDown && (IsKeyDown('z') || IsKeyDown('Z')))
	{
		_camera.DollyIn(-dy/10.0);
	}
	else if (_mouseLeftDown)
	{
    _camera.YawAboutCenter(dx/20.0);
    _camera.TiltAboutCenter(-dy/20.0);
	} 
	
  lastPosX = x;
  lastPosY = y;
}

void Visualizer_GLUT::SetKeyboardFunc(void(*callback)(unsigned char, int, int))
{
  glutKeyboardFunc(callback);
}

void DrawXYGrid(V3D center, double lenX, double lenY, int numCellsX, int numCellsY)
{
	V3D corner = center - V3D(lenX/2.0,lenY/2.0,0);
	double dx = lenX/(double)numCellsX;
	double dy = lenY/(double)numCellsY;

	int count;

	glBegin(GL_LINES);
	
	count=0;
	for(double y=corner.y; count<=numCellsY; y+=dy,count++)
	{	
		glVertex3d(corner.x,y,center.z);
		glVertex3d(corner.x+lenX,y,center.z);
	}

	count=0;
	for(double x=corner.x; count<=numCellsX; x+=dx,count++)
	{
		glVertex3d(x,corner.y,center.z);
		glVertex3d(x,corner.y+lenY,center.z);
	}
	glEnd();
}

GLuint Visualizer_GLUT::MakeVolumeCallList()
{
	GLuint list = glGenLists(1);
	glNewList(list, GL_COMPILE);

	// set up nice colors and so on.
	_glDraw->SetLighting(true);
	// enable color tracking
	glEnable(GL_COLOR_MATERIAL);
	// set material properties which will be assigned by glColor
	glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);

	// first the polys
	glEnable( GL_POLYGON_OFFSET_FILL );  
	glPolygonOffset( 1.f, 1.f );

	//glDisable( GL_POLYGON_OFFSET_FILL );
	
	glEnable(GL_LINE_SMOOTH);	

  glColor3f(.65f, .65f, .65f);
	// floor
	GLRectangle(V3F(0,0,0),V3F(0,0,-1),V3F(1,0,0),5,5,5,5);

	glDisable( GL_POLYGON_OFFSET_FILL );

  _glDraw->SetLighting(false);
  glPolygonMode(GL_FRONT_AND_BACK,GL_LINE); // wireframe mode!
  //glBegin(GL_LINES);
  glColor3d(.55, .55, .55);
  GLRectangle(V3F(0, 0, 0), V3F(0, 0, -1), V3F(1, 0, 0), 5, 5, 5, 5);
  //glEnd();
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	_glDraw->SetLighting(false);

	glColor3d(.2,.2,.2);

	//glDepthMask(GL_TRUE);
	_glDraw->SetLighting(true);

	glEndList();
	return list;
}

void Visualizer_GLUT::InitializeMenu(const vector<string>& strings)
{
  vector<string> tmp = strings;
  tmp.push_back("Toggle.RefTrajectory");
  tmp.push_back("Toggle.ActualTrajectory");
  tmp.push_back("Toggle.Thrusts");

  FILE* f = fopen("../config/Scenarios.txt","r");
  char buf[512]; buf[511] = 0;
  while (f && fgets(buf, 510, f))
  {
    string trimmed = SLR::Trim(string(buf));
    tmp.push_back(string("Scenario.") + trimmed);
  }
  fclose(f);

  f = fopen("../config/X_Scenarios.txt", "r");
  while (f && fgets(buf, 510, f))
  {
    string trimmed = SLR::Trim(string(buf));
    tmp.push_back(string("Scenario.") + trimmed);
  }
  if (f)
  {
    fclose(f);
  }

  glutSetWindow(_glutWindowNum);
  _menu.CreateMenu(tmp);
  _menu.OnMenu = MakeDelegate(this, &Visualizer_GLUT::OnMenu);
}

void Visualizer_GLUT::OnMenu(string cmd)
{
  if (cmd == "Toggle.RefTrajectory")
  {
    showRefTrajectory = !showRefTrajectory;
  }
  else if (cmd == "Toggle.ActualTrajectory")
  {
    showActualTrajectory = !showActualTrajectory;
  }
  else if (cmd == "Toggle.Thrusts")
  {
    showPropCommands = !showPropCommands;
  }
  else if (cmd.find("Scenario.")!=string::npos)
  {
    string name = string("../config/")+cmd.substr(9)+".txt";
    _delayedScenarioLoader = name;
  }
  else
  {
    graph->AddGraph(cmd);
  }
}

void Visualizer_GLUT::OnMainTimer()
{
  _timer_dt_ms = (float)_lastMainTimerEvent.Seconds()*1000.f;
  _lastMainTimerEvent.Reset();
}

bool Visualizer_GLUT::GetData(const string& name, float& ret) const
{
  if (name.find_first_of(".") == string::npos) return false;
  string leftPart = LeftOf(name, '.');
  string rightPart = RightOf(name, '.');
  
  if (ToUpper(leftPart) == "SIM")
  {
#define GETTER_HELPER(A,B) if (SLR::ToUpper(rightPart) == SLR::ToUpper(A)){ ret=(B); return true; }
    // UDACITY CONVENTION
    GETTER_HELPER("Draw_dt", _draw_dt_ms);
    GETTER_HELPER("Update_dt", _timer_dt_ms);
    GETTER_HELPER("DrawTime", _last_draw_time_ms);
#undef GETTER_HELPER
  }
  return false;
}

vector<string> Visualizer_GLUT::GetFields() const
{
  vector<string> ret;
  ret.push_back("Sim.Draw_dt");
  ret.push_back("Sim.Update_dt");
  ret.push_back("Sim.DrawTime");
  return ret;
}
