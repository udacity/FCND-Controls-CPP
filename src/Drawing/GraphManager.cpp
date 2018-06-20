#include "Common.h"
#include "GraphManager.h"
#include "../Utility/SimpleConfig.h"
#include "../Utility/StringUtils.h"
#include "DrawingFuncs.h"
#include "DataSource.h"

using namespace SLR;

GraphManager* _g_GraphManager=NULL;

void _g_OnGrapherDisplay()
{
  if (_g_GraphManager != NULL)
  {
    _g_GraphManager->Paint();
  }
}

void _g_OnGrapherReshape(int w, int h)
{
  if (_g_GraphManager != NULL)
  {
    _g_GraphManager->Paint();
  }
}

GraphManager::GraphManager(bool own_window)
{
	ParamsHandle config = SimpleConfig::GetInstance();

  _ownWindow = own_window;

  if (_ownWindow)
  {
    _g_GraphManager = this;

    glutInitWindowSize(500, 300);
    glutInitWindowPosition(0, 0);
    _glutWindowNum = glutCreateWindow("Grapher");
    glutSetWindow(_glutWindowNum);

    glutReshapeFunc(&_g_OnGrapherReshape);
    glutDisplayFunc(&_g_OnGrapherDisplay);

    InitPaint();
  }

  graph1.reset(new Graph("Graph1"));
  graph2.reset(new Graph("Graph1"));
}

GraphManager::~GraphManager()
{
  graph1.reset();
  graph2.reset();
  Sleep(100);
  _g_GraphManager = NULL;
}

void GraphManager::Reset()
{
  graph1->Reset();
  graph2->Reset();
}

void GraphManager::Clear()
{
  graph1->Clear();
  graph2->Clear();
}

void GraphManager::UpdateData(double time)
{
  if (graph1)
  {
    graph1->Update(time, _sources);
  }
  if (graph2)
  {
    graph2->Update(time, _sources);
  }
}

void GraphManager::DrawUpdate()
{
 
  if (_ownWindow)
  {
    glutSetWindow(_glutWindowNum);
    glutPostRedisplay();
  }
}

void GraphManager::InitPaint()
{
  glClearColor(0.0, 0.0, 0.0, 0.0);  // When screen cleared, use black.
  glShadeModel(GL_SMOOTH);  // How the object color will be rendered smooth or flat
}

void GraphManager::Paint()
{
  if (_ownWindow)
  {
    glutSetWindow(_glutWindowNum);

    int width = glutGet(GLUT_WINDOW_WIDTH);
    int height = glutGet(GLUT_WINDOW_HEIGHT);

    glViewport(0, 0, width, height);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);  //Clear the screen
  }
  else
  {

  }

  if (graph1 && graph1->_series.size())
  {
    glPushMatrix();
    if (graph2 && graph2->_series.size())
    {
      glTranslatef(0, .55f, 0);
    }
    else
    {
      glTranslatef(0, -.5f, 0);
    }
    glScalef(1, .5f, 1);

    glColor3f(0, 0, 0);
    glBegin(GL_QUADS);
    glVertex2f(-1, 1);
    glVertex2f(1, 1);
    glVertex2f(1, -1);
    glVertex2f(-1, -1);
    glEnd();

    graph1->Draw();
    glPopMatrix();
  }

  if (graph2 && graph2->_series.size())
  {
    glPushMatrix();
    glTranslatef(0, -.5f, 0);
    glScalef(1, .5f, 1);

    glColor3f(0, 0, 0);
    glBegin(GL_QUADS);
    glVertex2f(-1, 1);
    glVertex2f(1, 1);
    glVertex2f(1, -1);
    glVertex2f(-1, -1);
    glEnd();

    graph2->Draw();
    glPopMatrix();
  }

  glFlush();  // Render now

  if (_ownWindow)
  {
    glutSwapBuffers();
  }
}

void GraphManager::RegisterDataSource(shared_ptr<DataSource> src)
{
  _sources.push_back(src);
}

vector<string> GraphManager::GetGraphableStrings()
{
  vector<string> ret;
  for (auto i = _sources.begin(); i != _sources.end(); i++)
  {
    vector<string> s = (*i)->GetFields();
    for (auto j = s.begin(); j != s.end(); j++)
    {
      ret.push_back("AddGraph1."+*j);
      ret.push_back("AddGraph2." + *j);
    }
  }
  return ret;
}

void GraphManager::AddGraph(string path)
{
  if (path.find("AddGraph1.") == 0)
  {
    graph1->AddItem(path.substr(10));
  }
  if (path.find("AddGraph2.") == 0)
  {
    graph2->AddItem(path.substr(10));
  }
}
