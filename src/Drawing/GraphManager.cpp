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

  graph.reset(new Graph("Graph1"));
}

GraphManager::~GraphManager()
{
  graph.reset();
  Sleep(100);
  _g_GraphManager = NULL;
}

void GraphManager::Reset()
{
  graph->Reset();
}

void GraphManager::Clear()
{
  graph->Clear();
}

void GraphManager::UpdateData(double time)
{
  if (graph)
  {
    graph->Update(time, _sources);
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
    if (graph->_series.size())
    {
      // Draw black background
      glColor3f(0, 0, 0);
      glBegin(GL_QUADS);
      glVertex2f(-1, 1);
      glVertex2f(1, 1);
      glVertex2f(1, -1);
      glVertex2f(-1, -1);
      glEnd();
    }
  }

  if (graph)
  {
    graph->Draw();
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
      ret.push_back("AddGraph."+*j);
    }
  }
  return ret;
}

void GraphManager::AddGraph(string path)
{
  if (path.find("AddGraph.") == 0)
  {
    graph->AddItem(path.substr(9));
  }
}
