#include "Common.h"
#include "Trajectory.h"
#include "Utility/Timer.h"

#include "Drawing/Visualizer_GLUT.h"
#include "Simulation/QuadDynamics.h"
#include "Simulation/Simulator.h"
#include "Utility/SimpleConfig.h"
#include "Utility/StringUtils.h"
#include "Drawing/GraphManager.h"

using SLR::Quaternion;
using SLR::ToUpper;

void KeyboardInteraction(V3F& force, shared_ptr<Visualizer_GLUT> vis);
bool receivedResetRequest = true;
bool paused = false;
void PrintHelpText();
void ProcessConfigCommands(shared_ptr<Visualizer_GLUT> vis);

vector<QuadcopterHandle> quads;

shared_ptr<Visualizer_GLUT> visualizer;
shared_ptr<GraphManager> grapher;

float dtSim = 0.001f;
Timer lastDraw;
V3F force, moment;

float simulationTime=0;
int randomNumCarry=0;
string flightMode;

void OnTimer(int v);

vector<QuadcopterHandle> CreateVehicles();

int main(int argcp, char **argv)
{
  PrintHelpText();
 
  // load parameters
  ParamsHandle config = SimpleConfig::GetInstance();

  // initialize visualizer
  visualizer.reset(new Visualizer_GLUT(&argcp, argv));
  grapher.reset(new GraphManager(false));

  // create a quadcopter to simulate
  quads = CreateVehicles();
  
  grapher->RegisterDataSource(visualizer);

  visualizer->InitializeMenu(grapher->GetGraphableStrings());

  visualizer->quads = quads;
  visualizer->graph = grapher;

  ProcessConfigCommands(visualizer);
  
  glutTimerFunc(1,&OnTimer,0);
  
  glutMainLoop();

  return 0;
}

void ResetSimulation()
{
  ParamsHandle config = SimpleConfig::GetInstance();

  receivedResetRequest = false;
  simulationTime = 0;
  config->Reset();
  dtSim = config->Get("Sim.Timestep", 0.005f);
  
  flightMode = config->Get("Quad.SimMode","Full3D");
  
  for (unsigned i = 0; i<quads.size(); i++)
  {
    quads[i]->Reset();
  }
  grapher->Clear();
}

void OnTimer(int)
{
  ParamsHandle config = SimpleConfig::GetInstance();
  
  // logic to reset the simulation based on key input or reset conditions
  float endTime = config->Get("Sim.EndTime",-1.f);
  if(receivedResetRequest ==true ||
     (ToUpper(config->Get("Sim.RunMode", "Continuous"))=="REPEAT" && endTime>0 && simulationTime >= endTime))
  {
    ResetSimulation();
  }
  
  visualizer->OnMainTimer();
  
  // main loop
  if (!paused)
  {
    for(unsigned i=0;i<quads.size();i++)
    {
      quads[i]->Run(dtSim, simulationTime, randomNumCarry, force, moment, flightMode);
    }
    grapher->UpdateData(simulationTime);
    simulationTime += dtSim;
  }
  
  KeyboardInteraction(force, visualizer);
  
  if (lastDraw.ElapsedSeconds() > 0.030)
  {
    if (quads.size() > 0)
    {
      visualizer->SetArrow(quads[0]->Position() - force, quads[0]->Position());
    }
    visualizer->Update();
    grapher->DrawUpdate();
    lastDraw.Reset();
  }
  
  glutTimerFunc(5,&OnTimer,0);
}

vector<QuadcopterHandle> CreateVehicles()
{
  vector<QuadcopterHandle> ret;

  ParamsHandle config = SimpleConfig::GetInstance();
  int i = 1;
  while (1)
  {
    char buf[100];
    sprintf_s(buf, 100, "Sim.Vehicle%d", i);
    if (config->Exists(buf))
    {
      QuadcopterHandle q = QuadDynamics::Create(config->Get(buf, "Quad"),ret.size());
      grapher->RegisterDataSource(q);
      ret.push_back(q);
    }
    else
    {
      break;
    }
    i++;
  }
  return ret;

}

void KeyboardInteraction(V3F& force, shared_ptr<Visualizer_GLUT> visualizer)
{
  bool keyPressed = false;
  const float forceStep = 0.04f;

  if (visualizer->IsSpecialKeyDown(GLUT_KEY_LEFT))
  {
    force += V3F(0, -forceStep, 0);
    keyPressed = true;
  }
  if (visualizer->IsSpecialKeyDown(GLUT_KEY_UP))
  {
    force += V3F(0, 0, -forceStep);
    keyPressed = true;
  }
  if (visualizer->IsSpecialKeyDown(GLUT_KEY_RIGHT))
  {
    force += V3F(0, forceStep, 0);
    keyPressed = true;
  }
  if (visualizer->IsSpecialKeyDown(GLUT_KEY_DOWN))
  {
    force += V3F(0, 0, forceStep);
    keyPressed = true;
  }
  if (visualizer->IsKeyDown('w') || visualizer->IsKeyDown('W'))
  {
    force += V3F(forceStep, 0, 0);
    keyPressed = true;
  }
  if (visualizer->IsKeyDown('s') || visualizer->IsKeyDown('S'))
  {
    force += V3F(-forceStep, 0, 0);
    keyPressed = true;
  }

  if (!keyPressed)
  {
    force = V3F();
  }
  if (force.mag() > 2.f)
  {
    force = force / force.mag() * 2.f;
  }

  if (visualizer->IsKeyDown('c') || visualizer->IsKeyDown('C'))
  {
    visualizer->graph->graph->RemoveAllSeries();
  }

  if (visualizer->IsKeyDown('r') || visualizer->IsKeyDown('R'))
  {
    receivedResetRequest = true;
  }

  static bool key_space_pressed = false;

  if (visualizer->IsKeyDown(' '))
  {
    if (!key_space_pressed)
    {
      key_space_pressed = true;
      paused = !paused;
      visualizer->paused = paused;
    }
  }
  else
  {
    key_space_pressed = false;
  }
}

void ProcessConfigCommands(shared_ptr<Visualizer_GLUT> vis)
{
  ParamsHandle config = SimpleConfig::GetInstance();
  int i = 1;
  while (1)
  {
    char buf[100];
    sprintf_s(buf, 100, "Commands.%d", i);
    string cmd = config->Get(buf, "");
    if (cmd == "") break;
    vis->OnMenu(cmd);
    i++;
  }
}

void PrintHelpText()
{
  printf("SIMULATOR!\n");
  printf("Select main window to interact with keyboard/mouse:\n");
  printf("LEFT DRAG / X+LEFT DRAG / Z+LEFT DRAG = rotate, pan, zoom camera\n");
  printf("W/S/UP/LEFT/DOWN/RIGHT - apply force\n");
  printf("C - clear all graphs\n");
  printf("R - reset simulation\n");
  printf("Space - pause simulation\n");
}
