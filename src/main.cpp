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


QuadcopterHandle quad;
shared_ptr<Trajectory> followed_traj;

shared_ptr<Visualizer_GLUT> visualizer;
shared_ptr<GraphManager> grapher;

float dtSim = 0.001f;
Timer lastDraw;
V3F force, moment;

float simulationTime=0;
int randomNumCarry=0;
string flightMode;

void OnTimer(int v);

int main(int argcp, char **argv)
{
  PrintHelpText();
 
  // load parameters
  ParamsHandle config = SimpleConfig::GetInstance();

  // initialize visualizer
  visualizer.reset(new Visualizer_GLUT(&argcp, argv));
  grapher.reset(new GraphManager(false));

  // create a quadcopter to simulate
  quad = QuadDynamics::Create("Quad");

  // Initialise the trajectory log
  string followedTrajFile = string("../config/") + config->Get("Sim.LoggedStateFile", "");
  followed_traj.reset(new Trajectory());
  followed_traj->SetLogFile(followedTrajFile);
  quad->followedTrajectoryCallback = MakeDelegate(followed_traj.get(), &Trajectory::AddTrajectoryPoint);

  grapher->RegisterDataSource("Quad",quad);

  visualizer->InitializeMenu(grapher->GetGraphableStrings());

  visualizer->quad = quad;
  visualizer->graph = grapher;
  visualizer->followed_traj = followed_traj;

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
  
  // Reset the followed trajectory
  followed_traj->Clear();
  string followedTrajFile = string("../config/") + config->Get("Sim.LoggedStateFile", "");
  followed_traj->SetLogFile(followedTrajFile);
  
  quad->Reset();
  grapher->Clear();
}

void OnTimer(int v)
{
  ParamsHandle config = SimpleConfig::GetInstance();
  
  // logic to reset the simulation based on key input or reset conditions
  float endTime = config->Get("Sim.EndTime",-1.f);
  if(receivedResetRequest ==true ||
     (ToUpper(config->Get("Sim.RunMode", "Continuous"))=="REPEAT" && endTime>0 && simulationTime >= endTime))
  {
    ResetSimulation();
  }
  
  // main loop
  if (!paused)
  {
    grapher->UpdateData(simulationTime);
    quad->Run(dtSim, simulationTime, randomNumCarry, force, moment, flightMode);
    simulationTime += dtSim;
  }
  
  KeyboardInteraction(force, visualizer);
  
  if (lastDraw.ElapsedSeconds() > 0.030)
  {
    visualizer->SetArrow(quad->Position() - force, quad->Position());
    visualizer->Update();
    grapher->DrawUpdate();
    lastDraw.Reset();
  }
  
  glutTimerFunc(5,&OnTimer,0);
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
  if (visualizer->IsKeyDown('w'))
  {
    force += V3F(forceStep, 0, 0);
    keyPressed = true;
  }
  if (visualizer->IsKeyDown('s'))
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

  if (visualizer->IsKeyDown('c'))
  {
    visualizer->graph->graph->RemoveAllSeries();
  }

  if (visualizer->IsKeyDown('r'))
  {
    receivedResetRequest = true;
  }

  static bool key_p_pressed = false;
  static bool key_t_pressed = false;
  static bool key_space_pressed = false;

  if (visualizer->IsKeyDown('p'))
  {
    if (!key_p_pressed)
    {
      key_p_pressed = true;
      visualizer->showPropCommands = !visualizer->showPropCommands;
    }
  }
  else
  {
    key_p_pressed = false;
  }

  if (visualizer->IsKeyDown('t'))
  {
    if (!key_t_pressed)
    {
      key_t_pressed = true;
      visualizer->showTrajectory = !visualizer->showTrajectory;
    }
  }
  else
  {
    key_t_pressed = false;
  }

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
  printf("P - show thrusts\n");
  printf("T - show trajectory\n");
  printf("C - clear all graphs\n");
  printf("R - reset simulation\n");
  printf("Space - pause simulation\n");
}
