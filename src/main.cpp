#include "Common.h"
#include "Trajectory.h"
#include "Utility/Timer.h"

#include "Drawing/Visualizer_GLUT.h"
#include "Simulation/QuadDynamics.h"
#include "Simulation/Simulator.h"
#include "Utility/SimpleConfig.h"
#include "Utility/StringUtils.h"
#include "Drawing/GraphManager.h"
#include "GL/glut.h"

using SLR::Quaternion;
using SLR::ToUpper;

void KeyboardInteraction(V3F& force, Visualizer_GLUT& vis);
bool receivedResetRequest = true;
bool paused = false;
void PrintHelpText();
void ProcessConfigCommands(Visualizer_GLUT& vis);

int main(int argcp, char **argv)
{
  PrintHelpText();
 
  // load parameters
  ParamsHandle config = SimpleConfig::GetInstance();

  // initialize visualizer
  Visualizer_GLUT visualizer(&argcp, argv);
  shared_ptr<GraphManager> grapher(new GraphManager(false));

  // initialize simulation
  Simulator sim;

  // create a quadcopter to simulate
  QuadcopterHandle quad = QuadDynamics::Create("Quad");
  sim.AddVehicle(quad);

  // Initialise the trajectory log
  string followedTrajFile = string("../config/") + config->Get("Sim.LoggedStateFile", "");
  shared_ptr<Trajectory> followed_traj(new Trajectory());
  followed_traj->SetLogFile(followedTrajFile);
  quad->followedTrajectoryCallback = MakeDelegate(followed_traj.get(), &Trajectory::AddTrajectoryPoint);

  grapher->RegisterDataSource("Quad",quad);

  visualizer.InitializeMenu(grapher->GetGraphableStrings());

  visualizer.quad = quad;
  visualizer.graph = grapher;
  visualizer.followed_traj = followed_traj;

  float dtSim = 0.001f;
  Timer lastDraw;

  V3F force, moment;

  float simulationTime=0;
  int randomNumCarry=0;
  string flightMode;

  ProcessConfigCommands(visualizer);
  
  // main loop - everything is happening here
  while (!visualizer._exiting)
  {
    // logic to reset the simulation based on key input or reset conditions
    float endTime = config->Get("Sim.EndTime",-1.f);
    if(receivedResetRequest ==true || 
      (ToUpper(config->Get("Sim.RunMode", "Continuous"))=="REPEAT" && endTime>0 && simulationTime >= endTime))
    {
      receivedResetRequest = false;
      simulationTime = 0;
      config->Reset();
      dtSim = config->Get("Sim.Timestep", 0.001f);
      
      flightMode = config->Get("Quad.SimMode","Full3D");

      // Reset the followed trajectory
      followed_traj->Clear();
      string followedTrajFile = string("../config/") + config->Get("Sim.LoggedStateFile", ""); 
      followed_traj->SetLogFile(followedTrajFile);

      quad->Reset();
      grapher->Clear();
    }

    // main loop
    if (!paused)
    {
      grapher->UpdateData(simulationTime);
      quad->Run(dtSim, simulationTime, randomNumCarry, force, moment, flightMode);
    }

    KeyboardInteraction(force, visualizer);

    if (lastDraw.ElapsedSeconds() > 0.030)
    {
      visualizer.SetArrow(quad->Position() - force, quad->Position());
      visualizer.Update();
      grapher->DrawUpdate();
      lastDraw.Reset();
    }

    // Only increment the simulation time if not paused
    if (!paused)
    {
      simulationTime += dtSim;
    }
    Sleep((int)(dtSim * 1000));
  } // end of while loop

  return 0;
}

void KeyboardInteraction(V3F& force, Visualizer_GLUT& visualizer)
{
  bool keyPressed = false;
  const float forceStep = 0.04f;

  if (visualizer.IsSpecialKeyDown(GLUT_KEY_LEFT))
  {
    force += V3F(0, -forceStep, 0);
    keyPressed = true;
  }
  if (visualizer.IsSpecialKeyDown(GLUT_KEY_UP))
  {
    force += V3F(0, 0, -forceStep);
    keyPressed = true;
  }
  if (visualizer.IsSpecialKeyDown(GLUT_KEY_RIGHT))
  {
    force += V3F(0, forceStep, 0);
    keyPressed = true;
  }
  if (visualizer.IsSpecialKeyDown(GLUT_KEY_DOWN))
  {
    force += V3F(0, 0, forceStep);
    keyPressed = true;
  }
  if (visualizer.IsKeyDown('w'))
  {
    force += V3F(forceStep, 0, 0);
    keyPressed = true;
  }
  if (visualizer.IsKeyDown('s'))
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

  if (visualizer.IsKeyDown('c'))
  {
    visualizer.graph->graph->RemoveAllSeries();
  }

  if (visualizer.IsKeyDown('r'))
  {
    receivedResetRequest = true;
  }


  static bool key_p_pressed = false;
  static bool key_t_pressed = false;
  static bool key_space_pressed = false;

  if (visualizer.IsKeyDown('p'))
  {
    if (!key_p_pressed)
    {
      key_p_pressed = true;
      visualizer.showPropCommands = !visualizer.showPropCommands;
    }
  }
  else
  {
    key_p_pressed = false;
  }

  if (visualizer.IsKeyDown('t'))
  {
    if (!key_t_pressed)
    {
      key_t_pressed = true;
      visualizer.showTrajectory = !visualizer.showTrajectory;
    }
  }
  else
  {
    key_t_pressed = false;
  }

  if (visualizer.IsKeyDown(' '))
  {
    if (!key_space_pressed)
    {
      key_space_pressed = true;
      paused = !paused;
      visualizer.paused = paused;
    }
  }
  else
  {
    key_space_pressed = false;
  }
}

void ProcessConfigCommands(Visualizer_GLUT& vis)
{
  ParamsHandle config = SimpleConfig::GetInstance();
  int i = 1;
  while (1)
  {
    char buf[100];
    sprintf_s(buf, 100, "Commands.%d", i);
    string cmd = config->Get(buf, "");
    if (cmd == "") break;
    vis.OnMenu(cmd);
    i++;
  }
}

void PrintHelpText()
{
  printf("SIMULATOR!\n");
  printf("Select main window to interact with keyboard/mouse:\n");
  printf("LEFT DRAG / CTRL+LEFT DRAG / SHIFT+LEFT DRAG = rotate, pan, zoom camera\n");
  printf("W/S/UP/LEFT/DOWN/RIGHT - apply force\n");
  printf("P - show thrusts\n");
  printf("T - show trajectory\n");
  printf("C - clear all graphs\n");
  printf("R - reset simulation\n");
  printf("Space - pause simulation\n");
}
