#pragma once


#include "SandboxController.h"
#include "SimpleAttitudeController.h"
#include "FullCascadedController.h"



inline ControllerHandle CreateController(string controllerType, string config)
{
  ControllerHandle ret;

  if (controllerType == "SandboxController")
  {
    ret.reset(new SandboxController(config));
  }

  else if (controllerType == "SimpleAttitudeController")
  {
    ret.reset(new SimpleAttitudeController(config));
  }

  else if (controllerType == "FullCascadedController")
  {
    ret.reset(new FullCascadedController (config));
  }
  
  return ret;
}