#pragma once

#include "QuadControl.h"

inline ControllerHandle CreateController(string controllerType, string config)
{
  ControllerHandle ret;

  if (controllerType == "QuadControl")
  {
    ret.reset(new QuadControl(config));
  }
  
  return ret;
}