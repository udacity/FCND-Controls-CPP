// Random Number Utilities Header, super-lightweight-robotics library
// License: BSD-3-clause

#pragma once

// random number routines from Numerical Recipes
double gasdev(int &idum);
double ran1(int &idum);

inline float ran1_inRange(float min, float max, int& idum)
{
  return (float)(ran1(idum)*(max-min)+min);
}

inline double ran1_inRange(double min, double max, int& idum)
{
  return (double)(ran1(idum)*(max-min)+min);
}