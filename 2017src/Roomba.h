#pragma once

#include "Robot.h"
#include <tuple>

class Roomba: public Robot
{
 public:
  tuple<double x, double y, double z, long theta> simulate(double time);
  const double speed = 0.33;
};
