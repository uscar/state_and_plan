#pragma once

#include "Robot.h"
#include <tuple>

class Roomba: public Robot
{
 public:
	 Roomba(double x, double y, double z, double dir);
  std::tuple<double, double, double, long> simulate(double time);
  const double speed = 0.33;
};
