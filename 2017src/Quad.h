#pragma once

#include "Robot.h"

class Quad: public Robot
{
public:
	Quad(double x, double y, double z, double dir);
	void lookaround();
	void rotate(double rad);
	void move(double distance);
	void ascend(double z);
	void descend(double z);
	void shortHop();
	void evade();
}

