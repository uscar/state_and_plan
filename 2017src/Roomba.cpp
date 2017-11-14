#include "Roomba.h"
#include "math.h"

tuple<double x, double y, double z, long theta> simulate(double time){
	return make_tuple(x+(speed*cos(theta)), y+(speed*sin(theta)), z, theta);
}