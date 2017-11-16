#include "Roomba.h"
#include "math.h"

Roomba::Roomba(double x, double y, double z, double dir) : Robot(x, y, z, dir) {

}

std::tuple<double, double, double, long> simulate(double time){
	return std::make_tuple(0.0, 0.0, 0.0, 0);
	//return std::make_tuple(x+(speed*cos(theta)*time), y+(speed*sin(theta)*time), z, theta);
}