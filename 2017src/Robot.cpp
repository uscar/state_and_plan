#include "Robot.h"

//Constructor
Robot::Robot(double x, double y, double z, double dir) {
	mX = x;
	mY = y;
	mZ = z;
	mDir = dir;
}

//Getters
double Robot::GetX() {
	return mX;
}
double Robot::GetY() {
	return mY;
}
double Robot::GetZ() {
	return mZ;
}
double Robot::GetDir() {
	return mDir;
}

//Setters
double Robot::SetX(double x) {
	mX = x;
}
double Robot::SetY(double y) {
	mY = y;
}
double Robot::SetZ(double z) {
	mZ = z;
}
double Robot::SetDir(double dir) {
	mDir = dir;
}