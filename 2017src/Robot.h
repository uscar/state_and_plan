#pragma once

//Base robot class that roomba and quad will inherit from

class Robot {

public:

	//Constructor
	Robot(double x, double y, double z, double dir);

	//Getters
	double GetX();
	double GetY();
	double GetZ();
	double GetDir();

	//Setters
	double SetX(double x);
	double SetY(double y);
	double SetZ(double z);
	double SetDir(double dir);

private:

	double mX;
	double mY;
	double mZ;
	double mDir;

};