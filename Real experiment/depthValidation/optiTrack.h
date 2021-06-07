#ifndef _OPTITRACK_H_
#define _OPTITRACK_H_

// Velocity calculation
typedef struct
{
	float linearVelX;
	float linearVelY;
	float linearVelZ;
	float angleVelX;
	float angleVelY;
	float angleVelZ;
} Velocity;

// Acceleration calculation
typedef struct
{
	float linearAccX;
	float linearAccY;
	float linearAccZ;
} Acceleration;

// Define macro variable
#define MATH_PI 3.14159265F

// World Up Axis
extern int upAxis;

// Obtain the time stamp
extern double timeStampLast, timeStampSecLast;

// Save the previous pose to calculate the velocity and acceleration
extern float xPosLast, yPosLast, zPosLast;
extern float xOrienLast, yOrienLast, zOrienLast;

extern float xPosSecLast, yPosSecLast, zPosSecLast;

extern Velocity vel;
extern Acceleration acc;

void ConvertRHSPosZupToYUp(float& x, float& y, float& z);
void ConvertRHSRotZUpToYUp(float& qx, float& qy, float& qz, float& qw);

#endif // !_OPTITRACK_H_

