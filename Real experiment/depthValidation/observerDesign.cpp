#include "depthValidation.h"
#include "optiTrack.h"

// The proposed observer
double observer(int x, int y, int xLast, int yLast, double & rdHatLast, double *pCurVel, double *pLastVel, double *pLastAcc)
{
	// Define the variables
	double depth, dHat;
	double fai, faiLast, faiLastDer1t;
	double phi;
	Eigen::Vector2d g1, g2, faiLastDer1p;
	double g3;

	// Set the gain and the period, 30 is the fps
	double k = 0.000008, period = 1.0 / 30;
	
	// Camera calibration using MATLAB toolbox, focal length is expressed in pixels
	int focalLen = 1060;
	int principalPointX = 955, principalPointY = 542;

	// Obtain the coordinates in the image system
	int imgX = x - principalPointX, imgY = y - principalPointY;
	int imgXLast = xLast - principalPointX, imgYLast = yLast - principalPointY;

	// Define the variables
	double vxLast, vyLast, vzLast, wxLast, wyLast, wzLast, accxLast, accyLast, acczLast;
	double vx, vy, vz;

	// Obtain the current and previous velocity and acceleration
	vx = pCurVel[0]; vy = pCurVel[1]; vz = pCurVel[2];
	
	vxLast = pLastVel[0]; vyLast = pLastVel[1]; vzLast = pLastVel[2];
	wxLast = pLastVel[3]; wyLast = pLastVel[4]; wzLast = pLastVel[5];

	accxLast = pLastAcc[0]; accyLast = pLastAcc[1]; acczLast = pLastAcc[2];
		
	// Calculate the depth, using the "ode1" method, and the symbols are from the IROS2018 paper
	g1(0) = -vxLast*focalLen + vzLast*imgXLast;
	g1(1) = -vyLast*focalLen + vzLast*imgYLast;
	g2(0) = imgXLast*imgYLast*wxLast / focalLen - (focalLen + pow(imgXLast, 2) / focalLen)*wyLast + imgYLast*wzLast;
	g2(1) = (focalLen + pow(imgYLast, 2) / focalLen)*wxLast - imgXLast*imgYLast*wyLast / focalLen - imgXLast*wzLast;
	g3 = (wxLast*imgYLast - wyLast*imgXLast) / focalLen;

	faiLast = -(vxLast*imgXLast + vyLast*imgYLast)*focalLen + 0.5*vzLast*(pow(imgXLast, 2) + pow(imgYLast, 2));
	faiLastDer1p(0) = -vxLast*focalLen + vzLast*imgXLast;
	faiLastDer1p(1) = -vyLast*focalLen + vzLast*imgYLast;
	faiLastDer1t = -(accxLast*imgXLast + accyLast*imgYLast)*focalLen + 0.5*acczLast*(pow(imgXLast, 2) + pow(imgYLast, 2));

	phi = 1.0 / log(20) * g3 - k*(faiLastDer1p.transpose()*g2 + faiLastDer1t) -
		pow(20, rdHatLast + k*faiLast)*(-vzLast / log(20) + k*faiLastDer1p.transpose()*g1);
	dHat = rdHatLast + period*phi;

	fai = -(vx*imgX + vy*imgY)*focalLen + 0.5*vz*(pow(imgX, 2) + pow(imgY, 2));
	depth = 1.0 / pow(20, dHat + k*fai);

	rdHatLast = dHat;
	return depth;
}