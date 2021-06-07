#ifndef _DEPTH_VALIDATION_H_
#define _DEPTH_VALIDATION_H_

// Include the necessary header files
#include <Windows.h>
#include <Kinect.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/ml/ml.hpp>
#include <math.h>
#include <time.h>
#include <ctype.h>
#include <algorithm>
#include <vector>
#include <iterator>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <tchar.h>
#include <conio.h>
#include <thread>
#include <fstream>
#include <iostream>
#include <cstring>

// Parallel computing
#include <omp.h>

// Mat_lab 
#include <engine.h>

// Ei_gen lib
#include <Eigen/Core>
#include <Eigen/Dense>

// Opti_track system
#include "NatNetClient.h"
#include "NatNetTypes.h"
#include "optiTrack.h"

using namespace std;
using namespace cv;
using namespace Eigen;

// Declare the global variable
extern int iConnectionType;
extern char szMyIPAddress[];
extern char szServerIPAddress[];
extern int analogSamplesPerMocapFrame;
extern unsigned int MyServersDataPort;
extern unsigned int MyServersCommandPort;

class NatNetClient;
extern NatNetClient * theClient;


// Declare the template function to release the interface pointer safely
template<typename Interface> 
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

// Declare the function for optiTrack motion capture system
int createClient(int iConnectionType);
int optiTrack();

// Receives data from the server
void __cdecl DataHandler(sFrameOfMocapData* data, void* pUserData);
// Receives NatNet error messages
void __cdecl MessageHandler(int msgType, char* msg);		            
void resetClient();

// Design the observer
double movingAveFilter(double *originalSignal, int sizeWindow, int idxImg);
double medianFilter(double *originalSignal, int sizeWindow, int idxImg);
double observer(int x, int y, int xLast, int yLast, double & rdHatLast, double *pCurVel, double *pLastVel, double *pLastAcc);

// Compare the estimated depth with ground-truth by plotting using the mat_lab engine
int plotDepth(double trueDep[][4], double estiDep[][4], double velCamArray[][6], double accCamArray[][3], int nImage);

#endif // !_DEPTH_VALIDATION_H_
