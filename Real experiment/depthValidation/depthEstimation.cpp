/*
*************************************************************************************
*                                                                                   *
*  Author: iMotion          Version: 1.0.0            Goal: depth validation        *
*                                                                                   *
*************************************************************************************

Usage [optional]:

depthValidation [ServerIP] [LocalIP] [OutputFilename]

[ServerIP]			IP address of the server (e.g. 127.0.0.1) ( defaults to local machine)
[OutputFilename]	Name of points file (pts) to write out (defaults to output.pts)
*/

#include <depthValidation.h>

// Define the global variable
NatNetClient * theClient = NULL;
int analogSamplesPerMocapFrame = 0;
unsigned int MyServersDataPort = 1510;
unsigned int MyServersCommandPort = 1511;

char szMyIPAddress[128] = "";
char szServerIPAddress[128] = "";

int iConnectionType = ConnectionType_Multicast;


int main(int argc, char * argv[])
{
	/*Step I. Set the OptiTrack motion cpature system****************************/
	// Optitrack motion capture system, parse command line arguments
	if (argc > 1)
	{
		// Specified on command line
		strcpy_s(szServerIPAddress, argv[1]);
		printf("Connecting to server at %s...\n", szServerIPAddress);
	}
	else
	{
		// Not specified - assume server is local machine
		strcpy_s(szServerIPAddress, "");
		printf("Connecting to server at LocalMachine\n");
	}

	if (argc > 2)
	{
		// Specified on command line
		strcpy_s(szMyIPAddress, argv[2]);
		printf("Connecting from %s...\n", szMyIPAddress);
	}
	else
	{
		// Not specified - assume server is local machine
		strcpy_s(szMyIPAddress, "");
		printf("Connecting from Local Machine...\n");
	}

	// Connection to a NatNet server application is accomplished by an instance of NatNetClient object
	int iResult;
	iResult = createClient(iConnectionType);
	if (iResult != ErrorCode_OK)
	{
		printf("Error initializing client.  See log for details.  Exiting...\n");
		return 1;
	}
	else
	{
		printf("Client initialized and ready. Congratulations!\n");
	}

	// Output the relevant information about the optiTrack system
	iResult = optiTrack();
	if (iResult != ErrorCode_OK)
	{
		cout << "Output the info of optiTrack in error. Exiting...\n" << endl;
		return 1;
	}
	else
	{
		cout << "Output the info of optiTrack correctly. Congratulations!\n" << endl;
	}

	// Call the opti_track motion capture system to obtain the camera velocity, set the callback handlers
	theClient->SetVerbosityLevel(Verbosity_Warning);
	theClient->SetMessageCallback(MessageHandler);

	// Receive data from the server by the call back function
	theClient->SetDataCallback(DataHandler, theClient);
	/*End of Step I. Set the OptiTrack motion cpature system****************************/
	
	
	/*Step II. Define the variables for velocity measurement and sequence tracking**************/
	// Define the variable to fix the tracking sequences to prevent numbered disturbances
	double massCenSeqLast[4][2] = { 0 };

	// Define the index variable
	int numImg = 0;

	// Define the array to store the ground-truth and estimated depth
	double groundTruth[5000][4] = { 0 };
	double estimatedDepth[5000][4] = { 0 };

	int massCenterCoorLast[4][2] = { 0 };
	int massCenterCoorX = 0, massCenterCoorY = 0;

	// Define the 2D array to store the velocities and the accelerations of the camera
	double velCameraOrig[5000][6] = { 0 }, accCameraOrig[5000][3] = { 0 };
	double velCameraFilt[5000][6] = { 0 }, accCameraFilt[5000][3] = { 0 };

	// Define the 2D array to store the image feature coordinates in pixel
	int coorImgFeat[5000][8] = { 0 };

	// Define the 1D array to store the current and last velocity and acceleration after filtering
	double currentVel[6] = { 0 }, currentAcc[3] = { 0 };
	double lastTimeVel[6] = { 0 }, lastTimeAcc[3] = { 0 };

	// Initialize the dHatLast
	double dHatLast[4] = { 0.11906, 0.11906, 0.11906, 0.11906 };

	//  Save the data for plotting using MATLAB
	ofstream outVel, outAcc, outGndTruth, outEstDepth, outImgFeat;
	outVel.open("F:\\lixiangfei\\07 Depth estimation experiment\\data\\velocity.txt", ios::out | ios::trunc);
	outAcc.open("F:\\lixiangfei\\07 Depth estimation experiment\\data\\acceleration.txt", ios::out | ios::trunc);
	outGndTruth.open("F:\\lixiangfei\\07 Depth estimation experiment\\data\\trueDepth.txt", ios::out | ios::trunc);
	outEstDepth.open("F:\\lixiangfei\\07 Depth estimation experiment\\data\\estiDepth.txt", ios::out | ios::trunc);
	outImgFeat.open("F:\\lixiangfei\\07 Depth estimation experiment\\data\\imgFeat.txt", ios::out | ios::trunc);
	/*End of Step II. Define the variables for velocity measurement and sequence tracking**************/


	/*Step III. Set the Kinect for image capture and processing**************/
	// KINECT V2 sensor
	IKinectSensor * pSensor = nullptr;
	HRESULT hResult = S_OK;
	hResult = GetDefaultKinectSensor(&pSensor);
	if (FAILED(hResult))
	{
		cerr << "Error : GetDefaultKinectSensor" << endl;
		return -1;
	}

	// Open the sensor
	hResult = pSensor->Open();
	if (FAILED(hResult))
	{
		cerr << "Error : IKinectSensor::Open()" << endl;
		return -1;
	}

	// Get the source
	IColorFrameSource * pColorSource = nullptr;
	hResult = pSensor->get_ColorFrameSource(&pColorSource);
	if (FAILED(hResult))
	{
		cerr << "Error : IKinectSensor::get_ColorFrameSource()" << endl;
		return -1;
	}

	IDepthFrameSource * pDepthSource = nullptr;
	hResult = pSensor->get_DepthFrameSource(&pDepthSource);
	if (FAILED(hResult))
	{
		cerr << "Error : IKinectSensor::get_DepthFrameSource()" << endl;
		return -1;
	}

	// Get the reader
	IColorFrameReader * pColorReader = nullptr;
	hResult = pColorSource->OpenReader(&pColorReader);
	if (FAILED(hResult))
	{
		cerr << "Error : IColorFrameSource::OpenReader()" << endl;
		return -1;
	}

	IDepthFrameReader * pDepthReader = nullptr;
	hResult = pDepthSource->OpenReader(&pDepthReader);
	if (FAILED(hResult))
	{
		cerr << "Error : IDepthFrameSource::OpenReader()" << endl;
		return -1;
	}

	// Color frame description
	IFrameDescription * pColorDescription = nullptr;
	hResult = pColorSource->get_FrameDescription(&pColorDescription);
	if (FAILED(hResult))
	{
		cerr << "Error : IColorFrameSource::get_FrameDescription()" << endl;
		return -1;
	}

	int colorBufferWidth = 0, colorBufferHeight = 0;
	pColorDescription->get_Width(&colorBufferWidth); // 1920
	pColorDescription->get_Height(&colorBufferHeight); // 1080
	unsigned int colorBufferSize = colorBufferWidth * colorBufferHeight * 4 * sizeof(unsigned char);

	// Define mat variable used in OpenCV
	Mat colorBufferMat(colorBufferHeight, colorBufferWidth, CV_8UC4);

	// Define mat variable used for the image processing and circle center detection
	int startRectX = 800, startRectY = 400;
	int rectColorWidth = 320, rectColorHeight = 240;
	Rect rect(startRectX, startRectY, rectColorWidth, rectColorHeight);
	Mat rectColorMat(rectColorHeight, rectColorWidth, CV_8UC4);

	// Depth frame description
	IFrameDescription * pDepthDescription = nullptr;
	hResult = pDepthSource->get_FrameDescription(&pDepthDescription);
	if (FAILED(hResult))
	{
		cerr << "Error : IDepthFrameSource::get_FrameDescription()" << endl;
		return -1;
	}

	int depthWidth = 0, depthHeight = 0;
	pDepthDescription->get_Width(&depthWidth); // 512
	pDepthDescription->get_Height(&depthHeight); // 424
	unsigned int depthBufferSize = depthWidth * depthHeight * sizeof(unsigned short);

	// Define mat variable used for the depth data processing
	Mat depthBufferMat(depthHeight, depthWidth, CV_16UC1);
	Mat depthMat(depthHeight, depthWidth, CV_8UC1);

	/* Get the depth data and camera space point, and use "new" to build a memory session.
	Note that the following two line code must be placed outside the while loop, or the memory will explode*/
	UINT16 * depthData = new UINT16[512 * 424];
	CameraSpacePoint *  p_cameraSpacePoints = new CameraSpacePoint[1920 * 1080];

	// Coordinate Mapper to get the depth
	ICoordinateMapper * pCoordinateMapper = nullptr;
	hResult = pSensor->get_CoordinateMapper(&pCoordinateMapper);
	if (FAILED(hResult))
	{
		cerr << "Error : IKinectSensor::get_CoordinateMapper()" << endl;
		return -1;
	}

	// Define frame pointers
	IColorFrame * pColorFrame = nullptr;
	IDepthFrame * pDepthFrame = nullptr;

	// Calculate the running time of the program
	clock_t startTime = clock();

	// Image capture loop
	while (true)
	{
		// Color Frame
		hResult = pColorReader->AcquireLatestFrame(&pColorFrame);
		if (FAILED(hResult) || !pColorFrame)
		{
			continue;
		}
		if (SUCCEEDED(hResult))
		{
			hResult = pColorFrame->CopyConvertedFrameDataToArray(colorBufferSize, reinterpret_cast<BYTE*>(colorBufferMat.data), ColorImageFormat::ColorImageFormat_Bgra);
			if (SUCCEEDED(hResult))
			{
				// Get the region-of-interest image to reduce the computation load
				colorBufferMat(rect).copyTo(rectColorMat);
			}
		}

		// Show the intercepted image
		//imshow("Original image", colorBufferMat);
		//imshow("Rectangle color image", rectColorMat);

		Mat grayMat, binaryMat, cannyMat;

		// Turn the color image into the gray image
		cvtColor(rectColorMat, grayMat, CV_BGR2GRAY);
		//GaussianBlur(grayMat, grayMat, Size(9, 9), 2, 2);

		// Gray image threshold
		threshold(grayMat, binaryMat, 70, 180, CV_THRESH_BINARY);

		// Detect the edges using Canny algorithm
		Canny(binaryMat, cannyMat, 100, 100 * 2, 3);

		// Find the contours in the region-of-interest image
		std::vector<std::vector<cv::Point>> contours, realContours;
		std::vector<cv::Vec4i> hierarchy;

		findContours(cannyMat, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);

		// Remove the repeated contours
		std::vector<int> numRealCont;
		for (int i = 0; i < contours.size();)
		{
			int next = hierarchy[i][0];
			numRealCont.push_back(i);
			realContours.push_back(contours[i]);
			i = next;
			if (i == -1)
			{
				break;
			}
		}

		// Define the variables for mass center calculation 
		vector<Moments> contourMoment(realContours.size());
		vector<Point2d> massCenter(realContours.size());
		vector<Point> massCenterInWholeImg(realContours.size());

		// Define the variable for contour drawing
		Mat contourMat = Mat::zeros(cannyMat.size(), CV_8U);

		// Obtain the original velocities and accelerations of camera
		velCameraOrig[numImg][0] = vel.linearVelX;
		velCameraOrig[numImg][1] = vel.linearVelY;
		velCameraOrig[numImg][2] = vel.linearVelZ;
		velCameraOrig[numImg][3] = vel.angleVelX;
		velCameraOrig[numImg][4] = vel.angleVelY;
		velCameraOrig[numImg][5] = vel.angleVelZ;

		accCameraOrig[numImg][0] = acc.linearAccX;
		accCameraOrig[numImg][1] = acc.linearAccY;
		accCameraOrig[numImg][2] = acc.linearAccZ;

		// Increase the index
		numImg++;
		
		// Obtain the every column data of velocity and acceleration
		double **ppVelWindows = new double *[6], **ppAccWindows = new double *[3];
		for (int j = 0; j < 6; j++)
		{
			ppVelWindows[j] = new double[numImg];
			for (int i = 0; i < numImg; i++)
			{
				ppVelWindows[j][i] = velCameraOrig[i][j];
			}
		}

		for (int j = 0; j < 3; j++)
		{
			ppAccWindows[j] = new double[numImg];
			for (int i = 0; i < numImg; i++)
			{
				ppAccWindows[j][i] = accCameraOrig[i][j];
			}
		}

		// Decrease the index
		numImg--;

		// Filter the velocity signals by the moving average filter
		currentVel[0] = movingAveFilter((double *)ppVelWindows[0], 1, numImg);
		currentVel[1] = movingAveFilter((double *)ppVelWindows[1], 5, numImg);
		currentVel[2] = movingAveFilter((double *)ppVelWindows[2], 1, numImg);
		currentVel[3] = movingAveFilter((double *)ppVelWindows[3], 5, numImg);
		currentVel[4] = movingAveFilter((double *)ppVelWindows[4], 5, numImg);
		currentVel[5] = movingAveFilter((double *)ppVelWindows[5], 5, numImg);

		// Filter the acceleration signals by combining the median filter and the moving average filter
		currentAcc[0] = movingAveFilter((double *)ppAccWindows[0], 4, numImg);
		currentAcc[1] = movingAveFilter((double *)ppAccWindows[1], 4, numImg);
		currentAcc[2] = movingAveFilter((double *)ppAccWindows[2], 4, numImg);

		// Delete the array for velocity and acceleration
		for (int i = 0; i < 6; i++)
		{
			delete[] ppVelWindows[i];
		}

		for (int i = 0;i < 3;i++)
		{
			delete[] ppAccWindows[i];
		}
		delete[] ppVelWindows;
		delete[] ppAccWindows;

		// Depth Frame
		hResult = pDepthReader->AcquireLatestFrame(&pDepthFrame);
		if (SUCCEEDED(hResult) && realContours.size() == 4)
		{
			// Map the color frame to camera space
			pDepthFrame->CopyFrameDataToArray(depthWidth*depthHeight, depthData);
			hResult = pCoordinateMapper->MapColorFrameToCameraSpace(512 * 424, depthData, 1920 * 1080, p_cameraSpacePoints);

			// Define the new index
			unsigned int idxNew, pIndex;

			// Obtain the depth of mass centers
			vector<Point3d> massCenterCameraCoor(realContours.size());

			// Parallel computing
			// #pragma omp parallel for
			for (int idx = 0; idx < realContours.size(); idx++)
			{
				// Calculate the moments
				contourMoment[idx] = moments(contours[numRealCont[idx]], false);
				if (contourMoment[idx].m00 < FLT_EPSILON)
				{
					massCenter[idx] = Point2d(0.0, 0.0);
					continue;
				}
				else
				{
					// Calculate the mass centers
					massCenter[idx] = Point2d(contourMoment[idx].m10 / contourMoment[idx].m00,
						contourMoment[idx].m01 / contourMoment[idx].m00);

					if (numImg == 0)
					{
						massCenSeqLast[idx][0] = massCenter[idx].x;
						massCenSeqLast[idx][1] = massCenter[idx].y;

						massCenterInWholeImg[idx].x = cvRound(massCenter[idx].x) + startRectX;
						massCenterInWholeImg[idx].y = cvRound(massCenter[idx].y) + startRectY;

						pIndex = massCenterInWholeImg[idx].y * 1920 + massCenterInWholeImg[idx].x;
						if (pIndex >= 1920 * 1080)
						{
							continue;
						}
						else
						{
							// Calculate the mass center in camera coordinate system to obtain the true depth
							massCenterCameraCoor[idx] = Point3d(p_cameraSpacePoints[pIndex].X, p_cameraSpacePoints[pIndex].Y,
								p_cameraSpacePoints[pIndex].Z);

							// Obtain the ground-true
							groundTruth[numImg][idx] = massCenterCameraCoor[idx].z;

							// Obtain the estimated depth using the observer
							massCenterCoorX = massCenterInWholeImg[idx].x;
							massCenterCoorY = massCenterInWholeImg[idx].y;

							estimatedDepth[numImg][idx] = observer(massCenterCoorX, massCenterCoorY,
								massCenterCoorLast[idx][0], massCenterCoorLast[idx][1], dHatLast[idx],
								currentVel, lastTimeVel, lastTimeAcc);

							// Save the previous mass center coordinates
							massCenterCoorLast[idx][0] = massCenterCoorX;
							massCenterCoorLast[idx][1] = massCenterCoorY;

							// Save the mass center coordinates to the outImgFeat file
							outImgFeat << massCenterCoorX << "\t" << massCenterCoorY << "\t";
						}
					}
					else
					{
						// Fix the tracking sequences by calculating the distance between the current and last mass centers
						std::vector<double> distMassCenters;
						for (int j = 0; j < 4; j++)
						{
							distMassCenters.push_back(fabs(massCenter[idx].x - massCenSeqLast[j][0]) + fabs(massCenter[idx].y - massCenSeqLast[j][1]));
						}

						// Index the position of the smallest
						auto smallest = std::min_element(std::begin(distMassCenters), std::end(distMassCenters));
						idxNew = static_cast<int>(std::distance(std::begin(distMassCenters), smallest));

						// Clear the all variables in the vector distMassCenters
						distMassCenters.clear();

						// Save the current mass centers
						massCenSeqLast[idxNew][0] = massCenter[idx].x;
						massCenSeqLast[idxNew][1] = massCenter[idx].y;

						// Map the mass centers to the entire image
						Point tempMassCenter = Point(cvRound(massCenter[idx].x), cvRound(massCenter[idx].y));
						massCenterInWholeImg[idxNew].x = tempMassCenter.x + startRectX;
						massCenterInWholeImg[idxNew].y = tempMassCenter.y + startRectY;

						// Remove the outliers
						pIndex = massCenterInWholeImg[idxNew].y * 1920 + massCenterInWholeImg[idxNew].x;
						if (pIndex >= 1920 * 1080)
						{
							continue;
						}
						else
						{
							// Calculate the mass center in camera coordinate system to obtain the true depth
							massCenterCameraCoor[idxNew] = Point3d(p_cameraSpacePoints[pIndex].X, p_cameraSpacePoints[pIndex].Y,
								p_cameraSpacePoints[pIndex].Z);

							// Obtain the ground-true
							groundTruth[numImg][idxNew] = massCenterCameraCoor[idxNew].z;

							// Obtain the estimated depth using the observer
							massCenterCoorX = massCenterInWholeImg[idxNew].x;
							massCenterCoorY = massCenterInWholeImg[idxNew].y;

							estimatedDepth[numImg][idxNew] = observer(massCenterCoorX, massCenterCoorY,
								massCenterCoorLast[idxNew][0], massCenterCoorLast[idxNew][1], dHatLast[idxNew],
								currentVel, lastTimeVel, lastTimeAcc);

							// Save the previous mass center coordinates
							massCenterCoorLast[idxNew][0] = massCenterCoorX;
							massCenterCoorLast[idxNew][1] = massCenterCoorY;

							// Save the mass center coordinates to the outImgFeat file
							outImgFeat << massCenterCoorX << "\t" << massCenterCoorY << "\t";
						}

						// Draw the contours
						/*string showWord = "m" + to_string(idxNew);
						Point tempPoint = Point(cvRound(massCenter[idx].x), cvRound(massCenter[idx].y));
						drawContours(contourMat, realContours, -1, Scalar(255, 0, 255), 1, 8);
						circle(contourMat, massCenter[idx], 2, Scalar(255, 0, 255), -1, 8, 0);
						putText(contourMat, showWord, tempPoint, CV_FONT_HERSHEY_COMPLEX, 0.8, Scalar(255, 0, 255));*/
					}
				}
			}

			// Save the previous velocity and acceleration
			memcpy(lastTimeVel, currentVel, 6 * sizeof(currentVel[0]));
			memcpy(lastTimeAcc, currentAcc, 3 * sizeof(currentAcc[0]));

			// Update the velocity and acceleration after removing the noise
			memcpy(velCameraFilt[numImg], currentVel, 6 * sizeof(currentVel[0]));
			memcpy(accCameraFilt[numImg], currentAcc, 3 * sizeof(currentAcc[0]));

			// Output the data including velocity, acceleration, etc, to the file
			for (int i = 0; i < 6; i++)
			{
				outVel << velCameraFilt[numImg][i] << "\t";
			}
			outVel << "\n";

			for (int i = 0; i < 3; i++)
			{
				outAcc << accCameraFilt[numImg][i] << "\t";
			}
			outAcc << "\n";

			for (int i = 0; i < 4; i++)
			{
				outGndTruth << groundTruth[numImg][i] << "\t";
				outEstDepth << estimatedDepth[numImg][i] << "\t";
			}
			outGndTruth << "\n";
			outEstDepth << "\n";
			outImgFeat << "\n";

			// Count the number of image
			numImg++;
		}

		// Show the image
		//imshow("Contour image", contourMat);

		// Pointers pColorFrame and pDepthFrame must be released safely, or the sensor fails to "AcquireLatestFrame"		
		SafeRelease(pColorFrame);
		SafeRelease(pDepthFrame);

		// Escape the while loop by setting a constant image number
		if (numImg == 4000) break;

		// Press the ESC button escape the while loop
		//if (cv::waitKey(30) == VK_ESCAPE)  break;

		// Calculate the running time of the program
		clock_t endTime = clock();
		printf("\nThe run time is: %f s\n\n", double(endTime - startTime) / CLOCKS_PER_SEC);
		startTime = endTime;
	}

	// Release the pointer safely
	SafeRelease(pColorSource);
	SafeRelease(pDepthSource);
	SafeRelease(pColorReader);
	SafeRelease(pDepthReader);
	SafeRelease(pColorDescription);
	SafeRelease(pDepthDescription);
	SafeRelease(pCoordinateMapper);
	if (pSensor)
	{
		pSensor->Close();
	}
	SafeRelease(pSensor);
	cv::destroyAllWindows();

	// Clean up the optiTrack client
	theClient->Uninitialize();

	// Close the file
	outVel.close();
	outAcc.close();
	outGndTruth.close();
	outEstDepth.close();
	outImgFeat.close();

	// Call MATLAB engine to show the difference between the ground and the estimated values 
	int plotFlag = plotDepth(groundTruth, estimatedDepth, velCameraFilt, accCameraFilt, numImg);

	if (plotFlag == EXIT_SUCCESS)
	{
		cout << "Depth plotting finished!" << endl;
	}

	return 0;
}


