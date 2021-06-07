#include <depthValidation.h>
#include <optiTrack.h>

int plotDepth(double trueDep[][4], double estiDep[][4], double velCamArray[][6], double accCamArray[][3], int nImage)
{
	// Examine the engine
	Engine *pEng = NULL;
	if (!(pEng = engOpen(NULL)))
	{
		fprintf(stderr, "\n Can't start MATLAB engine \n");
		getchar();
		return EXIT_FAILURE;
	}

	// Produce the time axis,the KINECT and the opti_Track are set to 30fps
	double *time = new double[nImage];
	for (int i = 0; i < nImage; i++)
	{
		time[i] = (double) 1 / 30 * i;
	}

	mxArray *Time = NULL;
	Time = mxCreateDoubleMatrix(1, nImage, mxREAL);
	memcpy((void *)mxGetPr(Time), (void *)time, nImage*sizeof(time[0]));

	// Produce the data axis for four circle centers
	double **pptrueDep = new double *[4];
	double **ppestiDep = new double *[4];
	for (int j = 0; j < 4; j++)
	{
		pptrueDep[j] = new double[nImage];
		ppestiDep[j] = new double[nImage];
		for (int i = 0; i < nImage; i++)
		{
			pptrueDep[j][i] = trueDep[i][j];
			ppestiDep[j][i] = estiDep[i][j];
		}
	}

	mxArray *trueDepArr[4] = { NULL }, *estiDepArr[4] = { NULL };
	for (int k = 0; k < 4; k++)
	{
		trueDepArr[k] = mxCreateDoubleMatrix(1, nImage, mxREAL);
		estiDepArr[k] = mxCreateDoubleMatrix(1, nImage, mxREAL);
		memcpy((void *)mxGetPr(trueDepArr[k]), (void *)pptrueDep[k], nImage*sizeof(pptrueDep[k][0]));
		memcpy((void *)mxGetPr(estiDepArr[k]), (void *)ppestiDep[k], nImage*sizeof(ppestiDep[k][0]));
	}

	// Produce the data axis for velocities
	double **ppvelCamArray = new double *[6];
	for (int j = 0; j < 6; j++)
	{
		ppvelCamArray[j] = new double[nImage];
		for (int i = 0; i < nImage; i++)
		{
			ppvelCamArray[j][i] = velCamArray[i][j];
		}
	}

	mxArray *pvelCamArray[6] = { NULL };
	for (int k = 0; k < 6; k++)
	{
		pvelCamArray[k] = mxCreateDoubleMatrix(1, nImage, mxREAL);
		memcpy((void *)mxGetPr(pvelCamArray[k]), (void *)ppvelCamArray[k], nImage*sizeof(ppvelCamArray[k][0]));
	}

	// Produce the data axis for accelerations
	double **ppaccCamArray = new double *[3];
	for (int j = 0; j < 3; j++)
	{
		ppaccCamArray[j] = new double[nImage];
		for (int i = 0; i < nImage; i++)
		{
			ppaccCamArray[j][i] = accCamArray[i][j];
		}
	}

	mxArray *paccCamArray[3] = { NULL };
	for (int k = 0; k < 3; k++)
	{
		paccCamArray[k] = mxCreateDoubleMatrix(1, nImage, mxREAL);
		memcpy((void *)mxGetPr(paccCamArray[k]), (void *)ppaccCamArray[k], nImage*sizeof(ppaccCamArray[k][0]));
	}

	// Plot the ground-truth and the estimated depth
	engPutVariable(pEng, "T", Time);
	engPutVariable(pEng, "G0", trueDepArr[0]);
	engPutVariable(pEng, "G1", trueDepArr[1]);
	engPutVariable(pEng, "G2", trueDepArr[2]);
	engPutVariable(pEng, "G3", trueDepArr[3]);
	engPutVariable(pEng, "E0", estiDepArr[0]);
	engPutVariable(pEng, "E1", estiDepArr[1]);
	engPutVariable(pEng, "E2", estiDepArr[2]);
	engPutVariable(pEng, "E3", estiDepArr[3]);
	engEvalString(pEng, "figure(1)");
	engEvalString(pEng, "plot(T,G0,T,G1,T,G2,T,G3);");
	engEvalString(pEng, "xlabel('Time (s)');");
	engEvalString(pEng, "ylabel('Ground truth (m)');");
	engEvalString(pEng, "legend('Mass center 0','Mass center 1','Mass center 2','Mass center 3');");
	engEvalString(pEng, "figure(2)");
	engEvalString(pEng, "plot(T,E0,T,E1,T,E2,T,E3);");
	engEvalString(pEng, "xlabel('Time (s)');");
	engEvalString(pEng, "ylabel('Estimated depth (m)');");
	engEvalString(pEng, "legend('Mass center 0','Mass center 1','Mass center 2','Mass center 3');");

	// plot the velocities and accelerations
	engPutVariable(pEng, "T", Time);
	engPutVariable(pEng, "V0", pvelCamArray[0]);
	engPutVariable(pEng, "V1", pvelCamArray[1]);
	engPutVariable(pEng, "V2", pvelCamArray[2]);
	engPutVariable(pEng, "V3", pvelCamArray[3]);
	engPutVariable(pEng, "V4", pvelCamArray[4]);
	engPutVariable(pEng, "V5", pvelCamArray[5]);
	engPutVariable(pEng, "A0", paccCamArray[0]);
	engPutVariable(pEng, "A1", paccCamArray[1]);
	engPutVariable(pEng, "A2", paccCamArray[2]);
	engEvalString(pEng, "figure(3)");
	engEvalString(pEng, "plot(T,V0,T,V1,T,V2);");
	engEvalString(pEng, "xlabel('Time (s)');");
	engEvalString(pEng, "ylabel('Linear velocity (m/s)');");
	engEvalString(pEng, "legend('X','Y','Z');");
	engEvalString(pEng, "figure(4)");
	engEvalString(pEng, "plot(T,V3,T,V4,T,V5);");
	engEvalString(pEng, "xlabel('Time (s)');");
	engEvalString(pEng, "ylabel('Angular velocity (rad/s)');");
	engEvalString(pEng, "legend('Pitch','Yaw','Roll');");
	engEvalString(pEng, "figure(5)");
	engEvalString(pEng, "plot(T,A0,T,A1,T,A2);");
	engEvalString(pEng, "xlabel('Time (s)');");
	engEvalString(pEng, "ylabel('Linear acceleration (m/s^2)');");
	engEvalString(pEng, "legend('X','Y','Z');");

	printf("Hit return to continue\n\n");
	fgetc(stdin);

	// Delete the array for depth
	mxDestroyArray(Time);
	delete[] time;
	for (int i = 0; i < 4; i++)
	{
		mxDestroyArray(trueDepArr[i]);
		mxDestroyArray(estiDepArr[i]);
		delete[] pptrueDep[i];
		delete[] ppestiDep[i];
	}

	delete[] pptrueDep;
	delete[] ppestiDep;

	// Delete the array for velocity
	for (int i = 0; i < 6; i++)
	{
		mxDestroyArray(pvelCamArray[i]);
		delete[] ppvelCamArray[i];
	}

	delete[] ppvelCamArray;

	// Delete the array for accelerations
	for (int i = 0; i < 3; i++)
	{
		mxDestroyArray(paccCamArray[i]);
		delete[] ppaccCamArray[i];
	}

	delete[] ppaccCamArray;

	engEvalString(pEng, "close;");
	engEvalString(pEng, "exit;");

	engClose(pEng);

	return EXIT_SUCCESS;
}