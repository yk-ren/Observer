#include "depthValidation.h"

double movingAveFilter(double *originalSignal, int sizeWindow, int idxImg)
{
	// Variable sizeWindow denotes the moving window length
	double sum = 0;
	int number;

	for (int i = 0; i < sizeWindow; i++)
	{
		/* When the window does not have enough data yet,
		the algorithm computes the average of all the previous data*/
		sum += originalSignal[idxImg - i];

		if (i == idxImg)
		{
			break;
		}
	}

	// Save the number
	number = idxImg < sizeWindow ? ++idxImg : sizeWindow;

	return sum / number;
}


double medianFilter(double *originalSignal, int sizeWindow, int idxImg)
{
	sort(originalSignal - sizeWindow + 1, originalSignal);

	return originalSignal[idxImg - (int)(sizeWindow / 2)];
}