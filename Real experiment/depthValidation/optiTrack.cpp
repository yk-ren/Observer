#include "depthValidation.h"
#include "natUtils.h"
#include "optiTrack.h"

// World Up Axis (default to Y)
int upAxis = 1;

// Save the previous time stamp 
double timeStampLast, timeStampSecLast;

// Save the previous pose to calculate the velocity and acceleration
float xPosLast, yPosLast, zPosLast;
float xOrienLast, yOrienLast, zOrienLast;

float xPosSecLast, yPosSecLast, zPosSecLast;

Velocity vel;
Acceleration acc;

int optiTrack()
{
	// Send or Receive test request
	printf("Sending Test Request...\n");
	void * response;
	int nBytes;
	int iResult;
	iResult = theClient->SendMessageAndWait("TestRequest", &response, &nBytes);
	if (iResult == ErrorCode_OK)
	{
		printf("Received: %s", (char*)response);
	}

	// Retrieve Data Descriptions from server
	printf("Requesting Data Descriptions...");
	sDataDescriptions * pDataDefs = NULL;
	int nBodies = theClient->GetDataDescriptions(&pDataDefs);
	if (!pDataDefs)
	{
		printf("Unable to retrieve Data Descriptions, please check your code...");
	}
	else
	{
		printf("Received %d Data Descriptions:\n", pDataDefs->nDataDescriptions);
		for (int i = 0; i < pDataDefs->nDataDescriptions; i++)
		{
			printf("Data Description # %d (type=%d)\n", i, pDataDefs->arrDataDescriptions[i].type);
			if (pDataDefs->arrDataDescriptions[i].type == Descriptor_MarkerSet)
			{
				// MarkerSet
				sMarkerSetDescription * pMS = pDataDefs->arrDataDescriptions[i].Data.MarkerSetDescription;
				printf("MarkerSet Name : %s\n", pMS->szName);
				for (int i = 0; i < pMS->nMarkers; i++)
				{
					printf("%s\n", pMS->szMarkerNames[i]);
				}
			}
			else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_RigidBody)
			{
				// RigidBody
				sRigidBodyDescription * pRB = pDataDefs->arrDataDescriptions[i].Data.RigidBodyDescription;
				printf("RigidBody Name : %s\n", pRB->szName);
				printf("RigidBody ID : %d\n", pRB->ID);
				printf("RigidBody Parent ID : %d\n", pRB->parentID);
				printf("Parent Offset : %3.6f,%3.6f,%3.6f\n", pRB->offsetx, pRB->offsety, pRB->offsetz);
			}
			else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_Skeleton)
			{
				// Skeleton
				sSkeletonDescription * pSK = pDataDefs->arrDataDescriptions[i].Data.SkeletonDescription;
				printf("Skeleton Name : %s\n", pSK->szName);
				printf("Skeleton ID : %d\n", pSK->skeletonID);
				printf("RigidBody (Bone) Count : %d\n", pSK->nRigidBodies);
				for (int j = 0; j < pSK->nRigidBodies; j++)
				{
					sRigidBodyDescription * pRB = &pSK->RigidBodies[j];
					printf("  RigidBody Name : %s\n", pRB->szName);
					printf("  RigidBody ID : %d\n", pRB->ID);
					printf("  RigidBody Parent ID : %d\n", pRB->parentID);
					printf("  Parent Offset : %3.6f,%3.6f,%3.6f\n", pRB->offsetx, pRB->offsety, pRB->offsetz);
				}
			}
			else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_ForcePlate)
			{
				// Force Plate
				sForcePlateDescription * pFP = pDataDefs->arrDataDescriptions[i].Data.ForcePlateDescription;
				printf("Force Plate ID : %d\n", pFP->ID);
				printf("Force Plate Serial : %s\n", pFP->strSerialNo);
				printf("Force Plate Width : %3.2f\n", pFP->fWidth);
				printf("Force Plate Length : %3.2f\n", pFP->fLength);
				printf("Force Plate Electrical Center Offset (%3.3f, %3.3f, %3.3f)\n", pFP->fOriginX, pFP->fOriginY, pFP->fOriginZ);
				for (int iCorner = 0; iCorner < 4; iCorner++)
				{
					printf("Force Plate Corner %d : (%3.4f, %3.4f, %3.4f)\n", iCorner, pFP->fCorners[iCorner][0], pFP->fCorners[iCorner][1], pFP->fCorners[iCorner][2]);
				}
				printf("Force Plate Type : %d\n", pFP->iPlateType);
				printf("Force Plate Data Type : %d\n", pFP->iChannelDataType);
				printf("Force Plate Channel Count : %d\n", pFP->nChannels);
				for (int iChannel = 0; iChannel < pFP->nChannels; iChannel++)
				{
					printf("\tChannel %d : %s\n", iChannel, pFP->szChannelNames[iChannel]);
				}

			}
			else
			{
				// Unknown
				printf("Unknown data type.");
			}
		}
	}

	return ErrorCode_OK;
}

// DataHandler receives data from the server, and calculates the velocity of the rigid body
void __cdecl DataHandler(sFrameOfMocapData* data, void* pUserData)
{
	NatNetClient * pClient = (NatNetClient*)pUserData;

	// FrameOfMocapData parameters
	bool bIsRecording = ((data->params & 0x01) != 0);
	bool bTrackedModelsChanged = ((data->params & 0x02) != 0);
	if (bIsRecording)
	{
		printf("RECORDING\n");
	}

	if (bTrackedModelsChanged)
	{
		printf("Models Changed.\n");
	}

	// Time code - for systems with an eSync and SMPTE time code generator - decode to values
	int hour, minute, second, frame, subframe;
	bool bValid = pClient->DecodeTimecode(data->Timecode, data->TimecodeSubframe, &hour, &minute, &second, &frame, &subframe);
	// Decode to friendly string
	char szTimecode[128] = "";
	pClient->TimecodeStringify(data->Timecode, data->TimecodeSubframe, szTimecode, 128);
	printf("Time code : %s\n", szTimecode);

	// Get the coordinate frame description, default to Y.
	void * response;
	int ret = 0;
	int nBytes = 0;
	
	ret = pClient->SendMessageAndWait("UpAxis", &response, &nBytes);
	if (ret == ErrorCode_OK)
	{
		upAxis = *(long*)response;
	}

	// Get the rigid bodies info
	int order;
	EulerAngles ea;
	Quat q;

	// Define the temp variable
	float xPos, yPos, zPos;
	float qxOrien, qyOrien, qzOrien, qwOrien;

	// Define the variable to obtain the velocity
	double timeStamp;

	printf("Rigid Bodies [Count=%d]\n", data->nRigidBodies);
	for (int i = 0; i < data->nRigidBodies; i++)
	{
		// 0x01 : bool, rigid body was successfully tracked in this frame
		bool bTrackingValid = data->RigidBodies[i].params & 0x01;

		// Output the data of makers
		if (false)
		{
			printf("\tRigid body markers [Count=%d]\n", data->RigidBodies[i].nMarkers);
			for (int iMarker = 0; iMarker < data->RigidBodies[i].nMarkers; iMarker++)
			{
				printf("\t\t");
				if (data->RigidBodies[i].MarkerIDs)
				{
					printf("MarkerID:%d", data->RigidBodies[i].MarkerIDs[iMarker]);
				}
				if (data->RigidBodies[i].MarkerSizes)
				{
					printf("\tMarkerSize:%3.2f", data->RigidBodies[i].MarkerSizes[iMarker]);
				}
				if (data->RigidBodies[i].Markers)
				{
					printf("\tMarkerPos:%3.2f,%3.2f,%3.2f\n",
						data->RigidBodies[i].Markers[iMarker][0],
						data->RigidBodies[i].Markers[iMarker][1],
						data->RigidBodies[i].Markers[iMarker][2]);
				}
			}
		}

		
		// Output the data of rigid body
		printf("Rigid Body [ID=%d  Error=%3.6f  Valid=%d]\n", data->RigidBodies[i].ID, data->RigidBodies[i].MeanError, bTrackingValid);
		printf("FrameID : %d, Timestamp: %3.6lf\n", data->iFrame, data->fTimestamp);

		// Save the data into the temp variable
		xPos = data->RigidBodies[i].x;  
		yPos = data->RigidBodies[i].y;   
		zPos = data->RigidBodies[i].z;
		qxOrien = data->RigidBodies[i].qx;  
		qyOrien = data->RigidBodies[i].qy;   
		qzOrien = data->RigidBodies[i].qz;
		qwOrien = data->RigidBodies[i].qw;
		
		// If Motive is streaming Z-up, convert to this renderer's Y-up coordinate system
		if (upAxis == 2)
		{
			// Convert position
			ConvertRHSPosZupToYUp(xPos, yPos, zPos);
			// Convert orientation
			ConvertRHSRotZUpToYUp(qxOrien, qyOrien, qzOrien, qwOrien);
		}
		
		// Convert Motive quaternion output to Euler angles
		// Motive coordinate conventions : X(Pitch), Y(Yaw), Z(Roll), Relative, RHS
		q.x = qxOrien; q.y = qyOrien; q.z = qzOrien; q.w = qwOrien;
		order = EulOrdXYZr;
		ea = Eul_FromQuat(q, order);

		// Change the radian to degree to examine the correctness of the code
		if (false)
		{
			ea.x = NATUtils::RadiansToDegrees(ea.x);
			ea.y = NATUtils::RadiansToDegrees(ea.y);
			ea.z = NATUtils::RadiansToDegrees(ea.z);
		}
		
		// Output the pose coordinate
		printf("\txPos\tyPos\tzPos\txEa\tyEa\tzEa\n");
		printf("\t%3.6f\t%3.6f\t%3.6f\t%3.6f\t%3.6f\t%3.6f\t\n",
				xPos, yPos, zPos, ea.x, ea.y, ea.z);

		// Get the time stamp
		timeStamp = data->fTimestamp;
		
		// Base coordinate systems of robot and optiTrack are different as follows, so conversion between these two coordinate systems is needed
		float tempPos, tempOrien;
		tempPos = zPos;
		zPos = xPos;
		xPos = -tempPos;
		yPos = yPos;

		tempOrien = 0.1 * ea.z;
		ea.z = ea.x;
		ea.x = -tempOrien;
		ea.y = ea.y;

		// Calculate the current velocity of the rigid body using 
		vel.linearVelX = (xPos - xPosLast) / (timeStamp - timeStampLast);
		vel.linearVelY = (yPos - yPosLast) / (timeStamp - timeStampLast);
		vel.linearVelZ = (zPos - zPosLast) / (timeStamp - timeStampLast);
		vel.angleVelX = (ea.x - xOrienLast) / (timeStamp - timeStampLast);
		vel.angleVelY = (ea.y - yOrienLast) / (timeStamp - timeStampLast);
		vel.angleVelZ = (ea.z - zOrienLast) / (timeStamp - timeStampLast);


		// Calculate the current linear acceleration of the rigid body
		acc.linearAccX = ((xPos - xPosLast) / (timeStamp - timeStampLast) - (xPosLast - xPosSecLast) / (timeStampLast - timeStampSecLast)) / (timeStamp - timeStampLast);
		acc.linearAccY = ((yPos - yPosLast) / (timeStamp - timeStampLast) - (yPosLast - yPosSecLast) / (timeStampLast - timeStampSecLast)) / (timeStamp - timeStampLast);
		acc.linearAccZ = ((zPos - zPosLast) / (timeStamp - timeStampLast) - (zPosLast - zPosSecLast) / (timeStampLast - timeStampSecLast)) / (timeStamp - timeStampLast);


		// Save the previous values of the time stamp
		timeStampSecLast = timeStampLast;
		timeStampLast = timeStamp;

		// Save the previous values of the velocity and acceleration
		xPosSecLast = xPosLast;
		yPosSecLast = yPosLast;
		zPosSecLast = zPosLast;

		xPosLast = xPos;
		yPosLast = yPos;
		zPosLast = zPos;
		xOrienLast = ea.x;
		yOrienLast = ea.y;
		zOrienLast = ea.z;

	}
}

// MessageHandler receives NatNet error/debug messages
void __cdecl MessageHandler(int msgType, char* msg)
{
	printf("\n%s\n", msg);
}

// Reset the client
void resetClient()
{
	int iSuccess;

	printf("\n\nre-setting Client\n\n.");

	iSuccess = theClient->Uninitialize();
	if (iSuccess != 0)
	{
		printf("\nerror reset client\n");
	}

	iSuccess = theClient->Initialize(szMyIPAddress, szServerIPAddress);
	if (iSuccess != 0)
	{
		printf("\n error reset client \n");
	}
}

void ConvertRHSPosZupToYUp(float& x, float& y, float& z)
{
	/*
	[RHS, Y-Up]     [RHS, Z-Up]

	                      Y
	Y                 Z /
	|__ X             |/__ X
	/
	Z

	X_yup  =  X_zup
	Y_yup  =  Z_zup
	Z_yup  =  -Y_zup
	*/
	float yOriginal = y;
	y = z;
	z = -yOriginal;
}

void ConvertRHSRotZUpToYUp(float& qx, float& qy, float& qz, float& qw)
{
	// -90 deg rotation about +X
	float qRx, qRy, qRz, qRw;
	float angle = -90.0f * MATH_PI / 180.0f;
	qRx = sin(angle / 2.0f);
	qRy = 0.0f;
	qRz = 0.0f;
	qRw = cos(angle / 2.0f);

	// rotate quaternion using quaternion multiply
	float qxNew, qyNew, qzNew, qwNew;
	qxNew = qw*qRx + qx*qRw + qy*qRz - qz*qRy;
	qyNew = qw*qRy - qx*qRz + qy*qRw + qz*qRx;
	qzNew = qw*qRz + qx*qRy - qy*qRx + qz*qRw;
	qwNew = qw*qRw - qx*qRx - qy*qRy - qz*qRz;

	qx = qxNew;
	qy = qyNew;
	qz = qzNew;
	qw = qwNew;
}