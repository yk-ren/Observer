#include <depthValidation.h>

int createClient(int iConnectionType)
{
	// Release previous server
	if (theClient)
	{
		theClient->Uninitialize();
		delete theClient;
	}

	// Create NatNet client
	theClient = new NatNetClient(iConnectionType);

	// Print version info
	unsigned char ver[4];
	theClient->NatNetVersion(ver);
	printf("\nThis NatNet Client (NatNet ver. %d.%d.%d.%d)\n", ver[0], ver[1], ver[2], ver[3]);

	// Initialize the client and connect to NatNet server to use NatNet default port assignments
	int retCode = theClient->Initialize(szMyIPAddress, szServerIPAddress);
	// To use a different port for commands and/or data:
	// int retCode = theClient->Initialize(szMyIPAddress, szServerIPAddress, MyServersCommandPort, MyServersDataPort);
	if (retCode != ErrorCode_OK)
	{
		printf("Unable to connect to server.  Error code: %d. Exiting", retCode);
		return ErrorCode_Internal;
	}
	else
	{
		// Get the number of analog samples per motion capture frame of data
		void * pResult;
		int ret = 0;
		int nBytes = 0;
		ret = theClient->SendMessageAndWait("AnalogSamplesPerMocapFrame", &pResult, &nBytes);
		if (ret == ErrorCode_OK)
		{
			analogSamplesPerMocapFrame = *((int*)pResult);
			printf("Analog Samples Per Motion Capture Frame : %d", analogSamplesPerMocapFrame);
		}

		// Print server information
		sServerDescription ServerDescription;
		memset(&ServerDescription, 0, sizeof(ServerDescription));
		theClient->GetServerDescription(&ServerDescription);
		if (!ServerDescription.HostPresent)
		{
			printf("Unable to connect to server. Host not present. Exiting.");
			return 1;
		}
		printf("Server application info:\n");
		printf("Application: %s (ver. %d.%d.%d.%d)\n", ServerDescription.szHostApp, ServerDescription.HostAppVersion[0],
			ServerDescription.HostAppVersion[1], ServerDescription.HostAppVersion[2], ServerDescription.HostAppVersion[3]);
		printf("NatNet Version: %d.%d.%d.%d\n", ServerDescription.NatNetVersion[0], ServerDescription.NatNetVersion[1],
			ServerDescription.NatNetVersion[2], ServerDescription.NatNetVersion[3]);
		printf("Client IP:%s\n", szMyIPAddress);
		printf("Server IP:%s\n", szServerIPAddress);
		printf("Server Name:%s\n\n", ServerDescription.szHostComputerName);
	}

	return ErrorCode_OK;
}