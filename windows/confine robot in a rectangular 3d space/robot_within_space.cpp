// ExampleGetCodeVersion.cpp : Defines the entry point for the console application.
//

#include <windows.h>
#include "CommandLayer.h"
#include <conio.h>
#include <vector>
#include "KinovaTypes.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>

using namespace std;

//A handle to the API.
HINSTANCE commandLayer_handle;

int (*MyGetCodeVersion)(int[CODE_VERSION_COUNT]);
int (*MyInitAPI)();
int (*MyCloseAPI)();
int(*MyEraseAllProtectionZones)();
int(*MySetProtectionZone)(ZoneList Command);

// Standard protection box  ( 4 walls + 1 floor); can be personalized depending on your needs
float limitXMin = 0.520f;
float limitXMax = limitXMin + 0.030f;

float limitYMin = limitXMin;
float limitYMax = limitYMin + 0.030f;

float limitZCeil = 1.00f;
float limitZFloor = 0.00f; // Floor height
float limitZFloorMin = -2.010f;




int main(int argc, char* argv[])
{
	bool result = true;

	bool initOK = false;
	int test;

	if (argc >= 2)
	{
		limitXMin = atof(argv[1]);
		limitYMin = atof(argv[1]);

		limitXMax = limitXMin + 0.03f;
		limitYMax = limitYMin + 0.03f;
	}

	if (argc >= 3)
	{
		limitZFloor = atof(argv[2]);
		
	}

	cout << "Protection zone will use:" << endl;
	cout << "LimitX:" << limitXMin << endl;
	cout << "LimitY:" << limitYMin << endl ;
	cout << "Z floor:" << limitZFloor << endl << endl;


	int data[CODE_VERSION_COUNT];

	for (int i = 0; i < CODE_VERSION_COUNT; i++)
	{
		data[i] = 0;
	}
	//We load the API.
	commandLayer_handle = LoadLibrary(L"CommandLayerWindows.dll");

	MyInitAPI = (int(*)()) GetProcAddress(commandLayer_handle, "InitAPI");
	MyCloseAPI = (int(*)()) GetProcAddress(commandLayer_handle, "CloseAPI");

	cout << "InitAPI" << endl;
	result = (*MyInitAPI)();

	if ((MyGetCodeVersion = (int(*)(int[CODE_VERSION_COUNT])) GetProcAddress(commandLayer_handle, "GetCodeVersion")) == NULL)
	{
		cout << "GetCodeVersion was NOT initialized correctly." << endl;
	}
	else
	{
		result = (*MyGetCodeVersion)(data);

		cout << "DSP's firmware version is : " << std::hex << data[0] << "." << std::hex << data[1] << "." << std::hex << data[2] << "." << std::hex << data[30] << endl;

		cout << "Actuator 1's firmware version is : " << std::hex << data[3] << "." << std::hex << data[4] << "." << std::hex << data[5] << endl;
		cout << "Actuator 2's firmware version is : " << std::hex << data[6] << "." << std::hex << data[7] << "." << std::hex << data[8] << endl;
		cout << "Actuator 3's firmware version is : " << std::hex << data[9] << "." << std::hex << data[10] << "." << std::hex << data[11] << endl;
		cout << "Actuator 4's firmware version is : " << std::hex << data[12] << "." << std::hex << data[13] << "." << std::hex << data[14] << endl;
		cout << "Actuator 5's firmware version is : " << std::hex << data[15] << "." << std::hex << data[16] << "." << std::hex << data[17] << endl;
		cout << "Actuator 6's firmware version is : " << std::hex << data[18] << "." << std::hex << data[19] << "." << std::hex << data[20] << endl;

		cout << "Finger 1's firmware version is : " << std::hex << data[21] << "." << std::hex << data[22] << "." << std::hex << data[23] << endl;
		cout << "Finger 2's firmware version is : " << std::hex << data[24] << "." << std::hex << data[25] << "." << std::hex << data[26] << endl;
		cout << "Finger 3's firmware version is : " << std::hex << data[27] << "." << std::hex << data[28] << "." << std::hex << data[29] << endl;

		cout << "CAN interface 1's firmware version is : " << std::hex << data[31] << "." << std::hex << data[32] << "." << std::hex << data[33] << endl;
		cout << "CAN interface 2's firmware version is : " << std::hex << data[34] << "." << std::hex << data[35] << "." << std::hex << data[36] << endl;

		for (int j = 0; j < CODE_VERSION_COUNT; j++)
		{
			if (data[j] != 0)
			{
				initOK = true;
			}
		}
	}

	if (initOK)
	{

		if ((MyEraseAllProtectionZones = (int(*)()) GetProcAddress(commandLayer_handle, "EraseAllProtectionZones")) == NULL)
		{
			cout << "EraseAllProtectionZones was NOT initialized correctly." << endl;
		}
		else
		{
			result = (*MyEraseAllProtectionZones)();

			cout << "MyEraseAllProtectionZones Done" << endl;
		}

		if ((MySetProtectionZone = (int(*)(ZoneList Command)) GetProcAddress(commandLayer_handle, "SetProtectionZone")) == NULL)
		{
			cout << "SetProtectionZone was NOT initialized correctly." << endl;
		}
		else
		{
			ZoneList zl;
			zl.NbZones = 5;

			int i = 0;
			//ZONE 0
			zl.Zones[i].ID = i;
			zl.Zones[i].zoneShape.shapeType = ShapeType::PrismSquareBase_Z;

			zl.Zones[i].zoneShape.Points[0].InitStruct();
			zl.Zones[i].zoneShape.Points[0].X = -limitXMax;
			zl.Zones[i].zoneShape.Points[0].Y = -limitYMax;
			zl.Zones[i].zoneShape.Points[0].Z = limitZFloor;

			zl.Zones[i].zoneShape.Points[1].InitStruct();
			zl.Zones[i].zoneShape.Points[1].X = -limitXMax;
			zl.Zones[i].zoneShape.Points[1].Y = limitYMax;
			zl.Zones[i].zoneShape.Points[1].Z = limitZFloor;

			zl.Zones[i].zoneShape.Points[2].InitStruct();
			zl.Zones[i].zoneShape.Points[2].X = -limitXMin;
			zl.Zones[i].zoneShape.Points[2].Y = limitYMax;
			zl.Zones[i].zoneShape.Points[2].Z = limitZFloor;

			zl.Zones[i].zoneShape.Points[3].InitStruct();
			zl.Zones[i].zoneShape.Points[3].X = -limitXMin;
			zl.Zones[i].zoneShape.Points[3].Y = -limitYMax;
			zl.Zones[i].zoneShape.Points[3].Z = limitZFloor;

			zl.Zones[i].zoneShape.Points[4].InitStruct();
			zl.Zones[i].zoneShape.Points[4].X = 0.0f;
			zl.Zones[i].zoneShape.Points[4].Y = 0.0f;
			zl.Zones[i].zoneShape.Points[4].Z = limitZCeil;

			zl.Zones[i].zoneLimitation.speedParameter1 = 0.0f;
			zl.Zones[i].zoneLimitation.speedParameter2 = 0.0f;

			i++;

			//ZONE 1
			zl.Zones[i].ID = i;
			zl.Zones[i].zoneShape.shapeType = ShapeType::PrismSquareBase_Z;

			zl.Zones[i].zoneShape.Points[0].InitStruct();
			zl.Zones[i].zoneShape.Points[0].X = limitXMin;
			zl.Zones[i].zoneShape.Points[0].Y = -limitYMax;
			zl.Zones[i].zoneShape.Points[0].Z = limitZFloor;

			zl.Zones[i].zoneShape.Points[1].InitStruct();
			zl.Zones[i].zoneShape.Points[1].X = limitXMin;
			zl.Zones[i].zoneShape.Points[1].Y = limitYMax;
			zl.Zones[i].zoneShape.Points[1].Z = limitZFloor;

			zl.Zones[i].zoneShape.Points[2].InitStruct();
			zl.Zones[i].zoneShape.Points[2].X = limitXMax;
			zl.Zones[i].zoneShape.Points[2].Y = limitYMax;
			zl.Zones[i].zoneShape.Points[2].Z = limitZFloor;

			zl.Zones[i].zoneShape.Points[3].InitStruct();
			zl.Zones[i].zoneShape.Points[3].X = limitXMax;
			zl.Zones[i].zoneShape.Points[3].Y = -limitYMax;
			zl.Zones[i].zoneShape.Points[3].Z = limitZFloor;

			zl.Zones[i].zoneShape.Points[4].InitStruct();
			zl.Zones[i].zoneShape.Points[4].X = 0.0f;
			zl.Zones[i].zoneShape.Points[4].Y = 0.0f;
			zl.Zones[i].zoneShape.Points[4].Z = limitZCeil;

			zl.Zones[i].zoneLimitation.speedParameter1 = 0.0f;
			zl.Zones[i].zoneLimitation.speedParameter2 = 0.0f;


			i++;

			//ZONE 2
			zl.Zones[i].ID = i;
			zl.Zones[i].zoneShape.shapeType = ShapeType::PrismSquareBase_Z;

			zl.Zones[i].zoneShape.Points[0].InitStruct();
			zl.Zones[i].zoneShape.Points[0].X = -limitXMax;
			zl.Zones[i].zoneShape.Points[0].Y = -limitYMax;
			zl.Zones[i].zoneShape.Points[0].Z = limitZFloor;

			zl.Zones[i].zoneShape.Points[1].InitStruct();
			zl.Zones[i].zoneShape.Points[1].X = -limitXMax;
			zl.Zones[i].zoneShape.Points[1].Y = -limitYMin;
			zl.Zones[i].zoneShape.Points[1].Z = limitZFloor;

			zl.Zones[i].zoneShape.Points[2].InitStruct();
			zl.Zones[i].zoneShape.Points[2].X = limitXMax;
			zl.Zones[i].zoneShape.Points[2].Y = -limitYMin;
			zl.Zones[i].zoneShape.Points[2].Z = limitZFloor;

			zl.Zones[i].zoneShape.Points[3].InitStruct();
			zl.Zones[i].zoneShape.Points[3].X = limitXMax;
			zl.Zones[i].zoneShape.Points[3].Y = -limitXMax;
			zl.Zones[i].zoneShape.Points[3].Z = limitZFloor;

			zl.Zones[i].zoneShape.Points[4].InitStruct();
			zl.Zones[i].zoneShape.Points[4].X = 0.0f;
			zl.Zones[i].zoneShape.Points[4].Y = 0.0f;
			zl.Zones[i].zoneShape.Points[4].Z = limitZCeil;

			zl.Zones[i].zoneLimitation.speedParameter1 = 0.0f;
			zl.Zones[i].zoneLimitation.speedParameter2 = 0.0f;


			i++;

			//ZONE 3
			zl.Zones[i].ID = i;
			zl.Zones[i].zoneShape.shapeType = ShapeType::PrismSquareBase_Z;

			zl.Zones[i].zoneShape.Points[0].InitStruct();
			zl.Zones[i].zoneShape.Points[0].X = -limitXMax;
			zl.Zones[i].zoneShape.Points[0].Y = limitYMin;
			zl.Zones[i].zoneShape.Points[0].Z = limitZFloor;

			zl.Zones[i].zoneShape.Points[1].InitStruct();
			zl.Zones[i].zoneShape.Points[1].X = -limitXMax;
			zl.Zones[i].zoneShape.Points[1].Y = limitYMax;
			zl.Zones[i].zoneShape.Points[1].Z = limitZFloor;

			zl.Zones[i].zoneShape.Points[2].InitStruct();
			zl.Zones[i].zoneShape.Points[2].X = limitXMax;
			zl.Zones[i].zoneShape.Points[2].Y = limitYMax;
			zl.Zones[i].zoneShape.Points[2].Z = limitZFloor;

			zl.Zones[i].zoneShape.Points[3].InitStruct();
			zl.Zones[i].zoneShape.Points[3].X = limitXMax;
			zl.Zones[i].zoneShape.Points[3].Y = limitYMin;
			zl.Zones[i].zoneShape.Points[3].Z = limitZFloor;

			zl.Zones[i].zoneShape.Points[4].InitStruct();
			zl.Zones[i].zoneShape.Points[4].X = 0.0f;
			zl.Zones[i].zoneShape.Points[4].Y = 0.0f;
			zl.Zones[i].zoneShape.Points[4].Z = limitZCeil;

			zl.Zones[i].zoneLimitation.speedParameter1 = 0.0f;
			zl.Zones[i].zoneLimitation.speedParameter2 = 0.0f;


			i++;

			//ZONE 4
			zl.Zones[i].ID = i;
			zl.Zones[i].zoneShape.shapeType = ShapeType::PrismSquareBase_Z;

			zl.Zones[i].zoneShape.Points[0].InitStruct();
			zl.Zones[i].zoneShape.Points[0].X = -limitXMax;
			zl.Zones[i].zoneShape.Points[0].Y = -limitYMax;
			zl.Zones[i].zoneShape.Points[0].Z = limitZFloorMin;

			zl.Zones[i].zoneShape.Points[1].InitStruct();
			zl.Zones[i].zoneShape.Points[1].X = -limitXMax;
			zl.Zones[i].zoneShape.Points[1].Y = limitYMax;
			zl.Zones[i].zoneShape.Points[1].Z = limitZFloorMin;

			zl.Zones[i].zoneShape.Points[2].InitStruct();
			zl.Zones[i].zoneShape.Points[2].X = limitXMax;
			zl.Zones[i].zoneShape.Points[2].Y = limitYMax;
			zl.Zones[i].zoneShape.Points[2].Z = limitZFloorMin;

			zl.Zones[i].zoneShape.Points[3].InitStruct();
			zl.Zones[i].zoneShape.Points[3].X = limitXMax;
			zl.Zones[i].zoneShape.Points[3].Y = -limitYMax;
			zl.Zones[i].zoneShape.Points[3].Z = limitZFloorMin;

			zl.Zones[i].zoneShape.Points[4].InitStruct();
			zl.Zones[i].zoneShape.Points[4].X = 0.0f;
			zl.Zones[i].zoneShape.Points[4].Y = 0.0f;
			zl.Zones[i].zoneShape.Points[4].Z = limitZFloor;

			zl.Zones[i].zoneLimitation.speedParameter1 = 0.0f;
			zl.Zones[i].zoneLimitation.speedParameter2 = 0.0f;

			result = (*MySetProtectionZone)(zl);

			cout << "MySetProtectionZone Done" << endl;
		}
	}
	else
	{
		cout << endl << "Error communicating with the Robot, please check connexion or drivers." << endl;
	}

	result = (*MyCloseAPI)();

	FreeLibrary(commandLayer_handle);

	cout << endl << "Press any key to exit." << endl;

	_getch();

	return 0;
}
