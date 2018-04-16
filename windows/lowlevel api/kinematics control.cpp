/*
* This example shows how to send RS-485 command through the USB port. Basically, that means communicating directly to the actuators
* via the USB port. If applied on a 6 axis JACO or MICO, this example will rotate the actuator #6, the one holding the end effector.
* It will also start a thread that read and display information during the process.
*/

//INCLUDE SECTION
#include <iostream>
#include <Windows.h>
#include <stdlib.h>
#include <stdio.h>
#include "CommunicationLayerWindows.h"
#include <string.h>
#include <stdio.h>
#include <conio.h>

using namespace std;

#define LOOPCOUNT 500

//This is the function that will be executed by the worker thread.(Information reading).
void* ReadPosition(void *ptr);

//A global variable that will be shared between the main thread(SEND) and the worker thread(RECEIVE).
volatile float Joint6Command;

//A handle needed to open the API(library).
HINSTANCE commLayer_Handle;

//A function that read the current position of the robot and some other informations.
void ReadPosition(RS485_Message MessageListIn[50]);

//Function pointers to access API's function.
int(*fptrInitCommunication)();
int(*MyRS485_Activate)();     // FUNCTION TO ACTIVATE USB - RS485 MODE //
int(*MyRS485_Read)(RS485_Message* PackagesIn, int QuantityWanted, int &ReceivedQtyIn);
int(*MyRS485_Write)(RS485_Message* PackagesOut, int QuantityWanted, int &ReceivedQtyIn);

//MAIN(SEND information)
int main()
{
	//Variable used during the communication process.
	int WriteCount = 0;
	int ReadCount = 0;

	//Flag used during initialization.
	bool Actuator6Initialized = false;

	//Variable needed during worker thread creation but not used after.
	int ThreadArgument = 0;

	//Message to initialize the actuator 6's command.
	RS485_Message InitMessage;

	//Message to receive the actuator 6's position.
	RS485_Message ReceiveInitMessage[50];

	//Message to move the actuator 6.
	RS485_Message TrajectoryMessage;

	cout << "RS-485 communication Example." << endl;

	//We load the API.
	commLayer_Handle = LoadLibrary(L"CommunicationLayerWindows.dll");

	//Initialization of the fucntion pointers.
	fptrInitCommunication = (int(*)()) GetProcAddress(commLayer_Handle, "InitCommunication");
	MyRS485_Activate = (int(*)()) GetProcAddress(commLayer_Handle, "OpenRS485_Activate");
	MyRS485_Read = (int(*)(RS485_Message*, int, int &)) GetProcAddress(commLayer_Handle, "OpenRS485_Read");
	MyRS485_Write = (int(*)(RS485_Message*, int, int &)) GetProcAddress(commLayer_Handle, "OpenRS485_Write");
	fptrCloseCommunication = (int(*)()) GetProcAddress(commLayer_Handle, "CloseCommunication");

	//If all functions are loaded correctly.
	if (fptrInitCommunication != NULL && MyRS485_Activate != NULL && MyRS485_Read != NULL && MyRS485_Write != NULL)
	{
		//Initialization of the API
		int result = fptrInitCommunication();

		//If API's initialization is correct.
		if (result == NO_ERROR_KINOVA)
		{
			cout << "U S B   I N I T I A L I Z A T I O N   C O M P L E T E D" << endl << endl;

			//We activate the RS-485 comm API. From here you cannot control the robot with the Joystick or
			//with the normal USB function. Only RS-485 command will be accepted. Reboot the robot to get
			//back to normal control.
			MyRS485_Activate();

			//Initialize the INIT message
			InitMessage.Command = RS485_MSG_GET_ACTUALPOSITION; //Setting the command ID
			InitMessage.SourceAddress = 0x00;                   //Setting the source address (0 means the API)
			InitMessage.DestinationAddress = 0x10;              //Setting the destinaiton address(0x15 is the actuator 6's default value)

			//Those value are not used for this command.
			InitMessage.DataLong[0] = 0x00000000;
			InitMessage.DataLong[1] = 0x00000000;
			InitMessage.DataLong[2] = 0x00000000;
			InitMessage.DataLong[3] = 0x00000000;

			//Send the Init Message. 1 is the message's quantity and WriteCount will stored the Qty of messages sent.
			MyRS485_Write(&InitMessage, 1, WriteCount);

			//In case we did not received the answer, we continue reading until it's done
			while (!Actuator6Initialized)
			{
				MyRS485_Write(&InitMessage, 1, WriteCount);
				Sleep(4);
				MyRS485_Read(ReceiveInitMessage, 1, ReadCount);

				//We make sure that the mesage come from actuator 6(0x15) and that the command ID is RS485_MSG_SEND_ACTUALPOSITION
				//which is the answer of our message. (See document Kinova RS485 Communication protocol).
				if (ReceiveInitMessage[0].SourceAddress == 0x10 && ReceiveInitMessage[0].Command == RS485_MSG_SEND_ACTUALPOSITION)
				{
					Joint6Command = ReceiveInitMessage[0].DataFloat[1];
					Actuator6Initialized = true;
				}
			}
			MyRS485_Read(ReceiveInitMessage, 1, ReadCount);

			/*
			* Creation of a message to send a position to the actuator 6(default address 0x15)
			* If you check the RS 485 communication protocol document, you will see that the command
			* RS485_MSG_GET_POSITION_COMMAND_ALL_VALUES(0x14) takes the command in argument 0 and 1.
			* The argument 2 is a flag that indicates that you need the extended version of the answer.
			* Basically, this command let you move the robot and the answer is the updated data.
			*/
			TrajectoryMessage.Command = 0x0114;
			TrajectoryMessage.SourceAddress = 0x00;
			TrajectoryMessage.DestinationAddress = 0x10;
			TrajectoryMessage.DataFloat[0] = Joint6Command;
			TrajectoryMessage.DataFloat[1] = Joint6Command;
			TrajectoryMessage.DataLong[2] = 1;
			TrajectoryMessage.DataLong[3] = 0x00000000;

			RS485_Message MessageListIn[50];
			for (int i = 0; i < LOOPCOUNT; i++)
			{
				//We add an increment to move counter clockwise
				Joint6Command += (float)(40 * (0.0025));

				//We assign the new command (increment added)
				TrajectoryMessage.DataFloat[0] = Joint6Command;
				TrajectoryMessage.DataFloat[1] = Joint6Command;
				
				/*
				* Param1(IN):  The buffer that contains all the messages. In our case, only one.
				* Param2(IN):  Messages count.
				* Param3(OUT): Message sent count.
				*/
				MyRS485_Write(&TrajectoryMessage, 1, WriteCount);

				Sleep(2);

				ReadPosition(MessageListIn);
			}
			
			for (int i = 0; i < LOOPCOUNT; i++)
			{
				//We add an increment to move clockwise
				Joint6Command -= (float)(40 * (0.0025));

				TrajectoryMessage.DataFloat[0] = Joint6Command;
				TrajectoryMessage.DataFloat[1] = Joint6Command;

				MyRS485_Write(&TrajectoryMessage, 1, WriteCount);

				Sleep(2);
				ReadPosition(MessageListIn);
			}
		}
	}
	else
	{
		cout << "Errors while loading API's function" << endl;
	}

	int result = fptrCloseCommunication();

	return 0;
}

//Function to read response from a command ID RS485_MSG_GET_POSITION_COMMAND_ALL_VALUES
void ReadPosition(RS485_Message MessageListIn[50])
{
	
	int MessageReadCount = 0;

	short accelX = 0;
	short accelY = 0;
	short accelZ = 0;
	short temperature = 0;

	float fAccelX = 0;
	float fAccelY = 0;
	float fAccelZ = 0;
	float fTemperature = 0;

	MessageReadCount = 0;

	MyRS485_Read(MessageListIn, 3, MessageReadCount);

	for (int j = 0; j < MessageReadCount; j++)
	{
		if (MessageListIn[j].Command == RS485_MSG_SEND_ALL_VALUES_1)
		{
			printf("Current   = %f\n", MessageListIn[j].DataFloat[0]);
			printf("Position  = %f\n", MessageListIn[j].DataFloat[1]);
			printf("Velocity  = %f\n", MessageListIn[j].DataFloat[2]);
			printf("Torque    = %f\n", MessageListIn[j].DataFloat[3]);
		}

		if (MessageListIn[j].Command == RS485_MSG_SEND_ALL_VALUES_2)
		{
			accelX = (short)(MessageListIn[j].DataLong[2] & 0x0000FFFF);
			accelY = (short)((MessageListIn[j].DataLong[2] & 0xFFFF0000) >> 16);
			accelZ = (short)(MessageListIn[j].DataLong[3] & 0x0000FFFF);
			temperature = (short)((MessageListIn[j].DataLong[3] & 0xFFFF0000) >> 16);

			fAccelX = (float)accelX * 0.001f;
			fAccelY = (float)accelY * 0.001f;
			fAccelZ = (float)accelZ * 0.001f;
			fTemperature = (float)temperature * 0.01f;

			printf("Accel X      = %f\n", fAccelX);
			printf("Accel Y      = %f\n", fAccelY);
			printf("Accel Z      = %f\n", fAccelZ);
			printf("Temperature  = %f\n", fTemperature);
		}

		if (MessageListIn[j].Command == RS485_MSG_SEND_ALL_VALUES_3)
		{
			printf("Motor current     = %f\n", fAccelX);
			printf("Absolute position = %f\n", fAccelY);
		}
	}
}
