/*
 * This example shows how to send RS-485 command through the USB port. Basically, that means communicating directly to the actuators
 * via the USB port. If applied on a 6 axis JACO or MICO, this example will rotate the actuator #6, the one holding the end effector.
 * It will also start a thread that read and display information during the process.
 */

//INCLUDE SECTION
#include <iostream>
#include <dlfcn.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include "Kinova.API.EthCommLayerUbuntu.h"
#include <string.h>
#include <arpa/inet.h>

#include <time.h>
using namespace std;

#define LOOPCOUNT 5000

//This is the function that will be executed by the worker thread.(Information reading).
void* ReadPosition(void *ptr);

timespec diff(timespec start, timespec end); // for computing time differences

//A mutex to protect the access to the API.
pthread_mutex_t APIMutex;

//A global variable that will be shared between the main thread(SEND) and the worker thread(RECEIVE).
volatile float Joint1Command=0.0, Joint2Command=0.0, Joint3Command=0.0, Joint4Command=0.0, Joint5Command=0.0, Joint6Command=0.0;

//A handle needed to open the API(library).
void *commLayer_Handle;

//Function pointers to access API's function.
int(*fptrInitCommunication)(EthernetCommConfig & config); //ETHERNET
int(*MyRS485_Activate)();     // FUNCTION TO ACTIVATE USB - RS485 MODE //
int(*MyRS485_Read)(RS485_Message* PackagesIn, int QuantityWanted, int &ReceivedQtyIn);
int(*MyRS485_Write)(RS485_Message* PackagesOut, int QuantityWanted, int &ReceivedQtyIn);
int(*fptrCloseCommunication)(); //ETHERNET

//MAIN(SEND information)
int main()
{
	EthernetCommConfig gEthernetCommConfig;

	gEthernetCommConfig.localBcastPort = 25015;
	gEthernetCommConfig.localCmdport   = 25025;
	gEthernetCommConfig.robotPort      = 55000;
	gEthernetCommConfig.localIpAddress = inet_addr("192.168.100.100");
	gEthernetCommConfig.robotIpAddress = inet_addr("192.168.100.10");
	gEthernetCommConfig.subnetMask     = inet_addr("255.255.255.0");
	gEthernetCommConfig.rxTimeOutInMs  = 1000;

	//Variable used during the communication process.
	int WriteCount = 0;
	int ReadCount = 0;

	//Flag used during initialization.
	bool Actuator1Initialized = false;
	bool Actuator6Initialized = false;

	//Message to initialize the actuator 6's command.
	RS485_Message InitMessage;

	//Message to receive the actuator 6's position.
	RS485_Message ReceiveInitMessage[3];

	//Message to move the actuator 6.EthernetCommConfig & config
	RS485_Message TrajectoryMessage[3], TrajectoryMessage2[3];


	cout << "RS-485 communication Example." << endl;

	//We load the API.
	commLayer_Handle = dlopen("./libKinova.API.EthCommLayerUbuntu.so", RTLD_NOW|RTLD_GLOBAL);

	//Initialization of the function pointers.
	//ETHERNET
	fptrInitCommunication = (int (*)(EthernetCommConfig & config)) dlsym(commLayer_Handle,"Ethernet_Communication_InitCommunicationEthernet");
	MyRS485_Activate = (int (*)()) dlsym(commLayer_Handle,"Ethernet_Communication_OpenRS485_Activate");
	MyRS485_Read = (int (*)(RS485_Message* PackagesIn, int QuantityWanted, int &ReceivedQtyIn)) dlsym(commLayer_Handle,"Ethernet_Communication_OpenRS485_Read");
	MyRS485_Write = (int (*)(RS485_Message* PackagesOut, int QuantityWanted, int &ReceivedQtyIn)) dlsym(commLayer_Handle,"Ethernet_Communication_OpenRS485_Write");
	fptrCloseCommunication = (int (*)()) dlsym(commLayer_Handle,"Ethernet_Communication_CloseCommunication");

	//If all functions are loaded correctly.
	if(fptrInitCommunication != NULL && MyRS485_Activate != NULL && MyRS485_Read != NULL && MyRS485_Write != NULL)
	{
		//Initialization of the API
		int result = fptrInitCommunication(gEthernetCommConfig);

		//If API's initialization is correct.
		if(result == NO_ERROR_KINOVA)
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
			while(ReadCount!=1 && !Actuator1Initialized)
			{
				//MyRS485_Write(&InitMessage, 1, WriteCount);
				usleep(4000);
				MyRS485_Read(ReceiveInitMessage, 1, ReadCount);
				if (ReadCount==1)
				{
					cout << ReadCount << endl;
				}

				//We make sure that the mesage come from actuator 6(0x15) and that the command ID is RS485_MSG_SEND_ACTUALPOSITION
				//which is the answer of our message. (See document Kinova RS485 Communication protocol).
				if(ReceiveInitMessage[0].SourceAddress == 0x10 && ReceiveInitMessage[0].Command == RS485_MSG_SEND_ACTUALPOSITION)
				{
					Joint1Command = ReceiveInitMessage[0].DataFloat[1];
				}
			}

			//Initialize the INIT message
			InitMessage.Command = RS485_MSG_GET_ACTUALPOSITION; //Setting the command ID
			InitMessage.SourceAddress = 0x00;                   //Setting the source address (0 means the API)
			InitMessage.DestinationAddress = 0x15;              //Setting the destinaiton address(0x15 is the actuator 6's default value)

			//Those value are not used for this command.
			InitMessage.DataLong[0] = 0x00000000;
			InitMessage.DataLong[1] = 0x00000000;
			InitMessage.DataLong[2] = 0x00000000;
			InitMessage.DataLong[3] = 0x00000000;

			//Send the Init Message. 1 is the message's quantity and WriteCount will stored the Qty of messages sent.
			MyRS485_Write(&InitMessage, 1, WriteCount);
			ReadCount=0;
			//In case we did not received the answer, we continue reading until it's done
			while(ReadCount != 1 && !Actuator6Initialized)
			{
				//MyRS485_Write(&InitMessage, 1, WriteCount);
				usleep(4000);
				MyRS485_Read(ReceiveInitMessage, 1, ReadCount);
				if (ReadCount==1)
				{
					cout << ReadCount << endl;
				}

				//We make sure that the mesage come from actuator 6(0x15) and that the command ID is RS485_MSG_SEND_ACTUALPOSITION
				//which is the answer of our message. (See document Kinova RS485 Communication protocol).
				if(ReceiveInitMessage[0].SourceAddress == 0x15 && ReceiveInitMessage[0].Command == RS485_MSG_SEND_ACTUALPOSITION)
				{
					Joint6Command = ReceiveInitMessage[0].DataFloat[1];
					Actuator6Initialized = true;
				}
			}

			/*
			 * Creation of a message to send a position to the actuator 6(default address 0x15)
			 * If you check the RS 485 communication protocol document, you will see that the command
			 * RS485_MSG_GET_POSITION_COMMAND_ALL_VALUES(0x14) takes the command in argument 0 and 1.
			 * The argument 2 is a flag that indicates that you need the extended version of the answer.
			 * Basically, this command let you move the robot and the answer is the updated data.
			 */

			TrajectoryMessage[0].Command = RS485_MSG_GET_POSITION_COMMAND_ALL_VALUES;
			TrajectoryMessage[0].SourceAddress = 0x00;
			TrajectoryMessage[0].DestinationAddress = 0x10;
			TrajectoryMessage[0].DataFloat[0] = Joint1Command;
			TrajectoryMessage[0].DataFloat[1] = Joint1Command;
			TrajectoryMessage[0].DataLong[2] = 0x1;
			TrajectoryMessage[0].DataLong[3] = 0x00000000;

			TrajectoryMessage[1].Command = RS485_MSG_GET_POSITION_COMMAND_ALL_VALUES;
			TrajectoryMessage[1].SourceAddress = 0x00;
			TrajectoryMessage[1].DestinationAddress = 0x11;
			TrajectoryMessage[1].DataFloat[0] = Joint2Command;
			TrajectoryMessage[1].DataFloat[1] = Joint2Command;
			TrajectoryMessage[1].DataLong[2] = 0x1;
			TrajectoryMessage[1].DataLong[3] = 0x00000000;

			TrajectoryMessage[2].Command = RS485_MSG_GET_POSITION_COMMAND_ALL_VALUES;
			TrajectoryMessage[2].SourceAddress = 0x00;
			TrajectoryMessage[2].DestinationAddress = 0x12;
			TrajectoryMessage[2].DataFloat[0] = Joint3Command;
			TrajectoryMessage[2].DataFloat[1] = Joint3Command;
			TrajectoryMessage[2].DataLong[2] = 0x1;
			TrajectoryMessage[2].DataLong[3] = 0x00000000;

			//ETHERNET
			TrajectoryMessage2[0].Command = RS485_MSG_GET_POSITION_COMMAND_ALL_VALUES;
			TrajectoryMessage2[0].SourceAddress = 0x00;
			TrajectoryMessage2[0].DestinationAddress = 0x13;
			TrajectoryMessage2[0].DataFloat[0] = Joint4Command;
			TrajectoryMessage2[0].DataFloat[1] = Joint4Command;
			TrajectoryMessage2[0].DataLong[2] = 0x1;
			TrajectoryMessage2[0].DataLong[3] = 0x00000000;

			TrajectoryMessage2[1].Command = RS485_MSG_GET_POSITION_COMMAND_ALL_VALUES;
			TrajectoryMessage2[1].SourceAddress = 0x00;
			TrajectoryMessage2[1].DestinationAddress = 0x14;
			TrajectoryMessage2[1].DataFloat[0] = Joint5Command;
			TrajectoryMessage2[1].DataFloat[1] = Joint5Command;
			TrajectoryMessage2[1].DataLong[2] = 0x1;
			TrajectoryMessage2[1].DataLong[3] = 0x00000000;

			TrajectoryMessage2[2].Command = RS485_MSG_GET_POSITION_COMMAND_ALL_VALUES;
			TrajectoryMessage2[2].SourceAddress = 0x00;
			TrajectoryMessage2[2].DestinationAddress = 0x15;
			TrajectoryMessage2[2].DataFloat[0] = Joint6Command;
			TrajectoryMessage2[2].DataFloat[1] = Joint6Command;
			TrajectoryMessage2[2].DataLong[2] = 0x1;
			TrajectoryMessage2[2].DataLong[3] = 0x00000000;

			timespec time1, time2;
			timespec temp[LOOPCOUNT];

			RS485_Message MessageListIn [50];
			int MessageReadCount = 0;
			bool updated[6];
			int limloop=0, depassement=0;
			for(int i = 0; i < LOOPCOUNT; i++)
			{

				clock_gettime(CLOCK_MONOTONIC, &time1);
				limloop=0;
				for (int u=0; u<6; u++)
				{
					updated[u]=false;
				}
				//We add an increment to move counter clockwise
				Joint6Command += 40 * (0.0025);
				Joint1Command += 40 * (0.0025);

				//Message to three first actuators
				TrajectoryMessage[0].DataFloat[0] = Joint1Command;
				TrajectoryMessage[0].DataFloat[1] = Joint1Command;

				TrajectoryMessage[1].DataFloat[0] = Joint2Command;
				TrajectoryMessage[1].DataFloat[1] = Joint2Command;

				TrajectoryMessage[2].DataFloat[0] = Joint3Command;
				TrajectoryMessage[2].DataFloat[1] = Joint3Command;

				//Message to three last actuators
				TrajectoryMessage2[0].DataFloat[0] = Joint4Command;
				TrajectoryMessage2[0].DataFloat[1] = Joint4Command;

				TrajectoryMessage2[1].DataFloat[0] = Joint5Command;
				TrajectoryMessage2[1].DataFloat[1] = Joint5Command;

				TrajectoryMessage2[2].DataFloat[0] = Joint6Command;
				TrajectoryMessage2[2].DataFloat[1] = Joint6Command;


				/*
				 * Param1(IN):  The buffer that contains all the messages. In our case, only one.
				 * Param2(IN):  Messages count.
				 * Param3(OUT): Message sent count.
				 */

				MyRS485_Write(TrajectoryMessage, 3, WriteCount);
				usleep(1000);
				while ((updated[0]==false || updated[1]==false || updated[2]==false)&& limloop<10)
				{
					limloop++;
					if (limloop==10)
					{
						cout << "exceed loop" << endl;
					}
					MyRS485_Read(MessageListIn, 3, MessageReadCount);
					for(int j = 0; j < MessageReadCount; j++)
					{
						switch(MessageListIn[j].SourceAddress)
						{
						case 0x10:
							updated[0]=true;
							//cout << "1";
							break;
						case 0x11:
							updated[1]=true;
							//cout << "2";
							break;
						case 0x12:
							updated[2]=true;
							//cout << "3";
							break;
						}
					}
				}

				MyRS485_Write(TrajectoryMessage2, 3, WriteCount);
				usleep(1150);

				limloop=0;
				while ((updated[3]==false || updated[4]==false|| updated[5]==false)&& limloop<10)
				{
					limloop++;
					if (limloop==10)
					{
						cout << "exceed loop" << endl;
					}
					MyRS485_Read(MessageListIn, 3, MessageReadCount);
					for(int j = 0; j < MessageReadCount; j++)
					{
						switch(MessageListIn[j].SourceAddress)
						{
						case 0x13:
							updated[3]=true;
							//cout << "4";
							break;
						case 0x14:
							updated[4]=true;
							//cout << "4";
							break;
						case 0x15:
							updated[5]=true;
							//cout << "4";
							break;
						}
					}
				}

				clock_gettime(CLOCK_MONOTONIC, &time2);
				temp[i]=diff(time1,time2);
				if (temp[i].tv_nsec>0.0025*1000000000)
				{
					depassement++;
				}
			}
			float sum=0.0;
			for(int i = 0; i < LOOPCOUNT; i++)
			{
				cout << temp[i].tv_nsec << endl;
				sum+=temp[i].tv_nsec;
			}
			cout << "mean " << sum/LOOPCOUNT << endl;
		}
		result = fptrCloseCommunication();

	}
	else
	{
		cout << "Errors while loading API's function" << endl;
	}
	
	return 0;
}



timespec diff(timespec start, timespec end)
{
	timespec temp;
	if ((end.tv_nsec-start.tv_nsec)<0) {
		temp.tv_sec = end.tv_sec-start.tv_sec-1;
		temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
	} else {
		temp.tv_sec = end.tv_sec-start.tv_sec;
		temp.tv_nsec = end.tv_nsec-start.tv_nsec;
	}
	return temp;
}
