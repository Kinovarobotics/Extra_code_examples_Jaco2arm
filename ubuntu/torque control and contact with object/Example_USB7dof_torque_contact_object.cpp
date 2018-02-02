/*****************************************************************************************************/
/*                                                                                                   */
/* Uses the 7 dof robot and USB in Cartesian mode. The robot I used has firmware 6.1.9               */
/*                                                                                                   */
/*****************************************************************************************************/

#include <iostream>
#include <dlfcn.h>
#include <vector>
#include "../../../Kinova.API.EthCommLayerUbuntu.h"
#include "../../../API/Kinova.API.EthCommandLayerUbuntu.h"
#include "../../../API/KinovaTypes.h"
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <curses.h>
#include <linux/input.h>
#include <pthread.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>


template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}
bool quitSignal = 0;

int inputCaptureThreadStopSignal = 0;
int contactMode=0;

using namespace std;

//Handle for the library's command layer.
void * commandLayer_handle;

float threshold_force=5.0;

//Function pointers to the functions we need
//General connectivity functions
int (*MyInitAPI)();
int (*MyCloseAPI)();
int (*MyRefresDevicesList)();
int (*MyMoveHome)();
int (*MyInitFingers)();
int (*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
int (*MySetActiveDevice)(KinovaDevice device);

//Kinematics functions
int (*MySetCartesianControl)();
int (*MySetAngularControl)();
int (*MySendBasicTrajectory)(TrajectoryPoint trajectory);
int (*MySendAdvanceTrajectory)(TrajectoryPoint trajectory);
int (*MyGetCartesianPosition)(CartesianPosition& pt);
int (*MyGetAngularPosition)(AngularPosition& pt);
int(*MyStopRedundantJointNullSpaceMotion)();
int (*MySendCartesianForceCommand)(float Command[COMMAND_SIZE]);

//Reactive force control function
int(*MyStartReactiveForceControl)();
int(*MyStopReactiveForceControl)();
int(*MyGetCartesianForce)(CartesianPosition &);
int(*MySetCartesianInertiaDamping)(CartesianInfo inertia, CartesianInfo damping);
int(*MySetCartesianForceMaxMin)(CartesianInfo min, CartesianInfo max);

//gravity compensation functions
int (*MySetGravityOptimalParameter)(float Command[GRAVITY_PARAM_SIZE]);
int (*MySetGravityType)(GRAVITY_TYPE Type);
int(*MyGetAngularForceGravityFree)(AngularPosition &);

//Torque control functions
int(*MySetTorqueVibrationController)(float value);
int(*MySetTorqueControlType)(TORQUECONTROL_TYPE type);
int(*MySetTorqueActuatorDamping)(float Command[COMMAND_SIZE]);
int(*MySwitchTrajectoryTorque)(GENERALCONTROL_TYPE);
int(*MySetTorqueSafetyFactor)(float factor);

//Obscure torque functions
int (*MySetPositionLimitDistance)	(	float 	Command[COMMAND_SIZE]	);
int (*MySetTorquePositionLimitDampingGain)	(	float 	Command[COMMAND_SIZE]	);
int (*MySetTorquePositionLimitDampingMax )	(	float 	Command[COMMAND_SIZE]	);
int (*MySetTorquePositionLimitRepulsGain  )	(	float 	Command[COMMAND_SIZE]	);
int (*MySetTorquePositionLimitRepulsMax )	(	float 	Command[COMMAND_SIZE]	);
int (*MySetTorqueGainMax  )	(	float 	Command[COMMAND_SIZE]	);
int (*MySetTorqueActuatorGain  )	(	float 	Command[COMMAND_SIZE]	);



float sum(float table[], int size)
{
	float result=0;
	for (int i=0; i<size; i++)
	{
		result+=table[i];
	}
	return result;
}



void Init()
{
	//Stop Reactive force control if it was activated
	(*MyStopReactiveForceControl)();
	(*MyMoveHome)();

	//Send optimal gravity compensation parameters
	float OptimalzParam[OPTIMAL_Z_PARAM_SIZE_7DOF] = { 0.00119221,
			-1.45144,
			-0.0146742,
			-0.0237669,
			-0.00965469,
			-0.6024,
			-0.00240457,
			-0.00185153,
			0.00106714,
			-0.175085,
			0.00293676,
			-0.00673825,
			0.0156054,
			0.26914,
			-0.113577,
			-0.0127116,
			0.114563,
			-0.0499898,
			0.0178227};
	float Command[GRAVITY_PARAM_SIZE];
	for (int i=0; i<42; i++)
	{
		Command[i]=0.0;
	}
	for (int i=0; i<OPTIMAL_Z_PARAM_SIZE_7DOF; i++)
	{
		Command[i]=OptimalzParam[i];
	}
	(*MySetGravityOptimalParameter)(Command);
	MySetGravityType(OPTIMAL); //set optimal gravity compensation mode


	//(*MyStartReactiveForceControl)(); //uncomment for activating reactive force control

	//activate torque control
	 // Switch to torque control
	MySwitchTrajectoryTorque(TORQUE);
	// Set the safety factor to 0.6
	MySetTorqueSafetyFactor(1.0);
	// Set the Vibration controller to 0.5
	MySetTorqueVibrationController(0.5);
	// Set damping in torque mode
	float ActuatorDamping[COMMAND_SIZE];
	for (int i = 0; i < COMMAND_SIZE; i++)
	{
			ActuatorDamping[i] = 0;
	}
	float damping =0.1;
	ActuatorDamping[0] = damping;
	ActuatorDamping[1] = damping;
	ActuatorDamping[2] = damping;
	ActuatorDamping[3] = damping;
	ActuatorDamping[4] = damping;
	ActuatorDamping[5] = damping;
	ActuatorDamping[6] = damping;

	MySetTorqueActuatorDamping(ActuatorDamping);

	//virtual wall thing
	float PosLim[COMMAND_SIZE];
	for (int i = 0; i < COMMAND_SIZE; i++)
	{
		PosLim[i] = 5.0;
	}
	MySetPositionLimitDistance(PosLim);

	//this is a damping torque to prevent you from reaching the joint limits
	float DampingWallMax[COMMAND_SIZE]; //max value from this damping torque
	for (int i = 0; i < COMMAND_SIZE; i++)
	{
		DampingWallMax[i] = 20.0;
	}
	MySetTorquePositionLimitDampingMax(DampingWallMax);

	float DampingWall[COMMAND_SIZE]; //damping torque value
	for (int i = 0; i < COMMAND_SIZE; i++)
	{
		DampingWall[i] = 10.0;
	}
	MySetTorquePositionLimitDampingGain(DampingWall);

	//this is a repulsion torque to prevent you from reaching the joint limits
	float RepulsMax[COMMAND_SIZE]; // max value for this repulsion torque
	for (int i = 0; i < COMMAND_SIZE; i++)
	{
		RepulsMax[i] = 20.0;
	}
	MySetTorquePositionLimitRepulsMax(RepulsMax);

	float RepulsGain[COMMAND_SIZE]; //repulsion torque value
	for (int i = 0; i < COMMAND_SIZE; i++)
	{
		RepulsGain[i] = 10.0;
	}
	MySetTorquePositionLimitRepulsGain(RepulsGain);

	//max feedback gain in torque mode. This sets the max tolerated value for the feedback gain in torque mode
	float MaxFeedbackGain[COMMAND_SIZE];
	for (int i = 0; i < COMMAND_SIZE; i++)
	{
		MaxFeedbackGain[i] = 4.0;
	}
	MySetTorqueGainMax(MaxFeedbackGain);

	float FeedbackGain[COMMAND_SIZE]; //feedback gain in torque mode. The more you will increase this gain, the stiffer the control will be
	for (int i = 0; i < COMMAND_SIZE; i++)
	{
		FeedbackGain[i] = 1.0;
	}
	MySetTorqueActuatorGain(FeedbackGain);
}



// ApplyForce will apply a 5 N downward force for 10 seconds. Robot will apply a downward force for 10 sec
// (I got a downward force with a magnitude reading between 3 and 7 N depending on which surface from the gripper was in contact with the table)
// This function is only useful in torque mode (not reactive force control)
void ApplyForce()
{

	CartesianPosition cf;
	float FeedbackGain[COMMAND_SIZE]; //feedback gain in torque mode
	for (int i = 0; i < COMMAND_SIZE; i++)
	{
		FeedbackGain[i] = 2.0;
	}
	MySetTorqueActuatorGain(FeedbackGain);

	float TorqueCommand[COMMAND_SIZE];
	for (int i =0; i<COMMAND_SIZE; i++)
	{
		TorqueCommand[i]=0.0; //initialization
	}
	TorqueCommand[2]=-5.0;
	for (int i =0; i<1000; i++)
	{
		MySendCartesianForceCommand(TorqueCommand);
		MyGetCartesianForce(cf);
		wcout << cf.Coordinates.Z << endl;
		usleep(10000); //sleep 10 ms
	}


}

void SetAdmittanceParameterinContact()
{
	/*Increase the inertia and damping parameters for reactive force control - uncomment for admittance/reactive force control mode
	float facteur = 2.0;
	CartesianInfo inertia;
	inertia.X = facteur * 10;
	inertia.Y = facteur * 15;
	inertia.Z = facteur * 20;
	inertia.ThetaX = facteur*0.1;
	inertia.ThetaY = facteur*0.1;
	inertia.ThetaZ = facteur*0.1;

	CartesianInfo damping;
	damping.X = 2*facteur * 15;
	damping.Y = 2 * facteur * 20;
	damping.Z = 2 * facteur * 30;
	damping.ThetaX = 2 * facteur*0.4;
	damping.ThetaY = 2 * facteur*0.4;
	damping.ThetaZ = 2 * facteur*0.4;
	MySetCartesianInertiaDamping(inertia, damping);

    //Increases the minimum and maximum force thresholds for reactive force control
	CartesianInfo min;

	min.X = threshold_force;
	min.Y = threshold_force;
	min.Z = threshold_force;
	min.ThetaX = facteur/2*1.0;
	min.ThetaY = facteur/2*1.0;
	min.ThetaZ = facteur/2*0.6;

	CartesianInfo max;
	max.X = facteur * 18;
	max.Y = facteur * 18;
	max.Z = facteur * 25;
	max.ThetaX = facteur*5.0;
	max.ThetaY = facteur*5.0;
	max.ThetaZ = facteur*3.0;
	(*MySetCartesianForceMaxMin)(min, max);*/

	//wakes up the robot with switch torque
	MySwitchTrajectoryTorque(TORQUE);

	//increases the damping in torque mode for a more stable contact
	float ActuatorDamping[COMMAND_SIZE];
	float damping =0.3;

	ActuatorDamping[0] = damping;
	ActuatorDamping[1] = damping;
	ActuatorDamping[2] = damping;
	ActuatorDamping[3] = damping;
	ActuatorDamping[4] = damping;
	ActuatorDamping[5] = damping;
	ActuatorDamping[6] = damping;

	MySetTorqueActuatorDamping(ActuatorDamping);
	wcout << "New damping parameters applied" << endl;

}


void SatisfyingForce()
{
	ApplyForce();
}



void *inputCaptureThread(void *arg)
{
    wcout << "Input capture thread start()"<< endl;

    int fd;

    fd = open("/dev/input/event3", O_RDONLY);
    struct input_event ev;

    while (inputCaptureThreadStopSignal == 0)
    {
        usleep(20000);
        read(fd, &ev, sizeof(struct input_event));
        if(ev.type == 1)
        {

            if(ev.value == 1) // Key press
            {
           		switch (ev.code)
                {
                    case 16: //Q
                    	SetAdmittanceParameterinContact();
                        break;

                    case 18: //E
                    	SatisfyingForce();
                        break;
                }
            }
            else if(ev.value == 0) // Key released
                {
                //printf("Key released\n");

                }
        }
    }

    // Tell we have finish closing the thread
    inputCaptureThreadStopSignal = 2;
    pthread_exit(NULL);
}



int main()
{
	pthread_t hdl_inputThread; //hdl_mouseInputThread;
	int ret;
	int result;


	//We load the library
 	commandLayer_handle = dlopen("Kinova.API.USBCommandLayerUbuntu.so",RTLD_NOW|RTLD_GLOBAL);

	//We load the functions from the library
	MyInitAPI 				= (int (*)()) dlsym(commandLayer_handle,"InitAPI");
	MyCloseAPI 				= (int (*)()) dlsym(commandLayer_handle,"CloseAPI");
	MyRefresDevicesList 	= (int (*)()) dlsym(commandLayer_handle,"RefresDevicesList");
	MyMoveHome 				= (int (*)()) dlsym(commandLayer_handle,"MoveHome");
	MyInitFingers 			= (int (*)()) dlsym(commandLayer_handle,"InitFingers");
	MySetCartesianControl 	= (int (*)()) dlsym(commandLayer_handle,"SetCartesianControl");
	MySetAngularControl 	= (int (*)()) dlsym(commandLayer_handle,"SetAngularControl");
	MySendBasicTrajectory 	= (int (*)(TrajectoryPoint trajectory)) dlsym(commandLayer_handle,"SendBasicTrajectory");
	MySendAdvanceTrajectory = (int (*)(TrajectoryPoint trajectory)) dlsym(commandLayer_handle,"SendAdvanceTrajectory");
	MyGetCartesianPosition 	= (int (*)(CartesianPosition& pt)) dlsym(commandLayer_handle,"GetCartesianPosition");
	MyGetAngularPosition 	= (int (*)(AngularPosition& pt)) dlsym(commandLayer_handle,"GetAngularPosition");
	MyGetDevices 			= (int (*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) dlsym(commandLayer_handle,"GetDevices");
	MySetActiveDevice 		= (int (*)(KinovaDevice devices)) dlsym(commandLayer_handle,"SetActiveDevice");
	MyStartReactiveForceControl = (int(*)()) dlsym(commandLayer_handle, "StartForceControl");
	MyStopReactiveForceControl = (int(*)()) dlsym(commandLayer_handle, "StopForceControl");
	MySetGravityType = (int(*)(GRAVITY_TYPE Type)) dlsym(commandLayer_handle, "SetGravityType");
	MySetGravityOptimalParameter = (int(*)(float Command[GRAVITY_PARAM_SIZE])) dlsym(commandLayer_handle, "SetGravityOptimalZParam");
	MyGetCartesianForce = (int(*)(CartesianPosition &Response)) dlsym(commandLayer_handle, "GetCartesianForce");

	MySetCartesianInertiaDamping=(int(*)(CartesianInfo inertia, CartesianInfo damping)) dlsym(commandLayer_handle, "SetCartesianInertiaDamping");
	MySetCartesianForceMaxMin = (int(*)(CartesianInfo min, CartesianInfo max)) dlsym(commandLayer_handle, "SetCartesianForceMinMax");
	MyGetAngularForceGravityFree= (int(*)(AngularPosition &Response)) dlsym(commandLayer_handle, "GetAngularForceGravityFree");
	MySendCartesianForceCommand = (int (*)(float Command[COMMAND_SIZE])) dlsym(commandLayer_handle,"SendCartesianForceCommand");


	MyStopRedundantJointNullSpaceMotion = (int(*)()) dlsym(commandLayer_handle, "StopRedundantJointNullSpaceMotion");

	MySetTorqueSafetyFactor = (int(*)(float)) dlsym(commandLayer_handle, "SetTorqueSafetyFactor");
	MySetTorqueActuatorDamping = (int(*)(float Command[COMMAND_SIZE])) dlsym(commandLayer_handle, "SetTorqueActuatorDamping");

	MySwitchTrajectoryTorque = (int(*)(GENERALCONTROL_TYPE)) dlsym(commandLayer_handle, "SwitchTrajectoryTorque");

	MySetTorqueVibrationController = (int(*)(float)) dlsym(commandLayer_handle, "SetTorqueVibrationController");

	MySetPositionLimitDistance= (int(*)(float Command[COMMAND_SIZE])) dlsym(commandLayer_handle, "SetPositionLimitDistance");
	MySetTorquePositionLimitDampingGain= (int(*)(float Command[COMMAND_SIZE])) dlsym(commandLayer_handle, "SetTorquePositionLimitDampingGain");
	MySetTorquePositionLimitDampingMax= (int(*)(float Command[COMMAND_SIZE])) dlsym(commandLayer_handle, "SetTorquePositionLimitDampingMax");
	MySetTorquePositionLimitRepulsGain= (int(*)(float Command[COMMAND_SIZE])) dlsym(commandLayer_handle, "SetTorquePositionLimitRepulsGain");
	MySetTorquePositionLimitRepulsMax= (int(*)(float Command[COMMAND_SIZE])) dlsym(commandLayer_handle, "SetTorquePositionLimitRepulsMax");
	MySetTorqueGainMax= (int(*)(float Command[COMMAND_SIZE])) dlsym(commandLayer_handle, "SetTorqueGainMax");
	MySetTorqueActuatorGain= (int(*)(float Command[COMMAND_SIZE])) dlsym(commandLayer_handle, "SetTorqueActuatorGain");


	//If the was loaded correctly
	if((MyInitAPI == NULL) 				|| (MyCloseAPI == NULL) 		   || (MyGetDevices == NULL) 			||
	   (MyMoveHome == NULL) 			|| (MyInitFingers == NULL) 		   || (MySetCartesianControl == NULL) 	||
	   (MySetAngularControl== NULL) 	|| (MySendBasicTrajectory == NULL) || (MySendAdvanceTrajectory == NULL) ||
	   (MyGetCartesianPosition == NULL) || (MySetActiveDevice == NULL) 	   || (MyGetAngularPosition == NULL))
	{
		wcout << "* * *  E R R O R   D U R I N G   I N I T I A L I Z A T I O N  * * *" << endl;
	}
	else
	{
		wcout << "I N I T I A L I Z A T I O N   C O M P L E T E D" << endl << endl;

		result = (*MyInitAPI)();

		(*MyRefresDevicesList)();

		wcout << "Initialization's result :" << result << endl;

		KinovaDevice list[MAX_KINOVA_DEVICE];
		TrajectoryPoint command;
		CartesianPosition CartPos;
		AngularPosition AngPos;

		int devicesCount = MyGetDevices(list, result);

		for(int i = 0; i < devicesCount; i++)
		{
			wcout << "Found a robot on the network (" << list[i].SerialNumber << ")" << endl;

			//Setting the current device as the active device.
			MySetActiveDevice(list[i]);
			MyStopRedundantJointNullSpaceMotion();
			Init();

			ret = pthread_create(&hdl_inputThread, NULL, inputCaptureThread, NULL); //thread to capture keyboard inputs

			if(ret != 0)
			{
				printf("Error: Input pthread_create() failed\n");
			}

			wcout << "Now, drive to robot around until one hard surface from its gripper is in contact with the table"<<endl;
			while(quitSignal == 0)
			{
				usleep(1000);
			}
			inputCaptureThreadStopSignal = 1;

		}

		result = (*MyCloseAPI)();
	}

	dlclose(commandLayer_handle);

	return 0;
}
