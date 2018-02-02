// Example_Ethernet7dof_TorqueControl2.cpp : Defines the entry point for the console application.
//

/*****************************************************************************************************/
/*                                                                                                   */
/* This example uses the 7 dof robot and USB communication. It switches the robot to torque mode and */
/* then moves the fingers using virtual joystick API commands.                                       */
/*                                                                                                   */
/*****************************************************************************************************/

#include <Windows.h>
#include "CommunicationLayerWindows.h"
#include "CommandLayer.h"
#include <conio.h>
#include "KinovaTypes.h"
#include <iostream>
#include <fstream>

using namespace std;

int main()
{
        int result;

        //Handle for the library's command layer.
        HINSTANCE commandLayer_handle;

        //Function pointers to the functions we need
        int(*MyInitAPI)();
        int(*MyCloseAPI)();
        int(*MyRefresDevicesList)();
        int(*MyMoveHome)();
        int(*MyInitFingers)();
        int(*MySwitchTorque)(GENERALCONTROL_TYPE type);
        int(*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
        int(*MySetActiveDevice)(KinovaDevice device);
        int(*MySetSafetyFactorTorque)(float factor);
        int(*MySendJoystickCommand)(JoystickCommand joystickCommand);
        int(*MyStartControlAPI)();
        int(*MyGetControlMapping)(ControlMappingCharts &Response);
        int(*MySetControlMapping)(ControlMappingCharts Response);


        //We load the library
        //commandLayer_handle = LoadLibrary(L"CommandLayerEthernet.dll");
        commandLayer_handle = LoadLibrary(L"CommandLayerWindows.dll");

        //We load the functions from the library
        MyInitAPI = (int(*)()) GetProcAddress(commandLayer_handle, "InitAPI");
        MyInitFingers = (int(*)()) GetProcAddress(commandLayer_handle, "InitFingers");
        MyCloseAPI = (int(*)()) GetProcAddress(commandLayer_handle, "CloseAPI");
        MyRefresDevicesList = (int(*)()) GetProcAddress(commandLayer_handle, "RefresDevicesList");
        MyMoveHome = (int(*)()) GetProcAddress(commandLayer_handle, "MoveHome");
        MySwitchTorque = (int(*)(GENERALCONTROL_TYPE type)) GetProcAddress(commandLayer_handle, "SwitchTrajectoryTorque");

        MyGetDevices = (int(*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) GetProcAddress(commandLayer_handle, "GetDevices");
        MySetActiveDevice = (int(*)(KinovaDevice devices)) GetProcAddress(commandLayer_handle, "SetActiveDevice");
        MySetSafetyFactorTorque = (int(*)(float factor)) GetProcAddress(commandLayer_handle, "SetTorqueSafetyFactor");

        MySendJoystickCommand = (int(*)(JoystickCommand joystickCommand)) GetProcAddress(commandLayer_handle, "SendJoystickCommand");

        MyStartControlAPI = (int(*)()) GetProcAddress(commandLayer_handle, "StartControlAPI");
        MyGetControlMapping = (int(*)(ControlMappingCharts &Response)) GetProcAddress(commandLayer_handle, "GetControlMapping");
        MySetControlMapping = (int(*)(ControlMappingCharts Response)) GetProcAddress(commandLayer_handle, "SetControlMapping");

        //If the was loaded correctly
        if ((MyInitAPI == NULL) || (MyInitFingers == NULL) || (MyCloseAPI == NULL) || (MyGetDevices == NULL) || (MyMoveHome == NULL)
                || (MyRefresDevicesList == NULL) || (MySendJoystickCommand == NULL) || (MyStartControlAPI == NULL) || (MyGetControlMapping == NULL)
                || (MySetControlMapping == NULL) || (MySwitchTorque == NULL) || (MySetActiveDevice == NULL) || (MySetSafetyFactorTorque == NULL))
        {
                cout << "* * *  E R R O R   D U R I N G   I N I T I A L I Z A T I O N  * * *" << endl;
        }
        else
        {
                cout << "I N I T I A L I Z A T I O N   C O M P L E T E D" << endl << endl;

                result = (*MyInitAPI)();
                (*MyRefresDevicesList)();

                cout << "Initialization's result :" << result << endl;

                KinovaDevice list[MAX_KINOVA_DEVICE];
                wchar_t ansCalib;
                float OptimalzParam[OPTIMAL_Z_PARAM_SIZE_7DOF];

                float GravParamCommand[GRAVITY_PARAM_SIZE];
                for (int i = 0; i<GRAVITY_PARAM_SIZE; i++)
                {
                        GravParamCommand[i] = 0.0; //initialization
                }


                float TorqueCommand[COMMAND_SIZE];
                for (int i = 0; i<COMMAND_SIZE; i++)
                {
                        TorqueCommand[i] = 0.0; //initialization
                }


                int devicesCount = MyGetDevices(list, result);

                for (int i = 0; i < devicesCount; i++)
                {
                        cout << "Found a robot on the USB bus (" << list[i].SerialNumber << ")" << endl;

                        //Setting the current device as the active device.
                        MySetActiveDevice(list[i]);

                        // switch to position mode
                        MySwitchTorque(POSITION);

                        //switch to torque mode
                        MyMoveHome();
                        Sleep(10000);
                        result = MyInitFingers();
                        MySwitchTorque(TORQUE);
                        MySetSafetyFactorTorque(1.0);

                        cout << "Here you have 10 seconds to play with the robot in torque mode." << endl
                                << "Try to move it with your hand" << endl << endl;
                        Sleep(10000);

                        JoystickCommand virtualCommand;
                        //Initializing the command.
                        for (int i = 0; i < JOYSTICK_BUTTON_COUNT; i++)
                        {
                                virtualCommand.ButtonValue[i] = 0;
                        }
                        virtualCommand.InclineForwardBackward = 0;
                        virtualCommand.InclineLeftRight = 0;
                        virtualCommand.MoveForwardBackward = 0;
                        virtualCommand.MoveLeftRight = 0;
                        virtualCommand.PushPull = 0;
                        virtualCommand.Rotate = 0;
                        virtualCommand.Rotate = 0;
                        AngularPosition ang;

                        //do only once after a reset (switch to finger control mode). OR ELSE COMMENT OUT LINES 138 TO 168
                        //check switch button switches to finger control mode in the actual mapping. The default mapping for the API will normally have two modes A, with the second mode A controlling the fingers.
                        result = (*MyStartControlAPI)();
                        Sleep(2000);
                        ControlMappingCharts ans;
                        MyGetControlMapping(ans);

                        int index_button = 0;
                        for (int i = 0; i < 26; i++)
                        {
                                if (ans.Mapping[ans.ActualControlMapping-1].ModeControlsA[0].ControlButtons[i].OneClick == 5)
                                {
                                        index_button = i;
                                        cout << "Button " << i << " is switching to finger control mode" << endl;
                                        break;
                                }
                        }

                        //we simulate the click of a button
                        result = (*MyStartControlAPI)();
                        cout << "Switching to finger commmand mode" << endl;
                        virtualCommand.ButtonValue[index_button] = 1;

                        for (int pp = 0; pp < 10; pp++)
                        {
                                (*MySendJoystickCommand)(virtualCommand);
                                Sleep(5);
                        }
                        virtualCommand.ButtonValue[index_button] = 0;
                        (*MySendJoystickCommand)(virtualCommand);
                        Sleep(2000);

                        //This is the point that will be send to the robot.
                        //We prepare the virtual joystick command that will be sent to the robotic arm.
                        result = (*MyStartControlAPI)();
                        cout << "We take control of the robotic arm." << endl;
                        cout << "Closing, then opening the fingers" << endl;
                        for (int pp = 0; pp < 200; pp++)
                        {
                                virtualCommand.InclineForwardBackward = 1;
                                (*MySendJoystickCommand)(virtualCommand);
                                Sleep(5);
                        }
                        for (int pp = 0; pp < 200; pp++)
                        {
                                virtualCommand.InclineForwardBackward = -1;
                                (*MySendJoystickCommand)(virtualCommand);
                                Sleep(5);
                        }
                        Sleep(8000);

                        cout << "Back to Trajectory mode" << endl << endl;
                        MySwitchTorque(POSITION);

                }

                result = (*MyCloseAPI)();
        }

        FreeLibrary(commandLayer_handle);

        return 0;
}
