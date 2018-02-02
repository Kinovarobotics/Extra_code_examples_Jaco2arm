
#include <Windows.h>
#include "CommunicationLayerWindows.h"
#include "CommandLayer.h"
#include <conio.h>
#include "KinovaTypes.h"
#include <iostream>
#include<string>


using namespace std;

//A handle to the API.
HINSTANCE commandLayer_handle;

//Function pointers to the functions we need
int(*MyInitAPI)();
int(*MyCloseAPI)();
int(*MySendBasicTrajectory)(TrajectoryPoint command);
int(*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
int(*MySetActiveDevice)(KinovaDevice device);
int(*MyMoveHome)();
int(*MyInitFingers)();
int(*MyGetCartesianCommand)(CartesianPosition &);
int(*MyGetAngularCommand)(AngularPosition &);
int(*MyGetCartesianPosition)(CartesianPosition &);
int(*MyGetAngularPosition)(AngularPosition &);
int(*MyStartForceControl)();
int(*MyStopForceControl)();

struct PosTime {
        CartesianPosition MyPosition[2420];
        int ControlTime;
        string TaskName;
};

void GoFirstPoint(PosTime Postime)
{
        TrajectoryPoint pointToSend;
        pointToSend.InitStruct();
        pointToSend.Position.Type = CARTESIAN_POSITION;

        cout << endl << "Robot will go back to Initial position ";
        system("pause");

        for (int i = 0; i < Postime.ControlTime; i++)
        {
                pointToSend.Position.CartesianPosition.X = Postime.MyPosition[Postime.ControlTime - 1 - i].Coordinates.X;
                pointToSend.Position.CartesianPosition.Y = Postime.MyPosition[Postime.ControlTime - 1 - i].Coordinates.Y;
                pointToSend.Position.CartesianPosition.Z = Postime.MyPosition[Postime.ControlTime - 1 - i].Coordinates.Z;
                pointToSend.Position.CartesianPosition.ThetaX = Postime.MyPosition[Postime.ControlTime - 1 - i].Coordinates.ThetaX;
                pointToSend.Position.CartesianPosition.ThetaY = Postime.MyPosition[Postime.ControlTime - 1 - i].Coordinates.ThetaY;
                pointToSend.Position.CartesianPosition.ThetaZ = Postime.MyPosition[Postime.ControlTime - 1 - i].Coordinates.ThetaZ;
                pointToSend.Position.Fingers.Finger1 = Postime.MyPosition[Postime.ControlTime - 1 - i].Fingers.Finger1;
                pointToSend.Position.Fingers.Finger2 = Postime.MyPosition[Postime.ControlTime - 1 - i].Fingers.Finger2;
                MySendBasicTrajectory(pointToSend);
        }

}
void ExecuteTask(PosTime Postime)
{
        TrajectoryPoint pointToSend;
        pointToSend.InitStruct();
        pointToSend.Position.Type = CARTESIAN_POSITION;

        cout << "Executing task " << Postime.TaskName << endl;

        for (int i = 0; i < Postime.ControlTime; i++)
        {
                pointToSend.Position.CartesianPosition.X = Postime.MyPosition[i].Coordinates.X;
                pointToSend.Position.CartesianPosition.Y = Postime.MyPosition[i].Coordinates.Y;
                pointToSend.Position.CartesianPosition.Z = Postime.MyPosition[i].Coordinates.Z;
                pointToSend.Position.CartesianPosition.ThetaX = Postime.MyPosition[i].Coordinates.ThetaX;
                pointToSend.Position.CartesianPosition.ThetaY = Postime.MyPosition[i].Coordinates.ThetaY;
                pointToSend.Position.CartesianPosition.ThetaZ = Postime.MyPosition[i].Coordinates.ThetaZ;
                pointToSend.Position.Fingers.Finger1 = Postime.MyPosition[i].Fingers.Finger1;
                pointToSend.Position.Fingers.Finger2 = Postime.MyPosition[i].Fingers.Finger2;
                MySendBasicTrajectory(pointToSend);
        }

}

PosTime LearnTask(PosTime Postime)
{
        TrajectoryPoint pointToSend;
        pointToSend.InitStruct();

        cout << "Please name your task : ";
        cin >> Postime.TaskName;
        cout << endl;
        cout << "For how long do you want to teach the robot, Please enter a value in seconds (max 300 seconds) " << endl;
        cin >> Postime.ControlTime;
        cout << endl;
        while (Postime.ControlTime > 300 || Postime.ControlTime < 0)
        {
                cout << "Incorrect Value, please try again" << endl;
                cin >> Postime.ControlTime;
                cout << endl;
        }
        MyStartForceControl();
        cout << "The Robot will be in Teach Mode for " << Postime.ControlTime << " seconds learnig Task " << Postime.TaskName << ", Please put the Robot in his Initial Position ; ";
        system("pause");
        cout << endl;
        Postime.ControlTime = 8 * Postime.ControlTime;
        cout << "Control will begin in ";

        for (int i = 0; i < 3; i++)
        {
                if (i == 0)
                {
                        cout << 2 - i;
                        Sleep(1000);
                }
                else
                {
                        cout << "..." << 2 - i;
                        Sleep(1000);
                }

        }
        cout << endl << endl;
        cout << "Robot is in teach mode" << endl;

        MyGetCartesianCommand(Postime.MyPosition[0]);

        //CartesianPosition MyPosition;


        for (int i = 0; i < Postime.ControlTime; i++)
        {
                MyGetCartesianCommand(Postime.MyPosition[i]);
                Sleep(125);
                if (i % 8 == 0)
                {
                        cout <<((Postime.ControlTime-i)/8)-1 << " ";
                }
        }

        MyStopForceControl();
        pointToSend.Position.Type = CARTESIAN_POSITION;

        //do you want to do back to the first point recorded (recommended)?
        char YesNo = 0;
        cout << "Do you want to go your first recorded point using the reverse mode? Press Y or N" << endl;
        cin >> YesNo;

        while (YesNo != 'n' && YesNo != 'N' && YesNo != 'y' && YesNo != 'Y')
        {
                cout << "Invalid Answer, please try again." << endl;
                cin >> YesNo;
        }
        if (YesNo == 'y' || YesNo == 'Y')
        {
                GoFirstPoint(Postime);
        }
        return Postime;
}



int main(int argc, char* argv[])
{
        //We load the API.
        commandLayer_handle = LoadLibrary(L"CommandLayerWindows.dll");

        AngularPosition currentCommand;

        int programResult = 0;

        //We load the functions from the library (Under Windows, use GetProcAddress)
        MyInitAPI = (int(*)()) GetProcAddress(commandLayer_handle, "InitAPI");
        MyCloseAPI = (int(*)()) GetProcAddress(commandLayer_handle, "CloseAPI");
        MyMoveHome = (int(*)()) GetProcAddress(commandLayer_handle, "MoveHome");
        MyInitFingers = (int(*)()) GetProcAddress(commandLayer_handle, "InitFingers");
        MyGetDevices = (int(*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) GetProcAddress(commandLayer_handle, "GetDevices");
        MySetActiveDevice = (int(*)(KinovaDevice devices)) GetProcAddress(commandLayer_handle, "SetActiveDevice");
        MySendBasicTrajectory = (int(*)(TrajectoryPoint)) GetProcAddress(commandLayer_handle, "SendBasicTrajectory");
        MyGetCartesianCommand = (int(*)(CartesianPosition &)) GetProcAddress(commandLayer_handle, "GetCartesianCommand");
        MyGetAngularCommand = (int(*)(AngularPosition &)) GetProcAddress(commandLayer_handle, "GetAngularCommand");
        MyGetCartesianPosition = (int(*)(CartesianPosition &)) GetProcAddress(commandLayer_handle, "GetCartesianPosition");
        MyGetAngularPosition = (int(*)(AngularPosition &)) GetProcAddress(commandLayer_handle, "GetAngularPosition");
        MyStartForceControl = (int(*)()) GetProcAddress(commandLayer_handle, "StartForceControl");
        MyStopForceControl = (int(*)()) GetProcAddress(commandLayer_handle, "StopForceControl");


        //Verify that all functions has been loaded correctly
        if ((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MySendBasicTrajectory == NULL) ||
                (MyGetDevices == NULL) || (MySetActiveDevice == NULL) || (MyGetCartesianCommand == NULL) ||
                (MyMoveHome == NULL) || (MyInitFingers == NULL) || (MyGetAngularCommand == NULL) ||
                (MyGetCartesianPosition == NULL) || (MyGetAngularPosition == NULL) || (MyStartForceControl == NULL) ||
                (MyStopForceControl == NULL))

        {
                cout << "* * *  E R R O R   D U R I N G   I N I T I A L I Z A T I O N  * * *" << endl;
                programResult = 0;
        }
        else
        {
                cout << "I N I T I A L I Z A T I O N   C O M P L E T E D" << endl << endl;

                int result = (*MyInitAPI)();

                cout << "Initialization's result :" << result << endl;

                KinovaDevice list[MAX_KINOVA_DEVICE];

                int devicesCount = MyGetDevices(list, result);

                for (int i = 0; i < devicesCount; i++)
                {
                        cout << "Found a robot on the USB bus (" << list[i].SerialNumber << ")" << endl;

                        //Setting the current device as the active device.
                        MySetActiveDevice(list[i]);

                        cout << "Send the robot to HOME position" << endl;
                        MyMoveHome();

                        cout << "Initializing the fingers" << endl;
                        MyInitFingers();

                        TrajectoryPoint pointToSend;
                        pointToSend.InitStruct();

                        //We specify that this point will use Cartesian position
                        pointToSend.Position.Type = CARTESIAN_POSITION;

                        int menu = 0;
                        int TaskNumber = 0;
                        int task = 0;
                        PosTime PositionTime[8];


                        while (menu != 11)
                        {
                                system("cls");
                                cout << "Please choose an option" << endl;

                                if (TaskNumber < 9)
                                {
                                        cout << "0 : Teach the robot a new Task" << endl;

                                        for (int i = 0; i < TaskNumber; i++)
                                        {
                                                cout << i + 1 << " : " << PositionTime[i].TaskName << endl;
                                        }

                                        cout << "9 : Go to Home Position" << endl;
                                        cout << "11 : EXIT" << endl;
                                        cin >> menu;
                                        cout << endl;
                                }

                                if (menu == 0)
                                {
                                        PositionTime[TaskNumber] = LearnTask(PositionTime[TaskNumber]);
                                        TaskNumber = TaskNumber + 1;
                                }
                                else
                                {
                                        if (menu == 9)
                                        {
                                                MyMoveHome();
                                        }

                                        else
                                        {
                                                if (menu != 11 & menu - 1 < TaskNumber)
                                                {
                                                        ExecuteTask(PositionTime[menu - 1]);

                                                        //do you want to do back to the first point recorded (recommended)?
                                                        char YesNo = 0;
                                                        cout << "Do you want to go to your first recorded point using the reverse mode? Press Y or N" << endl;
                                                        cin >> YesNo;

                                                        while (YesNo != 'n' && YesNo != 'N' && YesNo != 'y' && YesNo != 'Y')
                                                        {
                                                                cout << "Invalid Answer, please try again." << endl;
                                                                cin >> YesNo;
                                                        }
                                                        if (YesNo == 'y' || YesNo == 'Y')
                                                        {
                                                                GoFirstPoint(PositionTime[menu - 1]);
                                                        }

                                                }
                                        }


                                }

                        }

                }

                cout << endl << "C L O S I N G   A P I" << endl;
                result = (*MyCloseAPI)();
                programResult = 1;
        }

        FreeLibrary(commandLayer_handle);

        return programResult;

}


