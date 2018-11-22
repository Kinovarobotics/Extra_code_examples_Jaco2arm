
#include <Windows.h>
#include "CommunicationLayerWindows.h"
#include "CommandLayer.h"
#include <conio.h>
#include "KinovaTypes.h"
#include <iostream>


using namespace std;

//A handle to the API.
HINSTANCE commandLayer_handle;

//Function pointers to the functions we need
int(*MyInitAPI)();
int(*MyCloseAPI)();
int(*MyGetClientConfigurations)(ClientConfigurations &command);

int main(int argc, char* argv[])
{
	//We load the API.
	commandLayer_handle = LoadLibrary(L"CommandLayerWindows.dll");

	int result;
	int programResult = 0;

	//We load the functions from the library
	MyInitAPI = (int(*)()) GetProcAddress(commandLayer_handle, "InitAPI");
	MyCloseAPI = (int(*)()) GetProcAddress(commandLayer_handle, "CloseAPI");
	MyGetClientConfigurations = (int(*)(ClientConfigurations &)) GetProcAddress(commandLayer_handle, "GetClientConfigurations");

	//If the was loaded correctly
	if ((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MyGetClientConfigurations == NULL))
	{
	}
	else
	{
		result = (*MyInitAPI)();
		ClientConfigurations data;
		(*MyGetClientConfigurations)(data); 
		
		cout << "\n\n\n\n\n**************************************************************************" << endl;
		cout << "Your arm model is : " << data.Model << endl;
		cout << "**************************************************************************\n\n\n\n\n\n\n\n\n\n" << endl;

		cin >> result;
		result = (*MyCloseAPI)();
	}
	return 0;
}
