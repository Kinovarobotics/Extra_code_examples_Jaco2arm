This compressed folder contains the .so libraries compiled for a NVIDIA Jetson Tx2 running Ubuntu 16.04. 
** Those libraries were minimally tested and are NOT officially supported by Kinova **
The compressed file contains the four .so libraries (Command layer and communication layer for Ethernet and USB connection), as well as a simple example we used to test the communication between the robot and the tx2 board

At the moment, the Ethernet communication with the robot is successful. The USB communication leads to a Segmentation fault. We might investigate this fault in the future if we see a clear need from the community. 
