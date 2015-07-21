RobocupSSL Low-Level 
==============================
Description of the content of the various folders:
 * Arduino : Source of the on-board arduino and Processing code for a serial data ploting tools
 * DebugTools : Testing tools for the usbPacket communication
 * JoystickDriver : PC joystick interface for teleoperation of the robot
 * RobocupV2_1 : Onboard low-level C code for C2000 MCU for the alpha version of the RobocupSSL's robot 
 * RobocupV2_2 : Onboard low-level C code for C2000 MCU for the beta version of the RobocupSSL's robot 
 * Robocup_RadioBase_VX_x : Onboard low-level C code for C2000 MCU the radio telecommunication between the Master computer and the team of robot.

##Starting low-level development##
 * Install  [Code Composer Studio(CCS) *version 5.5*](http://processors.wiki.ti.com/index.php/Download_CCS).
 * Clone the git repository at the root of the workspace of the Code Composer Studio(CCS) *version 5.5*.
 * NOTE: You don't need to install ControlSUITE. All the required C2000 f2802x librairies are in C2000_include_zipped.zip

##How to include the C2000 librairies##
On Linux execute the unzip_C2000_include.sh. The extracted files must be at the root of the CCS workspace.
Then in project's properties go in C2000 Linker -> File Search Path. In add include lib, write the following path : "${WORKSPACE_LOC}/C2000_LaunchPad/f2802x_common/lib/driverlib.lib"

On Windows unzip the C2000_include_zipped.zip at the root of the CCS workspace
