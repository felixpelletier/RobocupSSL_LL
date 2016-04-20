RobocupSSL Low-Level main repository
==============================
Description of the content of the various folders:
 * ArduinoRobot : Source code for the on-board arduino.
 * DebugTools : Various Testing tools.
 * C2000Robot : Onboard low-level C code for C2000 MCU of the RobocupSSL robot.
 * C2000RadioBase : Onboard low-level C code for C2000 MCU the radio telecommunication between the Master computer and the robot swarm.

##Starting low-level development##
 * Install  [Code Composer Studio(CCS) *version 5.5*](http://processors.wiki.ti.com/index.php/Download_CCS).
 * Clone the git repository at the root of the workspace of the Code Composer Studio(CCS) *version 5.5*.
 * NOTE: You don't need to install ControlSUITE. All the required C2000 f2802x librairies are in C2000_include_zipped.zip

##How to include the C2000 librairies##
On Linux execute the unzip_C2000_include.sh. The extracted files must be at the root of the CCS workspace.
Then in project's properties go in C2000 Linker -> File Search Path. In add include lib, write the following path : "{PROJECT_loc}/../driverlib.lib"

On Windows unzip the C2000_include_zipped.zip at the root of the CCS workspace
