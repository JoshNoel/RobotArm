Arduino Robot Arm

HARDWARE
Arduino, SSC32

INSTALLATION
Use "Github for Windows" (http://windows.github.com/Dowload) on your home computer - his will allow easy syncing of the project to github. Once it is installed, simply click "Clone to Windows" on the top of the github project page. From then on, just use Github for Windows to sync to new versions. Also, if you want to find the files to run/edit, simply right click the project in Github for Windows and select "open in explorer," or go to "Users\YOURNAME\Documents\GitHub\PVRobotArm" aka, the Github folder in My Documents. If you can't use Github for Windows, dowload the the .zip and unzip it, however, this will not stay up to date.

HOW TO USE AT SCHOOL
I recommend dowloading SyncToy (http://www.microsoft.com/en-us/download/details.aspx?id=15155) and setting it up to sync the local project files (e.g. "Documents\GitHub\PVRobotArm") to a folder in your USB (e.g. "E:\Robot Arm"). Set it to "synchronise" when setting up the folder pair. AFTER CREATING THE SYNC PAIR, GO TO "CHANGE OPTIONS" AND CHECK "EXCLUDE HIDDEN FILES". Using SyncToy you can keep your usb up to date with the files on your computer (make sure they are up to date too) to your USB, and when you come home from school, you can then sync your changes back into your files and upload them to the repository with Github for Windows. For quick acess, you may want to put a SyncToy shortcut on your desktop or USB - the files will not sync automatically.

INSTRUCTIONS
The main robot arm files are located in "PVRobotArm\Arduino 1.0\programs\Arm_PSX_VMPv7"
The libraries will also be required for usage, but if you just run the Arduino IDE included in the repositoory, everything should work out fine.

CREDITS
Special thanks to the Arduino team for making the Arduino IDE, Circuits@Home for the inverse kinematics code, and Martin Peris for his SSC32 arduino driver