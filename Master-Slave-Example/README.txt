TO SETUP TELEOPERATION:

Run Haptic_Config.exe

Select device type
Enter other computer's ip address
Check box if labview sensor data needs to be transmitted. 
Click apply to save configuration. 

Run program. 


Some parameters can be changed in the program's code. Just open the file AnchoredSpringForce.vcproj. The parameters are:

Spring stiffness
Damping coefficient
Spring force and Sensor force weights (in case sensors are being used)
Freedom/Collision threshold
Weight of the stylus and/or attached shaft (for gravity compensation)

They are constants defined and precisely commented in the first lines of the code file masterslave.cpp. 
Their values can be changed as needed. After making changes, recompile the code and run.