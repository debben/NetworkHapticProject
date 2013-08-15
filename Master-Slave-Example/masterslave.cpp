/*****************************************************************************
MASTER/SLAVE PROGRAM

this program make a haptic device act as a slave for the master device.

Pedro Affonso and Garret Kottman 2012

*******************************************************************************/
#ifdef  _WIN64
#pragma warning (disable:4996)
#endif

#include <stdio.h>

#if defined(WIN32)
# include <windows.h>
# include <conio.h>
#else
# include "conio.h"
# include <stdlib.h>
# include <ctype.h>
# include <string.h>
#endif

#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <stdio.h>
#include <math.h>
#include "udpnet.h"//This file contains the network functions written for this program
#include <stdio.h>

//char addr[80] = "192.168.1.2";

//#define EXPERIMENT
/*When this define is enabled, the program won't perform its normal function. Instead, it will perform a step
response experiment and record the data in a text file. As a rule, compile the program with this #define line commented
*/

static bool master;
//Tells if is this the master or slave device

static bool sensorsEnabled;
//Is getting data from force sensors or not (if not, gets data from haptic device position)

static HDdouble gSpringStiffness = 0.15;
//Strength of the spring force
//In the master, increases the amout of force feedback the user wil feel
//In slave, increases the speed and strength of motion
//Good values for this range from 0.05 to 0.25

static HDdouble gDamping = 0.001;
//Strength of the damping force
//Good values range from 0 to 0.002
//Higher values on the slave will make it move more accurately
//Lower values on the Master results in better feeling of the objects

static hduVector3Dd other_device_vel;
//Velocity informed by the coupled haptic device

static hduVector3Dd anchor;
//position of the coupled device

static hduVector3Dd forceB;
//force informations received from sensors

static boolean anchored;
//Tells if this device is following the other

static double weight = 0.5;
//Weight of the device plus attached equipment (in Newtons)
//stylus ~0.5
//shaft ~1.5

static double sensorW = 0.01, springW = 1;
//Multiplier coefficients for the spring and sensor forces, used for combining them when the sensors
//are enabled

static double touchT = 5; 
//Collision threshold distance between the master and slave. If they are more distant than this value,
//the program will suppose that the slave has touched a hard object
//Lower values (about 5) give better force feedback when touching objects
//Higher (about 15) will give less friction and freedom feeling when not touching objects

#ifdef EXPERIMENT
#define datasize 300
#define step_size 40
static double xvalues[datasize];
static double tvalues[datasize];
#endif

static HDdouble gMaxStiffness = 1.0;

HDSchedulerHandle gCallbackHandle = 0;

void mainLoop();
HDCallbackCode HDCALLBACK SetSpringStiffnessCallback(void *pUserData);
HDCallbackCode HDCALLBACK AnchoredSpringForceCallback(void *pUserData);

/*
Loads the configuration file in order to set important variables
-network parameters
-mode of operation (master/slave with/without sensors)
*/
void loadConfig(char* addr, int* remoteport, int* localport, bool* master, bool* sensors){
    FILE* f = fopen("config.txt", "r");
	if(f == NULL){
		printf("Could not open the file!");
		f = fopen("config.txt", "w");
		if(f != NULL){
			fprintf(f, "address: 192.168.1.1\nmaster: yes\nsensors: no");
		}
		fclose(f);
	}else{
	    char readv[20];
		//read ip address
		fscanf(f, "address: %s\n", addr);

		//read master attribute
		fscanf(f, "master: %s\n", readv);
		if(strcmp(readv, "yes") == 0){
            *master = true;
            *localport = 1121;
            *remoteport = 1122;
		}else{
            *master = false;
            *localport = 1122;
            *remoteport = 1121;
		}

		//read sensors attribute
		fscanf(f, "sensors: %s", readv);

		if(strcmp(readv, "yes") == 0){
            *sensors = true;
		}else{
            *sensors = false;
		}
		fclose(f);
	}
}

#ifdef EXPERIMENT
//Saves data collected in the experiment to a file
void savedata(){
	FILE* f = fopen("data.m", "w+");
	if(f == NULL){
		printf("could not save data collected\n");
		return;
	}
	int i;
	fprintf(f, "x_values = [");
	for(i = 0; i < datasize; i++){
		fprintf(f, "%.2f ", xvalues[i]);
	}
	fprintf(f, "];\nt_values = [");
	for(i = 0; i < datasize; i++){
		fprintf(f, "%.0f ", tvalues[i]);
	}
	fprintf(f, "];\n");
	fclose(f);
	printf("Data saved succescfully\n");
	getch();
}
#endif

/******************************************************************************
 Main function.
******************************************************************************/
int main(int argc, char* argv[])
{
    char addr[80];//ip address of the other computer
    int localport, remoteport;// ports to send/ receive data

    NetInit();//Starts network functions
    loadConfig(addr, &remoteport, &localport, &master, &sensorsEnabled);
	printf("Using configuration: %s, %d, %d, %s, %s\n", addr, remoteport, localport, master?"master":"slave", sensorsEnabled?"sensors":"no sensors");
	//create the sockets
	SetupSocketOnPort(localport);
	SetupSocketToHost(remoteport, addr);

    //variable initialize
	anchored = true;
	anchor[0] = 0;anchor[1] = 0;anchor[2] = 0;
	forceB[0]=0; forceB[1]=0; forceB[2]=0;
	other_device_vel[0] = 0;other_device_vel[1] = 0;other_device_vel[2] = 0;
    HDErrorInfo error;

    /*HHD hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to initialize haptic device");
        fprintf(stderr, "\nPress any key to quit.\n");
        getch();
        return -1;
    }*/

    printf("Haptic device surgery\n");

    ///* Schedule the haptic callback function for rendering the forces */
    //gCallbackHandle = hdScheduleAsynchronous(
    //    AnchoredSpringForceCallback, 0, HD_MAX_SCHEDULER_PRIORITY);

    //hdEnable(HD_FORCE_OUTPUT);

    ///* Query the max closed loop control stiffness that the device
    //   can handle.  */
    //hdGetDoublev(HD_NOMINAL_MAX_STIFFNESS, &gMaxStiffness);

    ///* Start the haptic rendering */
    //hdStartScheduler();
    //if (HD_DEVICE_ERROR(error = hdGetError()))
    //{
    //    hduPrintError(stderr, &error, "Failed to start scheduler");
    //    fprintf(stderr, "\nPress any key to quit.\n");
    //    getch();
    //    return -1;
    //}

    /* Start the main application loop. */
    mainLoop();

    ///* Cleanup by stopping the haptics loop, unscheduling the asynchronous
    //   callback, disabling the device. */
    //hdStopScheduler();
    //hdUnschedule(gCallbackHandle);
    //hdDisableDevice(hHD);

    return 0;
}


/******************************************************************************
 Receive information/control device loop
******************************************************************************/
void mainLoop()
{
    HDdouble stiffness = gSpringStiffness;
#ifdef EXPERIMENT
    /*
        Alternate main loop for the experiment mode
    */
	printf("press any key to start\n");
	getch();
	anchor[0] = step_size;
	anchor[1] = 0;
	anchor[2] = 0;
	anchored = true;
	DWORD start_time = GetTickCount();
	int current_record = 0;
	tvalues[0] = 0;
	xvalues[0] = 0;
	while (HD_TRUE && current_record < (datasize-1)){
		DWORD current_time = GetTickCount() - start_time;
		if(current_time >= 5 + tvalues[current_record]){
			hduVector3Dd position;
			current_record++;
			hdGetDoublev(HD_CURRENT_POSITION, position);
			tvalues[current_record] = current_time;
			xvalues[current_record] = position[0];
		}

        if (!hdWaitForCompletion(gCallbackHandle, HD_WAIT_CHECK_STATUS))
        {
            fprintf(stderr, "\nThe main scheduler callback has exited\n");
            fprintf(stderr, "\nPress any key to quit.\n");
            getch();
            return;
        }
	}
	savedata();
	return;
#endif

    while (HD_TRUE)
    {
		char str[80];
		MyReceive(str); //Receives information for rendering through the UDP socket
		//puts(str);

		int r = rand();
		if (r < 100)
		{
			printf(str);
			printf("\r\n");
		}

		float x, y, z, vx, vy, vz;

            //Interpret the received data as position/velocity of the other haptic device
			//puts(str);
		if(str[0] == 'f'){
			printf("f");
			//this is a force information
			//Interpret the received data as a force on y axis
			sscanf(str, "f %f", &y);
			y += weight;
			y = y>3.3?3.3:y;
			y = y<-3.3?-3.3:y;

			//printf("%f\n", y);
			forceB[0] = 0;
			forceB[1] = y;
			forceB[2] = 0;
			anchored = true;
		}else
		if(str[0] == 'p'){
			sscanf(str, "p(%f, %f, %f, %f, %f, %f)", &x, &y, &z, &vx, &vy, &vz);
			anchor[0] = x;
			anchor[1] = y;
			anchor[2] = z;
			other_device_vel[0] = vx;
			other_device_vel[1] = vy;
			other_device_vel[2] = vz;
			anchored = true;
			//puts(str);
			//printf("%f %f %f %f %f %f\n", x, y, z, vx, vy, vz);
		}


        ///* Check if the main scheduler callback has exited. */
        //if (!hdWaitForCompletion(gCallbackHandle, HD_WAIT_CHECK_STATUS))
        //{
        //    fprintf(stderr, "\nThe main scheduler callback has exited\n");
        //    fprintf(stderr, "\nPress any key to quit.\n");
        //    getch();
        //    return;
        //}
    }

	CloseConnection ();
	//Network cleanup
}

//
///******************************************************************************
// * Main scheduler callback for rendering forces.
// * Also sends position and velocity data through UDP socket to the other device
// *****************************************************************************/
//HDCallbackCode HDCALLBACK AnchoredSpringForceCallback(void *pUserData)
//{
//    //static hduVector3Dd anchor;
//
//    HDErrorInfo error;
//
//    hduVector3Dd position, velocity;
//    hduVector3Dd force, deltaP;
//	force[0] = 0; force[1] = 0; force[2] = 0;
//
//    hdBeginFrame(hdGetCurrentDevice());
//
//    hdGetDoublev(HD_CURRENT_POSITION, position);
//
//    if (anchored)
//    {
//        /* Compute force */
//
//        /*
//		Renders the force based on the formula
//        F = (p0-p)*k - v*b - weight
//		for the master the formula is more complicated
//        */
//        hdGetDoublev(HD_CURRENT_VELOCITY, velocity);
//
//        hduVecSubtract(deltaP, anchor, position);
//		force = deltaP;
//        hduVecScaleInPlace(force, gSpringStiffness);
//
//        hduVecScaleInPlace(velocity, -gDamping);
//        hduVecAdd(force, force, velocity);
//
//        //If this is the master and the slave isn't touching an object, no force is rendered
//        if(master && (magnitude(deltaP) < touchT)){
//            hduVecScaleInPlace(force, 0.0);
//		}else
//		if(master && (magnitude(deltaP) >= touchT)){
//			hduVecScaleInPlace(force, (magnitude(force)-touchT*gSpringStiffness)/magnitude(force));
//		}
//
//		//if sensors are enabled, add the sensor feedback with a weighted average
//		if(master && sensorsEnabled){
//			hduVector3Dd forceTemp;
//			hduVecScaleInPlace(force, springW);
//			forceTemp = forceB;
//			hduVecScaleInPlace(forceTemp, sensorW);
//			hduVecAdd(force, force, forceTemp);
//		}
//        
//		
//
//		force[1] = force[1] + weight;//removes gravity
//		double maxForce = 3.3;
//		if(magnitude(force) > maxForce){
//			hduVecScaleInPlace(force, maxForce/magnitude(force));
//		}
//		
//        hdSetDoublev(HD_CURRENT_FORCE, force);
//
//
//    }
//
//	//sends position and velocity information to the other device
//
//	char str[80];
//	sprintf(str, "p(%f, %f, %f, %f, %f, %f)", position[0], position[1], position[2], velocity[0], velocity[1], velocity[2]);
//	SendToHost(str);
//
//    hdEndFrame(hdGetCurrentDevice());
//
//    /* Check if an error occurred while attempting to render the force */
//    if (HD_DEVICE_ERROR(error = hdGetError()))
//    {
//        if (hduIsForceError(&error))
//        {
//			printf("Maximum force exceeded!\n");
//        }
//        else if (hduIsSchedulerError(&error))
//        {
//            return HD_CALLBACK_DONE;
//        }
//    }
//
//    return HD_CALLBACK_CONTINUE;
//}

//double magnitude(hduvector3dd v){
//    return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
//}
