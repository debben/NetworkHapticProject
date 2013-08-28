#include <stdlib.h>
#include <iostream>
#include <cstdio>
#include <cassert>

#include <HD/hd.h>

#include "helper.h"

#include <HDU/hduError.h>
#include <HDU/hduVector.h>

#include <stdio.h>
#include <time.h>

#if defined(WIN32)
# include <windows.h>
# include <conio.h>
#else
# include "conio.h"
# include <stdlib.h>
# include <ctype.h>
# include <string.h>
#endif


#include <math.h>
#include "udpnet.h"//This file contains the network functions written for this program

//[Reza] The cursors's position which will be updated as long as data is received
float CursorX = 0, CursorY = 0, CursorZ = 0;

long MinTimeDelay = -1, MaxTimeDelay = 0;

static double sphereSmall = 12.0;
static double sphereBig = 50;
/* Charge (positive/negative) */
int charge = 1;
int width, height;
static HHD ghHD = HD_INVALID_HANDLE;
static HDSchedulerHandle gSchedulerCallback = HD_INVALID_HANDLE;
float initX = 50;
/* Glut callback functions used by helper.cpp */
void displayFunction(void);
void handleIdle(void);


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

static HDdouble gMaxStiffness = 1.0;

HDSchedulerHandle gCallbackHandle = 0;

void mainLoop();
HDCallbackCode HDCALLBACK SetSpringStiffnessCallback(void *pUserData);
HDCallbackCode HDCALLBACK AnchoredSpringForceCallback(void *pUserData);
hduVector3Dd forceField(hduVector3Dd pos);

void loadConfig(char* addr, int* remoteport, int* localport, bool* master, bool* sensors);

char label1[50];
char label2[50];
char label3[50];
char label4[50];
/* Haptic device record. */
struct DeviceDisplayState
{
    HHD m_hHD;
    hduVector3Dd position;
    hduVector3Dd force;
	hduVector3Dd velocity;
	
};
DeviceDisplayState state;
HDdouble timer = 0;

//HDCallbackCode HDCALLBACK DeviceStateCallback(void *pUserData)
//{
//    DeviceDisplayState *pDisplayState = 
//        static_cast<DeviceDisplayState *>(pUserData);
	
 //   hdGetDoublev(HD_CURRENT_POSITION, pDisplayState->position);
 //   hdGetDoublev(HD_CURRENT_FORCE, pDisplayState->force);
	//hdGetDoublev(HD_CURRENT_VELOCITY, pDisplayState ->velocity );
//    // execute this only once.
//    return HD_CALLBACK_DONE;
//}

hduVector3Dd project(const hduVector3Dd &a, const hduVector3Dd &b)
{
	hduVector3Dd tmp= (a.dotProduct(b)*b / b.dotProduct(b));
	return tmp;
	
}
/*******************************************************************************
 Graphics main loop function.
*******************************************************************************/
hduVector3Dd getVirtual(hduVector3Dd velocity)
{
	hduVector3Dd velo=project(velocity , hduVector3Dd (1,0,0));
	initX+= (velo[0]>0 ? 1 :-1 ) *
	state.velocity.magnitude()*0.001;    
	return hduVector3Dd (initX, 0,0);
}
void displayFunction(void)
{
    // Setup model transformations.
    glMatrixMode(GL_MODELVIEW); 
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glPushMatrix();

    setupGraphicsState();
	
	GLfloat mau[3]={1,1,1};
	GLfloat mauDark[3]={.7,.7,.7};
	GLfloat mauDarker[3]={.4,.4,.4};
    glEnable(GL_COLOR_MATERIAL);
  
	// Draw the fixed sphere.
    
	static const float fixedSphereColor[4] = {.2, .8, .8, .8};
    GLUquadricObj* pQuadObj = gluNewQuadric();
    drawSphere(pQuadObj, hduVector3Dd (0,0,0), fixedSphereColor, sphereBig );
	
    // hdScheduleSynchronous(DeviceStateCallback, &state,
    //                    HD_MIN_SCHEDULER_PRIORITY);
	static const float dynamicSphereColor[4] = { .8, .2, .2, .8 };
	//hduVector3Dd currentPos= getVirtual(state.velocity );
	
	hduVector3Dd currentPos(CursorX, CursorY, CursorZ);

	drawSphere(pQuadObj,
               //state.position,
               currentPos,
               dynamicSphereColor,
               sphereSmall);	
	
	char chuoi[50];
	sprintf(chuoi, "Current velocity: %.4f", state.velocity.magnitude());
	
	drawHapticsString(label1, hduVector3Dd(width *-0.9,height*0.9,0), 0.1, mauDark);
    drawHapticsString(label2, hduVector3Dd(width *-0.9,height*0.8,0), 0.1, mau);
    drawHapticsString(label3, hduVector3Dd(width *-0.9,height*0.7,0), 0.1, mauDarker);
    drawHapticsString(label4, hduVector3Dd(width *-0.9,height*0.6,0), 0.1, mauDarker);
    
	hduVector3Dd forceVector = forceField(currentPos);
    

    drawForceVector(pQuadObj,
                    currentPos,
                    forceVector,
                    sphereSmall*.1);

    gluDeleteQuadric(pQuadObj);
  
    glPopMatrix();
    glutSwapBuffers();    
}
                                
/*******************************************************************************
 Called periodically by the GLUT framework.
*******************************************************************************/
void handleIdle(void)
{
    glutPostRedisplay();

	//[Reza] Here we call the mainLoop function from the master_slave code which is not a loop anymore! 
	//[Reza] It only receives and processes packets needed to display 1 video frame
	mainLoop();

    //if (!hdWaitForCompletion(gSchedulerCallback, HD_WAIT_CHECK_STATUS))
    //{
    //    printf("The main scheduler callback has exited\n");
    //    printf("Press any key to quit.\n");
    //    getchar();
    //    exit(-1);
    //}
}

/******************************************************************************
 Popup menu handler
******************************************************************************/
void handleMenu(int ID)
{
    switch(ID) 
    {
        case 0:
            exit(0);
            break;
        case 1:
            charge *= -1;
            break;
    }
}


/*******************************************************************************
 Given the position is space, calculates the (modified) coulomb force.
*******************************************************************************/
hduVector3Dd forceField(hduVector3Dd pos)
{
    double dist = pos.magnitude();
    
    hduVector3Dd forceVec(0,0,0);
    
        double loi = pos[0]-sphereBig;
        if (loi<0)
		{
			forceVec = hduVector3Dd (-10.0*loi,0,0);
			
		}
    return forceVec;
}



/*******************************************************************************
 Main callback that calculates and sets the force.
*******************************************************************************/
double am=0.01;
double freg=500;
double B=10;
double K=1;
hduVector3Dd calculateF(hduVector3Dd nguon, hduVector3Dd &origin, double radi)
{
	if ((nguon-origin).magnitude()<radi)
		return (radi-(nguon-origin).magnitude())* normalize(nguon-origin)
				;
	else return hduVector3Dd(0,0,0);
}


hduVector3Dd remVec(0,0,0);
//HDCallbackCode HDCALLBACK CoulombCallback(void *data)
//{
//    HHD hHD = hdGetCurrentDevice();
//	HDdouble instRate;
//    hduVector3Dd force(0,0,0);
//	
//    hdBeginFrame(hHD);
//	
//	hdGetDoublev(HD_INSTANTANEOUS_UPDATE_RATE, &instRate);
//
//	timer += 1.0 / instRate;
//	hduVector3Dd pos;
//	hdGetDoublev (HD_CURRENT_POSITION, pos);
//    
//	hduVector3Dd layforce=calculateF(pos,
//			hduVector3Dd(0,0.0,0),
//			sphereBig );
//	if (layforce.magnitude()>0.01*sphereBig 
//		&& timer>0.5
//		) //penetrate more than 1% and 5 seconds has passed
//	{
//		timer=0;
//		hdGetDoublev (HD_CURRENT_VELOCITY, remVec);
//	}	
//	force=am * remVec *exp(-B*timer) * sin(timer * freg);
//	
//
//	hduVector3Dd tam;
//	hdGetDoublev(HD_NOMINAL_MAX_FORCE, tam);
//	sprintf(label1, "Current Force: %.4f",force.magnitude());
//	hdGetDoublev(HD_SOFTWARE_VELOCITY_LIMIT, tam);
//	//sprintf(label3, "Current exponential: %.4f",exp(-B*timer));
//	sprintf(label4, "Current timer: %.4f", timer);
//	sprintf(label2, "Current Hook Force: %.4f",layforce.magnitude());
//	
//	force=force+ K* layforce	;
//	
//	if (force.magnitude()>1)
//	force= force/force.magnitude();
//		
//	hdSetDoublev(HD_CURRENT_FORCE, force);
//    
//	hdEndFrame(hHD);
//
//    //HDErrorInfo error;
//    //if (HD_DEVICE_ERROR(error = hdGetError()))
//    //{
//    //    hduPrintError(stderr, &error, "Error during scheduler callback");
//    //    if (hduIsSchedulerError(&error))
//    //    {
//    //        return HD_CALLBACK_DONE;
//    //    }
//    //}
//
//	
//    return HD_CALLBACK_CONTINUE;
//	
//}

/*******************************************************************************
 Schedules the coulomb force callback.
*******************************************************************************/
void CoulombForceField()
{
    std::cout << "haptics callback" << std::endl;
    //gSchedulerCallback = hdScheduleAsynchronous(
    //    CoulombCallback, 0, HD_DEFAULT_SCHEDULER_PRIORITY);

    //HDErrorInfo error;
    //if (HD_DEVICE_ERROR(error = hdGetError()))
    //{
    //    hduPrintError(stderr, &error, "Failed to initialize haptic device");
    //    fprintf(stderr, "\nPress any key to quit.\n");
    //    getchar();
    //    exit(-1);
    //}

    std::cout << "graphics callback" << std::endl;

    glutMainLoop(); // Enter GLUT main loop.
}

/******************************************************************************
 This handler gets called when the process is exiting. Ensures that HDAPI is
 properly shutdown
******************************************************************************/
void exitHandler()
{
    //hdStopScheduler();
    //hdUnschedule(gSchedulerCallback);

    //if (ghHD != HD_INVALID_HANDLE)
    //{
    //    hdDisableDevice(ghHD);
    //    ghHD = HD_INVALID_HANDLE;
    //}
}
void keyboard(unsigned char bam, int x, int y)
{
	if (bam=='x')
	{
		timer=0;
		
	}

}
/******************************************************************************
 Main function.
******************************************************************************/
int main(int argc, char* argv[])
{
    HDErrorInfo error;

    printf("Starting application\n");
    timer=0;
    atexit(exitHandler);

	//---------------------------------------------------
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
	//---------------------------------------------------------

	//mainLoop();



     //Initialize the device.  This needs to be called before any other
     //actions on the device are performed.
    //ghHD = hdInitDevice(HD_DEFAULT_DEVICE);
    //if (HD_DEVICE_ERROR(error = hdGetError()))
    //{
    //    hduPrintError(stderr, &error, "Failed to initialize haptic device");
    //    fprintf(stderr, "\nPress any key to quit.\n");
    //    getchar();
    //    exit(-1);
    //}

    //printf("Found device %s\n",hdGetString(HD_DEVICE_MODEL_TYPE));
    //
    //hdEnable(HD_FORCE_OUTPUT);
    //hdEnable(HD_MAX_FORCE_CLAMPING);

    //hdStartScheduler();
    //if (HD_DEVICE_ERROR(error = hdGetError()))
    //{
    //    hduPrintError(stderr, &error, "Failed to start scheduler");
    //    fprintf(stderr, "\nPress any key to quit.\n");
    //    getchar();
    //    exit(-1);
    //}
	
    initGlut(argc, argv);

    // Get the workspace dimensions.
    //HDdouble maxWorkspace[6];
    //hdGetDoublev(HD_MAX_WORKSPACE_DIMENSIONS, maxWorkspace);
	HDdouble maxWorkspace[6] = {-210, -110, -85, 210, 205, 130};
	 

    // Low/left/back point of device workspace.
    hduVector3Dd LLB(maxWorkspace[0], maxWorkspace[1], maxWorkspace[2]);
    // Top/right/front point of device workspace.
    hduVector3Dd TRF(maxWorkspace[3], maxWorkspace[4], maxWorkspace[5]);
    initGraphics(LLB, TRF, width, height);

    // Application loop.
    CoulombForceField();

    printf("Done\n");
    return 0;
}


/******************************************************************************
 Receive information/control device loop
******************************************************************************/
void mainLoop()
{
    HDdouble stiffness = gSpringStiffness;

	//[Reza] Run the data receive loop for 33 mellisecond (1/30 second)
	//[Reza] We do this to prepare a maximum of 30 video frames per second
	clock_t start_time = clock();
	while (clock() < start_time + 33)
    {
		char str[80];
		MyReceive(str); //Receives information for rendering through the UDP socket
		//puts(str);

	/*	int r = rand();
		if (r < 100)
		{
			printf(str);
			printf("\r\n");
		}*/

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
			CursorX = x;
			CursorY = y;
			CursorZ = z;
			sprintf(str, "p(%f, %f, %f, %f, %f, %f)", x, y, z, vx, vy, vz);
			//SendToHost(str);
			hduVector3Dd F = calculateF(hduVector3Dd(x,y,z), hduVector3Dd(0,0.0,0), sphereBig);
			sprintf(str, "f(%f, %f, %f)", F[0], F[1], F[2]);
			SendToHost(str);

			//Send client a timestamp for time-delay measurement
			sprintf(str, "t(%d)", clock());
			SendToHost(str);
		}else
		if(str[0] == 't')
		{
			//Calculate the time-delay when a timestamp is received back
			clock_t t;
			sscanf(str, "t(%d)", &t);
			t = clock() - t;
			if (t > MaxTimeDelay)
				MaxTimeDelay = t;
			if (t < MinTimeDelay || MinTimeDelay == -1)
				MinTimeDelay = t;
			if (rand() < 100)
			{
				//printf("Time delay:  min=%dms  max=%dms  now=%dms  \r", MinTimeDelay, MaxTimeDelay, t);
				sprintf(label1, "Time delay:");
				sprintf(label2, "     Now = %d ms", t);
				sprintf(label3, "     Min = %d ms", MinTimeDelay);
				sprintf(label4, "     Max = %d ms", MaxTimeDelay);

			}
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

	//CloseConnection ();
	//Network cleanup
}

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

/******************************************************************************/
