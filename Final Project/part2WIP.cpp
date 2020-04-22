/*****************************************************************************

  Copyright (c) 2004 SensAble Technologies, Inc. All rights reserved.

  OpenHaptics(TM) toolkit. The material embodied in this software and use of
  this software is subject to the terms and conditions of the clickthrough
  Development License Agreement.

  For questions, comments or bug reports, go to forums at:
http://dsc.sensable.com

Module Name:

main.cpp

Description:

The main file that performs all haptics-relevant operation. Within a
asynchronous callback the graphics thread reads the position and sets
the force. Within a synchronous callback the graphics thread gets the
position and constructs graphics elements (e.g. force vector).

 *******************************************************************************/

#include <stdlib.h>
#include <iostream>
#include <cstdio>
#include <cassert>

#include <HD/hd.h>

#include "helper.h"

#include <HDU/hduError.h>
#include <HDU/hduVector.h>

#define throttle_debug(r,x) { static int k=0; if(k++%r==0) { std::cout<<x<<std::endl; } }

 /////////////////////////////////START///////////////////////////////////////
 /////////////////////////////////////////////////////////////////////////////
 /////////////////////////////////////////////////////////////////////////////

 //These are global variables to get you started. You can also define your own variables to use.

 // HIP Parameters
const double proxy_radius = 5.0;
const float proxy_color[4] = { .8, .2, .2, .8 };

// Box surface Parameters
const double wall_hip_k = 0.48;  // Surface stiffness with HIP (N/mm)
const double wall_sphere_k = 4.00;  // Surface stiffness with sphere (N/mm)
const double side_length = 160; // Length of the sides of the box (mm)
const hduVector3Dd side_lengths(side_length, side_length, side_length);
const float box_color[4] = { .2, .2, .8, .2 };

// Object (big sphere) Parameters.  Note that for remote testing purposes, object may spawn on hip to provide force / have intial velocity / force components.
const double sphere_k = 0.48;  // Surface stiffness with HIP (N/mm)
const double sphere_damping = 0.5; // Sphere damping (N-s/mm)
const double sphere_mass = 0.005; // Sphere mass (Kg)
const double sphere_radius = 10.0; // Radius of sphere (mm)
float sphere_color[4] = { 1.0, 1.0, 1.0, 0.8 };
const float sphere_color_a[4] = { 1.0, 1.0, 1.0, 0.8 };
const float sphere_color_b[4] = { 0.2, 0.8, 0.8, 0.8 };

// State parameters 
//Note that HIP tool is at 0, -65, -88).
hduVector3Dd sphere_pos(40, 20, 0); // center of the object sphere
hduVector3Dd sphere_vel(0, 0, 0);
hduVector3Dd sphere_acc(0, 0, 0); //velocity and acceleration of the big sphere

//Calculate wall interactions based on radius and position.  Return force vector.  For cube (if want different side lengths take side_lengths as arg).
hduVector3Dd Interaction_Wall(const hduVector3Dd& position, const double& radius, const double& k, const double& side_length) {
    hduVector3Dd wallForce;
	for (int i = 0; i < 3; ++i){
		if (position[i] + radius > side_length / 2) {
        wallForce[i] += k * (side_length / 2 - position[i] - radius);
		}
		if (position[i] - radius < -side_length / 2) {
			wallForce[i] += k * (-side_length / 2 - position[i] + radius);
		}
	}
    return wallForce;
}


/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
//////////////////////////////////END////////////////////////////////////////

/* Charge (positive/negative) */
int charge = 1;

static HHD ghHD = HD_INVALID_HANDLE;
static HDSchedulerHandle gSchedulerCallback = HD_INVALID_HANDLE;

/* Glut callback functions used by helper.cpp */
void displayFunction(void);
void handleIdle(void);

/* Haptic device record. */
struct DeviceDisplayState
{
    HHD m_hHD;
    hduVector3Dd position;
    hduVector3Dd force;
};

/*******************************************************************************
  Client callback used by the graphics main loop function.
  Use this callback synchronously.
  Gets data, in a thread safe manner, that is constantly being modified by the
  haptics thread.
 *******************************************************************************/
HDCallbackCode HDCALLBACK DeviceStateCallback(void* pUserData)
{
    DeviceDisplayState* pDisplayState =
        static_cast<DeviceDisplayState*>(pUserData);

    hdGetDoublev(HD_CURRENT_POSITION, pDisplayState->position);
    hdGetDoublev(HD_CURRENT_FORCE, pDisplayState->force);

    // execute this only once.
    return HD_CALLBACK_DONE;
}


/*******************************************************************************
  Graphics main loop function. Insert your graphic edits in here.
 *******************************************************************************/
void displayFunction(void)
{
    // Setup model transformations.
    glMatrixMode(GL_MODELVIEW);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glPushMatrix();
    //glTranslatef(0,0,-200); %Move the box linearly

    setupGraphicsState();
    // enable color
    glEnable(GL_COLOR_MATERIAL);
    //drawAxes(sphere_radius*3.0);


    /////////////////////////////////START///////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////

    //// No need to modify this part////////////////////////////////////////////
    // Draw a cubic box 
    drawBox(side_length, box_color);

    // Get the current position of end effector and save in the 'state' variable.
    DeviceDisplayState state;
    hdScheduleSynchronous(DeviceStateCallback, &state,
        HD_MIN_SCHEDULER_PRIORITY);
    ////////////////////////////////////////////////////////////////////////////

    // The big sphere (object) center is determined and updated from the haptic loop as a global "sphere_pos" variable that you computed
    // Make a copy of the sphere_pos to a local variable, called proxy_object, for use in the graphic loop 
    hduVector3Dd proxy_object(sphere_pos);

    // Update the "proxy_object" variable such that the big sphere does not appear to pass the walls visually.
    // when the big sphere is not touching the walls, proxy_object is the same as sphere_pos
    
	//We could combine these for loops with the ones for the hip proxy, but I am seperating them for consistency with the given comments layout.
	//Note that because we define the proxy_object position first, it is given priority over the hip proxy position.
    for (int i = 0; i < 3; ++i) {
        if (proxy_object[i] + sphere_radius > side_length / 2) {
            proxy_object[i] = side_length / 2 - sphere_radius;
        }
        else if (proxy_object[i] - sphere_radius < -side_length / 2) {
            proxy_object[i] = -side_length / 2 + sphere_radius;
        }
    }


    // Draw the big sphere using the drawSphere fucntion using the sphere center and parameters defined above
    GLUquadricObj* pQuadObj = gluNewQuadric();
    drawSphere(pQuadObj, proxy_object, sphere_color, sphere_radius);
    gluDeleteQuadric(pQuadObj);

    // Note that the current state of the HIP is saved in the variable called "state" from the method above
    // To get the position of the HIP, use state.position. This represents the center of the HIP sphere
    // For example, to find the distance between the current user position and an object
    // you can use: hduVector3Dd var = state.position - object_pos;
    hduVector3Dd proxy_pos(state.position);


    // Proxy position variable defined from state variable of the HIP position to be used for graphic display of the user's position
    // Update the "proxy_pos" variable such that the user's HIP sphere does not penetrate into other surfaces graphically
    // when the HIP sphere is not touching any objects, proxy_pos is the same as current user's position

    // Consider if HIP sphere enters the walls. Find the proxy_pos.
    for (int i = 0; i < 3; ++i) {
        if (proxy_pos[i] + proxy_radius > side_length / 2) {
            proxy_pos[i] = side_length / 2 - proxy_radius;
        }
        else if (proxy_pos[i] - proxy_radius < -side_length / 2) {
            proxy_pos[i] = -side_length / 2 + proxy_radius;
        }
    }


    // Consider if HIP sphere enters the big sphere. Find the proxy_pos. Current big sphere position is the global variable sphere_pos, but we compare to proxy_obj in
	//case we are already offsetting the dynamic sphere's location.
    hduVector3Dd rSphereHIP = proxy_object - state.position;
    //If the distance vector has less magnitude than sum of radii, then we have collision.
    const double deltaDist = rSphereHIP.magnitude() - sphere_radius - proxy_radius;
    if (deltaDist < 0) {
        //Calculate force onto dynamic sphere based on its k value.  Apply this force to both the user and the sphere.
        //The force is in the opposite direction to rSphereHIP (the vector between the centers of the two spheres).  This vector points from the proxy to the dynamic sphere.
        rSphereHIP.normalize();
        proxy_pos = proxy_object - rSphereHIP * (sphere_radius + proxy_radius);
    }


    // Draw HIP sphere using the updated "proxy_pos" variable
    pQuadObj = gluNewQuadric();
    drawSphere(pQuadObj, proxy_pos, proxy_color, proxy_radius);
    gluDeleteQuadric(pQuadObj);

    /////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////END////////////////////////////////////////

    glPopMatrix();
    glutSwapBuffers();

}

/*******************************************************************************
  Called periodically by the GLUT framework.
 *******************************************************************************/
void handleIdle(void)
{
    glutPostRedisplay();

    if (!hdWaitForCompletion(gSchedulerCallback, HD_WAIT_CHECK_STATUS))
    {
        printf("The main scheduler callback has exited\n");
        printf("Press any key to quit.\n");
        getchar();
        exit(-1);
    }
}

/******************************************************************************
  Popup menu handler
 ******************************************************************************/
void handleMenu(int ID)
{
    switch (ID)
    {
    case 0:
        exit(0);
        break;
    case 1:
        charge *= -1;
        break;
    }
}
/***********************************************************************************
Optional: Example of how to create a function that can be called in the haptic loop
************************************************************************************/
hduVector3Dd Interaction_WallHIP(hduVector3Dd position, double HIPRadius, double wallWidth)
{

    hduVector3Dd forceVector;


    // collision detection

    // forceVector = ...

    return forceVector;
}


/*******************************************************************************
  Main callback that calculates and sets the force. HAPTIC FEEDBACK LOOP
 *******************************************************************************/
HDCallbackCode HDCALLBACK DynamicObjectsCallback(void* data)
{
    hdBeginFrame(hdGetCurrentDevice());


    /////////////////////////////////START///////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////// 


    // *****Make sure to comment the steps you take clearly and properly******

    // Get the position of the device. Note that this position variable is defined locally
    // and only known within this haptic loop
    hduVector3Dd position;
    hdGetDoublev(HD_CURRENT_POSITION, position);

    // Local variables for you to use. Add more variables as needed.
    hduVector3Dd f(0, 0, 0); //force on the HIP sphere to be outputted to user
    double timeStep = 0.001; //update rate for numerical integration
	//Define some force on the sphere for testing.
	hduVector3Dd sphere_f(0, 0, 0); //net force on the big sphere

	//Track the loop iterations so that we can limit print rate.
	static int ticker = 0;
	++ticker;

    //We render haptic feedback to the HIP only depending on the position of the sphere.  If the sphere is on the left half (box is centered at 0, 0), the sphere will be pushed left.
	//For simplicity, we use the spring constant from the cube walls.
	if (sphere_pos[0] < 0 && position[0]+proxy_radius > 0){
		f[0] = -wall_hip_k*(position[0]+proxy_radius);
	}
	if (sphere_pos[0] > 0 && position[0]-proxy_radius < 0){
		f[0] = -wall_hip_k*(position[0]-proxy_radius);
	}

    // Determine the net forces on the big sphere and HIP
    // Compute sphere_f which is the resultant force acting on the big sphere and f which is force on HIP sphere to be outputted to user
    // Remember there are three possible collisions that you need to consider: HIP sphere & walls, HIP sphere & big sphere, big sphere & walls

    /*Edits*/
    //The simulation is contained within a box, surrounding (0, 0, 0) with lengths side_length.
    //We model these walls simple spring system.

	//We want to change the sphere_color once per collision.  Since a collision can last multiple ticks of the callback function, we need to have a toggle bool tracking collisions,
	//so that the color change only occurs once per collision.
	static bool inWall = false;

    //Check for wall collision.  We use a penetration method for both the hip and dynamic sphere, though in a previous version we used a perfect reflection method
	//for the dynamic sphere.  Both function approximately the same.
    f = f + Interaction_Wall(position, proxy_radius, wall_hip_k, side_length);
	sphere_f = sphere_f + Interaction_Wall(sphere_pos, sphere_radius, wall_sphere_k, side_length); 

	//If the force on the sphere from the walls is non zero, then the sphere is inside a wall.
	if (sphere_f.magnitude() > 0 && !inWall){
		SphereColorToggle();
		inWall = true;
	}
	//If the sphere is no longer in the wall, we update the inWall variable to false.
	else if (sphere_f.magnitude() <= 0){
		inWall = false;
	}



    //Calculate collision forces between the HIP and dynamic sphere.  Normally this would require the mass of both points to calculate energy balance.
    //We are given a stiffness coefficient, so it is possible we assume infinite mass for HIP.

    //Check if the HIP has collided with the dynamic sphere.
    hduVector3Dd rSphereHIP = sphere_pos - position;
    //If the distance vector has less magnitude than sum of radii, then we have collision.
    const double deltaDist = rSphereHIP.magnitude() - sphere_radius - proxy_radius;
    if (deltaDist < 0) {
        //Calculate force onto dynamic sphere based on its k value.  Apply this force to both the user and the sphere.
        //The force is in the opposite direction to rSphereHIP (the vector between the centers of the two spheres).  This vector points from the proxy to the dynamic sphere.
        rSphereHIP.normalize();
        hduVector3Dd collisionForce = rSphereHIP * deltaDist * sphere_k;

        f = f + collisionForce;
        sphere_f = sphere_f - collisionForce;
    }

	//std::cout << position[0] << ", " << position[1] << ", " << position[2] << "\n";
	// example of how you can test your big sphere dynamic by generating a fake known force on it to see its movement. 
    // Note that you still need to define the correct equation of sphere_f above this line for the actual simulation
    //sphere_f.set(0.001,0,0); //force pushing the big sphere along +x direction


    // Knowing sphere_f, compute big sphere dynamics to update its position variable, sphere_pos. This is used in the graphic display function
    // Velocity and acceleration of the sphere (sphere_vel and sphere_acc) are already defined globally
    // sphere_pos = ??;

    //Integrate the effects of sphere_f onto its motion.
    //Callback looping at 1 kHz, so dt = 0.001 s.
    const double dt = 0.001;

    //Update accel.  Account for damping to prevent infinite movement.
    sphere_acc = sphere_f / sphere_mass - sphere_damping * sphere_vel;

    //Integrate for velocity.  Is += defined for hd vector?  Don't have libraries.  
    sphere_vel = sphere_vel + sphere_acc * dt;

    //Integrate for position.
    sphere_pos = sphere_pos + sphere_vel * dt;

	/*Print some info for debugging.  Printing every step will slow the simulation, so print every 300ms.
	if (ticker == 300 && false){
		ticker = 0;
		std::cout << "-------\ndynamic sphere:\n";
		for (int i = 0; i < 3; ++i){
			std::cout << i << " - f: " << sphere_f[i] << ", a: " << sphere_acc[i] << ", v: " << sphere_vel[i] << ", p: " << sphere_pos[i] << '\n';
		}
		std::cout << "||v||: " << sphere_vel.magnitude() << "\n\n";
		std::cout << "hip sphere:\n";
		for (int i = 0; i < 3; ++i){
			std::cout << i << " - f: " << f[i] << ", p: " << position[i] << '\n';
		}
	}
	*/

    f.set(0, 0, 0); //keep f to zero to keep the force output to remote device to 0 for safety.

    // Set the output force on HIP, assuming the force output variable is f. You can change the variable.
    hdSetDoublev(HD_CURRENT_FORCE, f);


    /////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////END/////////////////////////////////////////

    hdEndFrame(hdGetCurrentDevice());

    /* Check if an error occurred while attempting to render the force */
    HDErrorInfo error;
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        if (hduIsSchedulerError(&error))
        {
            return HD_CALLBACK_DONE;
        }
    }

    return HD_CALLBACK_CONTINUE;
}

/*******************************************************************************
  Schedules the force callback.
 *******************************************************************************/
void DynamicObjectsRendering()
{
	//Get the initial position of the HIP so that we can correctly spawn the box around the HIP, as we don't know the initial 

    std::cout << "haptics callback" << std::endl;
    gSchedulerCallback = hdScheduleAsynchronous(
        DynamicObjectsCallback, 0, HD_DEFAULT_SCHEDULER_PRIORITY);

    HDErrorInfo error;
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to initialize haptic device");
        fprintf(stderr, "\nPress any key to quit.\n");
        getchar();
        exit(-1);
    }

    std::cout << "graphics callback" << std::endl;

    glutMainLoop(); // Enter GLUT main loop.
}

/******************************************************************************
  This handler gets called when the process is exiting. Ensures that HDAPI is
  properly shutdown
 ******************************************************************************/
void exitHandler()
{
    hdStopScheduler();
    hdUnschedule(gSchedulerCallback);

    if (ghHD != HD_INVALID_HANDLE)
    {
        hdDisableDevice(ghHD);
        ghHD = HD_INVALID_HANDLE;
    }
}

/******************************************************************************
  Main function.
 ******************************************************************************/
int main(int argc, char* argv[])
{
    HDErrorInfo error;

    printf("Starting application\n");

    atexit(exitHandler);

    // Initialize the device.  This needs to be called before any other
    // actions on the device are performed.
    ghHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to initialize haptic device");
        fprintf(stderr, "\nPress any key to quit.\n");
        getchar();
        exit(-1);
    }

    printf("Found device %s\n", hdGetString(HD_DEVICE_MODEL_TYPE));

    hdEnable(HD_FORCE_OUTPUT);
    hdEnable(HD_MAX_FORCE_CLAMPING);

    hdStartScheduler();
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to start scheduler");
        fprintf(stderr, "\nPress any key to quit.\n");
        getchar();
        exit(-1);
    }

    initGlut(argc, argv);

    // Get the workspace dimensions.
    HDdouble maxWorkspace[6];
    hdGetDoublev(HD_MAX_WORKSPACE_DIMENSIONS, maxWorkspace);

    // Low/left/back point of device workspace.
    hduVector3Dd LLB(maxWorkspace[0], maxWorkspace[1], maxWorkspace[2]);
    // Top/right/front point of device workspace.
    hduVector3Dd TRF(maxWorkspace[3], maxWorkspace[4], maxWorkspace[5]);
    initGraphics(LLB, TRF);

    // Application loop.
    DynamicObjectsRendering();

    printf("Done\n");
    return 0;
}

/******************************************************************************/