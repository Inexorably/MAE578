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
const double side_length = 130; // Length of the sides of the box (mm)
const hduVector3Dd side_lengths(side_length,side_length,side_length); 
const float box_color[4] = { .2, .2, .8, .2 };

// Object (big sphere) Parameters
const double sphere_k = 0.48;  // Surface stiffness with HIP (N/mm)
const double sphere_damping = 0.002; // Sphere damping (N-s/mm)
const double sphere_mass = 0.005; // Sphere mass (Kg)
const double sphere_radius = 10.0; // Radius of sphere (mm)
const float sphere_color[4] = {.2, .8, .8, .8};

// State parameters 
hduVector3Dd sphere_pos(0,0,0); // center of the object sphere
hduVector3Dd sphere_vel(0,0,0), sphere_acc(0,0,0); //velocity and acceleration of the big sphere


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
HDCallbackCode HDCALLBACK DeviceStateCallback(void *pUserData)
{
  DeviceDisplayState *pDisplayState = 
    static_cast<DeviceDisplayState *>(pUserData);

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
 
  // Consider if HIP sphere enters the walls. Find the proxy_pos

  
  // Consider if HIP sphere enters the big sphere. Find the proxy_pos. Current big sphere position is the global variable sphere_pos. 
  

  
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
HDCallbackCode HDCALLBACK DynamicObjectsCallback(void *data)
{
  hdBeginFrame(hdGetCurrentDevice());

  
  /////////////////////////////////START///////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////// 

  // *****Make sure to comment the steps you take clearly and properly******

  // Get the position of the device. Note that this position variable is defined locally
  // and only known within this haptic loop
  hduVector3Dd position;
  hdGetDoublev(HD_CURRENT_POSITION, position);
  hduVector3Dd velocity;
  hdGetDoublev (HD_CURRENT_VELOCITY, velocity);
  hduVector3Dd acceleration;
  static double dt = .001;
  static double flag=1;
  
  // Local variables for you to use. Add more variables as needed.
  hduVector3Dd f(0,0,0); //force on the HIP sphere to be outputted to user
  double timeStep = 0.001; //update rate for numerical integration
  hduVector3Dd sphere_f(0,0,0); //net force on the big sphere
  hduVector3Dd xnormal(-1,0,0);
	hduVector3Dd ynormal(0,1,0);
	hduVector3Dd znormal(0,0,1);
	hduVector3Dd x1normal(1,0,0);
	hduVector3Dd y1normal(0,-1,0);
	hduVector3Dd z1normal(0,0,0);
	hduVector3Dd sphere_right(0,0,0);
	hduVector3Dd sphere_left(0,0,0);
	hduVector3Dd sphere_top(0,0,0);
	hduVector3Dd sphere_bottom(0,0,0);
	hduVector3Dd sphere_back(0,0,0);
	hduVector3Dd f_wall_sphere(0,0,0);
	hduVector3Dd xf(0,0,0);
	hduVector3Dd yf(0,0,0);
	hduVector3Dd zf(0,0,0);
	hduVector3Dd x1f(0,0,0);
	hduVector3Dd y1f(0,0,0);
	hduVector3Dd z1f(0,0,0);
	double  length = 65;
	double  k=.5;


// Right Wall
hduVector3Dd xpoint(length,length,length);
hduVector3Dd xdistance=position-xpoint;
HDdouble xcurrent=dotProduct(xnormal,xdistance);

if ( xcurrent > proxy_radius )
{
	xf.set(0,0,0);
}
else
{
	xf=-k*xnormal*(xcurrent-proxy_radius);
}

hduVector3Dd sph_xdistance=sphere_pos-xpoint;
HDdouble sph_xcurrent=dotProduct(xnormal,sph_xdistance);

if ( sph_xcurrent > sphere_radius )
{
	sphere_right.set(0,0,0);
}
else
{
	sphere_right=-k*xnormal*(sph_xcurrent-sphere_radius);
}

hduVector3Dd x1point(-length,length,length);
hduVector3Dd x1distance=position-x1point;
HDdouble x1current=dotProduct(x1normal,x1distance);

if ( x1current > proxy_radius )
{
	x1f.set(0,0,0);
}
else
{
	x1f=-k*x1normal*(x1current-proxy_radius);
}

hduVector3Dd sph_x1distance=sphere_pos-x1point;
HDdouble sph_x1current=dotProduct(x1normal,sph_x1distance);

if ( sph_x1current > sphere_radius )
{
	sphere_left.set(0,0,0);
}
else
{
	sphere_left=-k*x1normal*(sph_x1current-sphere_radius);
}


// Back Wall
znormal.set(0,0,1);
hduVector3Dd zpoint(length,length,-length);
hduVector3Dd zdistance=position-zpoint;
HDdouble zcurrent=dotProduct(znormal,zdistance);


if ( zcurrent > proxy_radius )
{
	zf.set(0,0,0);
}
else
{
	zf=-k*znormal*(zcurrent-proxy_radius);
}

hduVector3Dd sph_zdistance=sphere_pos-zpoint;
HDdouble sph_zcurrent=dotProduct(znormal,sph_zdistance);

if ( sph_zcurrent > sphere_radius )
{
	sphere_back.set(0,0,0);
}
else
{
	sphere_back=-k*znormal*(sph_zcurrent-sphere_radius);
}


// Bottom Wall
hduVector3Dd ypoint(length,-length,length);
hduVector3Dd ydistance=position-ypoint;
HDdouble ycurrent=dotProduct(ynormal,ydistance);


if ( ycurrent > proxy_radius )
{
	yf.set(0,0,0);
}
else
{
	yf=-k*ynormal*(ycurrent-proxy_radius);
}

hduVector3Dd sph_ydistance=sphere_pos-ypoint;
HDdouble sph_ycurrent=dotProduct(ynormal,sph_ydistance);

if ( sph_ycurrent > sphere_radius )
{
	sphere_bottom.set(0,0,0);
}
else
{
	sphere_bottom=-k*ynormal*(sph_ycurrent-sphere_radius);
}

// Top Wall
hduVector3Dd y1point(length,length,length);
hduVector3Dd y1distance=position-y1point;
HDdouble y1current=dotProduct(y1normal,y1distance);


if ( y1current > proxy_radius )
{
	y1f.set(0,0,0);
}
else
{
	y1f=-k*y1normal*(y1current-proxy_radius);
}

hduVector3Dd sph_y1distance=sphere_pos-y1point;
HDdouble sph_y1current=dotProduct(y1normal,sph_y1distance);

if ( sph_y1current > sphere_radius )
{
	sphere_top.set(0,0,0);
}
else
{
	sphere_top=-k*y1normal*(sph_y1current-sphere_radius);
}

// Big Sphere
hduVector3Dd fsphere(0,0,0);
//hduVector3Dd center(0,0,0); // center of sphere
hduVector3Dd distance=position-sphere_pos; // vector between position and center
HDdouble r=distance.magnitude(); // distance value
hduVector3Dd rhat=(1/r)*(distance); // normalize distance vector
HDdouble R = sphere_radius; // radius of sphere
	
if ( r < R ) // if the user position is inside the sphere
{
	fsphere=.5*(R-r)*rhat; // force feedback when inside the sphere
}
else
{
	fsphere.set(0,0,0); // no force when user position is outside the sphere
}



f=zf+xf+x1f+yf+y1f+fsphere;
f_wall_sphere=sphere_left+sphere_right+sphere_top+sphere_bottom+sphere_back;


  //std::cout<< position[0] << std::endl; //can uncomment this and use the command to output values on the screen for debugging
 hduVector3Dd ObjF(0,0,0); //Object force
 hduVector3Dd damping_force;
 sphere_f.set(1,0,0);

 damping_force=sphere_damping*sphere_vel;
	
	ObjF = -fsphere + f_wall_sphere + damping_force;
	//ObjF = sphere_f-damping_force;
    sphere_acc=ObjF/sphere_mass;
    sphere_vel+=sphere_acc*dt;
	sphere_pos+=sphere_vel*dt; 
	
 
  // Determine the net forces on the big sphere and HIP

  // Compute sphere_f which is the resultant force acting on the big sphere and f which is force on HIP sphere to be outputted to user
  // Remember there are three possible collisions that you need to consider: HIP sphere & walls, HIP sphere & big sphere, big sphere & walls
  

  // example of how you can test your big sphere dynamic by generating a fake known force on it to see its movement. 
  // Note that you still need to define the correct equation of sphere_f above this line for the actual simulation
  //sphere_f.set(1,0,0); //force pushing the big sphere along +x direction
 

  // Knowing sphere_f, compute big sphere dynamics to update its position variable, sphere_pos. This is used in the graphic display function
  // Velocity and acceleration of the sphere (sphere_vel and sphere_acc) are already defined globally
  // sphere_pos = ??;

  
  
  f.set(0,0,0); //keep f to zero to keep the force output to remote device to 0 for safety.
  
  // Set the output force on HIP, assuming the force output variable is f. You can change the variable.
  hdSetDoublev(HD_CURRENT_FORCE, f);


  //// Questions to ask
  // What's the penetration distance
  // How to integrate time step in the program
  // 
  
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

  printf("Found device %s\n",hdGetString(HD_DEVICE_MODEL_TYPE));

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
