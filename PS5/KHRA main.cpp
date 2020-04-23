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

// Define haptic interaction planes and normal vectors
  hduVector3Dd planeOrigin_left(-side_lengths[0]/2, 0.0, 0.0);
  hduVector3Dd planeNormal_left(1.0, 0.0, 0.0);
  hduVector3Dd planeOrigin_right(side_lengths[0]/2, 0.0, 0.0);
  hduVector3Dd planeNormal_right(-1.0, 0.0, 0.0);
  hduVector3Dd planeOrigin_bottom(0.0, -side_lengths[0]/2, 0.0);
  hduVector3Dd planeNormal_bottom(0.0, 1.0, 0.0);
  hduVector3Dd planeOrigin_top(0.0, side_lengths[0]/2, 0.0);
  hduVector3Dd planeNormal_top(0.0, -1.0, 0.0);
  hduVector3Dd planeOrigin_back(0.0, 0.0, -side_lengths[0]/2);
  hduVector3Dd planeNormal_back(0.0, 0.0, 1.0);

// Define sphere and HIP radius
  
double haptic_radius= proxy_radius + sphere_radius;

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
  // Determine big sphere and HIP position w.r.t. walls
  hduVector3Dd proxy_radius_x(proxy_radius, 0.0, 0.0);
  hduVector3Dd proxy_radius_y(0.0, proxy_radius, 0.0);
  hduVector3Dd proxy_radius_z(0.0, 0.0, proxy_radius);
  
  hduVector3Dd pos_HIP_left = state.position - proxy_radius_x - planeOrigin_left;
  hduVector3Dd pos_HIP_right = state.position + proxy_radius_x - planeOrigin_right;
  hduVector3Dd pos_HIP_bottom = proxy_pos - proxy_radius_y - planeOrigin_bottom;
  hduVector3Dd pos_HIP_top = proxy_pos + proxy_radius_y - planeOrigin_top;
  hduVector3Dd pos_HIP_back = proxy_pos + proxy_radius_z - planeOrigin_back;

  double dist_HIP_left = dotProduct(pos_HIP_left,planeNormal_left);
  double dist_HIP_right = dotProduct(pos_HIP_right,planeNormal_right);
  double dist_HIP_bottom = dotProduct(pos_HIP_bottom,planeNormal_bottom);
  double dist_HIP_top = dotProduct(pos_HIP_top,planeNormal_top);
  double dist_HIP_back = dotProduct(pos_HIP_back,planeNormal_back);

  hduVector3Dd sphere_radius_x(sphere_radius, 0.0, 0.0);
  hduVector3Dd sphere_radius_y(0.0, sphere_radius, 0.0);
  hduVector3Dd sphere_radius_z(0.0, 0.0, sphere_radius);
  
  hduVector3Dd pos_sphere_left = sphere_pos - sphere_radius_x - planeOrigin_left;
  hduVector3Dd pos_sphere_right = sphere_pos + sphere_radius_x - planeOrigin_right;
  hduVector3Dd pos_sphere_bottom = sphere_pos - sphere_radius_y - planeOrigin_bottom;
  hduVector3Dd pos_sphere_top = sphere_pos + sphere_radius_y - planeOrigin_top;
  hduVector3Dd pos_sphere_back = sphere_pos + sphere_radius_z - planeOrigin_back;

  double dist_sphere_left = dotProduct(pos_sphere_left,planeNormal_left);
  double dist_sphere_right = dotProduct(pos_sphere_right,planeNormal_right);
  double dist_sphere_bottom = dotProduct(pos_sphere_bottom,planeNormal_bottom);
  double dist_sphere_top = dotProduct(pos_sphere_top,planeNormal_top);
  double dist_sphere_back = dotProduct(pos_sphere_back,planeNormal_back);

  		////////// HIP wall ////////////
  if (dist_HIP_left < 0.0)
  {
	  proxy_pos.set(-side_lengths[0]/2 + proxy_radius, state.position[1], state.position[2]);
  }
  else if (dist_HIP_right < 0.0)
  {
	  proxy_pos.set(side_lengths[0]/2 - proxy_radius, state.position[1], state.position[2]);
  }
  else if (dist_HIP_bottom < 0.0)
  {
	  proxy_pos.set(state.position[0], -side_lengths[1]/2 + proxy_radius, state.position[2]);
  }
  else if (dist_HIP_top < 0.0)
  {
	  proxy_pos.set(state.position[0], side_lengths[1]/2 - proxy_radius, state.position[2]);
  }
  else if (dist_HIP_back < 0.0)
  {
	  proxy_pos.set(state.position[0], state.position[1], -side_lengths[2]/2 + proxy_radius);
  }
  
  ////////////// Big Sphere collision with wall  ////////////////////////
  		////////// sphere wall ////////////
  if (dist_sphere_left < 0.0)
  {
	  proxy_object.set(-side_lengths[0]/2 + sphere_radius, sphere_pos[1], sphere_pos[2]);
  }
  else if (dist_sphere_right < 0.0)
  {
	  proxy_object.set(side_lengths[0]/2 - sphere_radius, sphere_pos[1], sphere_pos[2]);
  }
  else if (dist_sphere_bottom < 0.0)
  {
	  proxy_object.set(sphere_pos[0], -side_lengths[1]/2 + sphere_radius, sphere_pos[2]);
  }
  else if (dist_sphere_top < 0.0)
  {
	  proxy_object.set(sphere_pos[0], side_lengths[1]/2 - sphere_radius, sphere_pos[2]);
  }
  else if (dist_sphere_back < 0.0)
  {
	  proxy_object.set(sphere_pos[0], sphere_pos[1], - side_lengths[2]/2 + sphere_radius);
  }
  else
  {
	  proxy_object = sphere_pos;
  }

  // Consider if HIP sphere enters the big sphere. Find the proxy_pos. Current big sphere position is the global variable sphere_pos. 
  
  hduVector3Dd dist_HIP_sphere = state.position - sphere_pos;
  double r = dist_HIP_sphere.magnitude();

  hduVector3Dd r_hat = dist_HIP_sphere;
  r_hat.normalize();

  if (r  < haptic_radius)
	{
		proxy_pos = haptic_radius*r_hat;
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
  
  // Local variables for you to use. Add more variables as needed.
  hduVector3Dd f(0,0,0); //force on the HIP sphere to be outputted to user
  double timeStep = 0.001; //update rate for numerical integration
  hduVector3Dd sphere_f(0,0,0); //net force on the big sphere


  //std::cout<< position[0] << std::endl; //can uncomment this and use the command to output values on the screen for debugging
  
  // Determine big sphere and HIP position w.r.t. walls
  hduVector3Dd proxy_radius_x(proxy_radius, 0.0, 0.0);
  hduVector3Dd proxy_radius_y(0.0, proxy_radius, 0.0);
  hduVector3Dd proxy_radius_z(0.0, 0.0, proxy_radius);
 
  hduVector3Dd sphere_radius_x(sphere_radius, 0.0, 0.0);
  hduVector3Dd sphere_radius_y(0.0, sphere_radius, 0.0);
  hduVector3Dd sphere_radius_z(0.0, 0.0, sphere_radius);
  
  hduVector3Dd pos_HIP_left = position - proxy_radius_x - planeOrigin_left;
  hduVector3Dd pos_HIP_right = position + proxy_radius_x - planeOrigin_right;
  hduVector3Dd pos_HIP_bottom = position - proxy_radius_y - planeOrigin_bottom;
  hduVector3Dd pos_HIP_top = position + proxy_radius_y - planeOrigin_top;
  hduVector3Dd pos_HIP_back = position + proxy_radius_z - planeOrigin_back;

  hduVector3Dd pos_sphere_left = sphere_pos - sphere_radius_x - planeOrigin_left;
  hduVector3Dd pos_sphere_right = sphere_pos + sphere_radius_x - planeOrigin_right;
  hduVector3Dd pos_sphere_bottom = sphere_pos - sphere_radius_y - planeOrigin_bottom;
  hduVector3Dd pos_sphere_top = sphere_pos + sphere_radius_y - planeOrigin_top;
  hduVector3Dd pos_sphere_back = sphere_pos + sphere_radius_z - planeOrigin_back;

  double dist_HIP_left = dotProduct(pos_HIP_left,planeNormal_left);
  double dist_HIP_right = dotProduct(pos_HIP_right,planeNormal_right);
  double dist_HIP_bottom = dotProduct(pos_HIP_bottom,planeNormal_bottom);
  double dist_HIP_top = dotProduct(pos_HIP_top,planeNormal_top);
  double dist_HIP_back = dotProduct(pos_HIP_back,planeNormal_back);

  double dist_sphere_left = dotProduct(pos_sphere_left,planeNormal_left);
  double dist_sphere_right = dotProduct(pos_sphere_right,planeNormal_right);
  double dist_sphere_bottom = dotProduct(pos_sphere_bottom,planeNormal_bottom);
  double dist_sphere_top = dotProduct(pos_sphere_top,planeNormal_top);
  double dist_sphere_back = dotProduct(pos_sphere_back,planeNormal_back);

  // Determine the net forces on the big sphere and HIP
  double force_mag_HIP_x = 0;
  double force_mag_HIP_y = 0;
  double force_mag_HIP_z = 0;

  double force_mag_sphere_x = 0;
  double force_mag_sphere_y = 0;
  double force_mag_sphere_z = 0;

  hduVector3Dd force_HIP_x(0.0, 0.0, 0.0);
  hduVector3Dd force_HIP_y(0.0, 0.0, 0.0);
  hduVector3Dd force_HIP_z(0.0, 0.0, 0.0);

  hduVector3Dd force_sphere_x(0.0, 0.0, 0.0);
  hduVector3Dd force_sphere_y(0.0, 0.0, 0.0);
  hduVector3Dd force_sphere_z(0.0, 0.0, 0.0);
  
  hduVector3Dd f_wall(0.0, 0.0, 0.0);
  hduVector3Dd f_sphere(0.0, 0.0, 0.0);

  hduVector3Dd f_wall_sphere(0.0, 0.0, 0.0);

		////////// HIP wall (x-axis) ////////////
  if (dist_HIP_left < 0.0)
  {
	  force_mag_HIP_x = -wall_hip_k * dist_HIP_left;
	  force_HIP_x = force_mag_HIP_x * planeNormal_left;
  }
  else if (dist_HIP_right < 0.0)
  {
	  force_mag_HIP_x = -wall_hip_k * dist_HIP_right;
	  force_HIP_x = force_mag_HIP_x * planeNormal_right;
  }
  else
  {
	  force_HIP_x.set(0.0, 0.0, 0.0);
  }

		////////// HIP wall (y-axis) ////////////
  if (dist_HIP_bottom < 0.0)
  {
	  force_mag_HIP_y = -wall_hip_k * dist_HIP_bottom;
	  force_HIP_y = force_mag_HIP_y * planeNormal_bottom;
  }
  else if (dist_HIP_top < 0.0)
  {
	  force_mag_HIP_y = -wall_hip_k * dist_HIP_top;
	  force_HIP_y = force_mag_HIP_y * planeNormal_top;
  }
  else
  {
	  force_HIP_y.set(0.0, 0.0, 0.0);
  }

		////////// HIP wall (z-axis) ////////////
   if (dist_HIP_back < 0.0)
  {
	  force_mag_HIP_z = -wall_hip_k * dist_HIP_back;
	  force_HIP_z = force_mag_HIP_z * planeNormal_back;
  }
  else
  {
	  force_HIP_z.set(0.0, 0.0, 0.0);
  }
  
  f_wall = force_HIP_x + force_HIP_y + force_HIP_z;

  // Compute sphere_f which is the resultant force acting on the big sphere and f which is force on HIP sphere to be outputted to user
  // Remember there are three possible collisions that you need to consider: HIP sphere & walls, HIP sphere & big sphere, big sphere & walls
  
  		////////// Sphere wall (x-axis) ////////////
  if (dist_sphere_left < 0.0)
  {
	  force_mag_sphere_x = -wall_sphere_k * dist_sphere_left;
	  force_sphere_x = force_mag_sphere_x * planeNormal_left;
  }
  else if (dist_sphere_right < 0.0)
  {
	  force_mag_sphere_x = -wall_sphere_k * dist_sphere_right;
	  force_sphere_x = force_mag_sphere_x * planeNormal_right;
  }
  else
  {
	  force_sphere_x.set(0.0, 0.0, 0.0);
  }

		////////// Sphere wall (y-axis) ////////////
  if (dist_sphere_bottom < 0.0)
  {
	  force_mag_sphere_y = -wall_sphere_k * dist_sphere_bottom;
	  force_sphere_y = force_mag_sphere_y * planeNormal_bottom;
  }
  else if (dist_sphere_top < 0.0)
  {
	  force_mag_sphere_y = -wall_sphere_k * dist_sphere_top;
	  force_sphere_y = force_mag_sphere_y * planeNormal_top;
  }
  else
  {
	  force_sphere_y.set(0.0, 0.0, 0.0);
  }

		////////// Sphere wall (z-axis) ////////////
   if (dist_sphere_back < 0.0)
  {
	  force_mag_sphere_z = -wall_sphere_k * dist_sphere_back;
	  force_sphere_z = force_mag_sphere_z * planeNormal_back;
  }
  else
  {
	  force_sphere_z.set(0.0, 0.0, 0.0);
  }
  
  f_wall_sphere = force_sphere_x + force_sphere_y + force_sphere_z;

  // Rigid sphere forces /////////////////////////////////

  hduVector3Dd dist_HIP_sphere = position - sphere_pos;
  double r = dist_HIP_sphere.magnitude();

  hduVector3Dd r_hat = dist_HIP_sphere;
  r_hat.normalize();

  if (r  < haptic_radius)
	{
		f_sphere = sphere_k*(haptic_radius-r)*r_hat;
	}

  f = f_wall + f_sphere;

  // example of how you can test your big sphere dynamic by generating a fake known force on it to see its movement. 
  // Note that you still need to define the correct equation of sphere_f above this line for the actual simulation
  sphere_f.set(0,0,-1); //force pushing the big sphere along +x direction
 

  // Knowing sphere_f, compute big sphere dynamics to update its position variable, sphere_pos. This is used in the graphic display function
  // Velocity and acceleration of the sphere (sphere_vel and sphere_acc) are already defined globally
  // sphere_pos = ??;
  static hduVector3Dd previous_sphere_vel(0,0,0);
  static hduVector3Dd previous_sphere_pos(0,0,0);
  
  
  hduVector3Dd sphere_acc = (1/sphere_mass)*(-f_sphere-sphere_damping*previous_sphere_vel+f_wall_sphere);
  // hduVector3Dd sphere_acc = (sphere_f/sphere_mass);
  sphere_vel = sphere_acc*timeStep + previous_sphere_vel;
  sphere_pos = previous_sphere_vel*timeStep + previous_sphere_pos;
  
  previous_sphere_vel = sphere_vel;
  previous_sphere_pos = sphere_pos;
  
  f.set(0,0,0); //keep f to zero to keep the force output to remote device to 0 for safety.
  
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
