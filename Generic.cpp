/*****************************************************************************

Copyright (c) 2008 SensAble Technologies, Inc. All rights reserved.

OpenHaptics(TM) toolkit. The material embodied in this software and use of
this software is subject to the terms and conditions of the clickthrough
Development License Agreement.

For questions, comments or bug reports, go to forums at: 
    http://dsc.sensable.com

Module Name:

  FrictionlessPlane.cpp

Description: 

  This example demonstrates how to haptically render contact with an infinite
  frictionless plane.  The plane allows popthrough such that if the user 
  applies enough force against the plane, the plane will reverse its sidedness
  and allow the user to interact with it from the opposite side.

*******************************************************************************/

#include <stdio.h>
#include <conio.h>
#include <iostream>

/* Sensable's includes */
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>

HDSchedulerHandle gCallbackHandle = 0;

void mainLoop();
HDCallbackCode HDCALLBACK FrictionlessPlaneCallback(void *pUserData);


/******************************************************************************
 Main : Program entry point
******************************************************************************/
int main(int argc, char* argv[])
{  
    HDErrorInfo error;

    // Initialize the default haptic device.
    HHD hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to initialize haptic device");
        fprintf(stderr, "\nPress any key to quit.\n");
        _getch();
        return -1;
    }

	// Schedule the frictionless plane callback, which will then run at 
    // servoloop rates and command forces if the user penetrates the plane.
    gCallbackHandle = hdScheduleAsynchronous(
        FrictionlessPlaneCallback, 0, HD_MAX_SCHEDULER_PRIORITY);

    printf("Move the stylus around. Can you feel the wall(s)?\n");
	printf("You're an official haptician now !!\n");
	printf("-----------------------------------\n");
	printf("Plane example.\n");
    printf("Press Q key to quit.\n\n");

    /* to enable the motors call:  hdEnable(HD_FORCE_OUTPUT); */
	hdEnable(HD_FORCE_OUTPUT);

    /* Start the haptic rendering loop. */
    hdStartScheduler();
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to start scheduler");
        fprintf(stderr, "\nPress any key to quit.\n");
        _getch();
        return -1;
    }

    /* Start the main application loop. */
    mainLoop();

    /* Cleanup by stopping the haptics loop, unscheduling the asynchronous
       callback, disabling the device. */
    hdStopScheduler();
    hdUnschedule(gCallbackHandle);
    hdDisableDevice(hHD);

    return 0;
}


/******************************************************************************
 Main loop.  
 Detects and interprets keypresses. 
******************************************************************************/
void mainLoop()
{
    int keypress;

    while (1)
    {       
        /* Check for keyboard input. */
        if (_kbhit())
        {
            keypress = _getch();
            keypress = toupper(keypress);
            
            if (keypress == 'Q')
            {
                return;
            }
        }

        /* Check if the main scheduler callback has exited. */
        if (!hdWaitForCompletion(gCallbackHandle, HD_WAIT_CHECK_STATUS))
        {
            fprintf(stderr, "\nThe main scheduler callback has exited\n");
            fprintf(stderr, "\nPress any key to quit.\n");
            _getch();
            return;
        }
    }
}


/******************************************************************************
 * Scheduler callback for reading the robot position.  Called every 1ms 
 *****************************************************************************/
HDCallbackCode HDCALLBACK FrictionlessPlaneCallback(void *pUserData)
{
    hdBeginFrame(hdGetCurrentDevice());

	// Get the position of the device.
    hduVector3Dd position;
    hdGetDoublev(HD_CURRENT_POSITION, position);

	//*** START EDITING HERE ***//////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////

    //We use the same stiffness k for all examples below.  In the lab, we will have different cpps for each example, so we will have to redefine it in each.
    //Note that k will be wrt the internal distance units / force units.
    HDdouble k = 10000;

    //////////////////////////////GENERIC PLANE/////////////////////////////////////////////////
    /* Implement the algorithm covered in class
    to create a frictionless oriented 3D plane with an offset, as defined by a point on the plane and the normal to the plane.
    Indicate the orientation of the plane you chose clearly in your comment*/

    //Note that in the future, this could be better done by having a plane struct with normal and point vector members.
    hduVector3Dd planeNormal(0, 1, 0);
    hduVector3Dd planePoint(0, 1, 2);

    //Make sure that planeNormal is a unit vector.
    planeNormal.normalize();

    //r is the vector from the point on the plane planePoint to the user position.
    hduVector3Dd r = position - planePoint;

    //d is the impression of r onto planeNormal: if d is negative, the user is on or in the wall.  If d is positive, the user is outside of the wall.
    HDdouble d = dotProduct(r, planeNormal);

    //We initalize our force output var.
    hduVector3Dd f(0, 0, 0);

    //If d is negative, the user is on or in the wall.  If d is positive, the user is outside of the wall.
    if (d <= 0) {
        f = k * d * planeNormal;
    }

    // command the desired force "f". You must determine the value of "f" before using this line.
	hdSetDoublev(HD_CURRENT_FORCE, f);

    //Displaying force prob important.  Use  [ ].

    ////////////////////////////////////////////////////////////5 Sided box.////////////////////////
    
    //Recall that we put our point into 'position'.
    //By far the simplest way to do this is with chained if statements.
    //We initalize our force output var.
    hduVector3Dd f(0, 0, 0);

    //I don't know what the scale of internal distance units, so we will arbitrarily decide to make a 10x10x10 in this.
    //May have to flip a sign in case the box is facing the wrong way (opening should face user).
    //Again, can improve 'elegance' by encapsulating in classes.  Check with professor if wants this.
    //z is the side facing use that we likely want open (positive z).
    //These ints set the limits of the box in the x, y, z axes.
    int xMin = -5;
    int xMax = 5;
    int yMin = -5;
    int yMax = 5;
    int zMin = -5;

    //Could also hide this in function and loop call but likely meaningless complication.
    //x
    if (position[0] > xMax) {
        f[0] += k * (xMax - position[0]);
    }
    //Use else if as HIP is a point.
    else if (position[0] < xMin) {
        f[0] += k * (xMin - position[0]);
    }
    //y
    if (position[1] > yMax) {
        f[1] += k * (yMax - position[0]);
    }
    //Use else if as HIP is a point.
    else if (position[1] < yMin) {
        f[1] += k * (yMin - position[0]);
    }
    //z -- note that box is open facing user.
    if (position[0] < zMin) {
        f[2] += k * (zMin - position[0]);
    }
    
    //Render the force.
    hdSetDoublev(HD_CURRENT_FORCE, f);

    ////////////////////////////////////////////////////Render a 3d rigid sphere//////////////////////////////
    //Define the center in x y z of the sphere.
    //Define the sphere radius.  Is there an advanrage to using the HDdouble over standard c++ types?
    hduVector3Dd sphereCenter(10, 10, 10);
    HDdouble sphereRadius = 5;

    hduVector3Dd f(0, 0, 0);

    //Division and square root etc are generally expensive operations.  Magnitudes are always positive.  
    //Thus, we can speed up processing by comparing the sphereRadius^2 to the distance between hip and sphereCenter squared.

    //The distance between position and the sphereCenter.  Note that pow might not be defined for HDdouble type...
    HDdouble distance = 0;
    //Loop through the axises as this is an opportunity to condense without it being too obfuscated.
    //Note: These loops are technically less efficient with n+1 additional operations (initialize i and increment).
    //Can also just use .magnitude() but is more than 4x expensive.
    for (int i = 0; i < 2; ++i) {
        distanceSquared += pow(position[i] - sphereCenter[i]), 2);
    }

    //If distance is less than sphereRadius, we are inside the sphere.
    //We compared the squared values as this is faster.
    if (distanceSquared < pow(sphereRadius, 2)) {
        //We find the unit vector from the center of the sphere to the HIP position.
        hduVector3Dd rHat;
        rHat = position - sphereCenter;
        //Normalize the vector.
        rHat.normalize();

        //Set f.  May have to static cast to double for distanceSquared.  Note that this is still optimal because we
        //don't need to do the sqrt operation in non collision cases.
        f = k * (sphereRadius - sqrt(distanceSquared)) * rHat;
    }

    hdSetDoublev(HD_CURRENT_FORCE, f);

    //////////////////////////////////////////////////////////Gravitational Pull//////////////////////////////////
    //Define some arbitrary gravitationalPoint.
    hduVector3Dd gravitationalPoint(5, 5, 5);

    //d is the vector from the gravitational point to the position HIP.
    hduVector3Dd d = position - gravitationalPoint;

    //Then just implement as per slide 27...  R is arbitraily defined and F(r) should be continuous.  Find k2 algorithmically based on r.
    //Note: can use .magnitude().
    
    



	//////////////////////////////////////////////////////////////////////////////////
	//*** STOP EDITING HERE ***//////////////////////////////////////////////////////

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
