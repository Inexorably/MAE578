/*****************************************************************************

Copyright (c) 2004 SensAble Technologies, Inc. All rights reserved.

OpenHaptics(TM) toolkit. The material embodied in this software and use of
this software is subject to the terms and conditions of the clickthrough
Development License Agreement.

For questions, comments or bug reports, go to forums at: 
    http://dsc.sensable.com

Module: 

  helper.cpp

Description:
        
  Utilities that set the graphics state.

*******************************************************************************/

// define PI to use acos function
#define PI 3.1415926

#include <stdio.h>
#include <stdlib.h>

#if defined(WIN32) || defined(linux)
# include <GL/glut.h>
#elif defined(__APPLE__)
# include <GLUT/glut.h>
#endif

#include <HDU/hduMatrix.h>

extern void displayFunction(void);
extern void handleIdle(void);

/******************************************************************************
 Initializes GLUT.
******************************************************************************/
void initGlut(int argc, char* argv[])
{
    // Initialize GLUT.
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(500, 500);
    glutCreateWindow("GLUT/Dynamic Objects");

    // Setup GLUT callbacks.
    glutDisplayFunc(displayFunction); 
    glutIdleFunc(handleIdle);
}

/******************************************************************************    
 Uses the haptic device coordinate space as model space for graphics.
 Defines orthographic projection to fit it.
 LLB: Low, Left, Back point of device workspace.
 TRF: Top, Right, Front point of device workspace. 
******************************************************************************/
void initGraphics(const hduVector3Dd &LLB, const hduVector3Dd &TRF)
{
    // Setup perspective projection.
    glMatrixMode(GL_PROJECTION); 
    glLoadIdentity();

    HDdouble centerScreen[3];
    centerScreen[0] = (TRF[0] + LLB[0])/2.0;
    centerScreen[1] = (TRF[1] + LLB[1])/2.0;
    centerScreen[2] = (TRF[2] + LLB[2])/2.0;

    HDdouble screenDims[3];
    screenDims[0] = TRF[0] - LLB[0];
    screenDims[1] = TRF[1] - LLB[1];
    screenDims[2] = TRF[2] - LLB[2];

    HDdouble maxDimXY = screenDims[0] > screenDims[1] ? 
        screenDims[0] : screenDims[1];
    HDdouble maxDim = maxDimXY > screenDims[2] ? 
        maxDimXY : screenDims[2];
    maxDim /= 2.0;

	// we use prospective projection
	gluPerspective(40, 1, 1, 1000);
    
    glShadeModel(GL_SMOOTH);

    // Setup model transformations.
    glMatrixMode(GL_MODELVIEW); 
    glLoadIdentity();

	// move camera back to see scene
	gluLookAt(0.0,  0.0,  300.0,
	  		  0.0,  0.0, -1.0,
			  0.0f, 1.0f, 0.0f);

	glDisable(GL_DEPTH_TEST);
}


/******************************************************************************    
 Sets up graphics pipeline, lights etc.
******************************************************************************/
void setupGraphicsState()
{
    glShadeModel(GL_SMOOTH);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_NORMALIZE);
    glEnable(GL_LIGHT_MODEL_TWO_SIDE);
    glShadeModel(GL_SMOOTH);
    
    GLfloat lightZeroPosition[] = { 10.0, 4.0, 100.0, 0.0 };
    GLfloat lightZeroColor[] = { 0.6, 0.6, 0.6, 1.0 }; // green-tinted.
    GLfloat lightOnePosition[] = { -1.0, -2.0, -100.0, 0.0 };
    GLfloat lightOneColor[] = { 0.6, 0.6, 0.6, 1.0 }; // red-tinted.
    
    GLfloat light_ambient[] = { 0.8, 0.8, 0.8, 1.0 }; // White diffuse light.
    GLfloat light_diffuse[] = { 0.0, 0.0, 0.0, 1.0 }; // White diffuse light.
    GLfloat light_position[] = { 0.0, 0.0, 100.0, 1.0 }; // Infinite light loc.
    
    glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, 1);
    glLightfv(GL_LIGHT0, GL_POSITION, lightZeroPosition);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightZeroColor);
    glLightfv(GL_LIGHT1, GL_POSITION, lightOnePosition);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, lightOneColor);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);
}

/******************************************************************************
 Draws the cartesian axes.
******************************************************************************/
void drawAxes(double axisLength)
{
    glDisable(GL_LIGHTING);
    glEnable(GL_COLOR_MATERIAL);
    glLineWidth(2.0);
    
    glBegin(GL_LINES);
    for (int i = 0; i < 3; i++) 
    {
        float color[3] = { 0, 0, 0 };
        color[i] = 1.0;
        glColor3fv(color);
        
        float vertex[3] = {0, 0, 0};
        vertex[i] = axisLength;
        glVertex3fv(vertex);
        glVertex3f(0, 0, 0);
    } 
    glEnd();
}

/******************************************************************************
 Draws a sphere to represent an electric charge.
******************************************************************************/
void drawSphere(GLUquadricObj* pQuadObj, 
                const hduVector3Dd &position,
                const float color[4],
                double sphereRadius)
{
    glMatrixMode(GL_MODELVIEW); 
    glPushMatrix();
    glEnable(GL_LIGHTING);
    glColor4fv(color);
    glTranslatef(position[0], position[1], position[2]);
    gluSphere(pQuadObj, sphereRadius, 20, 20); 
    glPopMatrix();
}

/******************************************************************************
 Draws the force vector.
******************************************************************************/
void drawForceVector(GLUquadricObj* pQuadObj,
                     const hduVector3Dd &position,
                     const hduVector3Dd &forceVector,
                     double arrowThickness)
{
    glDisable(GL_LIGHTING);
    
    glPushMatrix();

    glTranslatef(position[0], position[1], position[2]);

    // Change the force magnitude/direction by rotating the force vector.
    // Calculate the rotation angle.
    hduVector3Dd unitForceVectorAxis = normalize(forceVector);
    hduVector3Dd zAxis( 0.0, 0.0, 1.0 );
    hduVector3Dd toolRotAxis = zAxis.crossProduct(unitForceVectorAxis);
        
    double toolRotAngle = acos(unitForceVectorAxis[2]);
    hduMatrix rotMatrix = hduMatrix::createRotation(toolRotAxis, 
                                                    toolRotAngle);

    double rotVals[4][4];
    rotMatrix.get(rotVals);
    glMultMatrixd((double*) rotVals);

    // The force arrow: composed of a cylinder and a cone.
    glColor3f( 0.2, 0.7, 0.2 );
    
    double strength = forceVector.magnitude();
    
    // Draw arrow shaft.
    gluCylinder(pQuadObj,arrowThickness, arrowThickness, strength, 16, 2); 
    glTranslatef(0, 0, strength);
    glColor3f(0.2, 0.8, 0.3);
    
    // Draw arrow head.
    gluCylinder(pQuadObj, arrowThickness*2, 0.0, strength*.15, 16, 2); 
    
    glPopMatrix();
}

/******************************************************************************
 Draws a wall
******************************************************************************/
void drawWall(const hduVector3Dd &centerPosition,
				const hduVector3Dd &normal,
				double wallLength,
                const float color[4])
{
    glMatrixMode(GL_MODELVIEW); 
    glPushMatrix();
    glEnable(GL_LIGHTING);

	// translate wall
    glTranslatef(centerPosition[0], centerPosition[1], centerPosition[2]);

	// rotate the wall so that normal points in the right direction
	float n1x = 0; // original normal points in the z direction
	float n1y = 0;
	float n1z = 1;
	float n2x = normal[0]; // new normal is passed in 
	float n2y = normal[1];
	float n2z = normal[2];
	float x = n1y*n2z - n2y*n1z; // the vector about which we rotate the wall is the cross product of n1 and n2
	float y = n1z*n2x - n2z*n1x;
	float z = n1x*n2y - n2z*n1y;
	float angle = acos(n1x*n2x + n1y*n2y + n1z*n2z)*180.0/PI; // the angle is obtained from the dot product in degrees

	// only rotate if the cross product is not zero
	if (!((x == 0) && (y == 0) && (z == 0))) glRotatef(angle, x, y, z);

	glBegin( GL_QUADS );
	    glColor4fv(color);
		glVertex3f( wallLength/2.0, -wallLength/2.0, 0);
		glVertex3f( wallLength/2.0,  wallLength/2.0, 0);
		glVertex3f(-wallLength/2.0,  wallLength/2.0, 0);
		glVertex3f(-wallLength/2.0, -wallLength/2.0, 0);
	glEnd();
    glPopMatrix();
}


/******************************************************************************
 Draws a box
******************************************************************************/
void drawBox(double wallLength, const float color[4])
{
	// right wall
	const hduVector3Dd wallPosition1(wallLength/2, 0, 0);
	const hduVector3Dd normal1(1, 0, 0);
	drawWall(wallPosition1, normal1, wallLength, color);

	// left wall
	const hduVector3Dd wallPosition2(-wallLength/2, 0, 0);
	const hduVector3Dd normal2(1, 0, 0);
	drawWall(wallPosition2, normal2, wallLength, color);

	// top wall
	const hduVector3Dd wallPosition3(0, wallLength/2, 0);
	const hduVector3Dd normal3(0, 1, 0);
	drawWall(wallPosition3, normal3, wallLength, color);

	// bottom wall
	const hduVector3Dd wallPosition4(0, -wallLength/2, 0);
	const hduVector3Dd normal4(0, 1, 0);
	drawWall(wallPosition4, normal4, wallLength, color);

	// back wall
	const hduVector3Dd wallPosition5(0, 0, -wallLength/2);
	const hduVector3Dd normal5(0, 0, 1);
	drawWall(wallPosition5, normal5, wallLength, color);
}

/******************************************************************************/
