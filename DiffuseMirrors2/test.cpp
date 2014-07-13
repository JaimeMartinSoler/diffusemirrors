

#include "test.h"
#include "shapes.h"
// http://eigen.tuxfamily.org/dox/group__QuickRefPage.html
#include <Eigen/Dense>
using namespace Eigen;

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "engine.h"

// test function for testing
void test() {
	test_get_area();
}

// test get_area()
void test_get_area() {
	Point* p0 = new Point(1.0f, 1.0f, 1.0f);
	Point* p1 = new Point(3.0f, 1.0f, 1.0f);
	Point* p2 = new Point(3.0f, 2.0f, 1.0f);
	Point* p3 = new Point(1.0f, 3.0f, 1.0f);
	PointMesh* rect_0 = new PointMesh(vp_rectangle(p0, p1, p2, p3), RECTANGLE, p0, 1.0f);
	PointMesh* tri_0  = new PointMesh(vp_triangle(p0, p1, p2), TRIANGLE, p0, 1.0f);
	float area_rect_0 = (*rect_0).get_area();
	float area_tri_0  = (*tri_0).get_area();
	int erase_me = 0;
}


// test geometry term functions
void test_geometry_term() {
	Point v0(1.0f, 0.0f, 0.0f);
	Point v1(1.0f, 1.0f, 0.0f);
	// float cos_01 = cos_vectors(new Point(1.0f, 0.0f, 0.0f), new Point(1.0f, 1.0f, 0.0f));	// this creates 2 points in the memory heap permanently (till deletion in the code). BAD
	//float cos_01 = cos_vectors(&(Point(1.0f, 0.0f, 0.0f)), &(Point(1.0f, 1.0f, 1.0f)));		// this creates 2 points ONLY in the scope and they will be AUTOMATICALLY DELETED  . GOOD
	float cos_01 = get_cos_vectors(&(Point(v0)), &(Point(1.0f, 1.0f, 1.0f)));		// this creates 2 points ONLY in the scope and they will be AUTOMATICALLY DELETED  . GOOD

	Point p0(0.0f, 0.0f, 0.0f);
	Point n0(0.0f, 0.0f, -1.0f); n0.normalize();
	Point p1(0.0f, 0.0f, -1.0f);
	Point n1(0.0f, 1.0f, 1.0f); n1.normalize();
	float gt_01 = get_geometry_term(&p0, &n0, &p1, &n1);

}

/* $Revision: 1.1.6.4 $ */
/*
 *	engdemo.cpp
 *
 *	A simple program to illustrate how to call MATLAB
 *	Engine functions from a C++ program.
 *
 * Copyright 1984-2011 The MathWorks, Inc.
 * All rights reserved
 */
int test_MATLAB_engine() {
	
	const int BUFSIZE = 256;

	Engine *ep;
	mxArray *T = NULL, *result = NULL;
	char buffer[BUFSIZE+1];
	double time[10] = { 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0 };

	/*
	 * Call engOpen with a NULL string. This starts a MATLAB process 
     * on the current host using the command "matlab".
	 */
	if (!(ep = engOpen(""))) {
		fprintf(stderr, "\nCan't start MATLAB engine\n");
		return EXIT_FAILURE;
	}

	/*
	 * PART I
	 *
	 * For the first half of this demonstration, we will send data
	 * to MATLAB, analyze the data, and plot the result.
	 */

	/* 
	 * Create a variable for our data
	 */
	T = mxCreateDoubleMatrix(1, 10, mxREAL);
	memcpy((void *)mxGetPr(T), (void *)time, sizeof(time));
	/*
	 * Place the variable T into the MATLAB workspace
	 */
	engPutVariable(ep, "T", T);

	/*
	 * Evaluate a function of time, distance = (1/2)g.*t.^2
	 * (g is the acceleration due to gravity)
	 */
	engEvalString(ep, "D = .5.*(-9.8).*T.^2;");

	/*
	 * Plot the result
	 */
	engEvalString(ep, "plot(T,D);");
	engEvalString(ep, "title('Position vs. Time for a falling object');");
	engEvalString(ep, "xlabel('Time (seconds)');");
	engEvalString(ep, "ylabel('Position (meters)');");

	/*
	 * use fgetc() to make sure that we pause long enough to be
	 * able to see the plot
	 */
	printf("Hit return to continue\n\n");
	fgetc(stdin);
	/*
	 * We're done for Part I! Free memory, close MATLAB figure.
	 */
	printf("Done for Part I.\n");
	mxDestroyArray(T);
	engEvalString(ep, "close;");
	
	return EXIT_SUCCESS;

	/*
	 * PART II
	 *
	 * For the second half of this demonstration, we will request
	 * a MATLAB string, which should define a variable X.  MATLAB
	 * will evaluate the string and create the variable.  We
	 * will then recover the variable, and determine its type.
	 */
	  
	/*
	 * Use engOutputBuffer to capture MATLAB output, so we can
	 * echo it back.  Ensure first that the buffer is always NULL
	 * terminated.
	 */

	buffer[BUFSIZE] = '\0';
	engOutputBuffer(ep, buffer, BUFSIZE);
	while (result == NULL) {
	    char str[BUFSIZE+1];
	    /*
	     * Get a string input from the user
	     */
	    printf("Enter a MATLAB command to evaluate.  This command should\n");
	    printf("create a variable X.  This program will then determine\n");
	    printf("what kind of variable you created.\n");
	    printf("For example: X = 1:5\n");
	    printf(">> ");

	    fgets(str, BUFSIZE, stdin);
	  
	    /*
	     * Evaluate input with engEvalString
	     */
	    engEvalString(ep, str);
	    
	    /*
	     * Echo the output from the command.  
	     */
	    printf("%s", buffer);
	    
	    /*
	     * Get result of computation
	     */
	    printf("\nRetrieving X...\n");
	    if ((result = engGetVariable(ep,"X")) == NULL)
	      printf("Oops! You didn't create a variable X.\n\n");
	    else {
		printf("X is class %s\t\n", mxGetClassName(result));
	    }
	}

	/*
	 * We're done! Free memory, close MATLAB engine and exit.
	 */
	printf("Done!\n");
	mxDestroyArray(result);
	engClose(ep);
	
	return EXIT_SUCCESS;
}

