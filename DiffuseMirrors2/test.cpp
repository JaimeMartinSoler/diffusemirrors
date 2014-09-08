

#include "test.h"
#include "global.h"
#include "main.h"
#include "render.h"
#include "scene.h"
#include "data_sim.h"
#include "data.h"
#include "capturetool2.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "engine.h"

// test function for testing
void test() {

	test_create_raw_from_raw_takes();

	std::cout << "\n\nTest done...\n\n";
}


// test_gradient_descent()
void test_gradient_descent() {
	// TO-DO
}

// test_create_raw_from_raw_takes()
void test_create_raw_from_raw_takes() {
	
	char dir_name[1024] = "C:\\Users\\Natalia\\Documents\\Visual Studio 2013\\Projects\\DiffuseMirrors2\\CalibrationMatrix\\test_03";
	char file_name[1024] = "PMD";
	//char dir_name[1024] = "C:\\Users\\Natalia\\Documents\\Visual Studio 2013\\Projects\\DiffuseMirrors2\\DiffuseMirrors2\\CalibrationMatrix\\test_03";
	//char file_name[1024] = "PMD";
	Info info(dir_name, file_name);

	RawData raw_data_take_0 (info, 0);
	RawData raw_data_take_1 (info, 1);
	RawData raw_data_take_2 (info, 2);
	RawData raw_data		(info);
	
	Frame frame_take_0	(raw_data_take_0, 0, 0, 0, 0);
	Frame frame_take_1	(raw_data_take_1, 0, 0, 0, 0);
	Frame frame_take_2	(raw_data_take_2, 0, 0, 0, 0);
	Frame frame			(raw_data, 0, 0, 0, 0);
	
	frame_take_0.plot();
	frame_take_1.plot();
	frame_take_2.plot();
	frame.plot();
}


// test_calibration_matrix()
void test_calibration_matrix() {

	char dir_name[1024] = "C:\\Users\\Natalia\\Documents\\Visual Studio 2013\\Projects\\DiffuseMirrors2\\CalibrationMatrix\\test_01";
	char file_name[1024] = "PMD_Calibration_Matrix";
	Info info = Info(dir_name, file_name);

	CalibrationMatrix cm(info);
	
	std::cout << "\ndata_size        = " << cm.data_size;
	std::cout << "\nerror_code       = " << cm.error_code;
	std::cout << "\npath_dist_0_at(0,0) = " << cm.path_dist_0_at(0,0);
	std::cout << "\npath_dist_0(cen) = " << cm.path_dist_0_at(info.rows/2,info.cols/2);
	std::cout << "\npath_dist_0(cor) = " << cm.path_dist_0_at(info.rows-1,info.cols-1);
	
	int di_floor = 8;
	float path_dist = info.distV[di_floor] + 0.6f * (info.distV[di_floor+1] - info.distV[di_floor]);
	std::cout << "\npath_dist = " << path_dist;
	std::cout << "\nc(fi_max, " << path_dist << ", cen) = " << cm.c_coef(info.freqV.size()-1, info.rows/2, info.cols/2, path_dist);

}

// Reads data from a .dat and info.txt file setting the DATAPMD_READ variable
int test_data_main() {

	char dir_name[1024] = "f:\\tmp\\pmdtest2";
	char file_name[1024] = "PMD_test_meas";

	// Create instance of INFO
	INFO = Info(dir_name, file_name);
	// Create instance and store in DATAPMD_READ
	DATAPMD_READ = RawData(INFO);
	// We have to check if there were errors while creating DATAPMD_READ
	if (DATAPMD_READ.error_code)
		return 1;	// error

	// Tests
	// DATAPMD_READ
	Frame frame_read (DATAPMD_READ, 0, 0, 0, 0);
	frame_read.plot();
	// DATAPMD_CAPTURE
	Frame frame_captured (DATAPMD_CAPTURE, 0, 0, 0, 0);
	frame_captured.plot();
	// FRAME_00_CAPTURE, FRAME_90_CAPTURE
	FRAME_00_CAPTURE.plot();
	FRAME_90_CAPTURE.plot();

	return 0;	// no errors
}


// test_capturetool2_main(...)
int test_capturetool2_main() {
	
	// DiffuseMirrors2.exe "80 90 100" "0 1 2 3" "1920" f:\tmp\pmdtest2 PMD_test_meas COM6 1
	//int error_in_PMD_charArray_to_file = PMD_charArray_to_file (argc, argv);
	//return error_in_PMD_charArray_to_file;

	int error = 0;

	// DiffuseMirrors2.exe
	//std::vector<float> frequencies(1);	frequencies[0] = 100.0f;
	std::vector<float> frequencies(3);	frequencies[0] = 100.0f;	frequencies[1] = 80.0f;	frequencies[2] = 100.0f;
	//std::vector<float> frequencies(1000, 100.0f);
	
	//std::vector<float> delays(1);	delays[0] = 0.0f;
	std::vector<float> delays(4);	delays[0] = 0.0f;	delays[1] = 1.0f;	delays[2] = 2.0f;	delays[3] = 3.0f;

	std::vector<float> shutters_float(1); 
	shutters_float[0] = 1920.0f;
	
	char dir_name[1024] = "f:\\tmp\\pmdtest2";
	char file_name[1024] = "PMD_test_meas";
	char comport[128] = "COM6";

	int numtakes = 1;

	// Capture from PMD to file file_name
	if (PMD_params_to_file (frequencies,delays,shutters_float, dir_name, file_name, comport, numtakes))
		error = 1;
	
	// Capture directly from PMD to RawData (RawData DATAPMD_CAPTURE)	(DEPRECATED)
	//if (PMD_params_to_RawData (DATAPMD_CAPTURE, frequencies, delays, shutters_float, comport, numtakes, false))
	//	error = 1;
	
	// Capture directly from PMD to Frame (Frame FRAME_00_CAPTURE, Frame FRAME_90_CAPTURE)
	float frequency = 100.0f;
	float distance = 0.0f;
	float shutter = 1920.0f;
	Frame * frame_00_null = NULL;	// (*frame_00_null) in PMD_params_to_Frame(...), if we want to avoid this Frame meas
	Frame * frame_90_null = NULL;	// (*frame_90_null) in PMD_params_to_Frame(...), if we want to avoid this Frame meas
	// FRAME_00_CAPTURE, FRAME_90_CAPTURE
	if (PMD_params_to_Frame (FRAME_00_CAPTURE, FRAME_90_CAPTURE, frequency, distance, shutter, comport, false))
		error = 1;

	return error;
}


// test get_area()
void test_get_area() {
	Point* p0 = new Point(1.0f, 1.0f, 1.0f);
	Point* p1 = new Point(3.0f, 1.0f, 1.0f);
	Point* p2 = new Point(3.0f, 2.0f, 1.0f);
	Point* p3 = new Point(1.0f, 3.0f, 1.0f);
	//PointMesh* rect_0 = new PointMesh(vp_rectangle(p0, p1, p2, p3), RECTANGLE, p0, 1.0f);
	//PointMesh* tri_0  = new PointMesh(vp_triangle(p0, p1, p2), TRIANGLE, p0, 1.0f);
	//float area_rect_0 = (*rect_0).get_area();
	//float area_tri_0  = (*tri_0).get_area();
	int erase_me = 0;
}


// test geometry term functions
void test_geometry_term() {
	Point v0(1.0f, 0.0f, 0.0f);
	Point v1(1.0f, 1.0f, 0.0f);
	// float cos_01 = cos_vectors(new Point(1.0f, 0.0f, 0.0f), new Point(1.0f, 1.0f, 0.0f));	// this creates 2 points in the memory heap permanently (till deletion in the code). BAD
	//float cos_01 = cos_vectors(&(Point(1.0f, 0.0f, 0.0f)), &(Point(1.0f, 1.0f, 1.0f)));		// this creates 2 points ONLY in the scope and they will be AUTOMATICALLY DELETED  . GOOD
	//float cos_01 = get_cos_vectors(&(Point(v0)), &(Point(1.0f, 1.0f, 1.0f)));		// this creates 2 points ONLY in the scope and they will be AUTOMATICALLY DELETED  . GOOD

	Point p0(0.0f, 0.0f, 0.0f);
	Point n0(0.0f, 0.0f, -1.0f); n0.normalize();
	Point p1(0.0f, 0.0f, -1.0f);
	Point n1(0.0f, 1.0f, 1.0f); n1.normalize();
	//float gt_01 = get_geometry_term(&p0, &n0, &p1, &n1);

}


// engdemo.cpp	// Revision: 1.1.6.4
// opyright 1984-2011 The MathWorks, Inc.
// All rights reserved
int test_MATLAB_engine() {
	
	const int BUFSIZE = 256;

	Engine *ep;
	mxArray *T = NULL, *result = NULL;
	char buffer[BUFSIZE+1];
	double time[10] = { 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0 };

	/*
	 * Call engOpen with a NULL std::string. This starts a MATLAB process 
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
	 * a MATLAB std::string, which should define a variable X.  MATLAB
	 * will evaluate the std::string and create the variable.  We
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
	     * Get a std::string input from the user
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

