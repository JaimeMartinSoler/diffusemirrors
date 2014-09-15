
#ifndef __TEST_H
#define __TEST_H


#include "global.h"


// test function for testing
void test(char* dir_name, char* file_name);




// test_toPixSim(...)
void test_toPixSim(char* dir_name, char* file_name);

// test_numbers()
void test_numbers();

// test_gradient_descent()
void test_gradient_descent();

// test_create_raw_from_raw_takes()
void test_create_raw_from_raw_takes();

// test_RawData()
void test_RawData();
// test_CMX_iterator(). approximation considering camPos = lasPos
void test_CMX_iterator(char* dir_name, char* file_name);
// test_distHS(...). approximation considering camPos = lasPos
void test_distHS(Info & info, float pathWallOffset, float ERROR_ON_PURPOSE_DIST_WALL_OFFSET, int pixClearTimes);
// test_CMX()
void test_CMX(Info & info, float pathWallOffset, float ERROR_ON_PURPOSE_DIST_WALL_OFFSET, int pixClearTimes = 0); // Sets a Simulated Frame from a given dist_wall_cam. Used in test_CMX()
void test_FrameSimFromWallDist (Frame & frame, Info & info, float dist_wall_cam, int freq_idx, int renderTime_ms = 0, PixStoring ps = PIXELS_STORING_GLOBAL);
// test_calibration_matrix()
void test_calibration_matrix();

// Reads data from a .dat and info.txt file setting the DATAPMD_READ variable
int test_data_main();

// test_capturetool2_main(...)
int test_capturetool2_main();

// test get_area()
void test_get_area();

// test geometry term functions
void test_geometry_term();

int test_MATLAB_engine();

#endif


