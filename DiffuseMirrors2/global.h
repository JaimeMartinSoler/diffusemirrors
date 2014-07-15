
#ifndef __GLOBAL_H
#define __GLOBAL_H

#include "shapes.h"
#include "data_read.h"




// CAPTURETOOL PARAMETERS
#define SOURCE_PLUGIN "camboardnano"
#define SOURCE_PARAM ""

#define SYNTH_CLOCK 600.0000 // Sample clock of function generator
//#define SPEEDOFLIGHT	 299792458	// (m/s)
#define SPEEDOFLIGHT_AIR 299705000	// (m/s)
#define DUTYCYCLE 10 // 1 us exposure, 9 us delay
#define FILENAME_FORMAT "capture_take%02d_f%06.2f_d%05.2f" // Filename prefix: Frequency, delay in m
#define FILENAME_APPEND "_p%03d.%s"         // Append phase, shutter (=0 for HDR) and suffix to filename
#define FILE_DATA_NAME_SUFFIX ".dat"
#define FILE_INFO_NAME_SUFFIX "_info.txt"

#define PMD_WIDTH 165	// (MHz)
#define PMD_HEIGTH 120	// (MHz)

#define FREQUENCY_MIN 1.0	// (MHz)
#define FREQUENCY_MAX 180.0	// (MHz)
#define SHUTTER_MIN 3.0		// (us)
#define SHUTTER_MAX 1920.0	// (us)

// TRANSIENTPMD SETTINGS
#define COMPORT_FORMAT "\\\\.\\%s"

// OPENCV
#define WindowName "PMD Image"
#define NOJPEG





// SPEED OF LIGHT IN AIR and PI
const float C_LIGHT_AIR = 299705000.0f;	// (m/s)
const float PI = 3.14159265359f;

// CAMERA CONFIGURATION
// Pixels
const int CAMERA_PIX_X = 32;	// Real: 165
const int CAMERA_PIX_Y = 24;	// Real: 120
// Measuring the FoV of the camera
const float CAMERA_DIST_FOV_MEAS = 1.0f;	// distance from camera center and screen (with same normal)
const float CAMERA_FOV_X_METERS = 0.5f;
const float CAMERA_FOV_Y_METERS = CAMERA_FOV_X_METERS * (((float)CAMERA_PIX_Y) / ((float)CAMERA_PIX_X));	// square pixels assumption

// VOLUME PATCHES CONFIGURATION
const int VOLUME_GRID_SIZE_X = 4;
const int VOLUME_GRID_SIZE_Y = 4;
const int VOLUME_GRID_SIZE_Z = 4;
const float VOLUME_GRID_X_ANGLE_MIN = -90.0f;
const float VOLUME_GRID_X_ANGLE_MAX = 90.0f;
const float VOLUME_GRID_X_ANGLE_STEP = 10.0f;
const float VOLUME_GRID_Y_ANGLE_MIN = -180.0f;
const float VOLUME_GRID_Y_ANGLE_MAX = 180.0f;
const float VOLUME_GRID_Y_ANGLE_STEP = 10.0f;

// OBJECT3D_SET[i], i = constant index content
const int CAMERA		= 0;
const int LASER			= 1;
const int WALL			= 2;
const int OCCLUDER		= 3;
const int FLOOR			= 4;
const int VOLUME		= 5;
const int WALL_PATCHES	= 6;
const int CAMERA_FOV	= 7;
const int LASER_RAY		= 8;
const int VOLUME_PATCHES = 9;

// Image Formation Model
const float L_E = 1.0f;	// Le(l) in the paper. Radiance from the light point in the wall from the laser

// vectors with all the object3D to be studied (and rendered)
const int OBJECT3D_SET_SIZE = 10;
extern Object3D_Set OBJECT3D_SET;

// DataPMD and FrameObjects
extern DataPMD DATAPMD_READ;
extern DataPMD DATAPMD_CAPTURE;
extern Frame FRAME_00_CAPTURE;
extern Frame FRAME_90_CAPTURE;


#endif

