
#ifndef __GLOBAL_H
#define __GLOBAL_H

#include "shapes.h"
#include "data_read.h"

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

// DataPMD Objects
extern DataPMD DATAPMD_READ;
extern DataPMD DATAPMD_CAPTURE;


#endif

