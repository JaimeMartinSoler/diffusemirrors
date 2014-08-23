
#ifndef __GLOBAL_H
#define __GLOBAL_H

#include <vector>
#include <mutex>              // std::mutex, std::unique_lock
#include <condition_variable> // std::condition_variable

// enums
enum Scene {DIRECT_VISION_WALL, DIRECT_VISION_ANY, DIRECT_VISION_ANY_SIMULATION, DIFFUSED_MIRROR, FOV_MEASUREMENT, CALIBRATION_MATRIX, UNKNOWN_SCENE};
// The PMD camera stores 165x120 pixels but some of them in the edges can not be considered as valid pixel measurements
enum Pixels_storing {PIXELS_TOTAL, PIXELS_VALID, UNKNOWN_PIXELS_STORING};
enum Source {DATA_FILE, DATA_REAL_TIME, SIMULATION, UNKNOWN_SRC};


#define PIXELS_STORING_GLOBAL PIXELS_VALID		// PIXELS_TOTAL,    PIXELS_VALID,    UNKNOWN_PIXELS_STORING

// CAPTURETOOL PARAMETERS
#define SOURCE_PLUGIN "camboardnano"
#define SOURCE_PARAM ""

#define SYNTH_CLOCK 600.0000 // Sample clock of function generator
//#define SPEEDOFLIGHT	 299792458	// (m/s)
#define SPEEDOFLIGHT_AIR 299705000	// (m/s)
#define DUTYCYCLE_INVERSE_OLD 10 // 1 us exposure, 9 us delay
#define DUTYCYCLE 0.04f // ABSOLUTELY IMPORTANT for thermal stability: add delay to ensure a duty cycle below 4%
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
#define CV_WHILE_CAPTURING false

// SPEED OF LIGHT IN AIR and PI
const float C_LIGHT_AIR = 299705000.0f;	// (m/s)
const float PI = 3.14159265359f;

// CAMERA CONFIGURATION
// Pixels
const int CAMERA_PIX_X = 165;	// Real: 165
const int CAMERA_PIX_Y = 120;	// Real: 120
// Bad pixels left and right
const int CAMERA_PIX_X_BAD_LEFT = 1;	// = 1
const int CAMERA_PIX_X_BAD_RIGHT = 2;	// = 1
const int CAMERA_PIX_X_BAD = CAMERA_PIX_X_BAD_LEFT + CAMERA_PIX_X_BAD_RIGHT;
const int CAMERA_PIX_X_VALID = CAMERA_PIX_X - CAMERA_PIX_X_BAD;
// Bad pixels bottom and top
const int CAMERA_PIX_Y_BAD_TOP = 1;		// = 1
const int CAMERA_PIX_Y_BAD_BOTTOM = 0;	// = 0
const int CAMERA_PIX_Y_BAD = CAMERA_PIX_Y_BAD_TOP + CAMERA_PIX_Y_BAD_BOTTOM;
const int CAMERA_PIX_Y_VALID = CAMERA_PIX_Y - CAMERA_PIX_Y_BAD;
// Measuring the FoV of the camera
const float CAMERA_DIST_FOV_MEAS = 2.0f;	// distance from camera center and screen (with same normal)
const float CAMERA_FOV_X_METERS = 0.855f;
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
const int CAMERA		 = 0;
const int LASER			 = 1;
const int WALL			 = 2;
const int OCCLUDER		 = 3;
const int FLOOR			 = 4;
const int VOLUME		 = 5;
const int WALL_PATCHES	 = 6;
const int CAMERA_FOV	 = 7;
const int LASER_RAY		 = 8;
const int VOLUME_PATCHES = 9;
const int PIXEL_PATCHES  = 10;	// Only to represent the Direct-Vision-Any scene

// Image Formation Model
const float L_E = 1.0f;	// Le(l) in the paper. Radiance from the light point in the wall from the laser

// vectors with all the object3D to be studied (and rendered)
const int OBJECT3D_SET_SIZE = 11;
class PointMesh;
typedef std::vector<PointMesh*> Object3D;
typedef std::vector<Object3D*> Object3D_Set;
extern Object3D_Set OBJECT3D_SET;

// DataPMD and FrameObjects
class DataPMD;
class Frame;
extern DataPMD DATAPMD_READ;	// DataPMD Read from a File (.dat)
extern DataPMD DATAPMD_CAPTURE;	// DataPMD Captured (directly from the PMD to this object)
extern Frame FRAME_00_CAPTURE;	// Frame (phase=00) Captured (directly from the PMD to this object)
extern Frame FRAME_90_CAPTURE;	// Frame (phase=90) Captured (directly from the PMD to this object)

// To externally control the capability to loop of the capturetoolDM2.cpp functions (true by default)
extern bool PMD_LOOP_ENABLE;

// Syncornization
extern std::mutex mutex_frame_object;
extern std::condition_variable cv_frame_object;
extern bool UPDATED_NEW_FRAME;
extern bool UPDATED_NEW_OBJECT;

#endif

