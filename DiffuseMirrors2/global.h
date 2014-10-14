
#ifndef __GLOBAL_H
#define __GLOBAL_H

#include <vector>
#include <mutex>              // std::mutex, std::unique_lock
#include <condition_variable> // std::condition_variable

// enums
enum ShapeType { UNKNOWN_SHA, PT, LINE, TRIANGLE, QUAD };
enum SceneType { UNKNOWN_SCT, DIRECT_VISION_SINUSOID, DIRECT_VISION_SIMULATION, DIRECT_VISION_SIMULATION_FRAME, OCCLUSION, OCCLUSION_FRAME, FOV_MEASUREMENT, RAW_DATA, CALIBRATION_MATRIX, TEST, TEST_TEST };
enum PixStoring { UNKNOWN_PIS, PIXELS_TOTAL, PIXELS_VALID };	// The PMD camera stores 165x120 pixels but some of them in the edges can not be considered as valid pixel meas
enum Object3DType { CAMERA, LASER, WALL, OCCLUDER, FLOOR, VOLUME, WALL_PATCHES, CAMERA_FOV, LASER_RAY, VOLUME_PATCHES, PIXEL_PATCHES, UNKOWN_OBT};
#define SCENE_SIZE 11
enum BoxSides { FRONT = 0, RIGHT = 1, BACK = 2, LEFT = 3, BOTTOM = 4, TOP = 5 };
//enum Source {DATA_FILE, DATA_REAL_TIME, SIMULATION, UNKNOWN_SRC};


// classes data.h
class Info;
class RawData;
class CalibrationMatrix;
class Frame;
// classes scene.h
class Point;
class Shape;
class Object3D;
class Scene;


// structure for passing user-supplied additional data to the objective function.
struct OCCLUSION_ADATA {
	// constant parameters
	Info* info;
	CalibrationMatrix* cmx;
	Object3D* volPatchesRef;
	int numFaces;
	int numShapes;
	std::vector<int>* shapesPerFace;
	std::vector<int>* firstShapeIdx_of_face;
	std::vector<Point>* faceNRef;
	std::vector<Point*>* shapeN;	// unnused
	std::vector<float>* area;
	Point* walN;
	Point* walL;
	int freq_idx;
	PixStoring ps_;
	bool pSim_;
	int numPix;				// rows*cols
	int sizeofFrameData;	// rows*cols*sizeof(float) = rows*cols*4
	// constant parameters p bounds
	std::vector<float>* pL;	// lower bound
	std::vector<float>* pU;	// upper bound
	// variable parameters
	Scene* sceneCopy;			// semi-constant: sceneCopy.o[VOLUME_PATHCES] is modified in each iteration of levmar
	Frame* frameSim00;
	Frame* frameSim90;
	std::vector<Point>* faceN;
	int facesFacing_size;
	std::vector<int>* facesFacingIdx;
	std::vector<int>* firstShapeIdx_in_volPatchesRadiance_of_facesFacingIdx;
	int volPatchesRadiance_size;
	std::vector<int>* volPatchesRadianceIdx;
	std::vector<float>* volPatchesRadiance;
	std::vector<int>* transientImage_size;
	std::vector<std::vector<float>>* transientImageDist;
	std::vector<std::vector<float>>* transientImageAmpl;
	Point* traV;
};


#define PIXELS_STORING_GLOBAL PIXELS_VALID		// PIXELS_TOTAL,    PIXELS_VALID,    UNKNOWN_PIS
#define PIXELS_SIMULATION false		// determines if we will Simulate patches and pixels (when possible) with less resolution than the PMD 
#define PMD_SIM_ROWS 12
#define PMD_SIM_COLS 16


// CAPTURETOOL PARAMETERS
#define SOURCE_PLUGIN "camboardnano"
#define SOURCE_PARAM ""

#define SYNTH_CLOCK 600.0000 // Sample clock of function generator
//#define SPEEDOFLIGHT	 299792458	// (m/s)
#define SPEEDOFLIGHT_AIR 299705000	// (m/s)
#define DUTYCYCLE_INVERSE_OLD 10 // 1 us exposure, 9 us delay
#define DUTYCYCLE 0.04f		// ABSOLUTELY IMPORTANT for thermal stability: add delay to ensure a duty cycle below 4%
#define TAKES_PER_CAPTURE 4 // The PMD actually captures 4 frames per take, being internally the shutter time 4 times greater. It has to be taken into account for DUTYCYCLE calculations

#define FILENAME_FORMAT "capture_take%02d_f%06.2f_d%05.2f" // Filename prefix: Frequency, delay in m // For temporal files for CV_WHILE_CAPTURING
#define FILENAME_APPEND "_p%03d.%s"         // Append phase, shutter (=0 for HDR) and suffix to filename // For temporal files for CV_WHILE_CAPTURING

#define NUMTAKE_FILENAME_APPEND "_nt"	// Info file 
#define INF_FILENAME_SUFFIX ".inf"	// Info file 
#define RAW_FILENAME_SUFFIX ".raw"	// Raw Data file
#define CMX_FILENAME_SUFFIX ".cmx"	// Calibration Matrix file	// Stores the C matrix
#define CMD_FILENAME_SUFFIX ".cmd"	// Calibration Matrix Direct Vision file


#define PMD_WIDTH 165	// (MHz)
#define PMD_HEIGTH 120	// (MHz)

#define FREQUENCY_MIN 1.0f		// (MHz)
#define FREQUENCY_MAX 180.0f	// (MHz)
#define SHUTTER_MIN 3.0f		// (us)
#define SHUTTER_MAX 1920.0f		// (us)

// TRANSIENTPMD SETTINGS
#define COMPORT_FORMAT "\\\\.\\%s"

// OPENCV
#define WindowNameCVCAP "PMD Image"
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
const int CAMERA_PIX_X_BAD_RIGHT = 2;	// = 2
const int CAMERA_PIX_X_BAD = CAMERA_PIX_X_BAD_LEFT + CAMERA_PIX_X_BAD_RIGHT;
const int CAMERA_PIX_X_VALID = CAMERA_PIX_X - CAMERA_PIX_X_BAD;
// Bad pixels bottom and top
const int CAMERA_PIX_Y_BAD_TOP = 1;		// = 1
const int CAMERA_PIX_Y_BAD_BOTTOM = 0;	// = 0
const int CAMERA_PIX_Y_BAD = CAMERA_PIX_Y_BAD_TOP + CAMERA_PIX_Y_BAD_BOTTOM;
const int CAMERA_PIX_Y_VALID = CAMERA_PIX_Y - CAMERA_PIX_Y_BAD;
// Measuring the FoV of the camera
const float CAMERA_DIST_FOV_MEAS = 1.5f;	// distance from camera center and screen (with same normal)
const float CAMERA_FOV_X_METERS = 0.653f;
const float CAMERA_FOV_Y_METERS = CAMERA_FOV_X_METERS * (((float)CAMERA_PIX_Y) / ((float)CAMERA_PIX_X));	// square pixels assumption

// VOLUME PATCHES CONFIGURATION
const int VOLUME_GRID_SIZE_X = 2;
const int VOLUME_GRID_SIZE_Y = 2;
const int VOLUME_GRID_SIZE_Z = 2;
const float VOLUME_GRID_X_ANGLE_MIN = -90.0f;
const float VOLUME_GRID_X_ANGLE_MAX = 90.0f;
const float VOLUME_GRID_X_ANGLE_STEP = 10.0f;
const float VOLUME_GRID_Y_ANGLE_MIN = -180.0f;
const float VOLUME_GRID_Y_ANGLE_MAX = 180.0f;
const float VOLUME_GRID_Y_ANGLE_STEP = 10.0f;

// Image Formation Model
const float L_E = 1.0f;	// Le(l) in the paper. Radiance from the light point in the wall from the laser

// SCENE MAIN
extern Scene SCENEMAIN;

// RawData and FrameObjects
extern Info INFO;				// Info with the parameters of the .inf file
extern RawData DATAPMD_READ;	// RawData Read from a File (.dat)
extern RawData DATAPMD_CAPTURE;	// RawData Captured (directly from the PMD to this object)
extern Frame FRAME_00_CAPTURE;	// Frame (phase=00) Captured (directly from the PMD to this object)
extern Frame FRAME_90_CAPTURE;	// Frame (phase=90) Captured (directly from the PMD to this object)

// To externally control the capability to loop of the capturetool2.cpp functions (true by default)
extern bool PMD_LOOP_ENABLE;

// Syncornization
extern std::mutex mutex_frame_object;
extern std::condition_variable cv_frame_object;
extern bool UPDATED_NEW_FRAME;
extern bool UPDATED_NEW_SCENE;

#endif

