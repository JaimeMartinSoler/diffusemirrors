
#include "global.h"

#include "scene.h"
#include "data.h"

// SCENE MAIN
Scene SCENEMAIN;

// Info, RawData, Frame objects
Info INFO;
RawData DATAPMD_READ;		// RawData Read from a File (.dat)
RawData DATAPMD_CAPTURE;	// RawData Captured (directly from the PMD to this object)
Frame FRAME_00_CAPTURE;		// Frame (phase=00) Captured (directly from the PMD to this object)
Frame FRAME_90_CAPTURE;		// Frame (phase=90) Captured (directly from the PMD to this object)

// To externally control the capability to loop of the capturetool2.cpp functions (true by default)
bool PMD_LOOP_ENABLE = true;

// Syncornization
std::mutex mutex_frame_object;				// mutual exclusion frame-object
std::condition_variable cv_frame_object;	// condition variable frame-object
bool UPDATED_NEW_FRAME = false;				// bool for spurious wakes (setted to false to wait in the object until 1st Frame is drawn)
bool UPDATED_NEW_SCENE = true;				// bool for spurious wakes	


