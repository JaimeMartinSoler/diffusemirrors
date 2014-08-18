
#include "global.h"
#include "shapes.h"
#include "data_read.h"

// vectors with all the object3D to be studied (and rendered)
extern Object3D_Set OBJECT3D_SET(OBJECT3D_SET_SIZE);

// DataPMD Objects
DataPMD DATAPMD_READ;		// DataPMD Read from a File (.dat)
DataPMD DATAPMD_CAPTURE;	// DataPMD Captured (directly from the PMD to this object)
Frame FRAME_00_CAPTURE;		// Frame (phase=00) Captured (directly from the PMD to this object)
Frame FRAME_90_CAPTURE;		// Frame (phase=90) Captured (directly from the PMD to this object)

// To externally control the capability to loop of the capturetoolDM2.cpp functions (true by default)
bool PMD_LOOP_ENABLE = true;

// Syncornization
std::mutex mutex_frame_object;				// mutual exclusion frame-object
std::condition_variable cv_frame_object;	// condition variable frame-object
bool UPDATED_NEW_FRAME = false;				// bool for spurious wakes (setted to false to wait in the object until 1st Frame is drawn)
bool UPDATED_NEW_OBJECT = true;				// bool for spurious wakes	


