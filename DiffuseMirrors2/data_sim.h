
#ifndef __DATA_SIM_H
#define __DATA_SIM_H

#include <vector>
#include <map>

#include "global.h"




// ----------------------------------------------------------------------------------------------------------------------------------------
// ----- DIRECT VISION --------------------------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------------------

// This is the implementation of the BestFit using the Levenberg-Marquardt nonlinear least squares algorithms (slevmar_dif()): http://users.ics.forth.gr/~lourakis/levmar/
void updatePixelPatches_Simulation_BestFit_Optim (CalibrationMatrix & cmx, Scene & sceneCopy, Frame & frameSim00, Frame & frameSim90, Frame & frame00, Frame & frame90, Point & camC, Point & camN, Object3D & screenFoVmeasNs, PixStoring ps_, bool pSim_);

// model to be fitted to measurements
//     p: Input parameters to be fitted. p_size: number of parameters (only distance in this first approach)
//         p[0] = dist(camC,wall)
//     x: Output values to be fitted.    x_size: number of values (pixels in this case)
//         x[i]: value of simulated pixel i
//     adata: additional data
void set_DirectVision_Simulation_Frame_Optim(float* p, float* x, int p_size, int x_size, void* adata);


// This will include a minimization algorithm, but for now it will run some simulations manually and get the best fit
// is totally inefficient with this implementation, just to try the system
void updatePixelPatches_Simulation_BestFit(CalibrationMatrix & cmx, Scene & sceneCopy, Frame & frameSim00, Frame & frameSim90, Frame & frame00, Frame & frame90, Point & camC, Point & camN, Object3D & screenFoVmeasNs, PixStoring ps_ = PIXELS_STORING_GLOBAL, bool pSim_ = false);

// This will include a minimization algorithm, but for now it will run some simulations manually and get the best fit
// is totally inefficient with this implementation, just to try the system. Includes Tilt
void updatePixelPatches_Simulation_BestFit_WithTilt(CalibrationMatrix & cmx, Scene & sceneCopy, Object3D & NsMod, std::vector<float> & sinAG, Frame & frameSim00, Frame & frameSim90, Frame & frame00, Frame & frame90, Point & camC, Point & camN, Object3D & screenFoVmeasNs, PixStoring ps_, bool pSim_);

// sets a Simulated Frame for the Direct Vision case, from a Calibration Matrix
void set_DirectVision_Simulation_Frame(CalibrationMatrix & cmx, Scene & sceneCopy, Frame & frameSim00, Frame & frameSim90, int freq_idx, PixStoring ps_ = PIXELS_STORING_GLOBAL, bool pSim_ = false);

// Distance (Measurement, Simulation) function
float distHS(Frame & H00, Frame & H90, Frame & S00, Frame & S90);




// ----------------------------------------------------------------------------------------------------------------------------------------
// ----- OCCLUSION ------------------------------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------------------

// sets a Simulated Frame for the Occlusion case, from a Transient Image and a Calibration Matrix. This does all the calculations
void set_Occlusion_Simulation_Frame_Optim(float* p, float* x, int p_size, int x_size, void* adata);

// Part of set_Occlusion_Simulation_Frame_Optim(...)
// check if the parameters converge with the given bounds. If not, it sets big values in x
bool pInBounds(float* p, float* x, int p_size, int x_size, struct OCCLUSION_ADATA* ad);

// Part of set_Occlusion_Simulation_Frame_Optim(...)
// updates the current scene with the values of the given parameters p
void updateSceneOcclusion(float* p, struct OCCLUSION_ADATA* ad);

// For set_Occlusion_Simulation_Frame(...)
// gets the Radiance from each volume patch (radiance from each volume patch). L(x) in the paper. 
// It deals with patches backing (not facing) the wall (they are considered ALWAYS facing the wall)
void setAttTermV(struct OCCLUSION_ADATA* ad);

// For set_Occlusion_Simulation_Frame(...)
// gets the Transient pixel = Impulse response of the scene. alpha_r in Ref08
// vector of maps. One map for pixel representing:
//   x axis = key   = path length (r) in m
//   y axis = value = amplitude of the impulse response
void setTransientImage(struct OCCLUSION_ADATA* ad);

// For set_Occlusion_Simulation_Frame(...)
// sets a Simulated Frame for the Occlusion case, from a Transient Image and a Calibration Matrix. This does NOT do any calculations
void set_FrameSim(struct OCCLUSION_ADATA* ad);

/*
// Auxiliar (add-hoc) function for setAttTermV and setAttTermV
// returns the actual index of the shape s of the face f, where:
// idxS0ofF[i] = index of the volPatch j, first volPatch of the face[i], 
// idxS0ofF[faces] = total number of shapes
int idxFS(int fi, int si, std::vector<int> & idxS0ofF);
*/

// Auxiliar function for updateSceneOcclusion(...)
// sets the axisN and the rad from the parameters
void set_axisNrad_fromP(Point & axisN, float & rad, float* p);




// ----------------------------------------------------------------------------------------------------------------------------------------
// ----- OCCLUSION OLD IMPLEMENTATION -----------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------------------

// This will include a minimization algorithm, but for now it will run some simulations manually and get the best fit
// is totally inefficient with this implementation, just to try the system
void updateVolumePatches_Occlusion_OLD_BestFit(CalibrationMatrix & cmx, Scene & sceneCopy, Object3D volPatchesRef, Frame & frameSim00, Frame & frameSim90, Frame & frame00, Frame & frame90, Point & walN, Point & _vopN, float dRes, PixStoring ps_ = PIXELS_STORING_GLOBAL, bool pSim_ = false);

// sets a Simulated Frame for the Occlusion case, from a Transient Image and a Calibration Matrix. This does all the calculations
void set_Occlusion_Simulation_Frame(CalibrationMatrix & cmx, Scene & scene, Frame & frameSim00, Frame & frameSim90, Point & walN, int freq_idx, PixStoring ps_ = PIXELS_STORING_GLOBAL, bool pSim_ = false);




// ----------------------------------------------------------------------------------------------------------------------------------------
// ----- OTHER ----------------------------------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------------------

// Correlation with Sinusoid asumption. c(w,phi)(r) in Paper.
// frequency (MHz), phase (deg), r(ns), N (abs)
float correlation(float frequency_, float phase_, float r_, float N_);

// Plot a transient pixel with MATLAB Engine
void plot_transientPixel(std::vector<float> & transientPixDist, std::vector<float> & transientPixAmpl, int transientPixSize);

#endif

