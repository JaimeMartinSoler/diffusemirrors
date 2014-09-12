
#ifndef __DATA_SIM_H
#define __DATA_SIM_H

#include <vector>
#include <map>

#include "global.h"

// gets all the data results, once the scene (OBJECT3D_SET) has been configured
void get_data_sim_diffused_mirror();

// gets all the data results with the simple set-up, once the scene (OBJECT3D_SET) has been configured
void get_data_sim_direct_vision_wall();




// (2014-09-08)
void set_DirectVision_Simulation_Frame (CalibrationMatrix & cmx, Scene & sceneCopy, Frame & frameSim, int freq_idx, PixStoring ps_ = PIXELS_STORING_GLOBAL, bool pSim = false);



// gets the Radiance from each volume patch (radiance from each volume patch). L(x) in the paper. 
// It deals with patches backing (not facing) the wall (they are not taken into account)
// TO-DO: OCCLUSION
void get_radiance_volume_patches(std::vector<float> & radiance_volume_patches_);

// (Radiance from each volume patch)·(Area of each volume patch). L(x)·Area(x)
// As long as we deal with patches instead of points, we are relating an L(x) with
// the patch which x represents weighted with its Area.
// If we don't weight each patch with each area, 4 wall patches of area 1.0 would give a
// 4 times higher combined L(x) than 1 wall patch (the comb of the previous ones) of area 4.0
// Also used in the "MYSELF" part
void get_radiance_volume_patches_x_area(std::vector<float> & radiance_volume_patches_x_area_, std::vector<float> & radiance_volume_patches_);

// gets the Radiance from the wall patch (radiance from each wall patch). L(w) in the paper. 
// It deals with volume patches backing (not facing) wall patches (they are not taken into account)
// TO-DO: OCCLUSION
void get_radiance_wall_patches(std::vector<float> & radiance_wall_patches_, std::vector<float> & radiance_volume_patches_);



// gets the Transient pixel = Impulse response of the scene. alpha_r in Ref08
// vector of maps. One map for pixel representing
//   x axis = key   = time (r) in ns
//   y axis = value = amplitude of the impulse response
void get_transient_image(std::vector<std::multimap<float, float>> & transient_image_, std::vector<float> & radiance_volume_patches_);

void get_transient_image_simple(std::vector<std::multimap<float, float>> & transient_image_simple_);

// Pixels value. H(w,phi) in the paper
void get_pixels_value(std::vector<float> & pixels_value_, std::vector<std::multimap<float, float>> & transient_image_, float distance_, float frequency_, float phase_, float shutter_, float Em_);

// Correlation. c(w,phi)(r) in Paper. Here defined as a function. Later should be a matrix
// frequency (MHz), phase (deg), r(ns), N (abs)
float correlation(float frequency_, float phase_, float r_, float N_);



// Plot a transient pixel with MATLAB Engine
void plot_transient_pixel (std::vector<std::multimap<float, float>> & transient_image_, int pix_x_, int pix_y_);

/*
// Plot image pixels values with MATLAB Engine
void plot_image_pixels_values(std::vector<float> & pixels_value_, int heigth_, int width_);
*/

// (2014-09-08)
// This will include a minimization algorithm, but for now it will run some simulations manually and get the best fit
// is totally inefficient with this implementation, just to try the system
void updatePixelPatches_Simulation_BestFit(CalibrationMatrix & cmx, Scene & sceneCopy, Frame & frame00, Point & camC, Point & camN, Object3D & screenFoVmeasNs, PixStoring ps_ = PIXELS_STORING_GLOBAL, bool pSim_ = false);

// (2014-09-08)
float distMeasSim(Frame & H, Frame & S);



#endif

