
#include <vector>
#include <map>
#include "global.h"

#include "data_sim.h"
#include "data.h"
#include "scene.h"
// MATLAB
#include "engine.h"



// ----------------------------------------------------------------------------------------------------------------------------------------
// ----- DIRECT VISION --------------------------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------------------

// This will include a minimization algorithm, but for now it will run some simulations manually and get the best fit
// is totally inefficient with this implementation, just to try the system
void updatePixelPatches_Simulation_BestFit(CalibrationMatrix & cmx, Scene & sceneCopy, Frame & frameSim, Frame & frame00, Point & camC, Point & camN, Object3D & screenFoVmeasNs, PixStoring ps_, bool pSim_) {

	// pixPatchesBestFit
	Object3D pixPatchesBestFit;
	float distHSmin = FLT_MAX;
	float distHS_;

	// get freq_idx
	int freq_idx = get_freq_idx(*(cmx.info), frame00.freq);	// returns -1 if no idx correspondance was found
	if (freq_idx < 0) {
		std::cout << "\nWarning: Frame freq = " << frame00.freq << " is not a freq in .cmx freqV = ";
		print(cmx.info->freqV);
		return;
	}

	// we will discretize the simulation just by the distances to the camera (constant for all pixel patches as screenFoVmeasNs[i]*dist)
	float dRes = 0.05f;
	float dMin = 1.0f;
	float dMax = 5.0f + dRes / 2.0f;
	
	// COMIBATION ITERATOR
	/*
	float dRes = 0.05f;
	float dMin = 1.0f;
	float dMax = 5.0f + dRes / 2.0f;
	std::vector<float> pixPatchDist(numPix(ps_), 0.0f);
	bool finishLoop = false;
	int pos = 0;
	while (!finishLoop) {
	pixPatchDist[pos=0] += dRes;
	while (pixPatchDist[pos] > dMax) {
	if (pos >= pixPatchDist.size() - 1) {
	finishLoop = true;
	break;
	}
	pixPatchDist[pos] = dMin;
	pixPatchDist[++pos] += dRes;
	}	}
	*/

	for (float d = dMin; d < dMax; d += dRes) {
		// Update pixel patches
		for (int i = 0; i < sceneCopy.o[PIXEL_PATCHES].s.size(); i++) {
			sceneCopy.o[PIXEL_PATCHES].s[i].p[0].set(sceneCopy.o[CAMERA].s[0].c + screenFoVmeasNs.s[i].p[0] * d);	// Useless for meas but for rendering
			sceneCopy.o[PIXEL_PATCHES].s[i].p[1].set(sceneCopy.o[CAMERA].s[0].c + screenFoVmeasNs.s[i].p[1] * d);	// Useless for meas but for rendering
			sceneCopy.o[PIXEL_PATCHES].s[i].p[2].set(sceneCopy.o[CAMERA].s[0].c + screenFoVmeasNs.s[i].p[2] * d);	// Useless for meas but for rendering
			sceneCopy.o[PIXEL_PATCHES].s[i].p[3].set(sceneCopy.o[CAMERA].s[0].c + screenFoVmeasNs.s[i].p[3] * d);	// Useless for meas but for rendering
			sceneCopy.o[PIXEL_PATCHES].s[i].c.set(sceneCopy.o[CAMERA].s[0].c + screenFoVmeasNs.s[i].c    * d);	// Useful for meas
		}
		// get Simulation Frame (S)
		set_DirectVision_Simulation_Frame(cmx, sceneCopy, frameSim, freq_idx, ps_, pSim_);
		// get Distance(H,S)
		distHS_ = distHS(frame00, frameSim);
		// check if Distance(H,S) is better and update the pixPatchesBestFit
		if (distHS_ < distHSmin) {
			distHSmin = distHS_;
			pixPatchesBestFit = sceneCopy.o[PIXEL_PATCHES];
		}
	}
	sceneCopy.o[PIXEL_PATCHES] = pixPatchesBestFit;
}


// sets a Simulated Frame for the Direct Vision case, from a Calibration Matrix
void set_DirectVision_Simulation_Frame(CalibrationMatrix & cmx, Scene & sceneCopy, Frame & frameSim, int freq_idx, PixStoring ps_, bool pSim_) {

	// Simulated Image Vector
	float relativeAlbedo = 1.0f;
	std::vector<float> DirectVision_Simulation(numPix(ps_, pSim_));
	for (int r = 0; r < rows(ps_, pSim_); r++) {
		for (int c = 0; c < cols(ps_, pSim_); c++) {
			DirectVision_Simulation[rc2idx(r, c, ps_, pSim_)] = cmx.S_DirectVision(freq_idx, r, c,
				sceneCopy.o[LASER].s[0].c, sceneCopy.o[PIXEL_PATCHES].s[rc2idx(r, c, ps_, pSim_)].c, sceneCopy.o[CAMERA].s[0].c, relativeAlbedo, ps_, pSim_);
	}	}
	frameSim.set(DirectVision_Simulation, rows(ps_, pSim_), cols(ps_, pSim_), cmx.info->freqV[freq_idx], 0.0f, cmx.info->shutV[cmx.info->shutV.size() - 1], 0.0f, ps_, pSim_);
}


// Distance (Measurement, Simulation) function
float distHS(Frame & H, Frame & S) {

	// Create the (H/S) vector
	int sizeHS = H.data.size();
	std::vector<float> HS(sizeHS);
	for (size_t i = 0; i < sizeHS; i++)
		HS[i] = H.data[i] / S.data[i];

	// (H/S)mean0, (H/S)var0
	float mean0 = mean(HS);
	float var0 = var(HS);

	// Get rid of outliers
	int outliersSym = 8;	// it erases one min and one max outlier each time
	int min_idx, max_idx;
	for (int i = 0; i < outliersSym; i++) {
		min(HS, min_idx);
		max(HS, max_idx);
		HS[min_idx] = mean0;
		HS[max_idx] = mean0;
	}
	float var1 = var(HS);

	// dist(H,S) = [var1(H/S)^2] / var0(H/S)
	return (var1 * var1) / var0;
}




// ----------------------------------------------------------------------------------------------------------------------------------------
// ----- OCCLUSION ------------------------------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------------------

// This will include a minimization algorithm, but for now it will run some simulations manually and get the best fit
// is totally inefficient with this implementation, just to try the system
void updateVolumePatches_Occlusion_BestFit(CalibrationMatrix & cmx, Scene & sceneCopy, Object3D volPatchesCopy, Frame & frameSim, Frame & frame00, Point & walN, Point & refN, float dRes, PixStoring ps_, bool pSim_) {

	// pixPatchesBestFit
	Object3D volPatchesBestFit;
	float distHSmin = FLT_MAX;
	float distHS_;

	// get freq_idx
	int freq_idx = get_freq_idx(*(cmx.info), frame00.freq);	// returns -1 if no idx correspondance was found
	if (freq_idx < 0) {
		std::cout << "\nWarning: Frame freq = " << frame00.freq << " is not a freq in .cmx freqV = ";
		print(cmx.info->freqV);
		return;
	}
	
	// Combination Iterator: we will discretize the simulation with all the possible combinations of volume patches
	float dMin = 0.0f;
	float dMax = dRes * (VOLUME_GRID_SIZE_Z + 0.5f);
	std::vector<float> pixPatchDist(VOLUME_GRID_SIZE_X * VOLUME_GRID_SIZE_Y, 0.0f);
	bool finishLoop = false;
	int pos = 0;
	int iter = 0;
	while (!finishLoop) {
		iter++; Sleep(200);
		if (true || (iter % 100) == 0)
			std::cout << "\niter = " << iter;
		// get Simulation Frame (S)
		set_Occlusion_Frame(cmx, SCENEMAIN, frame00, frameSim, walN, freq_idx, ps_, pSim_);
		distHS_ = distHS(frame00, frameSim);
		// check if Distance(H,S) is better and update the pixPatchesBestFit
		if (distHS_ < distHSmin) {
			distHSmin = distHS_;
			volPatchesBestFit = SCENEMAIN.o[PIXEL_PATCHES];
		}

		// Combination Iteration:
		SCENEMAIN.o[VOLUME_PATCHES].s[0].tra(refN * pixPatchDist[0]);
		pixPatchDist[pos=0] += dRes;
		while (pixPatchDist[pos] > dMax) {
			if (pos >= pixPatchDist.size() - 1) {
				finishLoop = true;
				break;
			}
			pixPatchDist[pos] = dMin;
			SCENEMAIN.o[VOLUME_PATCHES].s[pos] = volPatchesCopy.s[pos];
			pixPatchDist[++pos] += dRes;
			SCENEMAIN.o[VOLUME_PATCHES].s[pos].tra(refN * pixPatchDist[pos]);
		}
	}
	SCENEMAIN.o[VOLUME_PATCHES] = volPatchesBestFit;
}

// sets a Simulated Frame for the Occlusion case, from a Transient Image and a Calibration Matrix. This does all the calculations
void set_Occlusion_Frame(CalibrationMatrix & cmx, Scene & scene, Frame & frameReal, Frame & frameSim, Point & walN, int freq_idx, PixStoring ps_, bool pSim_) {
	
	// L_E;		// Le(l) in the paper. Radiance from the light point in the wall from the laser

	// Radiance from each volume patch. L(x) in the paper. 
	int radiance_volPatches_size = SCENEMAIN.o[VOLUME_PATCHES].s.size();
	// set volPatches Normals
	std::vector<Point> radiance_volPatchesN(radiance_volPatches_size);
	for (std::size_t i = 0; i < scene.o[VOLUME_PATCHES].s.size(); i++)
		radiance_volPatchesN[i] = scene.o[VOLUME_PATCHES].s[i].normalQUAD();
	std::vector<float> radiance_volPatches(radiance_volPatches_size);
	set_radiance_volPatches(radiance_volPatches, radiance_volPatchesN, scene, scene.o[LASER_RAY].s[0].p[1], walN, true, true);

	// Transient pixel = Impulse response of the scene. alpha_r in Ref08
	// vector of maps. One map for pixel representing:
	//   x axis = key   = path length (r) in m
	//   y axis = value = amplitude of the impulse response
	std::vector<std::multimap<float, float>> transientImage(numPix(ps_, pSim_));
	set_TransientImage(transientImage, radiance_volPatches, radiance_volPatchesN, scene, scene.o[LASER_RAY].s[0].p[1], walN);

	// Pixels value. H(w,phi) in the paper
	set_FrameSim(transientImage, cmx, frameSim, freq_idx, ps_, pSim_);


	// Plot a transient pixel with MATLAB Engine, from TransientImage
	/*
	int pix_x = (CAMERA_PIX_X - 1) / 2;	// max = CAMERA_PIX_X-1
	int pix_y = (CAMERA_PIX_Y - 1) / 2;	// max = CAMERA_PIX_Y-1
	// MATLAB Engine takes too much time to start, comment out next line unless you need to debugg
	plot_transient_pixel(TransientImage, pix_x, pix_y);
	*/
}

// For set_Occlusion_Frame(...)
// gets the Radiance from each volume patch (radiance from each volume patch). L(x) in the paper. 
// It deals with patches backing (not facing) the wall (they are considered ALWAYS facing the wall)
void set_radiance_volPatches(std::vector<float> & radiance_volPatches, std::vector<Point> & radiance_volPatchesN, Scene & scene, Point & walL, Point & walN, bool normArea, bool constArea) {

	// geometry term
	float geometryTerm_;

	// gets all (facing) rad_volPatches_
	for (std::size_t i = 0; i < scene.o[VOLUME_PATCHES].s.size(); i++) {
		geometryTerm_ = geometryTerm(walL, walN, scene.o[VOLUME_PATCHES].s[i].c, radiance_volPatchesN[i]);
		if (geometryTerm_ > 0.0f)
			radiance_volPatches[i] = L_E * scene.o[VOLUME_PATCHES].s[i].albedo * geometryTerm_;
		else
			radiance_volPatches[i] = -L_E * scene.o[VOLUME_PATCHES].s[i].albedo * geometryTerm_;
	}

	// normalize with the Area of the pixel patch (if so)
	if (normArea) {
		if (constArea) {
			float area = scene.o[VOLUME_PATCHES].s[0].areaRECTANGLE();
			for (std::size_t i = 0; i < scene.o[VOLUME_PATCHES].s.size(); i++)
				radiance_volPatches[i] *= area;
		} else {
			for (std::size_t i = 0; i < scene.o[VOLUME_PATCHES].s.size(); i++)
				radiance_volPatches[i] *= scene.o[VOLUME_PATCHES].s[i].areaRECTANGLE();
	}	}
}

// For set_Occlusion_Frame(...)
// gets the Transient pixel = Impulse response of the scene. alpha_r in Ref08
// vector of maps. One map for pixel representing:
//   x axis = key   = path length (r) in m
//   y axis = value = amplitude of the impulse response
void set_TransientImage(std::vector<std::multimap<float, float>> & transientImage, std::vector<float> & radiance_volPatches_, std::vector<Point> & radiance_volPatchesN, Scene & scene, Point & walL, Point & walN) {

	// geometry_term_xw, alpha_r, r
	float geometryTerm_xw;
	float alpha_r;	// alpha_r =    L_E * albedo_w * alpha_x =    L_E * albedo_w * g(x) * v(x) =    albedo_w * L(x) * geometryTerm_xw
	float r;		// path length (r) in m, when the value alpha_r arrives to the pixel

	// gets Transient Image
	for (std::size_t wi = 0; wi < transientImage.size(); wi++) {
		std::multimap<float, float> transient_pixel;
		for (std::size_t vi = 0; vi < radiance_volPatches_.size(); vi++) {
			geometryTerm_xw = geometryTerm(scene.o[VOLUME_PATCHES].s[vi].c, radiance_volPatchesN[vi], scene.o[WALL_PATCHES].s[wi].c, walN);
			if (geometryTerm_xw > 0.0f)
				alpha_r = scene.o[WALL_PATCHES].s[wi].albedo * radiance_volPatches_[vi] * geometryTerm_xw;
			else
				alpha_r = -scene.o[WALL_PATCHES].s[wi].albedo * radiance_volPatches_[vi] * geometryTerm_xw;
			r = distPath5(scene.o[LASER].s[0].c, walL, scene.o[VOLUME_PATCHES].s[vi].c, scene.o[WALL_PATCHES].s[wi].c, scene.o[CAMERA].s[0].c);
			transient_pixel.insert(std::pair<float, float>(r, alpha_r));	// auto sorted by key
		}
		transientImage[wi] = transient_pixel;
	}
}

// For set_Occlusion_Frame(...)
// sets a Simulated Frame for the Occlusion case, from a Transient Image and a Calibration Matrix. This does NOT do any calculations
void set_FrameSim(std::vector<std::multimap<float, float>> & transientImage, CalibrationMatrix & cmx, Frame & frameSim, int freq_idx, PixStoring ps_, bool pSim_) {

	// Simulated Image Vector
	int idx;
	std::vector<float> vectorSim(numPix(ps_, pSim_), 0.0f);
	// Go pixel by pixel
	for (int r = 0; r < rows(ps_, pSim_); r++) {
		for (int c = 0; c < cols(ps_, pSim_); c++) {
			idx = rc2idx(r, c, ps_, pSim_);
			// Fill with the values of the Transient Pixel
			for (std::multimap<float, float>::iterator it = transientImage[idx].begin(); it != transientImage[idx].end(); ++it)
				vectorSim[idx] += (*it).second * cmx.C_atX(freq_idx, (*it).first, r, c, ps_, pSim_);
	}	}
	// set the new Frame Simulated
	frameSim.set(vectorSim, rows(ps_, pSim_), cols(ps_, pSim_), cmx.info->freqV[freq_idx], 0.0f, cmx.info->shutV[cmx.info->shutV.size() - 1], 0.0f, ps_, pSim_);
}




// ----------------------------------------------------------------------------------------------------------------------------------------
// ----- OTHER ----------------------------------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------------------

// Correlation with Sinusoid asumption. c(w,phi)(r) in Paper.
// frequency (MHz), phase (deg), r(ns), N (abs)
float correlation(float frequency_, float phase_, float r_, float N_) {
	return (N_/(2*frequency_)) * cos(2*PI*frequency_*r_/1000.0f + phase_*PI/180.0f);
}

// Plot a transient pixel with MATLAB Engine
void plot_transient_pixel (std::vector<std::multimap<float, float>> & transientImage, int pix_x_, int pix_y_) {
	
	// MATLAB variables
	Engine *ep;
	mxArray *T = NULL;
	mxArray *V = NULL;
	// Variables. Array structure needed to deal with MATLAB functions
	int pos_in_vector = pix_y_*CAMERA_PIX_X + pix_x_;
	std::multimap<float, float>* transient_pixel = & transientImage[pos_in_vector];	// pos_in_vector = pix_y_*CAMERA_PIX_X + pix_x_;
	int size = (*transient_pixel).size();
	double* time_ns = new double[size];
	double* value = new double[size];

	// Fill the arrays with their corresponding values
	int i = 0;
	for (std::multimap<float,float>::iterator it=(*transient_pixel).begin(); it!=(*transient_pixel).end(); ++it) {
		time_ns[i] = (double)(*it).first;
		value[i] = (double)(*it).second;
		i++;
	}

	// Call engOpen(""). This starts a MATLAB process on the current host using the command "matlab"
	if (!(ep = engOpen("")))
		fprintf(stderr, "\nCan't start MATLAB engine\n");

	// Create MATLAB variables from C++ variables
	T = mxCreateDoubleMatrix(1, size, mxREAL);
	memcpy((void *)mxGetPr(T), (void *)time_ns, size*sizeof(time_ns));
	V = mxCreateDoubleMatrix(1, size, mxREAL);
	memcpy((void *)mxGetPr(V), (void *)value, size*sizeof(value));
	
	// Place MATLAB variables into the MATLAB workspace
	engPutVariable(ep, "T", T);
	engPutVariable(ep, "V", V);

	// Plot the result
	engEvalString(ep, "stem(T,V);");
	engEvalString(ep, "title('Impulse Response: value(time)');");
	engEvalString(ep, "xlabel('time (ns)');");
	engEvalString(ep, "ylabel('value');");

	// use std::cin freeze the plot
	std::cout << "\nWrite any std::string and click ENTER to continue\n";
	std::string answer;
	std::cin >> answer;

	// Free memory, close MATLAB figure.
	mxDestroyArray(T);
	mxDestroyArray(V);
	engEvalString(ep, "close;");
	engClose(ep);
}

/*
// Plot image pixels values with MATLAB Engine
void plot_image_pixels_values(std::vector<float> & pixels_value_, int heigth_, int width_) {

	// MATLAB variables
	Engine *ep;
	mxArray *I = NULL;
	mxArray *heigth = NULL;
	mxArray *width = NULL;

	// Variables. Array structure needed to deal with MATLAB functions
	int size = pixels_value_.size();
	double* pixels_value_array = new double[size];

	// Fill the arrays with their corresponding values
	int i = 0;
	for (std::vector<float>::iterator it = pixels_value_.begin(); it != pixels_value_.end(); ++it) {
		pixels_value_array[i++] = (double)(*it);
	}

	// Call engOpen(""). This starts a MATLAB process on the current host using the command "matlab"
	if (!(ep = engOpen("")))
		fprintf(stderr, "\nCan't start MATLAB engine\n");

	// Create MATLAB variables from C++ variables
	I = mxCreateDoubleMatrix(1, size, mxREAL);
	memcpy((void *)mxGetPr(I), (void *)pixels_value_array, size*sizeof(pixels_value_array));
	heigth = mxCreateDoubleScalar(heigth_);
	width = mxCreateDoubleScalar(width_);

	// Place MATLAB variables into the MATLAB workspace
	engPutVariable(ep, "I", I);
	engPutVariable(ep, "heigth", heigth);
	engPutVariable(ep, "width", width);

	// Manage I and plot the result 
	engEvalString(ep, "I");	
	engEvalString(ep, "I = I - min(min(I));");
	engEvalString(ep, "I = I / max(max(I));");
	engEvalString(ep, "I = flipud([reshape(I, width, heigth)]');");
	engEvalString(ep, "imshow(I);");
	engEvalString(ep, "title('Image Pixels Values');");

	// use fgetc() to make sure that we pause long enough to be able to see the plot
	printf("Press ENTER to continue\n\n");
	fgetc(stdin);
	// Free memory, close MATLAB figure.
	mxDestroyArray(I);
	mxDestroyArray(heigth);
	mxDestroyArray(width);
	engEvalString(ep, "close;");
	engClose(ep);
}
*/

