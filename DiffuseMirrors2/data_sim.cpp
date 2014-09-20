
#include <vector>
#include <map>
#include "global.h"

#include "data_sim.h"
#include "data.h"
#include "scene.h"
// MATLAB
#include "engine.h"

#include <levmar.h>


// ----------------------------------------------------------------------------------------------------------------------------------------
// ----- DIRECT VISION --------------------------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------------------

// structure for passing user-supplied data to the objective function (and its Jacobian). Parameters of set_DirectVision_Simulation_Frame(...)
struct set_DirectVision_Simulation_Frame_STRUCT_DATA {
	// Parameters for the Scene modification
	Object3D* screenFoVmeasNs;
	// Parameters of set_DirectVision_Simulation_Frame(...)
	CalibrationMatrix * cmx;
	Scene* sceneCopy;
	Frame* frameSim00;
	Frame* frameSim90;
	int freq_idx;
	PixStoring ps_;
	bool pSim_;
};

// model to be fitted to measurements
//     p: Input parameters to be fitted. p_size: number of parameters (only distance in this first approach)
//         p[0] = dist(camC,wall)
//     x: Output values to be fitted.    x_size: number of values (pixels in this case)
//         x[i]: value of simulated pixel i
//     adata: additional data
void set_DirectVision_Simulation_Frame_Optim(float* p, float* x, int p_size, int x_size, void* adata) {
	
	// Set the data structure
	struct set_DirectVision_Simulation_Frame_STRUCT_DATA* ad = (struct set_DirectVision_Simulation_Frame_STRUCT_DATA *) adata;

	// Update pixel patches distances 
	for (int i = 0; i < ad->sceneCopy->o[PIXEL_PATCHES].s.size(); i++) {
		ad->sceneCopy->o[PIXEL_PATCHES].s[i].p[0].set(ad->sceneCopy->o[CAMERA].s[0].c + ad->screenFoVmeasNs->s[i].p[0] * p[0]);	// Useless for meas but for rendering
		ad->sceneCopy->o[PIXEL_PATCHES].s[i].p[1].set(ad->sceneCopy->o[CAMERA].s[0].c + ad->screenFoVmeasNs->s[i].p[1] * p[0]);	// Useless for meas but for rendering
		ad->sceneCopy->o[PIXEL_PATCHES].s[i].p[2].set(ad->sceneCopy->o[CAMERA].s[0].c + ad->screenFoVmeasNs->s[i].p[2] * p[0]);	// Useless for meas but for rendering
		ad->sceneCopy->o[PIXEL_PATCHES].s[i].p[3].set(ad->sceneCopy->o[CAMERA].s[0].c + ad->screenFoVmeasNs->s[i].p[3] * p[0]);	// Useless for meas but for rendering
		ad->sceneCopy->o[PIXEL_PATCHES].s[i].c.   set(ad->sceneCopy->o[CAMERA].s[0].c + ad->screenFoVmeasNs->s[i].c    * p[0]);	// Useful for meas
	}

	// get Simulation Frame (S)
	set_DirectVision_Simulation_Frame(*(ad->cmx), *(ad->sceneCopy), *(ad->frameSim00), *(ad->frameSim90), ad->freq_idx, ad->ps_, ad->pSim_);

	// store the Simulation Frame (S) into the output array x
	const int numFramePix = numPix(ad->ps_, ad->pSim_);			// rows*cols
	const int sizeofFrameData = numFramePix * sizeof(float);	// rows*cols*sizeof(float) = rows*cols*4
	memcpy(x, ad->frameSim00->data.data(), sizeofFrameData);
	memcpy(x + numFramePix, ad->frameSim90->data.data(), sizeofFrameData);
}

// This is the implementation of the BestFit using the Levenberg-Marquardt nonlinear least squares algorithms (slevmar_dif()): http://users.ics.forth.gr/~lourakis/levmar/
void updatePixelPatches_Simulation_BestFit_Optim (CalibrationMatrix & cmx, Scene & sceneCopy, Frame & frameSim00, Frame & frameSim90, Frame & frame00, Frame & frame90, Point & camC, Point & camN, Object3D & screenFoVmeasNs, PixStoring ps_, bool pSim_) {

	// get freq_idx
	int freq_idx = get_freq_idx(*(cmx.info), frame00.freq);	// returns -1 if no idx correspondance was found
	if (freq_idx < 0) {
		std::cout << "\nWarning: Frame freq = " << frame00.freq << " is not a freq in .cmx freqV = ";
		print(cmx.info->freqV);
		return;
	}

	// set the initial parameters (p) and values (x)
	const int p_size = 1;
	const int numFramePix = numPix(ps_, pSim_);					// rows*cols
	const int x_size = numFramePix * cmx.info->phasV.size();	// rows*cols*phases = rows*cols*2
	const int sizeofFrameData = numFramePix * sizeof(float);	// rows*cols*sizeof(float) = rows*cols*4
	float* p = new float[p_size];										// p[0] = dist(camC,wall)
	float* x = new float[x_size];										// x[i]: value of simulated pixel i
	p[0] = 2.0f;														// initial parameters estimate
	memcpy(x, frame00.data.data(), sizeofFrameData);					// actual measurement values to be fitted with the model
	memcpy(x + numFramePix, frame90.data.data(), sizeofFrameData);
	
	// additional data
	struct set_DirectVision_Simulation_Frame_STRUCT_DATA adata;
	adata.screenFoVmeasNs = &screenFoVmeasNs;
	// Parameters of set_DirectVision_Simulation_Frame(...)
	adata.cmx = &cmx;
	adata.sceneCopy = &sceneCopy;
	adata.frameSim00 = &frameSim00;
	adata.frameSim90 = &frameSim90;
	adata.freq_idx = freq_idx;
	adata.ps_ = ps_;
	adata.pSim_ = pSim_;

	// optimization control parameters; passing to levmar NULL instead of opts reverts to defaults
	float opts[LM_OPTS_SZ];
	opts[0] = LM_INIT_MU;
	opts[1] = 1E-15;  // 1E-15
	opts[2] = 1E-15;  // 1E-15
	opts[3] = 1E-20;  // 1E-20
	opts[4] = LM_DIFF_DELTA; // relevant only if the finite difference Jacobian version is used (not this case)

	// info parameters. Output of the optimization function about internal parameters such as number of iterations (info[5]) etc
	float info[LM_INFO_SZ];
	
	// other unused parameters (work, covar)
	float* work = NULL;
	float* covar = NULL;

	// invoke the optimization function (returns the number of iterations, -1 if failed)
	int maxIters = 1000;
	int numIters = slevmar_dif (set_DirectVision_Simulation_Frame_Optim, p, x, p_size, x_size, maxIters, opts, info, work, covar, (void*)&adata); // withOUT analytic Jacobian
	
	// Update Optimal pixel patches distances 
	for (int i = 0; i < sceneCopy.o[PIXEL_PATCHES].s.size(); i++) {
		sceneCopy.o[PIXEL_PATCHES].s[i].p[0].set(sceneCopy.o[CAMERA].s[0].c + screenFoVmeasNs.s[i].p[0] * p[0]);	// Useless for meas but for rendering
		sceneCopy.o[PIXEL_PATCHES].s[i].p[1].set(sceneCopy.o[CAMERA].s[0].c + screenFoVmeasNs.s[i].p[1] * p[0]);	// Useless for meas but for rendering
		sceneCopy.o[PIXEL_PATCHES].s[i].p[2].set(sceneCopy.o[CAMERA].s[0].c + screenFoVmeasNs.s[i].p[2] * p[0]);	// Useless for meas but for rendering
		sceneCopy.o[PIXEL_PATCHES].s[i].p[3].set(sceneCopy.o[CAMERA].s[0].c + screenFoVmeasNs.s[i].p[3] * p[0]);	// Useless for meas but for rendering
		sceneCopy.o[PIXEL_PATCHES].s[i].c.   set(sceneCopy.o[CAMERA].s[0].c + screenFoVmeasNs.s[i].c    * p[0]);	// Useful for meas
	}
	
	// clear dynamic memory
	delete[] p;
	delete[] x;

	// print results (if so)
	std::cout << "\ndist = " << p[0] << ". numIters = " << numIters;
}


// This will include a minimization algorithm, but for now it will run some simulations manually and get the best fit
// is totally inefficient with this implementation, just to try the system
void updatePixelPatches_Simulation_BestFit(CalibrationMatrix & cmx, Scene & sceneCopy, Frame & frameSim00, Frame & frameSim90, Frame & frame00, Frame & frame90, Point & camC, Point & camN, Object3D & screenFoVmeasNs, PixStoring ps_, bool pSim_) {

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
	float albedoRelRes = 0.1f;
	float albedoRelMin = 0.5f;
	float albedoRelMax = 1.5f + albedoRelRes / 2.0f;
	
	// For all distance
	for (float d = dMin; d < dMax; d += dRes) {
		// Update pixel patches distances 
		for (int i = 0; i < sceneCopy.o[PIXEL_PATCHES].s.size(); i++) {
			sceneCopy.o[PIXEL_PATCHES].s[i].p[0].set(sceneCopy.o[CAMERA].s[0].c + screenFoVmeasNs.s[i].p[0] * d);	// Useless for meas but for rendering
			sceneCopy.o[PIXEL_PATCHES].s[i].p[1].set(sceneCopy.o[CAMERA].s[0].c + screenFoVmeasNs.s[i].p[1] * d);	// Useless for meas but for rendering
			sceneCopy.o[PIXEL_PATCHES].s[i].p[2].set(sceneCopy.o[CAMERA].s[0].c + screenFoVmeasNs.s[i].p[2] * d);	// Useless for meas but for rendering
			sceneCopy.o[PIXEL_PATCHES].s[i].p[3].set(sceneCopy.o[CAMERA].s[0].c + screenFoVmeasNs.s[i].p[3] * d);	// Useless for meas but for rendering
			sceneCopy.o[PIXEL_PATCHES].s[i].c.   set(sceneCopy.o[CAMERA].s[0].c + screenFoVmeasNs.s[i].c    * d);	// Useful for meas
		}
		// For all albedoRel
		for (float albedoRel = albedoRelMin; albedoRel < albedoRelMax; albedoRel += albedoRelRes) {
			// Update pixel patches albedoRels 
			for (int i = 0; i < sceneCopy.o[PIXEL_PATCHES].s.size(); i++) {
				sceneCopy.o[PIXEL_PATCHES].s[i].albedo = albedoRel;	// Useful for meas
			}
			// get Simulation Frame (S)
			set_DirectVision_Simulation_Frame(cmx, sceneCopy, frameSim00, frameSim90, freq_idx, ps_, pSim_);
			// get Distance(H,S)
				distHS_ = distHS(frame00, frame90, frameSim00, frameSim90);
			// check if Distance(H,S) is better and update the pixPatchesBestFit
			if (distHS_ < distHSmin) {
				distHSmin = distHS_;
				pixPatchesBestFit.set(sceneCopy.o[PIXEL_PATCHES]);
	}	}	}
	sceneCopy.o[PIXEL_PATCHES].set(pixPatchesBestFit);
}

// This will include a minimization algorithm, but for now it will run some simulations manually and get the best fit
// is totally inefficient with this implementation, just to try the system. Includes Tilt
void updatePixelPatches_Simulation_BestFit_WithTilt(CalibrationMatrix & cmx, Scene & sceneCopy, Object3D & NsMod, std::vector<float> & sinAG, Frame & frameSim00, Frame & frameSim90, Frame & frame00, Frame & frame90, Point & camC, Point & camN, Object3D & screenFoVmeasNs, PixStoring ps_, bool pSim_) {

	// pixPatchesBestFit
	Object3D pixPatchesBestFit;
	float distHSmin = FLT_MAX;
	float distHS_;
	Point xCC, xP01C, xP23C;
	std::vector<float> xCen(cols(ps_, pSim_));
	std::vector<float> xP03(cols(ps_, pSim_));
	std::vector<float> xP12(cols(ps_, pSim_));
	int idx;

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
	float albedoRelRes = 0.1f;
	float albedoRelMin = 0.5f;
	float albedoRelMax = 1.5f + albedoRelRes / 2.0f;
	// for all distance
	for (float d = dMin; d < dMax; d += dRes) {
		// update pixel patches distances 
		for (int i = 0; i < sceneCopy.o[PIXEL_PATCHES].s.size(); i++) {
			sceneCopy.o[PIXEL_PATCHES].s[i].p[0].set(sceneCopy.o[CAMERA].s[0].c + screenFoVmeasNs.s[i].p[0] * d);	// Useless for meas but for rendering
			sceneCopy.o[PIXEL_PATCHES].s[i].p[1].set(sceneCopy.o[CAMERA].s[0].c + screenFoVmeasNs.s[i].p[1] * d);	// Useless for meas but for rendering
			sceneCopy.o[PIXEL_PATCHES].s[i].p[2].set(sceneCopy.o[CAMERA].s[0].c + screenFoVmeasNs.s[i].p[2] * d);	// Useless for meas but for rendering
			sceneCopy.o[PIXEL_PATCHES].s[i].p[3].set(sceneCopy.o[CAMERA].s[0].c + screenFoVmeasNs.s[i].p[3] * d);	// Useless for meas but for rendering
			sceneCopy.o[PIXEL_PATCHES].s[i].c.   set(sceneCopy.o[CAMERA].s[0].c + screenFoVmeasNs.s[i].c    * d);	// Useful for meas
		}
		// update x
		xCC = (sceneCopy.o[PIXEL_PATCHES].s[rc2idx(0, 0, ps_, pSim_)].c + sceneCopy.o[PIXEL_PATCHES].s[rc2idx(0, cols(ps_, pSim_)-1, ps_, pSim_)].c) / 2.0f;
		xP01C = (sceneCopy.o[PIXEL_PATCHES].s[rc2idx(0, 0, ps_, pSim_)].p[0] + sceneCopy.o[PIXEL_PATCHES].s[rc2idx(0, cols(ps_, pSim_)-1, ps_, pSim_)].p[1]) / 2.0f;
		xP23C = (sceneCopy.o[PIXEL_PATCHES].s[rc2idx(0, 0, ps_, pSim_)].p[3] + sceneCopy.o[PIXEL_PATCHES].s[rc2idx(0, cols(ps_, pSim_)-1, ps_, pSim_)].p[2]) / 2.0f;
		for (int c = 0; c < xCen.size() / 2; c++) {
			xCen[c] = dist(xCC, sceneCopy.o[PIXEL_PATCHES].s[rc2idx(0, c, ps_, pSim_)].c);
			xP03[c] = dist(xP01C, sceneCopy.o[PIXEL_PATCHES].s[rc2idx(0, c, ps_, pSim_)].p[0]);
			xP12[c] = dist(xP23C, sceneCopy.o[PIXEL_PATCHES].s[rc2idx(0, c, ps_, pSim_)].p[2]);
		}
		for (int c = xCen.size() / 2; c < xCen.size(); c++) {
			xCen[c] = - dist(xCC, sceneCopy.o[PIXEL_PATCHES].s[rc2idx(0, c, ps_, pSim_)].c);
			xP03[c] = - dist(xP01C, sceneCopy.o[PIXEL_PATCHES].s[rc2idx(0, c, ps_, pSim_)].p[0]);
			xP12[c] = - dist(xP23C, sceneCopy.o[PIXEL_PATCHES].s[rc2idx(0, c, ps_, pSim_)].p[2]);
		}
		// for all distance tilted
		for (size_t a = 0; a < sinAG.size(); a++) {
			// for all row 
			for (int r = 0; r < rows(ps_,pSim_); r++) {
				// for all column 
				for (int c = 0; c < cols(ps_,pSim_); c++) {
					idx = rc2idx(r, c, ps_, pSim_);
					sceneCopy.o[PIXEL_PATCHES].s[idx].p[0].set(sceneCopy.o[CAMERA].s[0].c + screenFoVmeasNs.s[idx].p[0] * (d + xP03[c] * sinAG[a] * NsMod.s[idx].p[0].x));	// Useless for meas but for rendering
					sceneCopy.o[PIXEL_PATCHES].s[idx].p[1].set(sceneCopy.o[CAMERA].s[0].c + screenFoVmeasNs.s[idx].p[1] * (d + xP12[c] * sinAG[a] * NsMod.s[idx].p[1].x));	// Useless for meas but for rendering
					sceneCopy.o[PIXEL_PATCHES].s[idx].p[2].set(sceneCopy.o[CAMERA].s[0].c + screenFoVmeasNs.s[idx].p[2] * (d + xP12[c] * sinAG[a] * NsMod.s[idx].p[2].x));	// Useless for meas but for rendering
					sceneCopy.o[PIXEL_PATCHES].s[idx].p[3].set(sceneCopy.o[CAMERA].s[0].c + screenFoVmeasNs.s[idx].p[3] * (d + xP03[c] * sinAG[a] * NsMod.s[idx].p[3].x));	// Useless for meas but for rendering
					sceneCopy.o[PIXEL_PATCHES].s[idx].c.   set(sceneCopy.o[CAMERA].s[0].c + screenFoVmeasNs.s[idx].c    * (d + xCen[c] * sinAG[a] * NsMod.s[idx].c.   x));	// Useful for meas
				}
			}
			// get Simulation Frame (S)
			set_DirectVision_Simulation_Frame(cmx, sceneCopy, frameSim00, frameSim90, freq_idx, ps_, pSim_);
			//Sleep(100);
			//frameSim.plot(1, false, "Frame Sim Tilt");
			// For all albedoRel. (For efficiency we avoid looping the FrameSim through the albedo while the FrameSim is proportional to the albedo)
			for (float albedoRel = albedoRelMin; albedoRel < albedoRelMax; albedoRel += albedoRelRes) {
				// update Frame from albedo
				mulElemToVector(albedoRel, frameSim00.data, frameSim00.data);
				mulElemToVector(albedoRel, frameSim90.data, frameSim90.data);
				// get Distance(H,S)
				distHS_ = distHS(frame00, frame90, frameSim00, frameSim90);
				// check if Distance(H,S) is better and update the pixPatchesBestFit
				if (distHS_ < distHSmin) {
					distHSmin = distHS_;
					pixPatchesBestFit.set(sceneCopy.o[PIXEL_PATCHES]);
				}
	}	}	}
	sceneCopy.o[PIXEL_PATCHES].set(pixPatchesBestFit);
}


// sets a Simulated Frame for the Direct Vision case, from a Calibration Matrix
void set_DirectVision_Simulation_Frame(CalibrationMatrix & cmx, Scene & sceneCopy, Frame & frameSim00, Frame & frameSim90, int freq_idx, PixStoring ps_, bool pSim_) {

	// Simulated Image Vector
	std::vector<float> DirectVision_Simulation00(numPix(ps_, pSim_));
	std::vector<float> DirectVision_Simulation90(numPix(ps_, pSim_));
	int sIdx;
	for (int r = 0; r < rows(ps_, pSim_); r++) {
		for (int c = 0; c < cols(ps_, pSim_); c++) {
			sIdx = rc2idx(r, c, ps_, pSim_);
			DirectVision_Simulation00[sIdx] = cmx.S_DirectVision(freq_idx, 0, r, c, sceneCopy.o[LASER].s[0].c, sceneCopy.o[PIXEL_PATCHES].s[sIdx].c, sceneCopy.o[CAMERA].s[0].c, sceneCopy.o[PIXEL_PATCHES].s[sIdx].albedo, ps_, pSim_);
			DirectVision_Simulation90[sIdx] = cmx.S_DirectVision(freq_idx, 1, r, c, sceneCopy.o[LASER].s[0].c, sceneCopy.o[PIXEL_PATCHES].s[sIdx].c, sceneCopy.o[CAMERA].s[0].c, sceneCopy.o[PIXEL_PATCHES].s[sIdx].albedo, ps_, pSim_);
	}	}
	frameSim00.set(DirectVision_Simulation00, rows(ps_, pSim_), cols(ps_, pSim_), cmx.info->freqV[freq_idx], 0.0f, cmx.info->shutV[cmx.info->shutV.size() - 1], cmx.info->phasV[0], ps_, pSim_);
	frameSim90.set(DirectVision_Simulation90, rows(ps_, pSim_), cols(ps_, pSim_), cmx.info->freqV[freq_idx], 0.0f, cmx.info->shutV[cmx.info->shutV.size() - 1], cmx.info->phasV[cmx.info->phasV.size() - 1], ps_, pSim_);
}


// dist(H,S) = Sum{(Hi-Si)^2}. Distance (Measurement, Simulation) function. Actually, sed for both Direct Vision and Occlusion problems
float distHS(Frame & H00, Frame & H90, Frame & S00, Frame & S90) {
	
	// dist(H,S) = Sum{(Hi-Si)^2}
	float distResult = 0.0f;
	float H00_S00_diffPow2;
	float H90_S90_diffPow2;
	for (size_t i = 0; i < H00.data.size(); i++) {
		H00_S00_diffPow2 = H00.data[i] - S00.data[i];
		H90_S90_diffPow2 = H90.data[i] - S90.data[i];
		distResult += ((H00_S00_diffPow2 * H00_S00_diffPow2) + (H90_S90_diffPow2 * H90_S90_diffPow2));
	}
	return distResult;
	//return distResult * H.data.size();	// to normalize the result to the number of pixels
	
	/*
	// dist(H,S) = Sum{(Hi-Si)^2}
	std::vector<float> H_S(H.data.size());
	subPow2VectorToVector(H.data, S.data, H_S);

	// erase some maximums (possible outliers)
	int eraseMaxNum = H.data.size() / 10; 
	int eraseIdx;
	for (int i = 0; i < eraseMaxNum; i++) {
		max(H_S, eraseIdx);
		H_S.erase(H_S.begin() + eraseIdx);
	}

	return sum(H_S);
	*/
}




// ----------------------------------------------------------------------------------------------------------------------------------------
// ----- OCCLUSION ------------------------------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------------------

// This will include a minimization algorithm, but for now it will run some simulations manually and get the best fit
// is totally inefficient with this implementation, just to try the system
void updateVolumePatches_Occlusion_BestFit(CalibrationMatrix & cmx, Scene & sceneCopy, Object3D volPatchesCopy, Frame & frameSim00, Frame & frameSim90, Frame & frame00, Frame & frame90, Point & walN, Point & _vopN, float dRes, PixStoring ps_, bool pSim_) {

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
	/*
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
		set_Occlusion_Simulation_Frame(cmx, sceneCopy, frame00, frameSim, walN, freq_idx, ps_, pSim_);
		distHS_ = distHS(frame00, frameSim);
		frameSim.plot(1, false, "Frame Sim Occ");
		// check if Distance(H,S) is better and update the pixPatchesBestFit
		if (distHS_ < distHSmin) {
			distHSmin = distHS_;
			volPatchesBestFit = sceneCopy.o[PIXEL_PATCHES];
		}
		sceneCopy.o[VOLUME_PATCHES].s[0].tra(_vopN * pixPatchDist[0]);
		pixPatchDist[pos=0] += dRes;
		// Combination Iteration:
		while (pixPatchDist[pos] > dMax) {
			if (pos >= pixPatchDist.size() - 1) {
				finishLoop = true;
				break;
			}
			pixPatchDist[pos] = dMin;
			sceneCopy.o[VOLUME_PATCHES].s[pos] = volPatchesCopy.s[pos];
			pixPatchDist[++pos] += dRes;
			sceneCopy.o[VOLUME_PATCHES].s[pos].tra(_vopN * pixPatchDist[pos]);
		}
	}
	*/

	// Distances Iterator: te set up is discretized by changing the distances of the volume patches (all together) and the albedoRels. dRes = 0.05f in updateVolumePatches_Occlusion(...)
	float dMin = -1.0f;
	float dMax = 1.0f + dRes / 2.0f;
	Point traV;
	float albedoRelRes = 0.1f;
	float albedoRelMin = 0.5f;
	float albedoRelMax = 1.5f + albedoRelRes / 2.0f;
	// For all distance
	for (float d = dMin; d < dMax; d += dRes) {
		//Sleep(100);
		//frameSim00.plot(1, false, "Frame Sim00 Occ");
		//frameSim90.plot(1, false, "Frame Sim90 Occ");
		// Update pixel patches distances 
		traV = _vopN * d;
		for (int i = 0; i < sceneCopy.o[VOLUME_PATCHES].s.size(); i++) {
			sceneCopy.o[VOLUME_PATCHES].s[i].p[0] = volPatchesCopy.s[i].p[0] + traV;	// Useless for meas but for rendering
			sceneCopy.o[VOLUME_PATCHES].s[i].p[1] = volPatchesCopy.s[i].p[1] + traV;	// Useless for meas but for rendering
			sceneCopy.o[VOLUME_PATCHES].s[i].p[2] = volPatchesCopy.s[i].p[2] + traV;	// Useless for meas but for rendering
			sceneCopy.o[VOLUME_PATCHES].s[i].p[3] = volPatchesCopy.s[i].p[3] + traV;	// Useless for meas but for rendering
			sceneCopy.o[VOLUME_PATCHES].s[i].c    = volPatchesCopy.s[i].c    + traV;	// Useful for meas
		}
		// For all albedoRel
		for (float albedoRel = albedoRelMin; albedoRel < albedoRelMax; albedoRel += albedoRelRes) {
			// Update pixel patches albedoRels 
			for (int i = 0; i < sceneCopy.o[VOLUME_PATCHES].s.size(); i++)
				sceneCopy.o[VOLUME_PATCHES].s[i].albedo = albedoRel;	// Useful for meas
			// get Simulation Frame (S)
			set_Occlusion_Simulation_Frame(cmx, sceneCopy, frameSim00, frameSim90, walN, freq_idx, ps_, pSim_);
			// get Distance(H,S)
			distHS_ = distHS(frame00, frame90, frameSim00, frameSim90);
			// check if Distance(H,S) is better and update the pixPatchesBestFit
			if (distHS_ < distHSmin) {
				distHSmin = distHS_;
				volPatchesBestFit.set(sceneCopy.o[VOLUME_PATCHES]);
	}	}	}

	sceneCopy.o[VOLUME_PATCHES].set(volPatchesBestFit);
}

// sets a Simulated Frame for the Occlusion case, from a Transient Image and a Calibration Matrix. This does all the calculations
void set_Occlusion_Simulation_Frame(CalibrationMatrix & cmx, Scene & scene, Frame & frameSim00, Frame & frameSim90, Point & walN, int freq_idx, PixStoring ps_, bool pSim_) {
	
	// L_E;		// Le(l) in the paper. Radiance from the light point in the wall from the laser

	// Radiance from each volume patch. L(x) in the paper. 
	int radiance_volPatches_size = SCENEMAIN.o[VOLUME_PATCHES].s.size();
	// set volPatches Normals
	std::vector<Point> radiance_volPatchesN(radiance_volPatches_size);
	for (std::size_t i = 0; i < scene.o[VOLUME_PATCHES].s.size(); i++)
		radiance_volPatchesN[i] = scene.o[VOLUME_PATCHES].s[i].normalQUAD();
	std::vector<float> radiance_volPatches(radiance_volPatches_size);
	bool normArea = true;	// normalize rediance to the volume path area. Useful to check how changes the result a change on the volume patches
	bool constArea = true;	// if the area of each volume patch the same (constant) or not
	set_radiance_volPatches(radiance_volPatches, radiance_volPatchesN, scene, scene.o[LASER_RAY].s[0].p[1], walN, normArea, constArea);

	// Transient pixel = Impulse response of the scene. alpha_r in Ref08
	// vector of maps. One map for pixel representing:
	//   x axis = key   = path length (r) in m
	//   y axis = value = amplitude of the impulse response
	std::vector<std::multimap<float, float>> transientImage(numPix(ps_, pSim_));
	set_TransientImage(transientImage, radiance_volPatches, radiance_volPatchesN, scene, scene.o[LASER_RAY].s[0].p[1], walN);

	// Pixels value. H(w,phi) in the paper
	set_FrameSim(transientImage, cmx, frameSim00, frameSim90, freq_idx, ps_, pSim_);


	// Plot a transient pixel with MATLAB Engine, from TransientImage
	/*
	int pixRow = frameReal.rows / 2;
	int pixCol = frameReal.cols / 2;
	// MATLAB Engine takes too much time to start, comment out next line unless you need to debugg
	plot_transientPixel(transientImage[rc2idx(pixRow, pixCol, ps_, pSim_)]);
	*/
}

// For set_Occlusion_Simulation_Frame(...)
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

// For set_Occlusion_Simulation_Frame(...)
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

// For set_Occlusion_Simulation_Frame(...)
// sets a Simulated Frame for the Occlusion case, from a Transient Image and a Calibration Matrix. This does NOT do any calculations
void set_FrameSim(std::vector<std::multimap<float, float>> & transientImage, CalibrationMatrix & cmx, Frame & frameSim00, Frame & frameSim90, int freq_idx, PixStoring ps_, bool pSim_) {

	// Simulated Image Vector
	int idx;
	std::vector<float> vectorSim00(numPix(ps_, pSim_), 0.0f);
	std::vector<float> vectorSim90(numPix(ps_, pSim_), 0.0f);
	// Go pixel by pixel
	for (int r = 0; r < rows(ps_, pSim_); r++) {
		for (int c = 0; c < cols(ps_, pSim_); c++) {
			idx = rc2idx(r, c, ps_, pSim_);
			// Fill with the values of the Transient Pixel
			for (std::multimap<float, float>::iterator it = transientImage[idx].begin(); it != transientImage[idx].end(); ++it) {
				vectorSim00[idx] += (*it).second * cmx.C_atX(freq_idx, (*it).first, 0, r, c, ps_, pSim_);
				vectorSim90[idx] += (*it).second * cmx.C_atX(freq_idx, (*it).first, 1, r, c, ps_, pSim_);
			}
	}	}
	// set the new Frame Simulated
	frameSim00.set(vectorSim00, rows(ps_, pSim_), cols(ps_, pSim_), cmx.info->freqV[freq_idx], 0.0f, cmx.info->shutV[cmx.info->shutV.size() - 1], cmx.info->phasV[0], ps_, pSim_);
	frameSim90.set(vectorSim90, rows(ps_, pSim_), cols(ps_, pSim_), cmx.info->freqV[freq_idx], 0.0f, cmx.info->shutV[cmx.info->shutV.size() - 1], cmx.info->phasV[cmx.info->phasV.size() - 1], ps_, pSim_);
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
void plot_transientPixel (std::multimap<float, float> & transientPixel) {
	
	// MATLAB variables
	Engine *ep;
	mxArray *D = NULL;
	mxArray *V = NULL;
	// Variables. Array structure needed to deal with MATLAB functions
	int size = transientPixel.size();
	double* dist_m = new double[size];
	double* value = new double[size];

	// Fill the arrays with their corresponding values
	int i = 0;
	// Fill with the values of the Transient Pixel
	for (std::multimap<float, float>::iterator it = transientPixel.begin(); it != transientPixel.end(); ++it) {
		dist_m[i] = (double)it->first;
		value[i] = (double)it->second;
		i++;
	}

	// Call engOpen(""). This starts a MATLAB process on the current host using the command "matlab"
	if (!(ep = engOpen("")))
		fprintf(stderr, "\nCan't start MATLAB engine\n");

	// Create MATLAB variables from C++ variables
	D = mxCreateDoubleMatrix(1, size, mxREAL);
	memcpy((void *)mxGetPr(D), (void *)dist_m, size*sizeof(dist_m));
	V = mxCreateDoubleMatrix(1, size, mxREAL);
	memcpy((void *)mxGetPr(V), (void *)value, size*sizeof(value));
	
	// Place MATLAB variables into the MATLAB workspace
	engPutVariable(ep, "D", D);
	engPutVariable(ep, "V", V);

	// Plot the result
	engEvalString(ep, "stem(D,V);");
	engEvalString(ep, "title('Impulse Response: value(time)');");
	engEvalString(ep, "xlabel('distance (m)');");
	engEvalString(ep, "ylabel('value');");

	// use std::cin freeze the plot
	std::cout << "\nWrite any std::string and click ENTER to continue\n";
	std::string answer;
	std::cin >> answer;

	// Free memory, close MATLAB figure.
	mxDestroyArray(D);
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

