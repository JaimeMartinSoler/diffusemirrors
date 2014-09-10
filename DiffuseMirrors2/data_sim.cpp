
#include <vector>
#include <map>
#include "global.h"

#include "data_sim.h"
#include "data.h"
#include "scene.h"
// MATLAB
#include "engine.h"

// gets all the data results, once the scene (OBJECT3D_SET) has been configured
void get_data_sim_diffused_mirror() {

	// PAPER ------------------------------------------------------------------------------------

	// L_E;		// Le(l) in the paper. Radiance from the light point in the wall from the laser
	
	// Radiance from each volume patch. L(x) in the paper. 
	std::cout << "\nHERE 00A: ";
	int radiance_volume_patches_size = SCENEMAIN.o[VOLUME_PATCHES].s.size();	// TO-DO
	std::cout << "\nHERE 00B";
	std::vector<float> radiance_volume_patches(radiance_volume_patches_size);
	get_radiance_volume_patches(radiance_volume_patches);
	// (Radiance from each volume patch)�(Area of each volume patch). L(x)�Area(x)
	// As long as we deal with patches instead of points, we are relating an L(x) with
	// the patch which x represents weighted with its Area.
	// If we don't weight each patch with each area, 4 wall patches of area 1.0 would give a
	// 4 times higher combined L(x) than 1 wall patch (the comb of the previous ones) of area 4.0
	// Also used in the "MYSELF" part
	std::vector<float> radiance_volume_patches_x_area(radiance_volume_patches_size);
	get_radiance_volume_patches_x_area(radiance_volume_patches_x_area, radiance_volume_patches);

	// Radiance from the wall patch. L(w) in the paper. 
	std::vector<float> radiance_wall_patches(CAMERA_PIX_X * CAMERA_PIX_Y);
	get_radiance_wall_patches(radiance_wall_patches, radiance_volume_patches);


	// MYSELF (Combined with Papers) ------------------------------------------------------------

	// Transient pixel = Impulse response of the scene. alpha_r in Ref08
	// vector of maps. One map for pixel representing
	//   x axis = key   = time (r) in ns
	//   y axis = value = amplitude of the impulse response
	std::vector<std::multimap<float, float>> transient_image(CAMERA_PIX_X * CAMERA_PIX_Y);
	get_transient_image(transient_image, radiance_volume_patches_x_area);
	
	// Pixels value. H(w,phi) in the paper
	float distance = 0.0f;		// (m)
	float frequency = 100.0f;	// (MHz)
	float phase = 0.0f;			// (deg)
	float shutter = 1920.0f;	// (us)
	float Em = 1.0f;
	std::vector<float> pixels_value(CAMERA_PIX_X * CAMERA_PIX_Y);
	get_pixels_value(pixels_value, transient_image, distance, frequency, phase, shutter, Em);
	
	// Plot a transient pixel with MATLAB Engine, from transient_image
	int pix_x = (CAMERA_PIX_X-1) / 2;	// max = CAMERA_PIX_X-1
	int pix_y = (CAMERA_PIX_Y-1) / 2;	// max = CAMERA_PIX_Y-1
	// MATLAB Engine takes too much time to start, comment out next line unless you need to debugg
	//plot_transient_pixel(transient_image, pix_x, pix_y);

	// Plot image pixels values with opencv, from pixels_value
	Frame pixels_value_frame(pixels_value, false, distance, frequency, phase, shutter);
	pixels_value_frame.plot();
}


// gets all the data results with the simple set-up, once the scene (OBJECT3D_SET) has been configured
void get_data_sim_direct_vision_wall() {

	// Transient pixel = Impulse response of the scene. alpha_r in Ref08
	// vector of maps. One map for pixel representing
	//   x axis = key   = time (r) in ns
	//   y axis = value = amplitude of the impulse response
	std::vector<std::multimap<float, float>> transient_image_simple(CAMERA_PIX_X * CAMERA_PIX_Y);
	get_transient_image_simple(transient_image_simple);

	// Pixels value. H(w,phi) in the paper
	float distance = 0.0f;		// (m)
	float frequency = 100.0f;	// (MHz)
	float phase = 0.0f;			// (deg)
	float shutter = 1920.0f;	// (us)
	float Em = 1.0f;
	std::vector<float> pixels_value_simple(CAMERA_PIX_X * CAMERA_PIX_Y);
	get_pixels_value(pixels_value_simple, transient_image_simple, distance, frequency, phase, shutter, Em);

	// Plot image pixels values with opencv, from pixels_value
	Frame pixels_value_simple_frame(pixels_value_simple, false, distance, frequency, phase, shutter);
	pixels_value_simple_frame.plot();
}




// (2014-09-08)
void set_DirectVision_Simulation_Frame (CalibrationMatrix & cmx, Scene & sceneCopy, Frame & frameSim, int freq_idx, PixStoring ps_) {
	
	// Simulated Image Vector
	float relativeAlbedo = 1.0f;
	std::vector<float> DirectVision_Simulation(numPix(ps_));
	for (int r = 0; r < rows(ps_); r++) {
		for (int c = 0; c < cols(ps_); c++) {
			DirectVision_Simulation[rc2idx(r,c,ps_)] = cmx.S_DirectVision (freq_idx, r, c,
				sceneCopy.o[LASER].s[0].c, sceneCopy.o[PIXEL_PATCHES].s[rc2idx(r,c,ps_)].c,  sceneCopy.o[CAMERA].s[0].c, relativeAlbedo, ps_);
	}	}
	frameSim.set(DirectVision_Simulation, rows(ps_), cols(ps_), cmx.info->freqV[freq_idx], 0.0f, cmx.info->shutV[cmx.info->shutV.size()-1], 0.0f, ps_);
}



// gets the Radiance from each volume patch (radiance from each volume patch). L(x) in the paper. 
// It deals with patches backing (not facing) the wall (they are considered ALWAYS facing the wall)
// TO-DO: SOLVE THIS
void get_radiance_volume_patches(std::vector<float> & radiance_volume_patches_) {
	/*
	// wall and laser ray intersection point
	Point* laser_ray_from = (*(*OBJECT3D_SET[LASER])[0]).c;			// center of laser
	Point* laser_ray_normal = (*(*OBJECT3D_SET[LASER])[0]).n[0];	// normal of laser
	PointMesh* wall_face = (*OBJECT3D_SET[WALL])[0];				// face of the wall
	Point* wall_l = &(Point(get_intersection_linePointNormal_pointmesh(laser_ray_from, laser_ray_normal, wall_face)));
	// wall normal vector
	Point* wall_n = &(*(*(*OBJECT3D_SET[WALL])[0]).n[0]);
	
	// volume patch center point
	Point* volume_patch_c;
	// volume patch normal vector
	Point* volume_patch_n;
	// volume patch albedo
	float volume_patch_albedo;

	// geometry term
	float geometry_term;

	// gets all (facing) rad_volume_patches_
	for (std::size_t iv = 0; iv < radiance_volume_patches_.size(); iv++) {
		volume_patch_c = (*(*OBJECT3D_SET[VOLUME_PATCHES])[iv]).c;
		volume_patch_n = (*(*OBJECT3D_SET[VOLUME_PATCHES])[iv]).n[0];
		volume_patch_albedo = (*(*OBJECT3D_SET[VOLUME_PATCHES])[iv]).albedo;
		geometry_term = get_geometry_term(wall_l, wall_n, volume_patch_c, volume_patch_n);
		if (geometry_term > 0.0f)
			radiance_volume_patches_[iv] = L_E * volume_patch_albedo * geometry_term;
		else
			radiance_volume_patches_[iv] = - L_E * volume_patch_albedo * geometry_term;
	}
	*/
}


// (Radiance from each volume patch)�(Area of each volume patch). L(x)�Area(x)
// As long as we deal with patches instead of points, we are relating an L(x) with
// the patch which x represents weighted with its Area.
// If we don't weight each patch with each area, 4 wall patches of area 1.0 would give a
// 4 times higher combined L(x) than 1 wall patch (the comb of the previous ones) of area 4.0
// Also used in the "MYSELF" part
void get_radiance_volume_patches_x_area(std::vector<float> & radiance_volume_patches_x_area_, std::vector<float> & radiance_volume_patches_) {
	// TO-DO
	/*
	// volume patch
	PointMesh* volume_patch;

	for (std::size_t iv = 0; iv < radiance_volume_patches_.size(); iv++) {
		volume_patch = (*OBJECT3D_SET[VOLUME_PATCHES])[iv];
		radiance_volume_patches_x_area_[iv] = radiance_volume_patches_[iv]*(*volume_patch).get_area();
	}
	*/
}


// gets the Radiance from the wall patch (radiance from each wall patch). L(w) in the paper. 
// It deals with volume patches backing (not facing) wall patches (they are considered ALWAYS facing the wall)
// TO-DO: Solve This
void get_radiance_wall_patches(std::vector<float> & radiance_wall_patches_, std::vector<float> & radiance_volume_patches_) {
	/*
	// wall patch center
	Point* wall_patch_c;
	// wall patch normal
	Point* wall_patch_n;
	// wall patch albedo
	float wall_patch_albedo;

	// volume patch center point
	Point* volume_patch_c;
	// volume patch normal vector
	Point* volume_patch_n;

	// geometry term
	float geometry_term;

	// gets all radiance_wall_patches_
	for (std::size_t iw = 0; iw < radiance_wall_patches_.size(); iw++) {
		wall_patch_c = (*(*OBJECT3D_SET[WALL_PATCHES])[iw]).c;
		wall_patch_n = (*(*OBJECT3D_SET[WALL_PATCHES])[iw]).n[0];
		wall_patch_albedo = (*(*OBJECT3D_SET[WALL_PATCHES])[iw]).albedo;
		radiance_wall_patches_[iw] = 0.0f;

		// gets the radiance for the wall patch iw
		for (std::size_t iv = 0; iv < radiance_volume_patches_.size(); iv++) {
			volume_patch_c = (*(*OBJECT3D_SET[VOLUME_PATCHES])[iv]).c;
			volume_patch_n = (*(*OBJECT3D_SET[VOLUME_PATCHES])[iv]).n[0];
			geometry_term = get_geometry_term(volume_patch_c, volume_patch_n, wall_patch_c, wall_patch_n);
			if (geometry_term > 0.0f)
				radiance_wall_patches_[iw] += radiance_volume_patches_[iv] * wall_patch_albedo * geometry_term;
			else
				radiance_wall_patches_[iw] -= radiance_volume_patches_[iv] * wall_patch_albedo * geometry_term;
		}
	}
	*/
}


// gets the Transient pixel = Impulse response of the scene. alpha_r in Ref08
// vector of maps. One map for pixel representing
//   x axis = key   = time (r) in ns
//   y axis = value = amplitude of the impulse response
void get_transient_image(std::vector<std::multimap<float, float>> & transient_image_, std::vector<float> & radiance_volume_patches_) {

	// TO-DO
	/*
	// laser center
	Point* laser_c = (*(*OBJECT3D_SET[LASER])[0]).c;				// center of laser
	// wall and laser ray intersection point
	Point* laser_ray_normal = (*(*OBJECT3D_SET[LASER])[0]).n[0];	// normal of laser
	PointMesh* wall_face = (*OBJECT3D_SET[WALL])[0];				// face of the wall
	Point* wall_l = &(Point(get_intersection_linePointNormal_pointmesh(laser_c, laser_ray_normal, wall_face)));
	// camera center
	Point* camera_c = (*(*OBJECT3D_SET[CAMERA])[0]).c;				// center of camera

	// wall patch center
	Point* wall_patch_c;
	// wall patch normal
	Point* wall_patch_n;
	// wall patch albedo
	float wall_patch_albedo;

	// volume patch center point
	Point* volume_patch_c;
	// volume patch normal vector
	Point* volume_patch_n;

	// geometry_term_xw, alpha_r, r
	float geometry_term_xw;
	float alpha_r;	// alpha_r	= L_E * albedo_w * alpha_x		=
					//			= L_E * albedo_w * g(x) * v(x)	= 
					//			= albedo_w * L(x) * geometry_term_xw
	float r;	// time (in ns) when the value alpha_r arrives to the pixel

	// gets Transient Image
	for (std::size_t iw = 0; iw < transient_image_.size(); iw++) {
		wall_patch_c = (*(*OBJECT3D_SET[WALL_PATCHES])[iw]).c;
		wall_patch_n = (*(*OBJECT3D_SET[WALL_PATCHES])[iw]).n[0];
		wall_patch_albedo = (*(*OBJECT3D_SET[WALL_PATCHES])[iw]).albedo;
		std::multimap<float, float> transient_pixel;

		for (std::size_t iv = 0; iv < radiance_volume_patches_.size(); iv++) {
			volume_patch_c = (*(*OBJECT3D_SET[VOLUME_PATCHES])[iv]).c;
			volume_patch_n = (*(*OBJECT3D_SET[VOLUME_PATCHES])[iv]).n[0];
			geometry_term_xw = get_geometry_term(volume_patch_c, volume_patch_n, wall_patch_c, wall_patch_n);
			if (geometry_term_xw > 0.0f)
				alpha_r = wall_patch_albedo * radiance_volume_patches_[iv] * geometry_term_xw;
			else
				alpha_r = - wall_patch_albedo * radiance_volume_patches_[iv] * geometry_term_xw;
			r = 1000000000.0f * dist_5(laser_c, wall_l, volume_patch_c, wall_patch_c, camera_c) / C_LIGHT_AIR;
			transient_pixel.insert(std::pair<float, float>(r, alpha_r));	// auto sorted by key
		}
		transient_image_[iw] = transient_pixel;
	}
	*/
}



// Transient pixel = Impulse response of the scene. alpha_r in Ref08
// vector of maps. One map for pixel representing
//   x axis = key   = time (r) in ns
//   y axis = value = amplitude of the impulse response
void get_transient_image_simple(std::vector<std::multimap<float, float>> & transient_image_simple_) {
	
	// TO-DO
	/*
	// laser center
	Point* laser_c = (*(*OBJECT3D_SET[LASER])[0]).c;				// center of laser
	// camera center
	Point* camera_c = (*(*OBJECT3D_SET[CAMERA])[0]).c;				// center of camera
	// wall patch center
	Point* wall_patch_c;
	// wall patch albedo
	float wall_patch_albedo;

	float r;		// time (in ns) when the value alpha_r arrives to the pixel
	float alpha_r;	// alpha_r	= L_E / (dist(laser, wall_patch))^2
	float dist_l_w;	// dist(laser, wall_patch)

	// get get_transient_image_simple
	for (std::size_t i = 0; i < transient_image_simple_.size(); i++) {

		wall_patch_c = (*(*OBJECT3D_SET[WALL_PATCHES])[i]).c;
		wall_patch_albedo = (*(*OBJECT3D_SET[WALL_PATCHES])[i]).albedo;

		dist_l_w = dist_2(laser_c, wall_patch_c);
		alpha_r = L_E / (dist_l_w * dist_l_w);
		r = 1000000000.0f * dist_3(laser_c, wall_patch_c, camera_c) / C_LIGHT_AIR;

		std::multimap<float, float> transient_pixel_simple;
		transient_pixel_simple.insert(std::pair<float, float>(r, alpha_r));
		transient_image_simple_[i] = transient_pixel_simple;
	}
	*/
}


// Pixels value. H(w,phi) in the paper
void get_pixels_value(std::vector<float> & pixels_value_, std::vector<std::multimap<float, float>> & transient_image_, float distance_, float frequency_, float phase_, float shutter_, float Em_) {

	float wave_length = C_LIGHT_AIR / (frequency_ * 1000000.0f);
	float phase_total = phase_ + 360.f * distance_ / wave_length;	// (deg)
	float N = shutter_ * frequency_;

	for (std::size_t i = 0; i < pixels_value_.size(); i++) {
		pixels_value_[i] = 0.0f;
		for (std::multimap<float, float>::iterator it = transient_image_[i].begin(); it != transient_image_[i].end(); ++it) {
			pixels_value_[i] += Em_ * (*it).second * correlation(frequency_, phase_total, (*it).first, N);
		}
	}
}


// Correlation. c(w,phi)(r) in Paper. Here defined as a function. Later should be a matrix
// frequency (MHz), phase (deg), r(ns), N (abs)
float correlation(float frequency_, float phase_, float r_, float N_) {
	return (N_/(2*frequency_)) * cos(2*PI*frequency_*r_/1000.0f + phase_*PI/180.0f);
}



// Plot a transient pixel with MATLAB Engine
void plot_transient_pixel (std::vector<std::multimap<float, float>> & transient_image_, int pix_x_, int pix_y_) {
	
	// MATLAB variables
	Engine *ep;
	mxArray *T = NULL;
	mxArray *V = NULL;
	// Variables. Array structure needed to deal with MATLAB functions
	int pos_in_vector = pix_y_*CAMERA_PIX_X + pix_x_;
	std::multimap<float, float>* transient_pixel = & transient_image_[pos_in_vector];	// pos_in_vector = pix_y_*CAMERA_PIX_X + pix_x_;
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






// ----------------------------------------------------------------
// ----------------------------------------------------------------
// THE FUNCTIONS BELOW ARE JUST TEMPORTAL AND MAY BE NON-FUNCTIONAL:
// ----------------------------------------------------------------
// ----------------------------------------------------------------


// gets all the data results with the simple set-up, once the scene (OBJECT3D_SET) has been configured
void get_data_sim_direct_vision(Frame & frame_in, Frame & frame_out) {

	// Transient pixel = Impulse response of the scene. alpha_r in Ref08
	// vector of maps. One map for pixel representing
	//   x axis = key   = time (r) in ns
	//   y axis = value = amplitude of the impulse response
	std::vector<std::multimap<float, float>> transient_image_simple;
	if (frame_in.ps == PIXELS_TOTAL)
		transient_image_simple.resize(CAMERA_PIX_X * CAMERA_PIX_Y);
	else if (frame_in.ps == PIXELS_VALID)
		transient_image_simple.resize(CAMERA_PIX_X_VALID * CAMERA_PIX_Y_VALID);
	get_transient_image_simple(transient_image_simple);

	// Pixels value. H(w,phi) in the paper
	float distance	= frame_in.dist;	// (m)
	float frequency	= frame_in.freq;	// (MHz)
	float phase		= frame_in.phas;		// (deg)
	float shutter	= frame_in.shut;		// (us)
	float Em		= 1.0f;
	std::vector<float> pixels_value_simple;
	if (frame_in.ps == PIXELS_TOTAL)
		pixels_value_simple.resize(CAMERA_PIX_X * CAMERA_PIX_Y);
	else if (frame_in.ps == PIXELS_VALID)
		pixels_value_simple.resize(CAMERA_PIX_X_VALID * CAMERA_PIX_Y_VALID);
	get_pixels_value(pixels_value_simple, transient_image_simple, distance, frequency, phase, shutter, Em);

	// Plot image pixels values with opencv, from pixels_value
	Frame pixels_value_simple_frame(pixels_value_simple, false, distance, frequency, phase, shutter);
	pixels_value_simple_frame.plot();
}



// (2014-09-08)
// This will include a minimization algorithm, but for now it will run some simulations manually and get the best fit
// is totally inefficient with this implementation, just to try the system
void updatePixelPatches_Simulation_BestFit(CalibrationMatrix & cmx, Scene & sceneCopy, Frame & frame00, Point & camC, Point & camN, Object3D & screenFoVmeasNs, PixStoring ps_) {

	// pixPatchesBestFit
	Object3D pixPatchesBestFit;
	float distMin = FLT_MAX;
	float distNow;
	Frame frameSim;

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
	for (float d = dMin; d < dMax; d += dRes) {
		// Update pixel patches
		for (int i = 0; i < sceneCopy.o[PIXEL_PATCHES].s.size(); i++) {
			sceneCopy.o[PIXEL_PATCHES].s[i].p[0].set(sceneCopy.o[CAMERA].s[0].c + screenFoVmeasNs.s[i].p[0] * d);	// Useless for meas but for rendering
			sceneCopy.o[PIXEL_PATCHES].s[i].p[1].set(sceneCopy.o[CAMERA].s[0].c + screenFoVmeasNs.s[i].p[1] * d);	// Useless for meas but for rendering
			sceneCopy.o[PIXEL_PATCHES].s[i].p[2].set(sceneCopy.o[CAMERA].s[0].c + screenFoVmeasNs.s[i].p[2] * d);	// Useless for meas but for rendering
			sceneCopy.o[PIXEL_PATCHES].s[i].p[3].set(sceneCopy.o[CAMERA].s[0].c + screenFoVmeasNs.s[i].p[3] * d);	// Useless for meas but for rendering
			sceneCopy.o[PIXEL_PATCHES].s[i].c.set   (sceneCopy.o[CAMERA].s[0].c + screenFoVmeasNs.s[i].c    * d);	// Useful for meas
		}
		// Get Simulation Frame (S)
		set_DirectVision_Simulation_Frame (cmx, sceneCopy, frameSim, freq_idx, ps_);
		// Get Distance(H,S)
		distNow = distMeasSim(frame00, frameSim);
		// Check if Distance(H,S) is better and update the pixPatchesBestFit
		if (distNow < distMin) {
			distMin = distNow;
			pixPatchesBestFit = sceneCopy.o[PIXEL_PATCHES];
	}	}
	sceneCopy.o[PIXEL_PATCHES] = pixPatchesBestFit;
}


// (2014-09-08)
float distMeasSim(Frame & H, Frame & S) {
	
	// Create the (H/S) vector
	int sizeHS = H.data.size();
	std::vector<float> HS(sizeHS);	
	for (size_t i = 0; i < sizeHS; i++)
		HS[i] = H.data[i] / S.data[i];

	// (H/S)mean0, (H/S)var0
	float mean0 = mean(HS);
	float var0  = var(HS);

	// Get rid of outliers
	int outliersSym = 8;	// it erases one min and one max outlier each time
	int min_idx, max_idx;
	for (int i = 0; i < outliersSym; i++) {
		min(HS, min_idx);
		max(HS, max_idx);
		HS[min_idx] = mean0;
		HS[max_idx] = mean0;
	}
	float var1  = var(HS);

	// dist(H,S) = [var1(H/S)^2] / var0(H/S)
	return (var1 * var1) / var0;
}


