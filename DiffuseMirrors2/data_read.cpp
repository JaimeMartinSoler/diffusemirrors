
#include <iostream> 
#include <stdlib.h>     // atof

#include "global.h"
#include "data_read.h"
#include "data_sim.h"
#include "capturetoolDM2.h"
#include "scene.h"

#include <math.h>		// round, fmodf

#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>	

// ----- INFO -----------------------------------------------------------------------------------------------------------------------------
// Constructor
Info::Info(char* dir_name_, char* file_name_) {
	
	// External Parameters (file names):
	dir_name = dir_name_;
	file_name = file_name_;
	char inf_full_file_name_[1024];
	char raw_full_file_name_[1024];
	char cmx_full_file_name_[1024];
	char cmd_full_file_name_[1024];
	sprintf(inf_full_file_name_,"%s\\%s%s", dir_name_, file_name_, INF_FILENAME_SUFFIX);
	sprintf(raw_full_file_name_,"%s\\%s%s", dir_name_, file_name_, RAW_FILENAME_SUFFIX);
	sprintf(cmx_full_file_name_,"%s\\%s%s", dir_name_, file_name_, CMX_FILENAME_SUFFIX);
	sprintf(cmd_full_file_name_,"%s\\%s%s", dir_name_, file_name_, CMD_FILENAME_SUFFIX);
	inf_full_file_name = inf_full_file_name_;
	raw_full_file_name = raw_full_file_name_;
	cmx_full_file_name = cmx_full_file_name_;
	cmd_full_file_name = cmd_full_file_name_;

	// INFO FILE. Open with read permissions
	FILE* inf_file = fopen(inf_full_file_name, "r");
	if (inf_file == NULL) {
		std::cout << "\n\nError Reading \""<< inf_full_file_name << "\"\n\n";
		error_code = 1;
		return;
	}
	int line_number = 0;
	char delimiter = ':';
	char inf_file_line[8192];	// this is aprox a max of 1600 "values" per line (considering 5 bytes/chars per value, such as "100.0")
	
	// Default Calibration Matrix parameters (the info file could not have info about them if they raw file was not built as Calibration Matrix):
	sizeof_value_cmx = -1;
	laser_to_cam_offset_x = -1.0f;
	laser_to_cam_offset_y = -1.0f;
	laser_to_cam_offset_z = -1.0f;
	dist_wall_cam = -1.0f;
	// Explore line by line, getting the parameters
	while (fgets(inf_file_line, 8192, inf_file)) {
		//std::cout << inf_file_line;
		
		// bytes_per_value
		if (line_number == 3) {
			std::vector<float> sizeof_value_raw_vector;
			char_array_to_float_vector_from_delimiter (inf_file_line, sizeof_value_raw_vector, delimiter);
			sizeof_value_raw = (int)sizeof_value_raw_vector[0];
		}
		// width and heigth
		else if (line_number == 4) {
			std::vector<float> width_and_heigth;
			char_array_to_float_vector_from_delimiter (inf_file_line, width_and_heigth, delimiter);
			width = (int)width_and_heigth[0];
			heigth = (int)width_and_heigth[1];
		}
		// frequencies
		else if (line_number == 5)
			char_array_to_float_vector_from_delimiter (inf_file_line, frequencies, delimiter);
		// distances
		else if (line_number == 6)
			char_array_to_float_vector_from_delimiter (inf_file_line, distances, delimiter);
		// shutters_
		else if (line_number == 7)
			char_array_to_float_vector_from_delimiter (inf_file_line, shutters, delimiter);
		// phases
		else if (line_number == 8)
			char_array_to_float_vector_from_delimiter (inf_file_line, phases, delimiter);
		// numtakes
		else if (line_number == 9) {
			std::vector<float> numtakes_vector;
			char_array_to_float_vector_from_delimiter (inf_file_line, numtakes_vector, delimiter);
			numtakes = (int)numtakes_vector[0];
		}

		// Calibration Matrix parameters:
		// bytes_per_value_cmx;
		else if (line_number == 12) {
			std::vector<float> sizeof_value_cmx_vector;
			char_array_to_float_vector_from_delimiter (inf_file_line, sizeof_value_cmx_vector, delimiter);
			sizeof_value_cmx = (int)sizeof_value_cmx_vector[0];
		}
		// laser_to_cam_offset_x, laser_to_cam_offset_y, laser_to_cam_offset_z;
		else if (line_number == 13) {
			std::vector<float> laser_to_cam_offset_vector;
			char_array_to_float_vector_from_delimiter (inf_file_line, laser_to_cam_offset_vector, delimiter);
			laser_to_cam_offset_x = laser_to_cam_offset_vector[0];
			laser_to_cam_offset_y = laser_to_cam_offset_vector[1];
			laser_to_cam_offset_z = laser_to_cam_offset_vector[2];
		}
		// dist_wall_cam
		else if (line_number == 14) {
			std::vector<float> dist_wall_cam_vector;
			char_array_to_float_vector_from_delimiter (inf_file_line, dist_wall_cam_vector, delimiter);
			dist_wall_cam = dist_wall_cam_vector[0];
		}

		line_number++;
	}
	//std::cout << "\n";
	fclose(inf_file);
	error_code = 0;	// no errors
}
// Constructor
Info::Info(	char* dir_name_, char* file_name_, int sizeof_value_raw_, int width_, int heigth_,
			std::vector<float> & frequencies_, std::vector<float> & distances_, std::vector<float> & shutters_, std::vector<float> & phases_, int numtakes_, int error_code_,
			int sizeof_value_cmx_, float laser_to_cam_offset_x_, float laser_to_cam_offset_y_, float laser_to_cam_offset_z_, float dist_wall_cam_) {

	// External Parameters (file names):
	dir_name = dir_name_;
	file_name = file_name_;
	if ((dir_name_ != NULL) && (file_name_ != NULL)) {
		char inf_full_file_name_[1024];
		char raw_full_file_name_[1024];
		char cmx_full_file_name_[1024];
		char cmd_full_file_name_[1024];
		sprintf(inf_full_file_name_,"%s\\%s%s", dir_name_, file_name_, INF_FILENAME_SUFFIX);
		sprintf(raw_full_file_name_,"%s\\%s%s", dir_name_, file_name_, RAW_FILENAME_SUFFIX);
		sprintf(cmx_full_file_name_,"%s\\%s%s", dir_name_, file_name_, CMX_FILENAME_SUFFIX);
		sprintf(cmd_full_file_name_,"%s\\%s%s", dir_name_, file_name_, CMD_FILENAME_SUFFIX);
		inf_full_file_name = inf_full_file_name_;
		raw_full_file_name = raw_full_file_name_;
		cmx_full_file_name = cmx_full_file_name_;
		cmd_full_file_name = cmd_full_file_name_;
	}
	else {
		inf_full_file_name = NULL;
		raw_full_file_name = NULL;
		cmx_full_file_name = NULL;
		cmd_full_file_name = NULL;
	}

	// Info Parameters:
	sizeof_value_raw = sizeof_value_raw_;
	width = width_;
	heigth = heigth_;
	frequencies = frequencies_;
	distances = distances_;
	shutters = shutters_;
	phases = phases_;
	numtakes = numtakes_;
	// Calibration Matrix parameters:
	sizeof_value_cmx = sizeof_value_cmx_;
	laser_to_cam_offset_x = laser_to_cam_offset_x_;
	laser_to_cam_offset_y = laser_to_cam_offset_y_;
	laser_to_cam_offset_z = laser_to_cam_offset_z_;
	dist_wall_cam = dist_wall_cam_;

	error_code = error_code_;

}
// Constructor Default
Info::Info() {

	// External Parameters (file names):
	dir_name = NULL;
	file_name = NULL;
	inf_full_file_name = NULL;
	raw_full_file_name = NULL;
	cmx_full_file_name = NULL;
	cmd_full_file_name = NULL;

	// Info Parameters:
	sizeof_value_raw = 0;
	width = 0;
	heigth = 0;
	frequencies = std::vector<float>(0);;
	distances = std::vector<float>(0);;
	shutters = std::vector<float>(0);;
	phases = std::vector<float>(0);;
	numtakes = 0;
	// Calibration Matrix parameters:
	sizeof_value_cmx = 0;
	laser_to_cam_offset_x = 0.0f;
	laser_to_cam_offset_y = 0.0f;
	laser_to_cam_offset_z = 0.0f;
	dist_wall_cam = 0.0f;

	error_code = 0;
}


// ----- RAW DATA -------------------------------------------------------------------------------------------------------------------------
// Constructor
RawData::RawData(Info* info_) {

	// Info object pointer
	info = info_;

	int file_data_size_expected = (*info).frequencies.size() * (*info).distances.size() * (*info).shutters.size() * (*info).phases.size() * (*info).width * (*info).heigth;

	// DATA FILE. Open with read permissions
	FILE* raw_file = fopen((*info).raw_full_file_name, "rb");	// open in binary/raw mode
	if (raw_file == NULL) {
		std::cout << "\n\nError Reading \""<< (*info).raw_full_file_name << "\"\n\n";
		error_code = 1;
		return;
	}
	size_t fread_output_size;

	// get file_data_size
	fseek (raw_file , 0 , SEEK_END);
	data_size = ftell (raw_file) / (*info).sizeof_value_raw;
	rewind (raw_file);
	if (data_size != file_data_size_expected) {
		std::cout << "\n\nSize Incoherence Error while getting size of \""<< (*info).raw_full_file_name << "\"\n\n";
		error_code = 4;
		return;
	}

	// allocate memory to contain the whole file
	data = (unsigned short int*) malloc((*info).sizeof_value_raw*data_size);
	if (data == NULL) {
		std::cout << "\n\nMemory Error while allocating \""<< (*info).raw_full_file_name << "\"\n\n";
		error_code = 2;
		return;
	}

	// copy the file into the buffer:
	fread_output_size = fread (data, (*info).sizeof_value_raw, data_size, raw_file);
	if (fread_output_size != data_size) {
		std::cout << "\n\nSize Error while reading \""<< (*info).raw_full_file_name << "\"\n\n";
		error_code = 3;
		return;
	}

	fclose (raw_file);
	//free (data_size_);
	error_code = 0;		// no errors

}
// Constructor
RawData::RawData(Info* info_, unsigned short int* data_, int data_size_, int error_code_) {
	
	// External Parameters
	info = info_;

	// RawData Parameters
	data = data_;
	data_size = data_size_;	
	error_code = error_code_;
}
// Constructor Default
RawData::RawData() {
	
	// External Parameters
	info = NULL;

	// RawData Parameters
	data = NULL;
	data_size = 0;	
	error_code = 0;
}

// Returns the index in data[] corresponding to the parameter indices
int RawData::idx_in_data(int distances_idx, int frequencies_idx, int shutters_idx, int w, int h, int phases_idx) {
	return info->frequencies.size() * info->phases.size() * info->shutters.size() * info->heigth * info->width * distances_idx   +
	                                  info->phases.size() * info->shutters.size() * info->heigth * info->width * frequencies_idx +
	                                                        info->shutters.size() * info->heigth * info->width * phases_idx      +
	                                                                                info->heigth * info->width * shutters_idx    +
	                                                                                               info->width * h               +
	                                                                                                             w;
}

// Returns the value corresponding to the parameter indices = data[idx_in_data]
unsigned short int RawData::at(int distances_idx, int frequencies_idx, int shutters_idx, int w, int h, int phases_idx) {
	return data[idx_in_data(distances_idx, frequencies_idx, shutters_idx, w, h, phases_idx)];
}


// ----- CALIBRATION MATRIX ---------------------------------------------------------------------------------------------------------------
// Constructor. It creates a CalibrationMatrix object from the .cmx file noted in the info object
CalibrationMatrix::CalibrationMatrix(Info* info_, Pixels_storing pixels_storing_) { // by default: pixels_storing_ = PIXELS_VALID
	
	// External Parameters (RawData, Info)
	info = info_;
	RawData_src = NULL;

	// Calibration Matrix Parameters: data, data_size
	int file_data_size_expected = (*info).frequencies.size() * (*info).distances.size() * (*info).width * (*info).heigth;
	// DATA FILE. Open with read permissions
	FILE* cmx_file = fopen((*info).cmx_full_file_name, "rb");	// open in binary/raw mode
	if (cmx_file == NULL) {
		std::cout << "\n\nError Reading \""<< (*info).cmx_full_file_name << "\"\n\n";
		error_code = 1;
		return;
	}
	size_t fread_output_size;
	// get file_data_size
	fseek (cmx_file , 0 , SEEK_END);
	data_size = ftell (cmx_file) / (*info).sizeof_value_cmx;
	rewind (cmx_file);
	if (data_size != file_data_size_expected) {
		std::cout << "\n\nSize Incoherence Error while getting size of \""<< (*info).cmx_full_file_name << "\"\n\n";
		error_code = 4;
		return;
	}
	// allocate memory to contain the whole file
	data = (float*) malloc((*info).sizeof_value_cmx*data_size);
	if (data == NULL) {
		std::cout << "\n\nMemory Error while allocating \""<< (*info).cmx_full_file_name << "\"\n\n";
		error_code = 2;
		return;
	}
	// copy the file into the buffer:
	fread_output_size = fread (data, (*info).sizeof_value_cmx, data_size, cmx_file);
	if (fread_output_size != data_size) {
		std::cout << "\n\nSize Error while reading \""<< (*info).cmx_full_file_name << "\"\n\n";
		error_code = 3;
		return;
	}
	fclose (cmx_file);
	//free (data_size_);

	// Calibration Matrix Parameters: pixels_storing, width, heigth
	pixels_storing = pixels_storing_;
	if (pixels_storing_ == PIXELS_TOTAL) {
		width = info->width;
		heigth = info->heigth;
	}
	else if (pixels_storing_ == PIXELS_VALID) {
		width = CAMERA_PIX_X_VALID;
		heigth = CAMERA_PIX_Y_VALID;
	}

	// Calibration Matrix Parameters: path_dist_0
	std::vector<float> dist_laser_rc, dist_cam_rc;			// these dists are ordered as WALL_PATCHES and it is, by rows, from down to top, we want it from top to down
	set_scene_calibration_matrix (info, pixels_storing_);	// set the corresponding scene (camera, laser, wall and wall_patches)
	dist_2_centers( (*(*OBJECT3D_SET[LASER])[0]).c , (*OBJECT3D_SET[WALL_PATCHES]), dist_laser_rc);
	dist_2_centers( (*(*OBJECT3D_SET[CAMERA])[0]).c, (*OBJECT3D_SET[WALL_PATCHES]), dist_cam_rc);
	clear_scene();										// clear scene
	// path_dist_0
	path_dist_0 = cv::Mat(heigth, width, cv::DataType<float>::type);
	for (int h = 0; h < heigth; h++) {
			for (int w = 0; w < width; w++) {
				int pos_in_dists = (heigth-1-h)*width + w;	// these dists are ordered as WALL_PATCHES and it is, by rows, from down to top, we want it from top to down
				path_dist_0.at<float>(h, w) = dist_laser_rc[pos_in_dists] + dist_cam_rc[pos_in_dists];
	}	}

	error_code = 0;		// no errors
}
// Constructor
CalibrationMatrix::CalibrationMatrix(Info* info_, RawData* RawData_src_, float* data_, int data_size_, cv::Mat & path_dist_0_, Pixels_storing pixels_storing_, int width_, int heigth_, int error_code_) { // by default: error_code_ = 0
	
	// External Parameters (RawData, Info)
	info = info_;
	RawData_src = RawData_src_;

	// Calibration Matrix Parameters
	data = data_;
	data_size = data_size_;
	path_dist_0 = cv::Mat(path_dist_0_);
	pixels_storing = pixels_storing_;
	width = width_;
	heigth = heigth_;
	error_code = error_code_;

}
// Constructor Default
CalibrationMatrix::CalibrationMatrix() {

	// External Parameters (RawData, Info)
	info = NULL;
	RawData_src = NULL;

	// Calibration Matrix Parameters
	data = NULL;
	data_size = 0;
	path_dist_0 = cv::Mat(0, 0, cv::DataType<float>::type);
	pixels_storing = UNKNOWN_PIXELS_STORING;
	width = 0;
	heigth = 0;
	error_code = 0;
}

// Returns the index in data[], corresponding to the parameter indices. Takes care of the pixels_storing internally
int CalibrationMatrix::idx_in_data(int frequencies_idx, int distances_idx, int w, int h) {

	if (pixels_storing = PIXELS_TOTAL) {
		return info->distances.size() * info->heigth * info->width * frequencies_idx +
	                                    info->heigth * info->width * distances_idx   +
	                                                   info->width * h               + w;
	} else if (pixels_storing = PIXELS_VALID) {
		return info->distances.size() * info->heigth * info->width * frequencies_idx            +
	                                    info->heigth * info->width * distances_idx              +
	                                                   info->width * (h + CAMERA_PIX_Y_BAD_TOP) +
													                 (w + CAMERA_PIX_X_BAD_LEFT);
	}
}

// Returns the value corresponding to the parameter indices = data[idx_in_data].
float CalibrationMatrix::at(int frequencies_idx, int distances_idx, int w, int h) {
	return data[idx_in_data(frequencies_idx, distances_idx, w, h)];
}

// Returns the value at any distance interpolating with the closest distances. Distance have to be equidistant
float CalibrationMatrix::at_any_path_dist(int frequencies_idx, float path_dist, int w, int h) {

	float dist_offset = path_dist - path_dist_0.at<float>(h,w);
	float dist_res = (info->distances[1] - info->distances[0]);
	// dist_idx
	int dist_idx_floor = (dist_offset - info->distances[0]) / dist_res + 0.5f;	// + 0.5f, to let int truncate properly
	int dist_idx_ceil = dist_idx_floor + 1;
	// dist_scales
	float dist_scale_ceil = fmodf(dist_offset, dist_res) / dist_res;
	float dist_scale_floor = 1.0f - dist_scale_ceil;

	return (dist_scale_floor * at(frequencies_idx, dist_idx_floor, w, h)) + (dist_scale_ceil * at(frequencies_idx, dist_idx_ceil, w, h));

}



// ----- FRAME ----------------------------------------------------------------------------------------------------------------------------
// Constructor from RawData oriented
Frame::Frame(Info* info_, RawData* RawData_src_, int distance_idx_, int frequency_idx_, int shutter_idx_, int phase_idx_, Pixels_storing pixels_storing_) {
	
	// External Parameters (RawData, Info)
	Info* info = info_;
	RawData_src = RawData_src_;
	frequency_idx = frequency_idx_;
	distance_idx = distance_idx_;
	shutter_idx = shutter_idx_;
	phase_idx = phase_idx_;
	
	// Frame Parameters
	pixels_storing = pixels_storing_;
	if (pixels_storing_ == PIXELS_TOTAL) {
		width = info->width;
		heigth = info->heigth;
	}
	else if (pixels_storing_ == PIXELS_VALID) {
		width = CAMERA_PIX_X_VALID;
		heigth = CAMERA_PIX_Y_VALID;
	}
	frequency = info->frequencies[frequency_idx_];
	distance = info->distances[distance_idx_];
	shutter = info->shutters[shutter_idx_];
	phase = info->phases[phase_idx_];
	matrix = cv::Mat(heigth, width, cv::DataType<float>::type);
	if (pixels_storing_ == PIXELS_TOTAL) {
		for (int h = 0; h < heigth; h++) {
			for (int w = 0; w < width; w++) {
				// data is not stored properly. -32768 fixes it thanks to short int over-run
				// by default image is up-down-fliped so heigth_RawData = (heigth_Frame-1-h)
				matrix.at<float>(h, w) = (float)(RawData_src->at(distance_idx_, frequency_idx_, shutter_idx_, w, heigth - 1 - h, phase_idx_) - 32768);
	}	}	}
	else if (pixels_storing_ == PIXELS_VALID) {
		for (int h = 0; h < heigth; h++) {
			for (int w = 0; w < width; w++) {
				// Also, take into account that RawData only stores Pixels Total
				matrix.at<float>(h, w) = (float)(RawData_src->at(distance_idx_, frequency_idx_, shutter_idx_, w + CAMERA_PIX_X_BAD_LEFT, CAMERA_PIX_Y - CAMERA_PIX_Y_BAD_TOP - 1 - h, phase_idx_) - 32768);
	}	}	}
}
// Constructor from vector. Simulation oriented. For any Pixels_storing it consideres the vector matrix_vector properly arranged
Frame::Frame(std::vector<float> & matrix_vector, int heigth_, int width_, bool rows_up2down, float distance_, float frequency_, float shutter_, float phase_, Pixels_storing pixels_storing_) {
	
	// External Parameters (RawData, Info)
	RawData_src = NULL;
	info = NULL;
	frequency_idx = 0;
	distance_idx = 0;
	shutter_idx = 0;
	phase_idx = 0;
	
	// Frame Parameters
	pixels_storing = pixels_storing_;
	width = width_;
	heigth = heigth_;
	frequency = frequency_;
	distance = distance_;
	shutter = shutter_;
	phase = phase_;
	matrix = cv::Mat(heigth, width, cv::DataType<float>::type);
	if(rows_up2down) {
		for (int h = 0; h < heigth; h++) {
			for (int w = 0; w < width; w++) {
				matrix.at<float>(h, w) = matrix_vector[width * h + w];
	}	}	}
	else {
		for (int h = 0; h < heigth; h++) {
			for (int w = 0; w < width; w++) {
				matrix.at<float>(h, w) = matrix_vector[width * (heigth - 1 - h) + w];
	}	}	}
}
// Constructor from vector. Data real time capture oriented. heigth_ and width_ must be refered to the sizes of the total frame, regerdingless to the Pixels_storing
Frame::Frame(unsigned short int* data_, int heigth_, int width_, float distance_, float frequency_, float shutter_, float phase_, int phase_idx_, Pixels_storing pixels_storing_) {
	
	// External Parameters (RawData, Info)
	RawData_src = NULL;
	info = NULL;
	frequency_idx = 0;
	distance_idx = 0;
	shutter_idx = 0;
	phase_idx = 0;

	// Frame Parameters
	pixels_storing = pixels_storing_;
	if (pixels_storing_ == PIXELS_TOTAL) {
		width = width_;
		heigth = heigth_;
	}
	else if (pixels_storing_ == PIXELS_VALID) {
		width = CAMERA_PIX_X_VALID;
		heigth = CAMERA_PIX_Y_VALID;
	}
	frequency = frequency_;
	distance = distance_;
	shutter = shutter_;
	phase = phase_;
	matrix = cv::Mat(heigth, width, cv::DataType<float>::type);
	int idx_in_data;
	if (pixels_storing_ == PIXELS_TOTAL) {
		for (int h = 0; h < heigth; h++) {
			for (int w = 0; w < width; w++) {
				idx_in_data = (heigth_ * width_ * phase_idx_) + (width_ * (heigth_ - 1 - h)) + (w);
				matrix.at<float>(h, w) = (float)(data_[idx_in_data] - 32768);
	}	}	}
	else if (pixels_storing_ == PIXELS_VALID) {
		for (int h = 0; h < heigth; h++) {
			for (int w = 0; w < width; w++) {
				idx_in_data = (heigth_ * width_ * phase_idx_) + (width_ * (CAMERA_PIX_Y - CAMERA_PIX_Y_BAD_TOP - 1 - h)) + (w + CAMERA_PIX_X_BAD_LEFT);
				matrix.at<float>(h, w) = (float)(data_[idx_in_data] - 32768);
	}	}	}
}
// Constructor Default
Frame::Frame() {
	
	// External Parameters (RawData, Info)
	RawData_src = NULL;
	info = NULL;
	frequency_idx = 0;
	distance_idx = 0;
	shutter_idx = 0;
	phase_idx = 0;

	// Frame Parameters
	matrix = cv::Mat(0, 0, cv::DataType<float>::type);
	pixels_storing = UNKNOWN_PIXELS_STORING;
	width = 0;
	heigth = 0;
	frequency = 0.0f;
	distance = 0.0f;
	shutter = 0.0f;
	phase = 0.0f;
}

// first_idx can be 0 (default) or 1, it is the first idx of the row and col we are considering
float Frame::at(int row, int col, int first_idx) {	// (int first_idx = 0) by default in data_read.h
	if (first_idx == 0)
		return matrix.at<float>(row, col);
	else
		return matrix.at<float>(row-1, col-1);
} 

// Plot frame with opencv
void Frame::plot_frame() {

	if ((width <= 0) || (heigth <= 0))
		return;

	// Get a normalized matrix (min=0.0, max=1.0)
	cv::Mat M_norm = matrix.clone();
	double min, max, new_value;
	cv::minMaxLoc(M_norm, &min, &max);
	max -= min;
	cv::MatIterator_<float> it, end;
	for(it = M_norm.begin<float>(), end = M_norm.end<float>(); it != end; ++it) {
		(*it) = ((*it)-min) / max;
	}
	
	// show the image
	cv::namedWindow("Frame", cv::WINDOW_AUTOSIZE);	// WINDOW_NORMAL, WINDOW_AUTOSIZE
	cv::imshow("Frame", M_norm);
	cv::waitKey(1000);
}

// For FoV measurement scene. Plot frame with opencv with syncronization
void plot_frame_fov_measurement(bool loop) {		// by default: loop = false

	// Syncronization
	std::unique_lock<std::mutex> locker_frame_object;	// Create a defered locker (a locker not locked yet)
	locker_frame_object = std::unique_lock<std::mutex>(mutex_frame_object,std::defer_lock);

	
	// show the image
	cv::Mat M_norm;
	int scale = 10;
	bool first_iter = true;
	cv::namedWindow("Frame", cv::WINDOW_AUTOSIZE);	// WINDOW_NORMAL, WINDOW_AUTOSIZE

	// --- LOOP ------------------------------------------------------------------------------------------------
	while(loop || first_iter) {

		if (!PMD_LOOP_ENABLE && !first_iter)
			break;
		first_iter = false;
		
		// Syncronization
		locker_frame_object.lock();		// Lock mutex_frame_object, any thread which used mutex_frame_object can NOT continue until unlock()
		while (!UPDATED_NEW_FRAME) {
			cv_frame_object.wait(locker_frame_object);
		}

		// this three lines are the only critical zone
		if ((FRAME_90_CAPTURE.width <= 0) || (FRAME_90_CAPTURE.heigth <= 0)) {
			return;
		}
		M_norm = FRAME_90_CAPTURE.matrix.clone();		
		
		// Syncronization
		UPDATED_NEW_FRAME = false;
		UPDATED_NEW_OBJECT = true;
		cv_frame_object.notify_all();	// Notify all cv_frame_object. All threads waiting for cv_frame_object will break the wait after waking up
		locker_frame_object.unlock();	// Unlock mutex_frame_object, now threads which used mutex_frame_object can continue

		// Get a normalized matrix (min=0.0, max=1.0)
		double min, max, new_value;
		cv::minMaxLoc(M_norm, &min, &max);
		max -= min;
		cv::MatIterator_<float> it, end;
		for(it = M_norm.begin<float>(), end = M_norm.end<float>(); it != end; ++it)
			(*it) = ((*it)-min) / max;
		cv::resize(M_norm, M_norm, cv::Size(), scale, scale, cv::INTER_NEAREST);

		// show window
		cv::imshow("Frame", M_norm);
		cv::waitKey(20);
	}
}



// Reads data from a .dat and info.txt file setting the DATAPMD_READ variable
int data_read_main() {

	// FRAME_00_CAPTURE, FRAME_90_CAPTURE
	FRAME_00_CAPTURE.plot_frame();
	FRAME_90_CAPTURE.plot_frame();

	return 0;	// no errors
}



// sets a vector of floats form a char array from a given delimiter
void char_array_to_float_vector_from_delimiter (char* char_array, std::vector<float> & float_vector, char delimiter) {

    //char_array[char_array_Length-1] = '\0';

	// Find the position in the array of the first number after the delimiter
    int idx_float_str = 0;
	while(char_array[idx_float_str++] != delimiter) {
		if (char_array[idx_float_str] == '\n' || char_array[idx_float_str] == '\0')
			return;
	}
	idx_float_str++;

	// Build the vector
    float float_value;
    char char_value;
    std::string float_str = "";
    while(true) {
        char_value = char_array[idx_float_str++];
        if (char_value == ' ' || char_value == '\n'|| char_value == '\0') {
            float_value = atof(float_str.c_str());
            float_str = "";
            float_vector.push_back(float_value);
            if (char_value == '\n'|| char_value == '\0')
                break;
        } else {
            float_str += char_value;
        }
    }  
}

