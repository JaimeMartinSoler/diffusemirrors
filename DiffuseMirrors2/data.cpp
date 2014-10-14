
#include <iostream> 
#include <stdlib.h>     // atof

#include "global.h"

#include "data.h"
#include "data_sim.h"
#include "capturetool2.h"
#include "scene.h"

#include <math.h>		// round, fmodf

#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>	




// ----------------------------------------------------------------------------------------------------------------------------------------
// ----- INFO -----------------------------------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------------------

// ----- CONSTRUCTORS ---------------------------- // Note that each Constructor just contains its corresponding Setter

// Constructor Default
Info::Info() {
	set ();
}
// Constructor Copy
// pointers are copied "as are", the data pointed is not duplicated. For this use .clone(...) (if implemented)
Info::Info(Info & info) {// External Parameters (file names):
	set(info);
}
// Constructor: All parameters
Info::Info(	char* dir_name_, char* file_name_, int sizeof_value_raw_, int rows_, int cols_,
			std::vector<float> & freqV_, std::vector<float> & distV_, std::vector<float> & shutV_, std::vector<float> & phasV_, int numtakes_, int error_code_,
			int sizeof_value_cmx_, float laser_to_cam_offset_x_, float laser_to_cam_offset_y_, float laser_to_cam_offset_z_, float dist_wall_cam_) {
	set (dir_name_, file_name_, sizeof_value_raw_, rows_, cols_, freqV_, distV_, shutV_, phasV_, numtakes_, error_code_,
		 sizeof_value_cmx_, laser_to_cam_offset_x_, laser_to_cam_offset_y_, laser_to_cam_offset_z_, dist_wall_cam_);
}
// Constructor from file
Info::Info(char* dir_name_, char* file_name_) {
	set (dir_name_, file_name_);
}
// Destructor
Info::~Info() {
	// empty: crashes �when trying to delete any char*
}

// ----- SETTERS --------------------------------- // Note that each Constructor just contains its corresponding Setter

// Setter Default
void Info::set () {

	// External Parameters (file names):
	dir_name = NULL;
	file_name = NULL;
	inf_full_file_name = NULL;
	raw_full_file_name = NULL;
	cmx_full_file_name = NULL;
	cmd_full_file_name = NULL;
	raw_take_full_file_name.resize(0);

	// Info Parameters:
	sizeof_value_raw = 0;
	rows = 0;
	cols = 0;
	freqV = std::vector<float>(0);;
	distV = std::vector<float>(0);;
	shutV = std::vector<float>(0);;
	phasV = std::vector<float>(0);;
	numtakes = 0;
	// Calibration Matrix parameters:
	sizeof_value_cmx = 0;
	laser_to_cam_offset_x = 0.0f;
	laser_to_cam_offset_y = 0.0f;
	laser_to_cam_offset_z = 0.0f;
	dist_wall_cam = 0.0f;

	error_code = 0;
}
// Setter Copy
// pointers are copied "as are", the data pointed is not duplicated. For this use .clone(...) (if implemented)
void Info::set(Info & info) {// External Parameters (file names):
	
	dir_name = info.dir_name;
	file_name = info.file_name;
	inf_full_file_name = info.inf_full_file_name;
	raw_full_file_name = info.raw_full_file_name;
	cmx_full_file_name = info.cmx_full_file_name;
	cmd_full_file_name = info.cmd_full_file_name;
	raw_take_full_file_name = info.raw_take_full_file_name;

	// Info Parameters:
	sizeof_value_raw = info.sizeof_value_raw;	// bytes
	rows = info.rows;
	cols = info.cols;
	freqV = info.freqV;
	distV = info.distV;
	shutV = info.shutV;
	phasV = info.phasV;
	numtakes = info.numtakes;
	// Calibration Matrix parameters:
	int sizeof_value_cmx = info.sizeof_value_cmx;	// bytes
	float laser_to_cam_offset_x = info.laser_to_cam_offset_x;
	float laser_to_cam_offset_y = info.laser_to_cam_offset_y;
	float laser_to_cam_offset_z = info.laser_to_cam_offset_z;
	float dist_wall_cam = info.dist_wall_cam;
	int error_code = info.error_code;
}
// Setter All parameters
void Info::set (char* dir_name_, char* file_name_, int sizeof_value_raw_, int rows_, int cols_,
			std::vector<float> & freqV_, std::vector<float> & distV_, std::vector<float> & shutV_, std::vector<float> & phasV_, int numtakes_, int error_code_,
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
		raw_take_full_file_name.resize(numtakes_);
		for (int i = 0; i < numtakes_; i++) {
			char raw_take_full_file_name_i[1024];
			sprintf (raw_take_full_file_name_i,"%s\\%s%s%03d%s", dir_name, file_name, NUMTAKE_FILENAME_APPEND, i, RAW_FILENAME_SUFFIX);
			raw_take_full_file_name[i] = raw_take_full_file_name_i;
		}
	}
	else {
		inf_full_file_name = NULL;
		raw_full_file_name = NULL;
		cmx_full_file_name = NULL;
		cmd_full_file_name = NULL;
		raw_take_full_file_name.resize(numtakes_);
		for (int i = 0; i < numtakes_; i++)
			raw_take_full_file_name[i] = NULL;
	}

	// Info Parameters:
	sizeof_value_raw = sizeof_value_raw_;
	rows = rows_;
	cols = cols_;
	freqV = freqV_;
	distV = distV_;
	shutV = shutV_;
	phasV = phasV_;
	numtakes = numtakes_;
	// Calibration Matrix parameters:
	sizeof_value_cmx = sizeof_value_cmx_;
	laser_to_cam_offset_x = laser_to_cam_offset_x_;
	laser_to_cam_offset_y = laser_to_cam_offset_y_;
	laser_to_cam_offset_z = laser_to_cam_offset_z_;
	dist_wall_cam = dist_wall_cam_;

	error_code = error_code_;

}

// Setter from file
void Info::set (char* dir_name_, char* file_name_) {
	
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
	// std::vector<char*> raw_take_full_file_name; is built below

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
			std::vector<float> rows_and_cols;
			char_array_to_float_vector_from_delimiter (inf_file_line, rows_and_cols, delimiter);
			rows = (int)rows_and_cols[0];
			cols = (int)rows_and_cols[1];
		}
		// frequencies
		else if (line_number == 5)
			char_array_to_float_vector_from_delimiter (inf_file_line, freqV, delimiter);
		// distances
		else if (line_number == 6)
			char_array_to_float_vector_from_delimiter (inf_file_line, distV, delimiter);
		// shutters_
		else if (line_number == 7)
			char_array_to_float_vector_from_delimiter (inf_file_line, shutV, delimiter);
		// phases
		else if (line_number == 8)
			char_array_to_float_vector_from_delimiter (inf_file_line, phasV, delimiter);
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

	// std::vector<char*> raw_take_full_file_name
	raw_take_full_file_name.resize(numtakes);
	for (int i = 0; i < numtakes; i++) {
		char* raw_take_full_file_name_i = new char[1024];
		sprintf (raw_take_full_file_name_i,"%s\\%s%s%03d%s", dir_name, file_name, NUMTAKE_FILENAME_APPEND, i, RAW_FILENAME_SUFFIX);
		raw_take_full_file_name[i] = raw_take_full_file_name_i;
	}

	error_code = 0;	// no errors
}





// ----------------------------------------------------------------------------------------------------------------------------------------
// ----- RAW DATA -------------------------------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------------------

// ----- CONSTRUCTORS ---------------------------- // Note that each Constructor just contains its corresponding Setter

// Constructor Default
RawData::RawData() {
	set ();
}
// Constructor Copy
// pointers are copied "as are", the data pointed is not duplicated. For this use .clone(...) (if implemented)
RawData::RawData(RawData & raw_data) {
	set (raw_data);
}
// Constructor All parameters
RawData::RawData(Info & info_, unsigned short int* data_, int data_size_, int take_, int error_code_,
	float* data_sim_PT_, float* data_sim_PV_, int data_sim_size_, int data_sim_rows_, int data_sim_cols_) {
	set(info_, data_, data_size_, take_, error_code_, data_sim_PT_, data_sim_PV_, data_sim_size_, data_sim_rows_, data_sim_cols_);
}
// Constructor from Info. It creates a CalibrationMatrix object from the .raw file noted in the info object
	// take: number of the raw_numtake file this is referencing to. take = -1 if refers to the normal raw file
RawData::RawData(Info & info_, int take_) { // by default: take = -1
	set (info_, take_);
}
// Destructor
RawData::~RawData() {
	delete[] data;
	delete[] data_sim_PT;
	delete[] data_sim_PV;
}

// ----- SETTERS --------------------------------- // Note that each Constructor just contains its corresponding Setter

// Setter Default
void RawData::set () {
	
	// External Parameters
	info = NULL;
	take = -1;

	// RawData Parameters
	data = NULL;
	data_size = 0;	
	error_code = 0;

	// Simulation Parameters
	data_sim_PT = NULL;
	data_sim_PV = NULL;
	data_sim_size = 0;
	data_sim_rows = 0;
	data_sim_cols = 0;
}
// Setter Copy
// pointers are copied "as are", the data pointed is not duplicated. For this use .clone(...) (if implemented)
void RawData::set (RawData & raw_data) {

	// External Parameters
	info = raw_data.info;
	take = raw_data.take;	

	// RawData Parameters
	data = raw_data.data;
	data_size = raw_data.data_size;	
	error_code = raw_data.error_code;

	// Simulation Parameters
	data_sim_PT = raw_data.data_sim_PT;
	data_sim_PV = raw_data.data_sim_PV;
	data_sim_size = raw_data.data_sim_size;
	data_sim_rows = raw_data.data_sim_rows;
	data_sim_cols = raw_data.data_sim_cols;
}
// Setter All parameters
void RawData::set (Info & info_, unsigned short int* data_, int data_size_, int take_, int error_code_,
	float* data_sim_PT_, float* data_sim_PV_, int data_sim_size_, int data_sim_rows_, int data_sim_cols_) {
	
	// External Parameters
	info = &info_;
	take = take_;	

	// RawData Parameters
	data = data_;
	data_size = data_size_;	
	error_code = error_code_;

	// Simulation Parameters
	data_sim_PT = data_sim_PT_;
	data_sim_PV = data_sim_PV_;
	data_sim_size = data_sim_size_;
	data_sim_rows = data_sim_rows_;
	data_sim_cols = data_sim_cols_;
}
// Setter from Info. It creates a CalibrationMatrix object from the .raw file noted in the info object
// take: number of the raw_numtake file this is referencing to. take = -1 if refers to the normal raw file
void RawData::set (Info & info_, int take_) { // by default: take = -1
	
	// External Parameters (Info) and take
	info = &info_;	// Info object pointer
	take = take_;	// number of the raw_numtake file this is referencing to. take = -1 if refers to the normal raw file

	// select the file name this instance refers to
	char* raw_fn;	
	if (take == -1)
		raw_fn = info->raw_full_file_name;
	else
		raw_fn = info->raw_take_full_file_name[take];

	// expected file size (in number of elements)
	int file_data_size_expected = info->freqV.size() * info->distV.size() * info->shutV.size() * info->phasV.size() * info->rows * info->cols;
	std::cout << "\nfreqV.size() = "<< info->freqV.size();
	std::cout << "\ndistV.size() = "<< info->distV.size();
	std::cout << "\nshutV.size() = "<< info->shutV.size();
	std::cout << "\nphasV.size() = "<< info->phasV.size() << "\n";

	// DATA FILE. Open with read permissions
	FILE* raw_file = fopen(raw_fn, "rb");	// open in binary/raw mode
	if (raw_file == NULL) {
		std::cout << "\n\nError Reading \""<< raw_fn << "\"\n\n";
		error_code = 1;
		return;
	}
	size_t fread_output_size;

	// get file_data_size
	fseek (raw_file , 0 , SEEK_END);
	data_size = ftell (raw_file) / (*info).sizeof_value_raw;
	rewind (raw_file);
	if (data_size != file_data_size_expected) {
		std::cout << "\n\nSize Incoherence Error while getting size of \""<< raw_fn << "\"\nexpected data size: " << file_data_size_expected << "\nactual data size  : " << data_size << "\n\n";
		error_code = 4;
		return;
	}

	// allocate memory to contain the whole file
	data = (unsigned short int*) malloc((*info).sizeof_value_raw * data_size);
	if (data == NULL) {
		std::cout << "\n\nMemory Error while allocating \""<< raw_fn << "\"\n\n";
		error_code = 2;
		return;
	}

	// copy the file into the buffer:
	fread_output_size = fread (data, (*info).sizeof_value_raw, data_size, raw_file);
	if (fread_output_size != data_size) {
		std::cout << "\n\nSize Error while reading \""<< raw_fn << "\"\n\n";
		error_code = 3;
		return;
	}

	fclose (raw_file);

	// Data Simulated
	int data_sim_size = info->freqV.size() * info->distV.size() * info->shutV.size() * info->phasV.size() * PMD_SIM_ROWS * PMD_SIM_COLS;
	data_sim_PT = new float[data_sim_size];
	data_sim_PV = new float[data_sim_size];
	data_sim_rows = PMD_SIM_ROWS;
	data_sim_cols = PMD_SIM_COLS;
	// data_sim_PT, data_sim_PV
	Frame frame_sim_PT, frame_sim_PV;
	double scalePTrows = (double)PMD_SIM_ROWS / CAMERA_PIX_Y;
	double scalePTcols = (double)PMD_SIM_COLS / CAMERA_PIX_X;
	double scalePVrows = (double)PMD_SIM_ROWS / CAMERA_PIX_Y_VALID;
	double scalePVcols = (double)PMD_SIM_COLS / CAMERA_PIX_X_VALID;
	for (int fi = 0; fi < info->freqV.size(); fi++) {
		for (int di = 0; di < info->distV.size(); di++) {
			for (int si = 0; si < info->shutV.size(); si++) {
				for (int pi = 0; pi < info->phasV.size(); pi++) {
					// data_sim_PT
					frame_sim_PT.set((*this), fi, di, si, pi, PIXELS_TOTAL, false);
					cv::Mat frame_sim_PT_Mat = cv::Mat(frame_sim_PT.rows, frame_sim_PT.cols, cv::DataType<float>::type);
					memcpy(frame_sim_PT_Mat.data, frame_sim_PT.data.data(), frame_sim_PT.data.size() * sizeof(float));
					cv::resize(frame_sim_PT_Mat, frame_sim_PT_Mat, cv::Size(PMD_SIM_COLS, PMD_SIM_ROWS), scalePTcols, scalePTrows, cv::INTER_AREA);
					memcpy(data_sim_PT + data_idx(fi, di, si, pi, 0, 0, PIXELS_TOTAL, true), frame_sim_PT_Mat.data, frame_sim_PT_Mat.rows * frame_sim_PT_Mat.cols * sizeof(float));
					// data_sim_PV
					frame_sim_PV.set((*this), fi, di, si, pi, PIXELS_VALID, false);
					cv::Mat frame_sim_PV_Mat = cv::Mat(frame_sim_PV.rows, frame_sim_PV.cols, cv::DataType<float>::type);
					memcpy(frame_sim_PV_Mat.data, frame_sim_PV.data.data(), frame_sim_PV.data.size() * sizeof(float));
					cv::resize(frame_sim_PV_Mat, frame_sim_PV_Mat, cv::Size(PMD_SIM_COLS, PMD_SIM_ROWS), scalePVcols, scalePVrows, cv::INTER_AREA);
					memcpy(data_sim_PV + data_idx(fi, di, si, pi, 0, 0, PIXELS_VALID, true), frame_sim_PV_Mat.data, frame_sim_PV_Mat.rows * frame_sim_PV_Mat.cols * sizeof(float));
	}	}	}	}
	
	error_code = 0;		// no errors
}


// ----- FUNCTIONS -------------------------------

// Returns the index in data[] corresponding to the parameter indices
// input r,c are considered like Matrix from 0 indexation. It also takes care on PixStoring
int RawData::data_idx(int freq_idx, int dist_idx, int shut_idx, int phas_idx, int r, int c, PixStoring ps, bool pixSim) { // by default: ps = PIXELS_STORING_GLOBAL

	if (pixSim) {
		return info->freqV.size() * info->phasV.size() * info->shutV.size() * data_sim_rows * data_sim_cols * dist_idx +
			                        info->phasV.size() * info->shutV.size() * data_sim_rows * data_sim_cols * freq_idx +
			                                             info->shutV.size() * data_sim_rows * data_sim_cols * phas_idx +
														                      data_sim_rows * data_sim_cols * shut_idx +
																			                  data_sim_cols * r + c;
	} else if (ps == PIXELS_VALID) {
		return info->freqV.size() * info->phasV.size() * info->shutV.size() * info->rows * info->cols * dist_idx +
	                                info->phasV.size() * info->shutV.size() * info->rows * info->cols * freq_idx +
	                                                     info->shutV.size() * info->rows * info->cols * phas_idx +
	                                                                          info->rows * info->cols * shut_idx +
	                                                                                       info->cols * (CAMERA_PIX_Y - CAMERA_PIX_Y_BAD_TOP - 1 - r) +
																						                (c + CAMERA_PIX_X_BAD_LEFT);
	} else if (ps == PIXELS_TOTAL) {
		return info->freqV.size() * info->phasV.size() * info->shutV.size() * info->rows * info->cols * dist_idx +
	                                info->phasV.size() * info->shutV.size() * info->rows * info->cols * freq_idx +
	                                                     info->shutV.size() * info->rows * info->cols * phas_idx +
	                                                                          info->rows * info->cols * shut_idx +
	                                                                                       info->cols * (info->rows - 1 - r) +
																						                c;
	}
}
// Returns data[data_idx(...)], the value corresponding to the parameter indices
// input r,c are considered like Matrix from 0 indexation. It also takes care on PixStoring
unsigned short int RawData::at(int freq_idx, int dist_idx, int shut_idx, int phas_idx, int r, int c, PixStoring ps) { // by default: ps = PIXELS_STORING_GLOBAL
	return data[data_idx(freq_idx, dist_idx, shut_idx, phas_idx, r, c, ps)];
}
// Returns (short int)(data[data_idx(...)] - 32768), fixing the 32768 default offset and converting it to (signed) short int
short int RawData::atSI(int freq_idx, int dist_idx, int shut_idx, int phas_idx, int r, int c, PixStoring ps) {
	return (short int)(data[data_idx(freq_idx, dist_idx, shut_idx, phas_idx, r, c, ps)] - 32768);
}
// Returns (int)(data[data_idx(...)] - 32768), fixing the 32768 default offset and converting it to (signed) int
int RawData::atI(int freq_idx, int dist_idx, int shut_idx, int phas_idx, int r, int c, PixStoring ps) {
	return (int)(data[data_idx(freq_idx, dist_idx, shut_idx, phas_idx, r, c, ps)] - 32768);
}
// Returns (float)(data[data_idx(...)] - 32768), fixing the 32768 default offset and converting it to float
float RawData::atF(int freq_idx, int dist_idx, int shut_idx, int phas_idx, int r, int c, PixStoring ps, bool pixSim) {
	if (pixSim) {
		if (ps == PIXELS_TOTAL)
			return data_sim_PT[data_idx(freq_idx, dist_idx, shut_idx, phas_idx, r, c, ps, pixSim)];
		else if (ps == PIXELS_VALID)
			return data_sim_PV[data_idx(freq_idx, dist_idx, shut_idx, phas_idx, r, c, ps, pixSim)];
	} else {
		return (float)(data[data_idx(freq_idx, dist_idx, shut_idx, phas_idx, r, c, ps)] - 32768);
	}
}



// ----------------------------------------------------------------------------------------------------------------------------------------
// ----- CALIBRATION MATRIX ---------------------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------------------

// ----- CONSTRUCTORS ---------------------------- // Note that each Constructor just contains its corresponding Setter

// Constructor Default
CalibrationMatrix::CalibrationMatrix() {
	set ();
}
// Constructor Copy
// pointers are copied "as are", the C pointed is not duplicated. For this use .clone(...) (if implemented)
CalibrationMatrix::CalibrationMatrix(CalibrationMatrix & cmx) {
	set (cmx);
}
// Constructor All parameters
CalibrationMatrix::CalibrationMatrix(Info & info_, float* C_, int C_size_, std::vector<float> & pathDist0_, int error_code_,
	float* C_sim_PT_, float* C_sim_PV_, int C_sim_size_, int C_sim_rows_, int C_sim_cols_, std::vector<float> & pathDist0_sim_PT_, std::vector<float> & pathDist0_sim_PV_) {
	set (info_, C_, C_size_, pathDist0_, error_code_, C_sim_PT_, C_sim_PV_,C_sim_size_, C_sim_rows_, C_sim_cols_, pathDist0_sim_PT_, pathDist0_sim_PV_);
}
// Constructor. It creates a CalibrationMatrix object from the .cmx file noted in the info object
CalibrationMatrix::CalibrationMatrix(Info & info_) { // by default: pixels_storing_ = PIXELS_VALID
	set (info_);
}
// Destructor
CalibrationMatrix::~CalibrationMatrix() {
	delete[] C;
	delete[] C_sim_PT;
	delete[] C_sim_PV;
}

// ----- SETTERS --------------------------------- // Note that each Constructor just contains its corresponding Setter

// Constructor Default
void CalibrationMatrix::set () {

	// External Parameters (RawData, Info)
	info = NULL;

	// Calibration Matrix Parameters
	C = NULL;
	C_size = 0;
	pathDist0 = std::vector<float>(0);
	error_code = 0;

	// Simulation Parameters
	C_sim_PT = NULL;
	C_sim_PV = NULL;
	C_sim_size = 0;
	C_sim_rows = 0;
	C_sim_cols = 0;
	pathDist0_sim_PT = std::vector<float>(0);
	pathDist0_sim_PV = std::vector<float>(0);
}
// Constructor Copy
// pointers are copied "as are", the C pointed is not duplicated. For this use .clone(...) (if implemented)
void CalibrationMatrix::set (CalibrationMatrix & cmx) {
	// External Parameters (RawData, Info)
	info = cmx.info;

	// Calibration Matrix Parameters
	C = cmx.C;
	C_size = cmx.C_size;
	pathDist0 = cmx.pathDist0;
	error_code = cmx.error_code;

	// Simulation Parameters
	C_sim_PT = cmx.C_sim_PT;
	C_sim_PV = cmx.C_sim_PV;
	C_sim_size = cmx.C_sim_size;
	C_sim_rows = cmx.C_sim_rows;
	C_sim_cols = cmx.C_sim_cols;
	pathDist0_sim_PT = cmx.pathDist0_sim_PT;
	pathDist0_sim_PV = cmx.pathDist0_sim_PV;
}
// Constructor All parameters
void CalibrationMatrix::set (Info & info_, float* C_, int C_size_, std::vector<float> & pathDist0_, int error_code_,
	float* C_sim_PT_, float* C_sim_PV_, int C_sim_size_, int C_sim_rows_, int C_sim_cols_, std::vector<float> & pathDist0_sim_PT_, std::vector<float> & pathDist0_sim_PV_) {
	
	// External Parameters (RawData, Info)
	info = &info_;

	// Calibration Matrix Parameters
	C = C_;
	C_size = C_size_;
	pathDist0 = pathDist0_;
	error_code = error_code_;

	// Simulation Parameters
	C_sim_PT = C_sim_PT_;
	C_sim_PV = C_sim_PV_;
	C_sim_size = C_sim_size_;
	C_sim_rows = C_sim_rows_;
	C_sim_cols = C_sim_cols_;
	pathDist0_sim_PT = pathDist0_sim_PT_;
	pathDist0_sim_PV = pathDist0_sim_PV_;
}
// Constructor. It creates a CalibrationMatrix object from the .cmx file noted in the info object
void CalibrationMatrix::set (Info & info_) { 
	
	// External Parameters (Info)
	info = &info_;

	// Function Parameters
	RawData rawData(info_);
	Scene scene;
	bool pSim = false;
	// cmx_data_value = c * Em * albedo = H * distSrcPixPow2,    distSrcPixPow2 = |r_src - r_x0(r,c)|^2,
	std::vector<float> distSrcPixPow2V, distSrcPixPow2V_sim_PT, distSrcPixPow2V_sim_PV;	
	int si = info_.shutV.size() - 1;	// shutter index. The calibration matrix is One shutter oriented

	// C_size, C
	C_size = info->freqV.size() * info->distV.size() * info->phasV.size() * info->rows * info->cols;
	C = new float[C_size];

	// Simulation Parameters
	C_sim_size = info->freqV.size() * info->distV.size() * info->phasV.size() * PMD_SIM_ROWS * PMD_SIM_COLS;
	C_sim_PT = new float[C_sim_size];
	C_sim_PV = new float[C_sim_size];
	C_sim_rows = PMD_SIM_ROWS;
	C_sim_cols = PMD_SIM_COLS;

	// Filling C
	distSrcPixPow2V.resize(numPix(PIXELS_TOTAL, pSim));
	scene.clear();
	scene.setScene_CalibrationMatrix(info_.laser_to_cam_offset_x, info_.laser_to_cam_offset_y, info_.laser_to_cam_offset_z, info_.dist_wall_cam, PIXELS_TOTAL, pSim);
	for (size_t i = 0; i < distSrcPixPow2V.size(); i++)	
		distSrcPixPow2V[i] = (scene.o[LASER].s[0].c - scene.o[WALL_PATCHES].s[i].c).modPow2();
	// data ordering: for(freq) { for(dist) { for(rows){ for(cols){ // here...}}}}
	for (size_t fi = 0; fi < info_.freqV.size(); fi++) {
		for (size_t di = 0; di < info_.distV.size(); di++) {
			for (size_t pi = 0; pi < info_.phasV.size(); pi++) {
				for (size_t r = 0; r < info_.rows; r++) {
					for (size_t c = 0; c < info_.cols; c++) {
						//cmx_data_value = c * Em * albedo = H * distSrcPixPow2,    distSrcPixPow2 = |r_src - r_x0(r,c)|^2,
						C[C_idx(fi, di, pi, r, c, PIXELS_TOTAL, pSim)] = rawData.atF(fi, di, si, pi, r, c, PIXELS_TOTAL, pSim) *  distSrcPixPow2V[rc2idx(r, c, PIXELS_TOTAL, pSim)];
	}	}	}	}	}
	
	// Filling C_sim_PT, C_sim_PV
	pSim = true;
	// FillingdistSrcPixPow2V_sim_PT
	distSrcPixPow2V_sim_PT.resize(numPix(PIXELS_TOTAL, pSim));
	scene.clear();
	scene.setScene_CalibrationMatrix(info_.laser_to_cam_offset_x, info_.laser_to_cam_offset_y, info_.laser_to_cam_offset_z, info_.dist_wall_cam, PIXELS_TOTAL, pSim);
	for (size_t i = 0; i < distSrcPixPow2V_sim_PT.size(); i++)	
		distSrcPixPow2V_sim_PT[i] = (scene.o[LASER].s[0].c - scene.o[WALL_PATCHES].s[i].c).modPow2();
	// FillingdistSrcPixPow2V_sim_PV
	distSrcPixPow2V_sim_PV.resize(numPix(PIXELS_VALID, pSim));
	scene.clear();
	scene.setScene_CalibrationMatrix(info_.laser_to_cam_offset_x, info_.laser_to_cam_offset_y, info_.laser_to_cam_offset_z, info_.dist_wall_cam, PIXELS_VALID, pSim);
	for (size_t i = 0; i < distSrcPixPow2V_sim_PV.size(); i++)	
		distSrcPixPow2V_sim_PV[i] = (scene.o[LASER].s[0].c - scene.o[WALL_PATCHES].s[i].c).modPow2();
	// data ordering: for(freq){ for(dist){ for(phas){ for(r){ for(c){ //here...}}}}}
	for (size_t fi = 0; fi < info_.freqV.size(); fi++) {
		for (size_t di = 0; di < info_.distV.size(); di++) {
			for (size_t pi = 0; pi < info_.phasV.size(); pi++) {
			for (size_t r = 0; r < C_sim_rows; r++) {
				for (size_t c = 0; c < C_sim_cols; c++) {
					//cmx_data_value = c * Em * albedo = H * distSrcPixPow2,    distSrcPixPow2 = |r_src - r_x0(r,c)|^2,
					C_sim_PT[C_idx(fi, di, pi, r, c, PIXELS_TOTAL, pSim)] = rawData.atF(fi, di, si, pi, r, c, PIXELS_TOTAL, pSim) *  distSrcPixPow2V_sim_PT[rc2idx(r, c, PIXELS_TOTAL, pSim)];
					C_sim_PV[C_idx(fi, di, pi, r, c, PIXELS_VALID, pSim)] = rawData.atF(fi, di, si, pi, r, c, PIXELS_VALID, pSim) *  distSrcPixPow2V_sim_PV[rc2idx(r, c, PIXELS_VALID, pSim)];
	}	}	}	}	}
	
	// Filling pathDist0
	pSim = false;
	pathDist0.resize(numPix(PIXELS_TOTAL, pSim)); // independent of PixStoring, the accessing depends on it. Ordering: for(r){ for(c){ // here...}} // r,c Matrix-like, 0-idx
	scene.clear();
	scene.setScene_CalibrationMatrix(info_.laser_to_cam_offset_x, info_.laser_to_cam_offset_y, info_.laser_to_cam_offset_z, info_.dist_wall_cam, PIXELS_TOTAL, pSim);
	for (size_t i = 0; i < pathDist0.size(); i++)
		pathDist0[i] = dist(scene.o[LASER].s[0].c, scene.o[WALL_PATCHES].s[i].c) + dist(scene.o[CAMERA].s[0].c, scene.o[WALL_PATCHES].s[i].c);

	// Filling pathDist0_sim_PT
	pSim = true;
	pathDist0_sim_PT.resize(numPix(PIXELS_TOTAL, pSim));
	scene.clear();
	scene.setScene_CalibrationMatrix(info_.laser_to_cam_offset_x, info_.laser_to_cam_offset_y, info_.laser_to_cam_offset_z, info_.dist_wall_cam, PIXELS_TOTAL, pSim);
	for (size_t i = 0; i < pathDist0_sim_PT.size(); i++)
		pathDist0_sim_PT[i] = dist(scene.o[LASER].s[0].c, scene.o[WALL_PATCHES].s[i].c) + dist(scene.o[CAMERA].s[0].c, scene.o[WALL_PATCHES].s[i].c);

	// Filling pathDist0_sim_PV
	pSim = true;
	pathDist0_sim_PV.resize(numPix(PIXELS_VALID, pSim)); 
	scene.clear();
	scene.setScene_CalibrationMatrix(info_.laser_to_cam_offset_x, info_.laser_to_cam_offset_y, info_.laser_to_cam_offset_z, info_.dist_wall_cam, PIXELS_VALID, pSim);
	for (size_t i = 0; i < pathDist0_sim_PV.size(); i++)
		pathDist0_sim_PV[i] = dist(scene.o[LASER].s[0].c, scene.o[WALL_PATCHES].s[i].c) + dist(scene.o[CAMERA].s[0].c, scene.o[WALL_PATCHES].s[i].c);

	// no errors
	error_code = 0;		
	
	/*
	// Calibration Matrix Parameters: C, C_size
	// expected file size (in number of elements)
	int file_C_size_expected = info->freqV.size() * info->distV.size() * info->rows * info->cols;
	// DATA FILE. Open with read permissions
	FILE* cmx_file = fopen((*info).cmx_full_file_name, "rb");	// open in binary/raw mode
	if (cmx_file == NULL) {
		std::cout << "\n\nError Reading \""<< (*info).cmx_full_file_name << "\"\n\n";
		error_code = 1;
		return;
	}
	size_t fread_output_size;
	// get file_C_size
	fseek (cmx_file , 0 , SEEK_END);
	C_size = ftell (cmx_file) / (*info).sizeof_value_cmx;
	rewind (cmx_file);
	if (C_size != file_C_size_expected) {
		std::cout << "\n\nSize Incoherence Error while getting size of \""<< (*info).cmx_full_file_name << "\"\n\n";
		error_code = 4;
		return;
	}
	// allocate memory to contain the whole file
	C = (float*) malloc((*info).sizeof_value_cmx*C_size);
	if (C == NULL) {
		std::cout << "\n\nMemory Error while allocating \""<< (*info).cmx_full_file_name << "\"\n\n";
		error_code = 2;
		return;
	}
	// copy the file into the buffer:
	fread_output_size = fread (C, (*info).sizeof_value_cmx, C_size, cmx_file);
	if (fread_output_size != C_size) {
		std::cout << "\n\nSize Error while reading \""<< (*info).cmx_full_file_name << "\"\n\n";
		error_code = 3;
		return;
	}
	fclose (cmx_file);
	//free (C_size_);
	*/
}


// ----- FUNCTIONS -------------------------------

// Returns the index in C[], corresponding to the parameter indices. 
// input r,c are considered like Matrix from 0 indexation. It also takes care on PixStoring
int CalibrationMatrix::C_idx (int freq_idx, int dist_idx, int phas_idx, int r, int c, PixStoring ps, bool pSim) { // default: ps = PIXELS_STORING_GLOBAL

	if (pSim) {
		return info->distV.size() * info->phasV.size() * C_sim_rows * C_sim_cols * freq_idx +
	                                info->phasV.size() * C_sim_rows * C_sim_cols * dist_idx +
														 C_sim_rows * C_sim_cols * phas_idx +
																      C_sim_cols * r + c;
	} else if (ps == PIXELS_VALID) {
		return info->distV.size() * info->phasV.size() * info->rows * info->cols * freq_idx +
	                                info->phasV.size() * info->rows * info->cols * dist_idx +
														 info->rows * info->cols * phas_idx +
																	  info->cols * (r + CAMERA_PIX_Y_BAD_TOP) +
																				   (c + CAMERA_PIX_X_BAD_LEFT);
	} else if (ps == PIXELS_TOTAL) {
		return info->distV.size() * info->phasV.size() * info->rows * info->cols * freq_idx +
	                                info->phasV.size() * info->rows * info->cols * dist_idx +
														 info->rows * info->cols * phas_idx +
																	  info->cols * r        + c;
	}
}
// Returns C[C.C_idx(...)], the value from the Calibration Matrix C corresponding to the parameters
// input r,c are considered like Matrix from 0 indexation. It also takes care on PixStoring
float CalibrationMatrix::C_at (int freq_idx, int dist_idx, int phas_idx, int r, int c, PixStoring ps, bool pSim) {
	
	if (pSim) {
		if (ps == PIXELS_TOTAL)
			return C_sim_PT[C_idx(freq_idx, dist_idx, phas_idx, r, c, ps, pSim)];
		else if (ps == PIXELS_VALID)
			return C_sim_PV[C_idx(freq_idx, dist_idx, phas_idx, r, c, ps, pSim)];
	}
	else {
		return C[C_idx(freq_idx, dist_idx, phas_idx, r, c, ps ,pSim)];
	}
}
// Returns the C[...] interpolating with the closest path distances
// input r,c are considered like Matrix from 0 indexation. It also takes care on PixStoring
float CalibrationMatrix::C_atX (int freq_idx, float pathDist, int phas_idx, int r, int c, PixStoring ps, bool pSim)  { // default: ps = PIXELS_STORING_GLOBAL
	
	float pathDist_offset = pathDist - pathDist0_at(r, c, ps, pSim);
	float pathDist_res = (info->distV[1] - info->distV[0]);	// dist of info store actually path distances as well

	// pathDist Indices
	int pathDist_idx_floor = (pathDist_offset - info->distV[0]) / pathDist_res;
	int pathDist_idx_ceil = pathDist_idx_floor + 1;
	// checking if out of bounds
	if (pathDist_idx_floor < 0) {
		std::cout << "\nWarning: path_dist(" << r << "," << c << ") = " << pathDist << " out of .cmx min bound = " << info->distV[0] << " ---> pathDist\n";
		return C_at(freq_idx, 0, phas_idx, r, c, ps, pSim);
	} else if (pathDist_idx_floor >= info->distV.size() - 1) {
		std::cout << "\nWarning: path_dist(" << r << "," << c << ") = " << pathDist << " out of .cmx max bound = " << info->distV[info->distV.size()-1] << " ---> pathDist\n";
		return C_at(freq_idx, info->distV.size()-1, phas_idx, r, c, ps, pSim);
	}

	// pathDist Scales
	float pathDist_scale_ceil = fmodf(pathDist_offset-info->distV[0], pathDist_res) / pathDist_res;	// -info->distances[0] to avoid bad negative behaviour of fmodf(...)
	float pathDist_scale_floor = 1.0f - pathDist_scale_ceil;
	/*
	std::cout << "\n\ndist_offset      = " << dist_offset;
	std::cout << "\ndist_res         = " << dist_res;
	std::cout << "\ndist_idx_floor   = " << dist_idx_floor;
	std::cout << "\ndist_idx_ceil    = " << dist_idx_ceil;
	std::cout << "\ndist_scale_floor = " << dist_scale_floor;
	std::cout << "\ndist_scale_ceil  = " << dist_scale_ceil;

	std::cout << "\n\npathDist0.at("<< h << "," << w << ") = " << pathDist0_at(r,c);
	std::cout << "\npath_dist_floor  = " << pathDist0_at(r,c) + info->distV[dist_idx_floor];
	std::cout << "\npath_dist_ceil   = " << pathDist0_at(r,c) + info->distV[dist_idx_ceil];
	std::cout << "\ndist[di="<< dist_idx_floor << "] = " << info->distV[dist_idx_floor];
	std::cout << "\ndist[di="<< dist_idx_ceil << "] = " << info->distV[dist_idx_ceil];
	std::cout << "\nat(fi_max, di=" << dist_idx_floor << ", cen) = " << at(info->freqV.size()-1, dist_idx_floor, w, h);
	std::cout << "\nat(fi_max, di=" << dist_idx_ceil << ", cen) = " << at(info->freqV.size()-1, dist_idx_ceil, w, h);
	*/
	return (pathDist_scale_floor * C_at(freq_idx, pathDist_idx_floor, phas_idx, r, c, ps, pSim)) + (pathDist_scale_ceil * C_at(freq_idx, pathDist_idx_ceil, phas_idx, r, c, ps, pSim));
}

// Returns the index in pathDist0, corresponding to the parameter indices.
// input r,c are considered like Matrix from 0 indexation. It also takes care on PixStoring
int CalibrationMatrix::pathDist0_idx (int r, int c, PixStoring ps, bool pSim) { // default: ps = PIXELS_STORING_GLOBAL
	
	if (pSim)
		return C_sim_cols * r + c;
	else if (ps == PIXELS_VALID)
		return info->cols * (r + CAMERA_PIX_Y_BAD_TOP) + (c + CAMERA_PIX_X_BAD_LEFT);
	else if (ps == PIXELS_TOTAL)
			return info->cols * r + c;
}
// Returns the value from the pathDist0 corresponding to the parameter indices = pathDist0[pathDist0_idx]
// input r,c are considered like Matrix from 0 indexation. It also takes care on PixStoring
float CalibrationMatrix::pathDist0_at (int r, int c, PixStoring ps, bool pSim) { 
	
	if (pSim) {
		if (ps == PIXELS_TOTAL)
			return pathDist0_sim_PT[pathDist0_idx(r,c,ps,pSim)];
		else if (ps == PIXELS_VALID)
			return pathDist0_sim_PV[pathDist0_idx(r,c,ps,pSim)];
	}
	else {
		return pathDist0[pathDist0_idx(r,c,ps,pSim)];
	}
}
	

// This is the Simulation term for the direct vision problem: S_{i\;\omega}^{r,c}(\tau^{r,c}) in the Master Thesis document
// Returns the value of the Simulation from the Calibration Matrix C at any path distance interpolating with the closest path distances. Path distances have to be equidistant in vector
// Uses C_atX(...) for interpolating path distances
float CalibrationMatrix::S_DirectVision (int freq_idx, int phas_idx, int r, int c, Point & r_src, Point & r_x,  Point & r_cam, float relative_albedo, PixStoring ps, bool pSim) { // by default: relative_albedo = 1.0f, ps = PIXELS_STORING_GLOBAL

	float dist_src_x = dist(r_src, r_x);
	float dist_cam_x = dist(r_cam, r_x);
	float pathDist = dist_src_x + dist_cam_x;
	
	return C_atX (freq_idx, pathDist, phas_idx, r, c, ps, pSim) * relative_albedo / (dist_src_x * dist_src_x);
}




// ----------------------------------------------------------------------------------------------------------------------------------------
// ----- FRAME ----------------------------------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------------------

// ----- CONSTRUCTORS ---------------------------- // Note that each Constructor just contains its corresponding Setter

// Constructor Default
Frame::Frame() {
	set ();
}
// Constructor Copy
// pointers are copied "as are", the data pointed is not duplicated. For this use .clone(...) (if implemented)
Frame::Frame (Frame & frame) {
	set (frame);
}
// Constructor All parameters
Frame::Frame (RawData & RawData_src_, int freq_idx_, int dist_idx_, int shut_idx_, int phas_idx_,
	std::vector<float> & data_, PixStoring ps_, bool pSim_, int rows_, int cols_, float freq_, float dist_, float shut_, float phas_) {
	set (RawData_src_, freq_idx_, dist_idx_, shut_idx_, phas_idx_, data_, ps_, pSim_, rows_, cols_, freq_, dist_, shut_, phas_);
}
// Constructor from RawData. RawData oriented
Frame::Frame(RawData & RawData_src_, int freq_idx_, int dist_idx_, int shut_idx_, int phas_idx_, PixStoring ps_, bool pSim_) { // by default: ps_ = PIXELS_STORING_GLOBAL
	set (RawData_src_, freq_idx_, dist_idx_, shut_idx_, phas_idx_, ps_, pSim_);
}
// Constructor from vector. Simulation oriented. For any PixStoring it consideres the vector matrix_vector properly arranged
// If rows_ and cols_ are > 0, they will be the new rows and cols (interesting while simulating arbitrary rows and cols. Otherwise rows and cols are made from ps_
Frame::Frame(std::vector<float> & data_, int rows_, int cols_, float freq_, float dist_, float shut_, float phas_, PixStoring ps_, bool pSim_ ) { // by default: r,c,f,d,s,p = 0, ps_ = PIXELS_STORING_GLOBAL
	set (data_, rows_, cols_, freq_, dist_ , shut_, phas_, ps_, pSim_);
}
// Constructor from ushort int*. Real Time capture oriented. rows_ and cols_ must be refered to the sizes of the total frame, regerdingless to the PixStoring
Frame::Frame(unsigned short int* data_, int rowsPT, int colsPT, float freq_, float dist_, float shut_, float phas_, int phas_idx_, PixStoring ps_, bool pSim_) { // by default: ps_ = PIXELS_STORING_GLOBAL
	set (data_, rowsPT, colsPT, freq_, dist_, shut_, phas_, phas_idx_, ps_, pSim_);
}


// ----- SETTERS --------------------------------- // Note that each Constructor just contains its corresponding

// Setter Default
void Frame::set () {
	
	// External Parameters (RawData, Info)
	RawData_src = NULL;
	freq_idx = 0;
	dist_idx = 0;
	shut_idx = 0;
	phas_idx = 0;

	// Frame Parameters
	data = std::vector<float>(0);
	ps = UNKNOWN_PIS;
	pSim = false;
	rows = 0;
	cols = 0;
	freq = 0.0f;
	dist = 0.0f;
	shut = 0.0f;
	phas = 0.0f;
}
// Setter Copy
// pointers are copied "as are", the data pointed is not duplicated. For this use .clone(...) (if implemented)
void Frame::set (Frame & frame) {

	// External Parameters (RawData, Info)
	RawData_src = frame.RawData_src;
	freq_idx = frame.freq_idx;
	dist_idx = frame.dist_idx;
	shut_idx = frame.shut_idx;
	phas_idx = frame.phas_idx;

	// Frame Parameters
	data = frame.data;
	ps = frame.ps;
	pSim = frame.pSim;
	rows = frame.rows;
	cols = frame.cols;
	freq = frame.freq;
	dist = frame.dist;
	shut = frame.shut;
	phas = frame.phas;
}
// Setter All parameters
void Frame::set (RawData & RawData_src_, int freq_idx_, int dist_idx_, int shut_idx_, int phas_idx_,
	std::vector<float> & data_, PixStoring ps_, bool pSim_, int rows_, int cols_, float freq_, float dist_, float shut_, float phas_) {

	// External Parameters (RawData, Info)
	RawData_src = &RawData_src_;
	freq_idx = freq_idx_;
	dist_idx = dist_idx_;
	shut_idx = shut_idx_;
	phas_idx = phas_idx_;

	// Frame Parameters
	data = data_;
	ps = ps_;
	pSim = pSim_;
	rows = rows_;
	cols = cols_;
	freq = freq_;
	dist = dist_;
	shut = shut_;
	phas = phas_;
}
// Setter from RawData. RawData oriented
void Frame::set(RawData & RawData_src_, int freq_idx_, int dist_idx_, int shut_idx_, int phas_idx_, PixStoring ps_, bool pSim_) { // by default: ps_ = PIXELS_STORING_GLOBAL
	
	// External Parameters (RawData, idices)
	RawData_src = &RawData_src_;
	freq_idx = freq_idx_;
	dist_idx = dist_idx_;
	shut_idx = shut_idx_;
	phas_idx = phas_idx_;
	
	// Frame Parameters
	ps = ps_;
	pSim = pSim_;
	if (pSim_) {
		rows = PMD_SIM_ROWS;
		cols = PMD_SIM_COLS;
	} else if (ps_ == PIXELS_VALID) {
		rows = CAMERA_PIX_Y_VALID;
		cols = CAMERA_PIX_X_VALID;
	} else if (ps_ == PIXELS_TOTAL) {
		rows = CAMERA_PIX_Y;
		cols = CAMERA_PIX_X;
	}
	freq = RawData_src->info->freqV[freq_idx];
	dist = RawData_src->info->distV[dist_idx];
	shut = RawData_src->info->shutV[shut_idx];
	phas = RawData_src->info->phasV[phas_idx];
	data.resize(rows*cols);
	for (int r = 0; r < rows; r++) {
		for (int c = 0; c < cols; c++) {
			data[data_idx(r,c)] = RawData_src_.atF(freq_idx, dist_idx, shut_idx, phas_idx, r, c, ps, pSim);
	}	}
}
// Setter from vector. Simulation oriented. For any PixStoring it consideres the vector matrix_vector properly arranged
// If rows_ and cols_ are > 0, they will be the new rows and cols (interesting while simulating arbitrary rows and cols. Otherwise rows and cols are made from ps_
void Frame::set(std::vector<float> & data_, int rows_, int cols_, float freq_, float dist_, float shut_, float phas_, PixStoring ps_, bool pSim_) { // by default: r,c,f,d,s,p = 0, ps_ = PIXELS_STORING_GLOBAL
	
	// External Parameters (RawData, indices)
	RawData_src = NULL;
	freq_idx = 0;
	dist_idx = 0;
	shut_idx = 0;
	phas_idx = 0;
	
	// Frame Parameters
	data = data_;
	ps = ps_;
	pSim = pSim_;
	if ((rows_ > 0) && (cols_ > 0)) {
		rows = rows_;
		cols = cols_;
	} else if (pSim_) {
		rows = PMD_SIM_ROWS;
		cols = PMD_SIM_COLS;
	} else if (ps == PIXELS_VALID) {
		rows = CAMERA_PIX_Y_VALID;
		cols = CAMERA_PIX_X_VALID;
	} else if (ps == PIXELS_TOTAL) {
		rows = CAMERA_PIX_Y;
		cols = CAMERA_PIX_X;
	}
	freq = freq_;
	dist = dist_;
	shut = shut_;
	phas = phas_;
}
// Setter from ushort int*. Real Time capture oriented. rowsPT and colsPT must be refered to the sizes of the total frame, regerdingless to the PixStoring
void Frame::set(unsigned short int* data_, int rowsPT, int colsPT, float freq_, float dist_, float shut_, float phas_, int phas_idx_, PixStoring ps_, bool pSim_) { // by default: ps_ = PIXELS_STORING_GLOBAL
	
	// External Parameters (RawData, indices)
	RawData_src = NULL;
	freq_idx = 0;
	dist_idx = 0;
	shut_idx = 0;
	phas_idx = phas_idx_;

	// Frame Parameters
	ps = ps_;
	if (ps == PIXELS_VALID) {
		rows = CAMERA_PIX_Y_VALID;
		cols = CAMERA_PIX_X_VALID;
	}
	else if (ps == PIXELS_TOTAL) {
		rows = CAMERA_PIX_Y;
		cols = CAMERA_PIX_X;
	}
	pSim = pSim_;
	freq = freq_;
	dist = dist_;
	shut = shut_;
	phas = phas_;
	data.resize(rows*cols);
	int idx_in_data;
	if (ps == PIXELS_VALID) {
		for (int r = 0; r < rows; r++) {
			for (int c = 0; c < cols; c++) {
				idx_in_data = (rowsPT * colsPT * phas_idx_) + (colsPT * (CAMERA_PIX_Y - CAMERA_PIX_Y_BAD_TOP - 1 - r)) + (c + CAMERA_PIX_X_BAD_LEFT);
				data[data_idx(r,c)] = (float)(data_[idx_in_data] - 32768); // data is not stored properly in raw file. -32768 fixes it
	}	}	}
	else if (ps == PIXELS_TOTAL) {
		for (int r = 0; r < rows; r++) {
			for (int c = 0; c < cols; c++) {
				idx_in_data = (rowsPT * colsPT * phas_idx_) + (colsPT * (rowsPT - 1 - r)) + c;
				data[data_idx(r,c)] = (float)(data_[idx_in_data] - 32768); // data is not stored properly in raw file. -32768 fixes it
	}	}	}

	// pSim 
	if (pSim_)
		toPixSim();
}


// ----- FUNCTIONS -------------------------------

// r,c Matrix-like, 0-idx.
int Frame::data_idx (int r, int c) {
	return	cols * r + c;
}
// r,c Matrix-like, 0-idx.
float Frame::at (int r, int c) {
	return data[data_idx(r,c)];
} 

// Frame Real to Frame with with Simulated Pixels
void Frame::toPixSim(int rowsSim, int colsSim) {

	// get scales
	double scaleRows, scaleCols;
	if (ps == PIXELS_TOTAL) {
		scaleRows = (double)rowsSim / CAMERA_PIX_Y;
		scaleCols = (double)colsSim / CAMERA_PIX_X;
	}
	else if (ps == PIXELS_VALID) {
		scaleRows = (double)rowsSim / CAMERA_PIX_Y_VALID;
		scaleCols = (double)colsSim / CAMERA_PIX_X_VALID;
	}

	// resize the frame
	cv::Mat frameMat = cv::Mat(rows, cols, cv::DataType<float>::type);
	memcpy(frameMat.data, data.data(), data.size()*sizeof(float));
	cv::resize(frameMat, frameMat, cv::Size(colsSim, rowsSim), scaleCols, scaleRows, cv::INTER_AREA);
	data.resize(rowsSim * colsSim);
	memcpy(data.data(), frameMat.data, data.size()*sizeof(float));

	// update parameters
	pSim = true;
	rows = rowsSim;
	cols = colsSim;

	// MY METHOD:
	/*
	// vector to fill
	std::vector<float> dataSim(rowsSim * colsSim, 0.0f);

	// variables
	float rowPixSim = PMD_SIM_ROWS;
	float colPixSim = PMD_SIM_COLS;
	float rRealation = rows / rowPixSim;
	float cRealation = cols / colPixSim;
	float simArea = rRealation * cRealation;	// pixSizeSim / pixSizeReal
	float rPixRealTopF, rPixRealBottomF, cPixRealLeftF, cPixRealRightF;
	int rPixRealTopI, rPixRealBottomI, cPixRealLeftI, cPixRealRightI;
	float rScaleTop, rScaleBottom, cScaleLeft, cScaleRight;

	// Fill the Simulated Pixels rows
	for (int rSim = 0; rSim < rowPixSim; rSim++) {
	// Real pixels floats rows
	rPixRealTopF = (float)rSim * rRealation;
	rPixRealBottomF = ((float)rSim + 1.0f) * rRealation;
	// Real pixels ints rows
	rPixRealTopI = (int)rPixRealTopF + 1;		// >= 1, OK
	rPixRealBottomI = (int)rPixRealBottomF - 1;	// <= rows - 1, needs fix
	// Scales of edges cols
	rScaleTop = (float)rPixRealTopI - rPixRealTopF;
	rScaleBottom = rPixRealBottomF - (float)rPixRealBottomI - 1.0f;
	if (rPixRealBottomI == rows - 1) { rPixRealBottomI--; rScaleBottom = 1.0f; } // <= rows - 2, fixed

	// Fill the Simulated Pixels cols
	for (int cSim = 0; cSim < colPixSim; cSim++) {
	// Real pixels floats cols
	cPixRealLeftF = (float)cSim * cRealation;
	cPixRealRightF = ((float)cSim + 1.0f) * cRealation;
	// Real pixels ints cols
	cPixRealLeftI = (int)cPixRealLeftF + 1;		// >= 1, OK
	cPixRealRightI = (int)cPixRealRightF - 1;	// <= cols - 1, needs fix
	// Scales of edges cols
	cScaleLeft = (float)cPixRealLeftI - cPixRealLeftF;
	cScaleRight = cPixRealRightF - (float)cPixRealRightI - 1.0f;
	if (cPixRealRightI == cols - 1) { cPixRealRightI--; cScaleRight = 1.0f; } // <= cols - 2, fixed

	// Fill the Simulated pixel with the corresponding Real Full pixels
	for (int rReal = rPixRealTopI; rReal <= rPixRealBottomI; rReal++) {
	for (int cReal = cPixRealLeftI; cReal <= cPixRealRightI; cReal++) {
	dataSim[rc2idxPSIM(rSim, cSim)] += data[rc2idx(rReal, cReal, ps)];
	}	}
	// Fill the Simulated pixel with the corresponding Real Edges pixels
	for (int rReal = rPixRealTopI; rReal <= rPixRealBottomI; rReal++) {
	dataSim[rc2idxPSIM(rSim, cSim)] += cScaleLeft * data[rc2idx(rReal, cPixRealLeftI - 1, ps)];
	dataSim[rc2idxPSIM(rSim, cSim)] += cScaleRight * data[rc2idx(rReal, cPixRealRightI + 1, ps)];
	}
	for (int cReal = cPixRealLeftI; cReal <= cPixRealRightI; cReal++) {
	dataSim[rc2idxPSIM(rSim, cSim)] += rScaleTop * data[rc2idx(rPixRealTopI - 1, cReal, ps)];
	dataSim[rc2idxPSIM(rSim, cSim)] += rScaleBottom * data[rc2idx(rPixRealBottomI + 1, cReal, ps)];
	}
	// Fill the Simulated pixel with the corresponding Real Corner pixels
	dataSim[rc2idxPSIM(rSim, cSim)] += rScaleTop * cScaleLeft * data[rc2idx(rPixRealTopI - 1, cPixRealLeftI - 1, ps)];
	dataSim[rc2idxPSIM(rSim, cSim)] += rScaleTop * cScaleRight * data[rc2idx(rPixRealTopI - 1, cPixRealRightI + 1, ps)];
	dataSim[rc2idxPSIM(rSim, cSim)] += rScaleBottom * cScaleLeft * data[rc2idx(rPixRealBottomI + 1, cPixRealLeftI - 1, ps)];
	dataSim[rc2idxPSIM(rSim, cSim)] += rScaleBottom * cScaleRight * data[rc2idx(rPixRealBottomI + 1, cPixRealRightI + 1, ps)];
	// Normalize with the area
	dataSim[rc2idxPSIM(rSim, cSim)] /= simArea;
	}	}

	// update the Frame parameters
	data = dataSim;
	pSim = true;
	rows = rowsSim;
	cols = colsSim;
	*/
}
// Frame Simulated to Frame Real (useless, but for testing)
void Frame::toPixReal() {

	// get scales
	double scaleRows, scaleCols;
	int rows_, cols_;
	if (ps == PIXELS_TOTAL) {
		scaleRows = CAMERA_PIX_Y / (double)rows;
		scaleCols = CAMERA_PIX_X / (double)cols;
		rows_ = CAMERA_PIX_Y;
		cols_ = CAMERA_PIX_X;
	}
	else if (ps == PIXELS_VALID) {
		scaleRows = CAMERA_PIX_Y_VALID / (double)rows;
		scaleCols = CAMERA_PIX_X_VALID / (double)cols;
		rows_ = CAMERA_PIX_Y_VALID;
		cols_ = CAMERA_PIX_X_VALID;
	}

	// resize the frame
	cv::Mat frameMat = cv::Mat(rows, cols, cv::DataType<float>::type);
	memcpy(frameMat.data, data.data(), data.size()*sizeof(float));
	cv::resize(frameMat, frameMat, cv::Size(cols_, rows_), scaleCols, scaleRows, cv::INTER_AREA);
	data.resize(rows_ * cols_);
	memcpy(data.data(), frameMat.data, data.size()*sizeof(float));

	// update parameters
	pSim = false;
	rows = rows_;
	cols = cols_;
}

// Plot frame with opencv
void Frame::plot(int delay_ms, bool destroyWindow_, char* windowName, float scale) { // by default: delay_ms = 1000, destroyWindow = false, windowName = NULL, scale = -1.0f

	if ((rows <= 0) || (cols <= 0))
		return;

	// Get a normalized matrix (min=0.0, max=1.0)
	cv::Mat M_norm = cv::Mat(rows, cols, cv::DataType<float>::type);
	memcpy(M_norm.data, data.data(), data.size()*sizeof(float));
	double min, max, new_value;
	cv::minMaxLoc(M_norm, &min, &max);
	max -= min;
	cv::MatIterator_<float> it, end;
	for(it = M_norm.begin<float>(), end = M_norm.end<float>(); it != end; ++it) {
		(*it) = ((*it)-min) / max;
	}
	
	// show the image
	if (scale > 0.0f)
		cv::resize(M_norm, M_norm, cv::Size(), scale, scale, cv::INTER_NEAREST);
	if (windowName == NULL)
		windowName = "Frame.plot()";
	cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);	// WINDOW_NORMAL, WINDOW_AUTOSIZE
	cv::imshow(windowName, M_norm);
	cv::waitKey(delay_ms);
	if (destroyWindow_)
		cv::destroyWindow(windowName);
}
// plot frame amplitude with sinusoidal assumption
void plot_frame(Frame & frame_00, Frame & frame_90, int delay_ms, bool destroyWindow_, char* windowName, float scale) { // by default: delay_ms = 1000, destroyWindow = false, windowName = NULL, scale = -1.0f

	if ((frame_00.rows <= 0) || (frame_00.cols <= 0) || (frame_90.rows <= 0) || (frame_90.cols <= 0))
		return;

	// Get a the module matrix
	cv::Mat M_out = cv::Mat(frame_00.rows, frame_00.cols, cv::DataType<float>::type);
	memcpy(M_out.data, frame_00.data.data(), frame_00.data.size()*sizeof(float));
	for(int r = 0; r < M_out.rows; r++) {
		for(int c = 0; c < M_out.cols; c++) {
			M_out.at<float>(r,c) = (frame_00.at(r,c) * frame_00.at(r,c)) + (frame_90.at(r,c) * frame_90.at(r,c));
	}	}

	// Get a normalized matrix (min=0.0, max=1.0)
	double min, max, new_value;
	cv::minMaxLoc(M_out, &min, &max);
	max -= min;
	cv::MatIterator_<float> it, end;
	for(it = M_out.begin<float>(), end = M_out.end<float>(); it != end; ++it) {
		(*it) = ((*it)-min) / max;
	}
	
	// show the image
	if (scale > 0.0f)
		cv::resize(M_out, M_out, cv::Size(), scale, scale, cv::INTER_NEAREST);
	if (windowName == NULL)
		windowName = "plot_frame(Frame,Frame)";
	cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);	// WINDOW_NORMAL, WINDOW_AUTOSIZE
	cv::imshow(windowName, M_out);
	cv::waitKey(delay_ms);
	if (destroyWindow_)
		cv::destroyWindow(windowName);
}
// For FoV measurement scene. Plot frame with opencv with syncronization
void plot_frame_fov_measurement(Frame & frame_00, Frame & frame_90, bool loop, bool destroyWindow_, char* windowName, bool line_center, int lines_grid) { // by default: (see data.h)

	// Syncronization
	std::unique_lock<std::mutex> locker_frame_object;	// Create a defered locker (a locker not locked yet)
	locker_frame_object = std::unique_lock<std::mutex>(mutex_frame_object,std::defer_lock);

	// show the image
	cv::Mat M_00, M_90;
	int scale = 10;
	bool first_iter = true;
	bool frame_00_empty = false;
	bool frame_90_empty = false;
	// check empty frames. The non-empty frame is rendered. If both are non-empty the amplitude is rendered
	
	if (windowName == NULL)
		windowName = "plot_frame_fov_measurement(Frame,Frame)";
	cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);	// WINDOW_NORMAL, WINDOW_AUTOSIZE

	// --- LOOP ------------------------------------------------------------------------------------------------
	while(loop || first_iter) {

		if (!PMD_LOOP_ENABLE && !first_iter)
			break;
		
		// Syncronization
		locker_frame_object.lock();		// Lock mutex_frame_object, any thread which used mutex_frame_object can NOT continue until unlock()
		while (!UPDATED_NEW_FRAME) {
			cv_frame_object.wait(locker_frame_object);
		}
		
		// check empty frames. The non-empty frame is rendered. If both are non-empty the amplitude is rendered
		if (first_iter) {
			if ((frame_00.rows <= 0) || (frame_00.cols <= 0))
				frame_00_empty = true;
			if ((frame_90.rows <= 0) || (frame_90.cols <= 0))
				frame_90_empty = true;
			if (frame_00_empty && frame_90_empty)
				return;
		}
		first_iter = false;

		// these lines are the only critical zone
		if (!frame_00_empty) {
			M_00 = cv::Mat(frame_00.rows, frame_00.cols, cv::DataType<float>::type);	// M_00 will also store the module: M_00 * M_00 + M_90 * M_90
			memcpy(M_00.data, frame_00.data.data(), frame_00.data.size()*sizeof(float));
			if (!frame_90_empty) {
				M_90 = cv::Mat(frame_90.rows, frame_90.cols, cv::DataType<float>::type);
				memcpy(M_90.data, frame_90.data.data(), frame_90.data.size()*sizeof(float));
			}
		} else {
			M_00 = cv::Mat(frame_90.rows, frame_90.cols, cv::DataType<float>::type);	// M_00 stores frame90 in this case, and M_90 does nothing
			memcpy(M_00.data, frame_90.data.data(), frame_90.data.size()*sizeof(float));
		}
		
		// Syncronization
		UPDATED_NEW_FRAME = false;
		UPDATED_NEW_SCENE = true;
		cv_frame_object.notify_all();	// Notify all cv_frame_object. All threads waiting for cv_frame_object will break the wait after waking up
		locker_frame_object.unlock();	// Unlock mutex_frame_object, now threads which used mutex_frame_object can continue
		
		// Get a the module matrix
		if (!frame_00_empty && !frame_90_empty) {
			for(int r = 0; r < M_00.rows; r++) {
				for(int c = 0; c < M_00.cols; c++) {
					M_00.at<float>(r,c) = (M_00.at<float>(r,c) * M_00.at<float>(r,c)) + (M_90.at<float>(r,c) * M_90.at<float>(r,c));
		}	}	}
		
		// Get a normalized matrix (min=0.0, max=1.0)
		double min, max, new_value;
		cv::minMaxLoc(M_00, &min, &max);
		max -= min;
		cv::MatIterator_<float> it, end;
		for(it = M_00.begin<float>(), end = M_00.end<float>(); it != end; ++it)
			(*it) = ((*it)-min) / max;
		cv::resize(M_00, M_00, cv::Size(), scale, scale, cv::INTER_NEAREST);

		// draw additional guide lines
		if (lines_grid >= 2) {
			for (int ri = 1; ri < lines_grid; ri++) {	// row lines
				int r = ri * (M_00.rows / lines_grid);
				cv::line(M_00, cv::Point2f(0, r), cv::Point2f(M_00.cols-1, r), cv::Scalar(0, 0, 0), 1, 8);
			}
			for (int ci = 1; ci < lines_grid; ci++) {	// column lines
				int c = ci * (M_00.cols / lines_grid);
				cv::line(M_00, cv::Point2f(c, 0), cv::Point2f(c, M_00.rows-1), cv::Scalar(0, 0, 0), 1, 8);
		}	}
		if (line_center) {
			cv::line(M_00, cv::Point2f(M_00.cols/2, 0), cv::Point2f(M_00.cols/2, M_00.rows-1), cv::Scalar(0, 0, 0), 2, 8);
			cv::line(M_00, cv::Point2f(0, M_00.rows/2), cv::Point2f(M_00.cols-1, M_00.rows/2), cv::Scalar(0, 0, 0), 2, 8);
		}

		// show window
		cv::imshow(windowName, M_00);
		cv::waitKey(1);
	}
	if (destroyWindow_)
		cv::destroyWindow(windowName);
}
// Plots a row (if >=0) or a col (otherwise and if >= 0) of a Frame using MATALAB engine
void plot_rowcol(Frame & frame, char* text, int row, int col, bool & epExtStarted, bool epExtUsing, Engine *epExt) {
	
	// MATLAB variables
	Engine *ep;
	mxArray *X = NULL;
	mxArray *V = NULL;
	if (epExtUsing)
		ep = epExt;

	// Variables. Array structure needed to deal with MATLAB functions
	int rc, size;
	if (row >= 0)
		size = frame.cols;
	else if (col >= 0)
		size = frame.rows;
	else
		return;
	double* x = new double[size];		// need to convert to double to deal with MATLAB
	double* value = new double[size];
	// Fill with the ampls of the Transient Pixel
	for (int i = 0; i < size; ++i) {
		x[i] = (double)i;
		if (row >= 0)
			value[i] = (double)(frame.at(row, i));
		else
			value[i] = (double)(frame.at(i, col));
	}

	// Call engOpen(""). This starts a MATLAB process on the current host using the command "matlab"
	if (!(ep = engOpen("")))
		fprintf(stderr, "\nCan't start MATLAB engine\n");
	
	// Create MATLAB variables from C++ variables
	X = mxCreateDoubleMatrix(1, size, mxREAL);
	V = mxCreateDoubleMatrix(1, size, mxREAL);
	memcpy((void *)mxGetPr(X), (void *)x, size*sizeof(x));
	memcpy((void *)mxGetPr(V), (void *)value, size*sizeof(value));
	
	// Place MATLAB variables into the MATLAB workspace
	engPutVariable(ep, "X", X);
	engPutVariable(ep, "V", V);
	
	// Plot the result
	engEvalString(ep, "plot(X,V);");
	char titleTxt[1024];
	char xTxt[1024] = "xlabel('%s of the %s #%d');";
	if (row >= 0) {
		sprintf(titleTxt,"title('%s. Values of each %s of the %s #%d');", text, "col", "row", row);
		sprintf(xTxt,"xlabel('%s of the %s #%d');", "cols", "row", row);
	} else {
		sprintf(titleTxt,"title('%s. Values of each %s of the %s #%d');", text, "row", "col", col);
		sprintf(xTxt,"xlabel('%s of the %s #%d');", "rows", "col", col);
	}
	engEvalString(ep, titleTxt);
	engEvalString(ep, xTxt);
	engEvalString(ep, "ylabel('values');");

	// use std::cin freeze the plot
	if (!epExtUsing) {
		std::cout << "\nWrite any std::string and click ENTER to continue\n";
		std::string answer;
		std::cin >> answer;
	}
	
	// Free memory, close MATLAB figure.
	mxDestroyArray(X);
	mxDestroyArray(V);
	if (!epExtUsing) {
		engEvalString(ep, "close;");
		engClose(ep);
	} else {
		epExtStarted = true;
	}
	delete [] x;
	delete [] value;
}
// Plots a row, a colum, the average of every row or the average every columns of 2 Frames using MATALAB engine
//   (row >= 0) && (col <  0) && (avg == false): Plots the raw
//   (row <  0) && (col >= 0) && (avg == false): Plots the col
//   (row >= 0) && (col <  0) && (avg == true ): Plots the average of every raw
//   (row <  0) && (col >= 0) && (avg == true ): Plots the average of every col
//   else: undefined behavior
void plot_rowcol2(Frame & frameR, Frame & frameS, char* textR, char* textS, int row, int col, bool avg, bool & epExtStarted, bool epExtUsing, Engine *epExt) {
	
	// MATLAB variables
	Engine *ep;
	mxArray *X = NULL;
	mxArray *VR = NULL;
	mxArray *VS = NULL;
	if (epExtUsing)
		ep = epExt;

	// Variables. Array structure needed to deal with MATLAB functions
	int size = frameR.cols;
	if (row < 0)
		size = frameR.rows;
	double* x = new double[size];		// need to convert to double to deal with MATLAB
	double* valueR = new double[size];
	double* valueS = new double[size];
	
	// Fill the corresponding values arrays
	if (avg) {
		if (row >= 0) {	// row, avg
			for (int c = 0; c < frameR.cols; ++c) {
				x[c] = (double)c;
				valueR[c] = 0.0;
				valueS[c] = 0.0;
				for (int r = 0; r < frameR.rows; ++r) {
					valueR[c] += (double)(frameR.at(r, c));
					valueS[c] += (double)(frameS.at(r, c));
				}
				valueR[c] /= (double)(frameR.rows);
				valueS[c] /= (double)(frameR.rows);
		}	} else {	// col, avg
			for (int r = 0; r < frameR.rows; ++r) {
				x[r] = (double)r;
				valueR[r] = 0.0; 
				valueS[r] = 0.0;
				for (int c = 0; c < frameR.cols; ++c) {
					valueR[r] += (double)(frameR.at(r, c));
					valueS[r] += (double)(frameS.at(r, c));
				}
				valueR[r] /= (double)(frameR.cols);
				valueS[r] /= (double)(frameR.cols);
	}	}	} else {
		if (row >= 0) {	// row, non-avg
			for (int c = 0; c < frameR.cols; ++c) {
				x[c] = (double)c;
				valueR[c] = (double)(frameR.at(row, c));
				valueS[c] = (double)(frameS.at(row, c));
		}	} else {	// col, non-avg
			for (int r = 0; r < frameR.rows; ++r) {
				x[r] = (double)r;
				valueR[r] = (double)(frameR.at(r, col)); 
				valueS[r] = (double)(frameS.at(r, col));
	}	}	}
	
	// Call engOpen(""). This starts a MATLAB process on the current host using the command "matlab"
	if (!(ep = engOpen("")))
		fprintf(stderr, "\nCan't start MATLAB engine\n");
	
	// Create MATLAB variables from C++ variables
	X = mxCreateDoubleMatrix(1, size, mxREAL);
	VR = mxCreateDoubleMatrix(1, size, mxREAL);
	VS = mxCreateDoubleMatrix(1, size, mxREAL);
	memcpy((void *)mxGetPr(X), (void *)x, size*sizeof(x));
	memcpy((void *)mxGetPr(VR), (void *)valueR, size*sizeof(valueR));
	memcpy((void *)mxGetPr(VS), (void *)valueS, size*sizeof(valueS));
	
	// Place MATLAB variables into the MATLAB workspace
	engPutVariable(ep, "X", X);
	engPutVariable(ep, "VR", VR);
	engPutVariable(ep, "VS", VS);
	
	// Plot the result
	engEvalString(ep, "plot(X, VR, 'b', X, VS, 'r');");
	char titleTxt[1024];
	char xTxt[1024];
	if (avg) {
		if (row >= 0) {
			sprintf(titleTxt,"title('Values of the averaged rows.   Blue=%s, Red=%s');", textR, textS);
			sprintf(xTxt,"xlabel('cols of the averaged rows');");
		} else {
			sprintf(titleTxt,"title('Values of the averaged cols.   Blue=%s, Red=%s');", textR, textS);
			sprintf(xTxt,"xlabel('rows of the averaged cols');");
	}	} else {
		if (row >= 0) {
			sprintf(titleTxt,"title('Values of each col of the row #%d.   Blue=%s, Red=%s');", row, textR, textS);
			sprintf(xTxt,"xlabel('cols of the row #%d');", row);
		} else {
			sprintf(titleTxt,"title('Values of each row of the col #%d.   Blue=%s, Red=%s');", col, textR, textS);
			sprintf(xTxt,"xlabel('rows of the col #%d');", col);
	}	}
	engEvalString(ep, titleTxt);
	engEvalString(ep, xTxt);
	engEvalString(ep, "ylabel('values');");
	//char legendTxt[1024];
	//sprintf(legendTxt,"legend('%s','%s');", textR, textS);
	//engEvalString(ep, legendTxt);		// the legend blinks iterating through an external ep

	// use std::cin freeze the plot
	if (!epExtUsing) {
		std::cout << "\nWrite any std::string and click ENTER to continue\n";
		std::string answer;
		std::cin >> answer;
	}
	
	// Free memory, close MATLAB figure.
	mxDestroyArray(X);
	mxDestroyArray(VR);
	mxDestroyArray(VS);
	if (!epExtUsing) {
		engEvalString(ep, "close;");
		engClose(ep);
	} else {
		epExtStarted = true;
	}
	delete [] x;
	delete [] valueR;
	delete [] valueS;
}
// Plots a row, a colum, the average of every row or the average every columns of 4 Frames using MATALAB engine
//   (row >= 0) && (col <  0) && (avg == false): Plots the raw
//   (row <  0) && (col >= 0) && (avg == false): Plots the col
//   (row >= 0) && (col <  0) && (avg == true ): Plots the average of every raw
//   (row <  0) && (col >= 0) && (avg == true ): Plots the average of every col
//   else: undefined behavior
void plot_rowcol4(Frame & frameR00, Frame & frameS00, Frame & frameR90, Frame & frameS90, char* textR00, char* textS00, char* textR90, char* textS90, int row, int col, bool avg, bool & epExtStarted, bool epExtUsing, Engine *epExt) {
	
	// MATLAB variables
	Engine *ep;
	mxArray *X = NULL;
	mxArray *VR00 = NULL;
	mxArray *VS00 = NULL;
	mxArray *VR90 = NULL;
	mxArray *VS90 = NULL;
	if (epExtUsing)
		ep = epExt;

	// Variables. Array structure needed to deal with MATLAB functions
	int size = frameR00.cols;
	if (row < 0)
		size = frameR00.rows;
	double* x = new double[size];			// need to convert to double to deal with MATLAB
	double* valueR00 = new double[size];
	double* valueS00 = new double[size];
	double* valueR90 = new double[size];
	double* valueS90 = new double[size];
	
	// Fill the corresponding values arrays
	if (avg) {
		if (row >= 0) {	// row, avg
			for (int c = 0; c < frameR00.cols; ++c) {
				x[c] = (double)c;
				valueR00[c] = 0.0;
				valueS00[c] = 0.0;
				valueR90[c] = 0.0;
				valueS90[c] = 0.0;
				for (int r = 0; r < frameR00.rows; ++r) {
					valueR00[c] += (double)(frameR00.at(r, c));
					valueS00[c] += (double)(frameS00.at(r, c));
					valueR90[c] += (double)(frameR90.at(r, c));
					valueS90[c] += (double)(frameS90.at(r, c));
				}
				valueR00[c] /= (double)(frameR00.rows);
				valueS00[c] /= (double)(frameR00.rows);
				valueR90[c] /= (double)(frameR00.rows);
				valueS90[c] /= (double)(frameR00.rows);
		}	} else {	// col, avg
			for (int r = 0; r < frameR00.rows; ++r) {
				x[r] = (double)r;
				valueR00[r] = 0.0; 
				valueS00[r] = 0.0;
				valueR90[r] = 0.0;
				valueS90[r] = 0.0;
				for (int c = 0; c < frameR00.cols; ++c) {
					valueR00[r] += (double)(frameR00.at(r, c));
					valueS00[r] += (double)(frameS00.at(r, c));
					valueR90[r] += (double)(frameR90.at(r, c));
					valueS90[r] += (double)(frameS90.at(r, c));
				}
				valueR00[r] /= (double)(frameR00.cols);
				valueS00[r] /= (double)(frameR00.cols);
				valueR90[r] /= (double)(frameR00.cols);
				valueS90[r] /= (double)(frameR00.cols);
	}	}	} else {
		if (row >= 0) {	// row, non-avg
			for (int c = 0; c < frameR00.cols; ++c) {
				x[c] = (double)c;
				valueR00[c] = (double)(frameR00.at(row, c));
				valueS00[c] = (double)(frameS00.at(row, c));
				valueR90[c] = (double)(frameR90.at(row, c));
				valueS90[c] = (double)(frameS90.at(row, c));
		}	} else {	// col, non-avg
			for (int r = 0; r < frameR00.rows; ++r) {
				x[r] = (double)r;
				valueR00[r] = (double)(frameR00.at(r, col)); 
				valueS00[r] = (double)(frameS00.at(r, col));
				valueR90[r] = (double)(frameR90.at(r, col)); 
				valueS90[r] = (double)(frameS90.at(r, col));
	}	}	}
	
	// Call engOpen(""). This starts a MATLAB process on the current host using the command "matlab"
	if (!(ep = engOpen("")))
		fprintf(stderr, "\nCan't start MATLAB engine\n");
	
	// Create MATLAB variables from C++ variables
	X = mxCreateDoubleMatrix(1, size, mxREAL);
	VR00 = mxCreateDoubleMatrix(1, size, mxREAL);
	VS00 = mxCreateDoubleMatrix(1, size, mxREAL);
	VR90 = mxCreateDoubleMatrix(1, size, mxREAL);
	VS90 = mxCreateDoubleMatrix(1, size, mxREAL);
	memcpy((void *)mxGetPr(X), (void *)x, size*sizeof(x));
	memcpy((void *)mxGetPr(VR00), (void *)valueR00, size*sizeof(valueR00));
	memcpy((void *)mxGetPr(VS00), (void *)valueS00, size*sizeof(valueS00));
	memcpy((void *)mxGetPr(VR90), (void *)valueR90, size*sizeof(valueR90));
	memcpy((void *)mxGetPr(VS90), (void *)valueS90, size*sizeof(valueS90));
	
	// Place MATLAB variables into the MATLAB workspace
	engPutVariable(ep, "X", X);
	engPutVariable(ep, "VR00", VR00);
	engPutVariable(ep, "VS00", VS00);
	engPutVariable(ep, "VR90", VR90);
	engPutVariable(ep, "VS90", VS90);
	
	// Plot the results of the figure 1 (00)
	engEvalString(ep, "figure(1);");
	engEvalString(ep, "plot(X, VR00, 'b', X, VS00, 'r');");
	char titleTxt00[1024];
	char xTxt00[1024];
	if (avg) {
		if (row >= 0) {
			sprintf(titleTxt00,"title('Values of the averaged rows.   Blue=%s, Red=%s');", textR00, textS00);
			sprintf(xTxt00,"xlabel('cols of the averaged rows');");
		} else {
			sprintf(titleTxt00,"title('Values of the averaged cols.   Blue=%s, Red=%s');", textR00, textS00);
			sprintf(xTxt00,"xlabel('rows of the averaged cols');");
	}	} else {
		if (row >= 0) {
			sprintf(titleTxt00,"title('Values of each col of the row #%d.   Blue=%s, Red=%s');", row, textR00, textS00);
			sprintf(xTxt00,"xlabel('cols of the row #%d');", row);
		} else {
			sprintf(titleTxt00,"title('Values of each row of the col #%d.   Blue=%s, Red=%s');", col, textR00, textS00);
			sprintf(xTxt00,"xlabel('rows of the col #%d');", col);
	}	}
	engEvalString(ep, titleTxt00);
	engEvalString(ep, xTxt00);
	engEvalString(ep, "ylabel('values');");
	if (!epExtUsing) {
		char legendTxt[1024];
		sprintf(legendTxt,"legend('%s','%s');", textR00, textS00);
		engEvalString(ep, legendTxt);		// the legend blinks iterating through an external ep
	}

	
	// Plot the results of the figure 2 (90)
	engEvalString(ep, "figure(2);");
	engEvalString(ep, "plot(X, VR90, 'b', X, VS90, 'r');");
	char titleTxt90[1024];
	char xTxt90[1024];
	if (avg) {
		if (row >= 0) {
			sprintf(titleTxt90,"title('Values of the averaged rows.   Blue=%s, Red=%s');", textR90, textS90);
			sprintf(xTxt90,"xlabel('cols of the averaged rows');");
		} else {
			sprintf(titleTxt90,"title('Values of the averaged cols.   Blue=%s, Red=%s');", textR90, textS90);
			sprintf(xTxt90,"xlabel('rows of the averaged cols');");
	}	} else {
		if (row >= 0) {
			sprintf(titleTxt90,"title('Values of each col of the row #%d.   Blue=%s, Red=%s');", row, textR90, textS90);
			sprintf(xTxt90,"xlabel('cols of the row #%d');", row);
		} else {
			sprintf(titleTxt90,"title('Values of each row of the col #%d.   Blue=%s, Red=%s');", col, textR90, textS90);
			sprintf(xTxt90,"xlabel('rows of the col #%d');", col);
	}	}
	engEvalString(ep, titleTxt90);
	engEvalString(ep, xTxt90);
	engEvalString(ep, "ylabel('values');");
	if (!epExtUsing) {
		char legendTxt[1024];
		sprintf(legendTxt,"legend('%s','%s');", textR90, textS90);
		engEvalString(ep, legendTxt);		// the legend blinks iterating through an external ep
	}

	// use std::cin freeze the plot
	if (!epExtUsing) {
		std::cout << "\nWrite any std::string and click ENTER to continue\n";
		std::string answer;
		std::cin >> answer;
	}
	
	// Free memory, close MATLAB figure.
	mxDestroyArray(X);
	mxDestroyArray(VR00);
	mxDestroyArray(VS00);
	mxDestroyArray(VR90);
	mxDestroyArray(VS90);
	if (!epExtUsing) {
		engEvalString(ep, "close;");
		engClose(ep);
	} else {
		epExtStarted = true;
	}
	delete [] x;
	delete [] valueR00;
	delete [] valueS00;
	delete [] valueR90;
	delete [] valueS90;
}


// ----------------------------------------------------------------------------------------------------------------------------------------
// ----- OTHER FUNCTIONS ------------------------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------------------

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

// returns the min, max, mean, variance value or index of the vector
float min(std::vector<float> & v) {
	float minf = v[0];
	for (size_t i = 1; i < v.size(); i++) {
		if (v[i] < minf)
			minf = v[i];
	}
	return minf;
}
float max(std::vector<float> & v) {
	float maxf = v[0];
	for (size_t i = 1; i < v.size(); i++) {
		if (v[i] > maxf)
			maxf = v[i];
	}
	return maxf;
}
float min(std::vector<float> & v, int & min_idx) {
	float minf = v[0];
	min_idx = 0;
	for (size_t i = 1; i < v.size(); i++) {
		if (v[i] < minf) {
			minf = v[i];
			min_idx = i;
	}	}
	return minf;
}
float max(std::vector<float> & v, int & max_idx) {
	float maxf = v[0];
	max_idx = 0;
	for (size_t i = 1; i < v.size(); i++) {
		if (v[i] > maxf) {
			maxf = v[i];
			max_idx = i;
	}	}
	return maxf;
}
float sum(std::vector<float> & v) {
	float sumf = v[0];
	for (size_t i = 1; i < v.size(); i++)
		sumf += v[i];
	return sumf;
}
float mean(std::vector<float> & v) {
	return sum(v) / v.size();
}
float var(std::vector<float> & v) {
	float meanf = mean(v);
	float diff = v[0] - meanf;
	float var = diff * diff;
	for (size_t i = 1; i < v.size(); i++) {
		diff = v[i] - meanf;
		var += diff * diff;
	}
	return var /= v.size();
}

// operates an element over a vector. Sizes must match, does not resizes
void sumElemToVector(float x, std::vector<float> & vIn, std::vector<float> & vOut) {
	for (size_t i = 1; i < vOut.size(); i++)
		vOut[i] = vIn[i] + x;
}
void subElemToVector(float x, std::vector<float> & vIn, std::vector<float> & vOut) {
	for (size_t i = 1; i < vOut.size(); i++)
		vOut[i] = vIn[i] - x;
}
void subPow2ElemToVector(float x, std::vector<float> & vIn, std::vector<float> & vOut) {
	for (size_t i = 1; i < vOut.size(); i++) {
		vOut[i] = vIn[i] - x;
		vOut[i] *= vOut[i];
	}
}
void mulElemToVector(float x, std::vector<float> & vIn, std::vector<float> & vOut) {
	for (size_t i = 1; i < vOut.size(); i++)
		vOut[i] = vIn[i] * x;
}
void divElemToVector(float x, std::vector<float> & vIn, std::vector<float> & vOut) {
	for (size_t i = 1; i < vOut.size(); i++)
		vOut[i] = vIn[i] / x;
}
// operates a vector over a vector. Sizes must match, does not resizes
void sumVectorToVector(std::vector<float> & vIn0, std::vector<float> & vIn1, std::vector<float> & vOut) {
	for (size_t i = 1; i < vOut.size(); i++)
		vOut[i] = vIn0[i] + vIn1[i];
}
void subVectorToVector(std::vector<float> & vIn0, std::vector<float> & vIn1, std::vector<float> & vOut) {
	for (size_t i = 1; i < vOut.size(); i++)
		vOut[i] = vIn0[i] - vIn1[i];
}
void subPow2VectorToVector(std::vector<float> & vIn0, std::vector<float> & vIn1, std::vector<float> & vOut) {
	for (size_t i = 1; i < vOut.size(); i++) {
		vOut[i] = vIn0[i] - vIn1[i];
		vOut[i] *= vOut[i];
	}
}
void mulVectorToVector(std::vector<float> & vIn0, std::vector<float> & vIn1, std::vector<float> & vOut) {
	for (size_t i = 1; i < vOut.size(); i++)
		vOut[i] = vIn0[i] * vIn1[i];
}
void divVectorToVector(std::vector<float> & vIn0, std::vector<float> & vIn1, std::vector<float> & vOut) {
	for (size_t i = 1; i < vOut.size(); i++)
		vOut[i] = vIn0[i] / vIn1[i];
}

// these functions returns the corresponding idx/stuff of a vector from r,c considering Matrix-like ordering 0-indexed.
int rc2idx(int r, int c, PixStoring ps, bool pSim) { // default: ps = PIXELS_STORING_GLOBAL, pSim = false
	if (pSim)
		return PMD_SIM_COLS * r + c;
	else if (ps == PIXELS_VALID)
		return CAMERA_PIX_X_VALID * r + c;
	else if (ps == PIXELS_TOTAL)
		return CAMERA_PIX_X * r + c;
	return -1;
}
int rc2idxCor(int r, int c, PixStoring ps, bool pSim) { // default: ps = PIXELS_STORING_GLOBAL, pSim = false
	if (pSim)
		return (PMD_SIM_COLS + 1) * r + c;
	else if (ps == PIXELS_VALID)
		return (CAMERA_PIX_X_VALID + 1) * r + c;
	else if (ps == PIXELS_TOTAL)
		return (CAMERA_PIX_X + 1) * r + c;
	return -1;
}
int rc2idxPT(int r, int c) {
	return CAMERA_PIX_X * r + c;
}
int rc2idxPV(int r, int c) {
	return CAMERA_PIX_X_VALID * r + c;
}
int rc2idxPSIM(int r, int c) {
	return PMD_SIM_COLS * r + c;
}
int rc2idxFromPT2PV(int r, int c, bool pSim) {	// returns -1 if the PT(r,c) is out of PV(r,c) range !!!
	if (pSim)
		return PMD_SIM_COLS * r + c;
	if ((r < CAMERA_PIX_Y_BAD_TOP) || (r >= (CAMERA_PIX_Y - CAMERA_PIX_Y_BAD_BOTTOM)) || (c < CAMERA_PIX_X_BAD_LEFT) || (c >= (CAMERA_PIX_X - CAMERA_PIX_X_BAD_RIGHT)))
		return -1;
	return CAMERA_PIX_X_VALID * (r - CAMERA_PIX_Y_BAD_TOP) + (c - CAMERA_PIX_X_BAD_LEFT);
}
int numPix(PixStoring ps, bool pSim) { // default: ps = PIXELS_STORING_GLOBAL, pSim = false
	if (pSim)
		return PMD_SIM_COLS * PMD_SIM_ROWS;
	else if (ps == PIXELS_VALID)
		return CAMERA_PIX_X_VALID * CAMERA_PIX_Y_VALID;
	else if (ps == PIXELS_TOTAL)
		return CAMERA_PIX_X * CAMERA_PIX_Y;
	return -1;
}
int numPixCor(PixStoring ps, bool pSim) { // default: ps = PIXELS_STORING_GLOBAL, pSim = false
	if (pSim)
		return (PMD_SIM_COLS + 1) * (PMD_SIM_ROWS + 1);
	else if (ps == PIXELS_VALID)
		return (CAMERA_PIX_X_VALID + 1) * (CAMERA_PIX_Y_VALID + 1);
	else if (ps == PIXELS_TOTAL)
		return (CAMERA_PIX_X + 1) * (CAMERA_PIX_Y + 1);
	return -1;
}
int numPixPT() {
	return CAMERA_PIX_X * CAMERA_PIX_Y;
}
int numPixPV() {
	return CAMERA_PIX_X_VALID * CAMERA_PIX_Y_VALID;
}
int numPixPSIM() {
	return PMD_SIM_COLS * PMD_SIM_ROWS;
}
int rows(PixStoring ps, bool pSim) { // default: ps = PIXELS_STORING_GLOBAL, pSim = false
	if (pSim)
		return PMD_SIM_ROWS;
	else if (ps == PIXELS_VALID)
		return CAMERA_PIX_Y_VALID;
	else if (ps == PIXELS_TOTAL)
		return CAMERA_PIX_Y;
	return -1;
}
int cols(PixStoring ps, bool pSim) { // default: ps = PIXELS_STORING_GLOBAL, pSim = false
	if (pSim)
		return PMD_SIM_COLS;
	else if (ps == PIXELS_VALID)
		return CAMERA_PIX_X_VALID;
	else if (ps == PIXELS_TOTAL)
		return CAMERA_PIX_X;
	return -1;
}
int rowsCor(PixStoring ps, bool pSim) { // default: ps = PIXELS_STORING_GLOBAL, pSim = false
	if (pSim)
		return PMD_SIM_ROWS + 1;
	else if (ps == PIXELS_VALID)
		return CAMERA_PIX_Y_VALID + 1;
	else if (ps == PIXELS_TOTAL)
		return CAMERA_PIX_Y + 1;
	return -1;
}
int colsCor(PixStoring ps, bool pSim) { // default: ps = PIXELS_STORING_GLOBAL, pSim = false
	if (pSim)
		return PMD_SIM_COLS + 1;
	else if (ps == PIXELS_VALID)
		return CAMERA_PIX_X_VALID + 1;
	else if (ps == PIXELS_TOTAL)
		return CAMERA_PIX_X + 1;
	return -1;
}

// returns the corresponding index. Return -1 if no correspondance found
int get_freq_idx (Info & info, float freq, float RelDiffMax) {
	for (size_t i = 0; i < info.freqV.size(); i++) {
		if (equalAproxf(freq, info.freqV[i]))
			return i;
	}
	return -1;
}
int get_dist_idx (Info & info, float dist, float RelDiffMax) {
	for (size_t i = 0; i < info.distV.size(); i++) {
		if (equalAproxf(dist, info.distV[i]))
			return i;
	}
	return -1;
}
int get_shut_idx (Info & info, float shut, float RelDiffMax) {
	for (size_t i = 0; i < info.shutV.size(); i++) {
		if (equalAproxf(shut, info.shutV[i]))
			return i;
	}
	return -1;
}
int get_phas_idx (Info & info, float phas, float RelDiffMax) {
	for (size_t i = 0; i < info.phasV.size(); i++) {
		if (equalAproxf(phas, info.phasV[i]))
			return i;
	}
	return -1;
}

// other auxiliar functions
bool equalAproxf (float f0, float f1, float RelDiffMax) {
	float diff = f1 - f0;
	float diffMax = abs(f0 * RelDiffMax);
	if (diffMax < 0.000001)	// this may happen if f0 is 0.0f or close to 0.0f
		diffMax = 0.000001;
	return ((diff < diffMax) && (diff > -diffMax));
}
void print(std::vector<float> & v, char* prefix, char* sufix) {
	if (prefix)
		std::cout << prefix;
	if (v.size() <= 0) {
		std::cout << "[empty]";
	} else {
		std::cout << "[" << v[0];
		for (size_t i = 1; i < v.size(); i++)
			std::cout << ", " << v[i];
		std::cout  << "]";
	}
	if (sufix)
		std::cout << sufix;
}
void print(std::vector<int> & v, char* prefix, char* sufix) {
	if (prefix)
		std::cout << prefix;
	if (v.size() <= 0) {
		std::cout << "[empty]";
	}
	else {
		std::cout << "[" << v[0];
		for (size_t i = 1; i < v.size(); i++)
			std::cout << ", " << v[i];
		std::cout << "]";
	}
	if (sufix)
		std::cout << sufix;
}
