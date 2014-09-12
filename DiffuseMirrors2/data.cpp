
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
	// empty: crashes ´when trying to delete any char*
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
		std::cout << "\n\nSize Incoherence Error while getting size of \""<< raw_fn << "\"\n\n";
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
	//free (data_size_);

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
					frame_sim_PT.set((*this), fi, di, si, pi, PIXELS_TOTAL, false);
					frame_sim_PV.set((*this), fi, di, si, pi, PIXELS_VALID, false);
					cv::Mat frame_sim_PT_Mat = cv::Mat(frame_sim_PT.rows, frame_sim_PT.cols, cv::DataType<float>::type);
					cv::Mat frame_sim_PV_Mat = cv::Mat(frame_sim_PV.rows, frame_sim_PV.cols, cv::DataType<float>::type);
					memcpy(frame_sim_PT_Mat.data, frame_sim_PT.data.data(), frame_sim_PT.data.size() * sizeof(float));
					memcpy(frame_sim_PV_Mat.data, frame_sim_PV.data.data(), frame_sim_PV.data.size() * sizeof(float));
					cv::resize(frame_sim_PT_Mat, frame_sim_PT_Mat, cv::Size(PMD_SIM_COLS, PMD_SIM_ROWS), scalePTcols, scalePTrows, cv::INTER_AREA);
					cv::resize(frame_sim_PV_Mat, frame_sim_PV_Mat, cv::Size(PMD_SIM_COLS, PMD_SIM_ROWS), scalePVcols, scalePVrows, cv::INTER_AREA);
					memcpy(data_sim_PT + data_idx(fi, di, si, pi, 0, 0, PIXELS_TOTAL, true), frame_sim_PT_Mat.data, frame_sim_PT_Mat.rows * frame_sim_PT_Mat.cols * sizeof(float));
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
CalibrationMatrix::CalibrationMatrix(Info & info_, float* C_, int C_size_, std::vector<float> & pathDist0_, int error_code_) { // by default: error_code_ = 0
	set (info_, C_, C_size_, pathDist0_, error_code_);
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
}
// Constructor All parameters
void CalibrationMatrix::set (Info & info_, float* C_, int C_size_, std::vector<float> & pathDist0_, int error_code_) { // by default: error_code_ = 0
	
	// External Parameters (RawData, Info)
	info = &info_;

	// Calibration Matrix Parameters
	C = C_;
	C_size = C_size_;
	pathDist0 = pathDist0_;
	error_code = error_code_;
}
// Constructor. It creates a CalibrationMatrix object from the .cmx file noted in the info object
void CalibrationMatrix::set (Info & info_) { // by default: pixels_storing_ = PIXELS_VALID
	
	// External Parameters (Info)
	info = &info_;

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

	// Calibration Matrix Parameters: pathDist0
	pathDist0 = std::vector<float>(numPix(PIXELS_TOTAL)); // independent of PixStoring, the accessing depends on it. Ordering: for(r){ for(c){ // here...}} // r,c Matrix-like, 0-idx
	Scene scene;	scene.clear();
	scene.setScene_CalibrationMatrix(info_.laser_to_cam_offset_x, info_.laser_to_cam_offset_y, info_.laser_to_cam_offset_z, info_.dist_wall_cam);
	for (size_t i = 0; i < numPix(PIXELS_TOTAL); i++)	// TO-DO: CHECK
		pathDist0[i] = dist(scene.o[LASER].s[0].c, scene.o[WALL_PATCHES].s[i].c) + dist(scene.o[CAMERA].s[0].c, scene.o[WALL_PATCHES].s[i].c);
	scene.clear();
	error_code = 0;		// no errors
}


// ----- FUNCTIONS -------------------------------

// Returns the index in C[], corresponding to the parameter indices. 
// input r,c are considered like Matrix from 0 indexation. It also takes care on PixStoring
int CalibrationMatrix::C_idx (int freq_idx, int dist_idx, int r, int c, PixStoring ps) { // default: ps = PIXELS_STORING_GLOBAL

	if (ps == PIXELS_VALID) {
		return info->distV.size() * info->rows * info->cols * freq_idx +
	                                info->rows * info->cols * dist_idx +
	                                             info->cols * (r + CAMERA_PIX_Y_BAD_TOP) +
													          (c + CAMERA_PIX_X_BAD_LEFT);
	} else if (ps == PIXELS_TOTAL) {
		return info->distV.size() * info->rows * info->cols * freq_idx +
	                                info->rows * info->cols * dist_idx +
	                                             info->cols * r        + c;
	}
}
// Returns C[C.C_idx(...)], the value from the Calibration Matrix C corresponding to the parameters
// input r,c are considered like Matrix from 0 indexation. It also takes care on PixStoring
float CalibrationMatrix::C_at (int freq_idx, int dist_idx, int r, int c, PixStoring ps) { // default: ps = PIXELS_STORING_GLOBAL
	return C[C_idx(freq_idx, dist_idx, r, c, ps)];
}
// Returns the C[...] interpolating with the closest path distances
// input r,c are considered like Matrix from 0 indexation. It also takes care on PixStoring
float CalibrationMatrix::C_atX (int freq_idx, float pathDist, int r, int c, PixStoring ps)  { // default: ps = PIXELS_STORING_GLOBAL
	
	float pathDist_offset = pathDist - pathDist0_at(r,c);
	float pathDist_res = (info->distV[1] - info->distV[0]);	// dist of info store actually path distances as well

	// pathDist Indices
	int pathDist_idx_floor = (pathDist_offset - info->distV[0]) / pathDist_res;
	int pathDist_idx_ceil = pathDist_idx_floor + 1;
	// checking if out of bounds
	if (pathDist_idx_floor < 0) {
		std::cout << "\nWarning: path_dist(" << r << "," << c << ") = " << pathDist << " out of .cmx min bound = " << info->distV[0] << " ---> pathDist\n";
		return C_at(freq_idx, 0, r, c, ps);
	} else if (pathDist_idx_floor >= info->distV.size() - 1) {
		std::cout << "\nWarning: path_dist(" << r << "," << c << ") = " << pathDist << " out of .cmx max bound = " << info->distV[info->distV.size()-1] << " ---> pathDist\n";
		return C_at(freq_idx, info->distV.size()-1, r, c, ps);
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
	return (pathDist_scale_floor * C_at(freq_idx, pathDist_idx_floor, r, c, ps)) + (pathDist_scale_ceil * C_at(freq_idx, pathDist_idx_ceil, r, c, ps));
}

// Returns the index in pathDist0, corresponding to the parameter indices.
// input r,c are considered like Matrix from 0 indexation. It also takes care on PixStoring
int CalibrationMatrix::pathDist0_idx (int r, int c, PixStoring ps) { // default: ps = PIXELS_STORING_GLOBAL
	
	if (ps == PIXELS_VALID) {
		return info->cols * (r + CAMERA_PIX_Y_BAD_TOP) +
			                (c + CAMERA_PIX_X_BAD_LEFT);
	} else if (ps == PIXELS_TOTAL) {
		return info->cols * r + c;
	}
}
// Returns the value from the pathDist0 corresponding to the parameter indices = pathDist0[pathDist0_idx]
// input r,c are considered like Matrix from 0 indexation. It also takes care on PixStoring
float CalibrationMatrix::pathDist0_at (int r, int c, PixStoring ps) { // default: ps = PIXELS_STORING_GLOBAL
	return pathDist0[pathDist0_idx(r,c,ps)];
}
	

// This is the Simulation term for the direct vision problem: S_{i\;\omega}^{r,c}(\tau^{r,c}) in the Master Thesis document
// Returns the value of the Simulation from the Calibration Matrix C at any path distance interpolating with the closest path distances. Path distances have to be equidistant in vector
// Uses C_atX(...) for interpolating path distances
float CalibrationMatrix::S_DirectVision (int freq_idx, int r, int c, Point & r_src, Point & r_x,  Point & r_cam, float relative_albedo, PixStoring ps) { // by default: relative_albedo = 1.0f, ps = PIXELS_STORING_GLOBAL

	float dist_src_x = dist(r_src, r_x);
	float dist_cam_x = dist(r_cam, r_x);
	float pathDist = dist_src_x + dist_cam_x;
	
	return C_atX (freq_idx, pathDist, r, c,ps) * relative_albedo / (dist_src_x * dist_src_x);
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
Frame::Frame(unsigned short int* data_, int rows_, int cols_, float freq_, float dist_, float shut_, float phas_, int phas_idx_, PixStoring ps_, bool pSim_) { // by default: ps_ = PIXELS_STORING_GLOBAL
	set (data_, rows_, cols_, freq_, dist_, shut_, phas_, phas_idx_, ps_, pSim_);
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
			data[data_idx(r,c)] = RawData_src->atF(freq_idx, dist_idx, shut_idx, phas_idx, r, c, ps, pSim);
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
// Setter from ushort int*. Real Time capture oriented. rows_ and cols_ must be refered to the sizes of the total frame, regerdingless to the PixStoring
void Frame::set(unsigned short int* data_, int rows_, int cols_, float freq_, float dist_, float shut_, float phas_, int phas_idx_, PixStoring ps_, bool pSim_) { // by default: ps_ = PIXELS_STORING_GLOBAL
	
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
		rows = RawData_src->info->rows;
		cols = RawData_src->info->cols;
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
				idx_in_data = (rows_ * cols_ * phas_idx_) + (cols_ * (CAMERA_PIX_Y - CAMERA_PIX_Y_BAD_TOP - 1 - r)) + (c + CAMERA_PIX_X_BAD_LEFT);
				data[data_idx(r,c)] = (float)(data_[idx_in_data] - 32768); // data is not stored properly in raw file. -32768 fixes it
	}	}	}
	else if (ps == PIXELS_TOTAL) {
		for (int r = 0; r < rows; r++) {
			for (int c = 0; c < cols; c++) {
				idx_in_data = (rows_ * cols_ * phas_idx_) + (cols_ * (rows_ - 1 - r)) + c;
				data[data_idx(r,c)] = (float)(data_[idx_in_data] - 32768); // data is not stored properly in raw file. -32768 fixes it
	}	}	}
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
void Frame::plot(int delay_ms, bool destroyWindow_, char* windowName) { // by default: delay_ms = 1000, destroyWindow = false, windowName = NULL

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
	if (windowName == NULL)
		windowName = "Frame.plot()";
	cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);	// WINDOW_NORMAL, WINDOW_AUTOSIZE
	cv::imshow(windowName, M_norm);
	cv::waitKey(delay_ms);
	if (destroyWindow_)
		cv::destroyWindow(windowName);
}
// plot frame amplitude with sinusoidal assumption
void plot_frame(Frame & frame_00, Frame & frame_90, int delay_ms, bool destroyWindow_, char* windowName) { // by default: delay_ms = 1000, destroyWindow = false, windowName = NULL

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
	if (windowName == NULL)
		windowName = "plot_frame(Frame,Frame)";
	cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);	// WINDOW_NORMAL, WINDOW_AUTOSIZE
	cv::imshow(windowName, M_out);
	cv::waitKey(delay_ms);
	if (destroyWindow_)
		cv::destroyWindow(windowName);
}
// For FoV measurement scene. Plot frame with opencv with syncronization
void plot_frame_fov_measurement(Frame & frame_00, Frame & frame_90, bool loop, bool destroyWindow_, char* windowName) { // by default: loop = false, destroyWindow = false, windowName = NULL

	// Syncronization
	std::unique_lock<std::mutex> locker_frame_object;	// Create a defered locker (a locker not locked yet)
	locker_frame_object = std::unique_lock<std::mutex>(mutex_frame_object,std::defer_lock);

	// show the image
	cv::Mat M_00, M_90;
	int scale = 10;
	bool first_iter = true;
	if (windowName == NULL)
		windowName = "plot_frame_fov_measurement(Frame,Frame)";
	cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);	// WINDOW_NORMAL, WINDOW_AUTOSIZE

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

		// this 6 lines are the only critical zone
		/*
		if ((FRAME_90_CAPTURE.width <= 0) || (FRAME_90_CAPTURE.heigth <= 0)) {
			return;
		}
		M_norm = FRAME_90_CAPTURE.matrix.clone();
		*/
		if ((frame_00.rows <= 0) || (frame_00.cols <= 0) || (frame_90.rows <= 0) || (frame_90.cols <= 0))
			return;
		M_00 = cv::Mat(frame_00.rows, frame_00.cols, cv::DataType<float>::type); // M_00 will also store the module: M_00 * M_00 + M_90 * M_90
		M_90 = cv::Mat(frame_90.rows, frame_90.cols, cv::DataType<float>::type);
		memcpy(M_00.data, frame_00.data.data(), frame_00.data.size()*sizeof(float));
		memcpy(M_90.data, frame_90.data.data(), frame_90.data.size()*sizeof(float));
		
		// Syncronization
		UPDATED_NEW_FRAME = false;
		UPDATED_NEW_SCENE = true;
		cv_frame_object.notify_all();	// Notify all cv_frame_object. All threads waiting for cv_frame_object will break the wait after waking up
		locker_frame_object.unlock();	// Unlock mutex_frame_object, now threads which used mutex_frame_object can continue
		
		// Get a the module matrix
		for(int r = 0; r < M_00.rows; r++) {
			for(int c = 0; c < M_00.cols; c++) {
				M_00.at<float>(r,c) = (M_00.at<float>(r,c) * M_00.at<float>(r,c)) + (M_90.at<float>(r,c) * M_90.at<float>(r,c));
		}	}
		
		// Get a normalized matrix (min=0.0, max=1.0)
		double min, max, new_value;
		cv::minMaxLoc(M_00, &min, &max);
		max -= min;
		cv::MatIterator_<float> it, end;
		for(it = M_00.begin<float>(), end = M_00.end<float>(); it != end; ++it)
			(*it) = ((*it)-min) / max;
		cv::resize(M_00, M_00, cv::Size(), scale, scale, cv::INTER_NEAREST);

		// show window
		cv::imshow(windowName, M_00);
		cv::waitKey(20);
	}
	if (destroyWindow_)
		cv::destroyWindow(windowName);
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
float mean(std::vector<float> & v) {
	float meanf = v[0];
	for (size_t i = 1; i < v.size(); i++)
		meanf += v[i];
	return meanf /= v.size();
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

// these functions returns the corresponding idx/stuff of a vector from r,c considering Matrix-like ordering 0-indexed.
int rc2idx(int r, int c, PixStoring ps) { // default: ps = PIXELS_STORING_GLOBAL
	if (ps == PIXELS_VALID)
		return CAMERA_PIX_X_VALID * r + c;
	else if (ps == PIXELS_TOTAL)
		return CAMERA_PIX_X * r + c;
	return -1;
}
int rc2idxCor(int r, int c, PixStoring ps) { // default: ps = PIXELS_STORING_GLOBAL
	if (ps == PIXELS_VALID)
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
int rc2idxFromPT2PV(int r, int c) {	// returns -1 if the PT(r,c) is out of PV(r,c) range !!!
	if ((r < CAMERA_PIX_Y_BAD_TOP) || (r >= (CAMERA_PIX_Y - CAMERA_PIX_Y_BAD_BOTTOM)) || (c < CAMERA_PIX_X_BAD_LEFT) || (c >= (CAMERA_PIX_X - CAMERA_PIX_X_BAD_RIGHT)))
		return -1;
	return CAMERA_PIX_X_VALID * (r - CAMERA_PIX_Y_BAD_TOP) + (c - CAMERA_PIX_X_BAD_LEFT);
}
int rc2idxPSIM(int r, int c) {
	return PMD_SIM_COLS * r + c;
}
int numPix(PixStoring ps) {
	if (ps == PIXELS_VALID)
		return CAMERA_PIX_X_VALID * CAMERA_PIX_Y_VALID;
	else if (ps == PIXELS_TOTAL)
		return CAMERA_PIX_X * CAMERA_PIX_Y;
	return -1;
}
int numPixCor(PixStoring ps) {
	if (ps == PIXELS_VALID)
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

int rows(PixStoring ps) {
	if (ps == PIXELS_VALID)
		return CAMERA_PIX_Y_VALID;
	else if (ps == PIXELS_TOTAL)
		return CAMERA_PIX_Y;
	return -1;
}
int cols(PixStoring ps) {
	if (ps == PIXELS_VALID)
		return CAMERA_PIX_X_VALID;
	else if (ps == PIXELS_TOTAL)
		return CAMERA_PIX_X;
	return -1;
}
int rowsCor(PixStoring ps) {
	if (ps == PIXELS_VALID)
		return CAMERA_PIX_Y_VALID + 1;
	else if (ps == PIXELS_TOTAL)
		return CAMERA_PIX_Y + 1;
	return -1;
}
int colsCor(PixStoring ps) {
	if (ps == PIXELS_VALID)
		return CAMERA_PIX_X_VALID + 1;
	else if (ps == PIXELS_TOTAL)
		return CAMERA_PIX_X + 1;
	return -1;
}

bool equalAproxf (float f0, float f1, float RelDiffMax) {
	float diff = f1 - f0;
	float diffMax = abs(f0 * RelDiffMax);
	if (diffMax < 0.000001)	// this may happen if f0 is 0.0f or close to 0.0f
		diffMax = 0.000001;
	return ((diff < diffMax) && (diff > -diffMax));
}
void print(std::vector<float> v, char* prefix, char* sufix) {
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

