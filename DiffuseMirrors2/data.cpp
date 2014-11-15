
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
	/*
	std::cout << "\nfreqV.size() = "<< info->freqV.size();
	std::cout << "\ndistV.size() = "<< info->distV.size();
	std::cout << "\nshutV.size() = "<< info->shutV.size();
	std::cout << "\nphasV.size() = "<< info->phasV.size() << "\n";
	*/

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
// Constructor Copy. pointers are copied "as are", the C pointer is not duplicated. For this use .clone(...) (if implemented)
CalibrationMatrix::CalibrationMatrix(CalibrationMatrix & cmx) {
	set (cmx);
}
// Constructor. It creates a CalibrationMatrix object from the Info and the averaging region bounds
CalibrationMatrix::CalibrationMatrix(Info & info_, int avgRowMin_, int avgRowMax_, int avgColMin_, int avgColMax_) { // by default: pixels_storing_ = PIXELS_VALID
	set (info_, avgRowMin_, avgRowMax_, avgColMin_, avgColMax_);
}
// Destructor
CalibrationMatrix::~CalibrationMatrix() {
	delete[] C;
}


// ----- SETTERS --------------------------------- // Note that each Constructor just contains its corresponding Setter

// Setter Default
void CalibrationMatrix::set () {

	// External Parameters (RawData, Info)
	info = NULL;

	// Calibration Matrix Parameters
	C = NULL;
	C_size = 0;

	// Averaging region
	avgRowMin = 0;
	avgRowMax = 0;
	avgColMin = 0;
	avgColMax = 0;
	avgPixels = 0;

	// pathDist
	pathDistV = std::vector<float>(0);
	distRes = 0.0f;
	distResInv = 0.0f;
	pathDistC = 0.0f;
	pathDistCmin = 0.0f;
	pathDistCmax = 0.0f;
	phasErrorMax = 0.0f;

	// saturation
	saturation = false;
	hAbsMax = 0.0f;
	hSatRatio = 0.0f;

	// errors
	error_code = 0;
}
// Setter Copy. pointers are copied "as are", the C pointer is not duplicated. For this use .clone(...) (if implemented)
void CalibrationMatrix::set (CalibrationMatrix & cmx) {

	// External Parameters (RawData, Info)
	info = cmx.info;

	// Calibration Matrix Parameters
	C = cmx.C;
	C_size = cmx.C_size;

	// Averaging region
	avgRowMin = cmx.avgRowMin;
	avgRowMax = cmx.avgRowMax;
	avgColMin = cmx.avgColMin;
	avgColMax = cmx.avgColMax;
	avgPixels = cmx.avgPixels;

	// pathDist
	pathDistV = cmx.pathDistV;
	distRes = cmx.distRes;
	distResInv = cmx.distResInv;
	pathDistC = cmx.pathDistC;
	pathDistCmin = cmx.pathDistCmin;
	pathDistCmax = cmx.pathDistCmax;
	phasErrorMax = cmx.phasErrorMax;

	// saturation
	saturation = cmx.saturation;
	hAbsMax = cmx.hAbsMax;
	hSatRatio = cmx.hSatRatio;

	// errors
	error_code = cmx.error_code;
}
// Setter. It creates a CalibrationMatrix object from the Info and the averaging region bounds
void CalibrationMatrix::set (Info & info_, int avgRowMin_, int avgRowMax_, int avgColMin_, int avgColMax_) { 
	
	// External Parameters (Info)
	info = &info_;
	
	// RawData
	RawData rawData(info_);
	int si = info->shutV.size() - 1;
	PixStoring rawData_ps = PIXELS_TOTAL;
	bool rawData_pSim = false;
	
	// Averaging region
	avgRowMin = avgRowMin_;
	avgRowMax = avgRowMax_;
	avgColMin = avgColMin_;
	avgColMax = avgColMax_;
	avgPixels = (avgRowMax - avgRowMin + 1) * (avgColMax - avgColMin + 1);
	 
	// C_size, C
	C_size = info->freqV.size() * info->distV.size() * info->phasV.size();
	C = new float[C_size];
	// filling C
	for (size_t fi = 0; fi < info->freqV.size(); ++fi) {
		for (size_t di = 0; di < info->distV.size(); ++di) {
			for (size_t pi = 0; pi < info->phasV.size(); ++pi) {
				C[C_idx(fi, di, pi)] = 0.0f;
				for (int r = avgRowMin; r <= avgRowMax; ++r) {
					for (int c = avgColMin; c <= avgColMax; ++c) {
						C[C_idx(fi, di, pi)] += rawData.atF(fi, di, si, pi, r,c, rawData_ps, rawData_pSim);
				}	}
				C[C_idx(fi, di, pi)] /= (float)avgPixels;
	}	}	}


	// Scene
	Scene scene;
	scene.clear();
	scene.setScene_CalibrationMatrix(info_.laser_to_cam_offset_x, info_.laser_to_cam_offset_y, info_.laser_to_cam_offset_z, info_.dist_wall_cam, rawData_ps, rawData_pSim);

	// pathDist
	Point avgRegionCenter = (scene.o[WALL_PATCHES].s[avgRowMin * cols(rawData_ps, rawData_pSim) + avgColMin].c +
							 scene.o[WALL_PATCHES].s[avgRowMin * cols(rawData_ps, rawData_pSim) + avgColMax].c +
							 scene.o[WALL_PATCHES].s[avgRowMax * cols(rawData_ps, rawData_pSim) + avgColMin].c +
							 scene.o[WALL_PATCHES].s[avgRowMax * cols(rawData_ps, rawData_pSim) + avgColMax].c ) / 4.0f;
	pathDistC = distPath3(scene.o[LASER].s[0].c, avgRegionCenter, scene.o[CAMERA].s[0].c);
	pathDistV = std::vector<float>(info->distV.size());
	for (size_t i = 0; i < pathDistV.size(); ++i) {
		pathDistV[i] = info->distV[i] + pathDistC;
	}
	// pathDistCmin/max/error
	std::vector<float> pathDistCall(avgPixels);
	for (int r = avgRowMin; r <= avgRowMax; ++r) {
		for (int c = avgColMin; c <= avgColMax; ++c) {
			int idx = r * cols(rawData_ps, rawData_pSim) + c;
			pathDistCall[idx] = distPath3(scene.o[LASER].s[0].c, scene.o[WALL_PATCHES].s[idx].c, scene.o[CAMERA].s[0].c);
	}	}
	pathDistCmin = min(pathDistCall);
	pathDistCmax = max(pathDistCall);
	phasErrorMax = (pathDistCmax - pathDistCmin) * ((info->freqV[info->freqV.size()-1] * 1E+06) / C_LIGHT_AIR);
	// distRes/Inv
	distRes = info->distV[1] - info->distV[0];
	distResInv = 1.0f / distRes;

	// saturation
	hAbsMax = 0.0f;
	for (int i = 0; i < rawData.data_size; ++i) {
		if (abs(rawData.data[i]) > hAbsMax) {
			hAbsMax = abs(rawData.data[i]);
	}	}
	hSatRatio = hAbsMax / 32768.0f;
	saturation = false;
	if (hSatRatio > H_SAT_RATIO_MAX)
		saturation = true;

	// errors
	error_code = 0;
}


// ----- FUNCTIONS -------------------------------

// Returns the index in C[], corresponding to the parameter indices. 
int CalibrationMatrix::C_idx (int freqIdx, int patdDistIdx, int phasIdx) { 
	return info->distV.size() * info->phasV.size() * freqIdx + info->phasV.size() * patdDistIdx + phasIdx;
}
// Returns the index in C[], corresponding to the parameter indices for all the phases (0,90). 
void CalibrationMatrix::C_idx_allPhas (int freqIdx, int patdDistIdx, int & C_idx_00, int & C_idx_90) { 
	C_idx_00 =  info->distV.size() * info->phasV.size() * freqIdx + info->phasV.size() * patdDistIdx;
	C_idx_90 =  C_idx_00 + 1;
}

// Returns C[C.C_idx(...)], the value from the Calibration Matrix C corresponding to the parameters
float CalibrationMatrix::C_at (int freqIdx, int patdDistIdx, int phasIdx) {
	return C[C_idx(freqIdx, patdDistIdx, phasIdx)];
}
// Returns C[C.C_idx(...)], the value from the Calibration Matrix C corresponding to the parameters for all the phases (0,90). 
void CalibrationMatrix::C_at_allPhas (int freqIdx, int patdDistIdx, float & C_00, float & C_90) {
	int C_idx_00, C_idx_90;
	C_idx_allPhas(freqIdx, patdDistIdx, C_idx_00, C_idx_90);
	C_00 = C[C_idx_00];
	C_90 = C[C_idx_90];
}

// Returns the C[...] interpolating with the closest patdDist's
float CalibrationMatrix::C_atX (int freqIdx, float pathDist, int phasIdx)  { 

	// pathDist Indices
	float pathDist_idx_float = (pathDist - pathDistV[0]) * distResInv;
	int pathDist_idx_floor = (int)pathDist_idx_float;
	int pathDist_idx_ceil = pathDist_idx_floor + 1;
	// checking if out of bounds
	if (pathDist_idx_floor < 0) {
		std::cout << "\nWarning: pathDist = " << pathDist << " out of Cmin bound = " << pathDistV[0] << " ---> pathDist\n";
		return C_at(freqIdx, 0, phasIdx);
	} else if (pathDist_idx_ceil >= pathDistV.size()) {
		std::cout << "\nWarning: pathDist = " << pathDist << " out of Cmax bound = " << pathDistV[pathDistV.size()-1] << " ---> pathDist\n";
		return C_at(freqIdx, pathDistV.size()-1, phasIdx);
	}

	// pathDist Scalings
	float pathDist_scale_ceil = pathDist_idx_float - (float)pathDist_idx_floor;
	float pathDist_scale_floor = 1.0f - pathDist_scale_ceil;
	return (pathDist_scale_floor * C_at(freqIdx, pathDist_idx_floor, phasIdx)) + (pathDist_scale_ceil * C_at(freqIdx, pathDist_idx_ceil, phasIdx));
}
// Returns the C[...] interpolating with the closest patdDist's for all the phases (0,90). 
void CalibrationMatrix::C_atX_allPhas (int freqIdx, float pathDist, float & Cx_00, float & Cx_90)  { 
	// pathDist Indices
	float pathDist_idx_float = (pathDist - pathDistV[0]) * distResInv;
	int pathDist_idx_floor = (int)pathDist_idx_float;
	int pathDist_idx_ceil = pathDist_idx_floor + 1;
	// checking if out of bounds
	if (pathDist_idx_floor < 0) {
		std::cout << "\nWarning: pathDist = " << pathDist << " out of Cmin bound = " << pathDistV[0] << " ---> pathDist\n";
		C_at_allPhas(freqIdx, 0, Cx_00, Cx_90);
		return;
	} else if (pathDist_idx_ceil >= pathDistV.size()) {
		std::cout << "\nWarning: pathDist = " << pathDist << " out of Cmax bound = " << pathDistV[pathDistV.size()-1] << " ---> pathDist\n";
		C_at_allPhas(freqIdx, pathDistV.size()-1, Cx_00, Cx_90);
		return;
	}

	// pathDist Scalings
	float pathDist_scale_ceil = pathDist_idx_float - (float)pathDist_idx_floor;
	float pathDist_scale_floor = 1.0f - pathDist_scale_ceil;
	int Cx_00_floorIdx = C_idx (freqIdx, pathDist_idx_floor, 0);
	int Cx_90_floorIdx = Cx_00_floorIdx + 1;
	int Cx_00_ceilIdx  = Cx_00_floorIdx + info->phasV.size();
	int Cx_90_ceilIdx  = Cx_00_ceilIdx + 1;
	Cx_00 = (pathDist_scale_floor * C[Cx_00_floorIdx]) + (pathDist_scale_ceil * C[Cx_00_ceilIdx]);
	Cx_90 = (pathDist_scale_floor * C[Cx_90_floorIdx]) + (pathDist_scale_ceil * C[Cx_90_ceilIdx]);
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
Frame::Frame(unsigned short int* data_, int rowsPT, int colsPT, float freq_, float dist_, float shut_, float phas_, int phas_idx_, PixStoring ps_, bool pSim_, bool first_iter) {
	set (data_, rowsPT, colsPT, freq_, dist_, shut_, phas_, phas_idx_, ps_, pSim_, first_iter);
}
// Constructor from std::vector<Frame>. Real Time capture oriented. Average of the vector of Frames
Frame::Frame(std::vector<Frame> & Frame_v, int Frame_v_size, bool first_iter) {
	set(Frame_v, Frame_v_size, first_iter);
}
// Constructor from 2 Frames. Real Time capture oriented. Amplitude of 2 Frames with sinusioud assumpotion
Frame::Frame(Frame & frame00, Frame & frame90, bool first_iter) {
	set(frame00, frame90, first_iter);
}
// Constructor stub from basic parameters
Frame::Frame(Info & info, PixStoring ps_, bool pSim_, int rows_, int cols_, float freq_, float dist_, float shut_, float phas_) {
	set(info, ps_, pSim_, rows_,cols_, freq_, dist_, shut_, phas_);
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
void Frame::set(unsigned short int* data_, int rowsPT, int colsPT, float freq_, float dist_, float shut_, float phas_, int phas_idx_, PixStoring ps_, bool pSim_, bool first_iter) { 
	
	if (first_iter) {
		// External Parameters (RawData, indices)
		RawData_src = NULL;
		freq_idx = 0;
		dist_idx = 0;
		shut_idx = 0;
		phas_idx = phas_idx_;

		// Frame Parameters
		ps = ps_;
		pSim = pSim_;
		freq = freq_;
		dist = dist_;
		shut = shut_;
		phas = phas_;
	}
	// Frame parameters out of first_iter
	if (ps == PIXELS_VALID) {
		rows = CAMERA_PIX_Y_VALID;
		cols = CAMERA_PIX_X_VALID;
	}
	else if (ps == PIXELS_TOTAL) {
		rows = CAMERA_PIX_Y;
		cols = CAMERA_PIX_X;
	}
	data.resize(rows*cols);
	// filling the data
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
// Setter from std::vector<Frame>. Real Time capture oriented. Average of the vector of Frames
void Frame::set(std::vector<Frame> & Frame_v, int Frame_v_size, bool first_iter) {

	if (first_iter) {
		// External Parameters (RawData, idices)
		RawData_src = Frame_v[0].RawData_src;
		freq_idx = Frame_v[0].freq_idx;
		dist_idx = Frame_v[0].dist_idx;
		shut_idx = Frame_v[0].shut_idx;
		phas_idx = Frame_v[0].phas_idx;
	
		// Frame Parameters
		ps = Frame_v[0].ps;
		pSim = Frame_v[0].pSim;
		rows = Frame_v[0].rows;
		cols = Frame_v[0].cols;
		freq = Frame_v[0].freq;
		dist = Frame_v[0].dist;
		shut = Frame_v[0].shut;
		phas = Frame_v[0].phas;
		data.resize(rows*cols);
	}

	// Fill the Frame with the averaged frames
	for (int i = 0; i < data.size(); ++i) {
		data[i] = 0.0f;
		for (int f = 0; f < Frame_v_size; ++f)
			data[i] += Frame_v[f].data[i];
		data[i] /= (float)Frame_v_size;
	}
}
// Setter from 2 Frames. Real Time capture oriented. Amplitude of 2 Frames with sinusioud assumpotion
void Frame::set(Frame & frame00, Frame & frame90, bool first_iter) {

	if (first_iter) {
		// External Parameters (RawData, idices)
		RawData_src = frame00.RawData_src;
		freq_idx = frame00.freq_idx;
		dist_idx = frame00.dist_idx;
		shut_idx = frame00.shut_idx;
		phas_idx = frame00.phas_idx;
	
		// Frame Parameters
		ps = frame00.ps;
		pSim = frame00.pSim;
		rows = frame00.rows;
		cols = frame00.cols;
		freq = frame00.freq;
		dist = frame00.dist;
		shut = frame00.shut;
		phas = frame00.phas;
		data.resize(rows*cols);
	}

	// Fill the Frame with the amplitude of the frames
	for (int i = 0; i < data.size(); ++i)
		data[i] = sqrt((frame00.data[i] * frame00.data[i]) + (frame90.data[i] * frame90.data[i]));
}
// Setter stub from basic parameters
void Frame::set(Info & info, PixStoring ps_, bool pSim_, int rows_, int cols_, float freq_, float dist_, float shut_, float phas_) {

	// External Parameters (RawData, indices)
	RawData_src = NULL;
	freq_idx = get_freq_idx(info, freq_);	// returns -1 if no idx correspondance was found
	dist_idx = get_dist_idx(info, dist_);
	shut_idx = get_shut_idx(info, shut_);
	phas_idx = get_phas_idx(info, phas_);
	// check the parameters are correct
	if (freq_idx < 0) {
		std::cout << "\nWarning (in Frame::set):\n  Frame freq = " << freq_ << " is not a freq in .cmx freqV = ";
		print(info.freqV);
		std::cout << "\nending...";
	}
	if (dist_idx < 0) {
		std::cout << "\nWarning (in Frame::set):\n  Frame dist = " << dist_ << " is not a dist in .cmx distV = ";
		print(info.distV);
		std::cout << "\nending...";
	}
	if (shut_idx < 0) {
		std::cout << "\nWarning (in Frame::set):\n  Frame shut = " << shut_ << " is not a shut in .cmx shutV = ";
		print(info.shutV);
		std::cout << "\nending...";
	}
	if (phas_idx < 0) {
		std::cout << "\nWarning (in Frame::set):\n  Frame phas = " << phas_ << " is not a phas in .cmx phasV = ";
		print(info.phasV);
		std::cout << "\nending...";
	}
	
	// Frame Parameters
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
	data = std::vector<float>(rows*cols, 0.0f);
	freq = freq_;
	dist = dist_;
	shut = shut_;
	phas = phas_;
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
			M_out.at<float>(r,c) = sqrt((frame_00.at(r,c) * frame_00.at(r,c)) + (frame_90.at(r,c) * frame_90.at(r,c)));
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
					M_00.at<float>(r,c) = sqrt((M_00.at<float>(r,c) * M_00.at<float>(r,c)) + (M_90.at<float>(r,c) * M_90.at<float>(r,c)));
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
//   (row >= 0) && (col <  0) && (avg == false): Plots the row
//   (row <  0) && (col >= 0) && (avg == false): Plots the col
//   (row >= 0) && (col <  0) && (avg == true ): Plots the average of every row
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
//   (row >= 0) && (col <  0) && (avg == false): Plots the row
//   (row <  0) && (col >= 0) && (avg == false): Plots the col
//   (row >= 0) && (col <  0) && (avg == true ): Plots the average of every row
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
// Plots a row, a colum, the average of every row or the average every columns of all the Frames of a vector of frames using MATALAB engine
//   (row >= 0) && (col <  0) && (avg == false): Plots the row
//   (row <  0) && (col >= 0) && (avg == false): Plots the col
//   (row >= 0) && (col <  0) && (avg == true ): Plots the average of every row
//   (row <  0) && (col >= 0) && (avg == true ): Plots the average of every col
//   else: undefined behavior
void plot_rowcolV(std::vector<Frame*> & frameV, std::vector<char*> & textV, std::vector<float> & colorV, float lineWidth, int row, int col, bool avg, bool legend, bool freezePlot, bool & epExtStarted, bool epExtUsing, Engine *epExt) {
	
	// MATLAB variables
	Engine *ep;
	const int frames =  frameV.size();
	// mxArrays
	std::vector<mxArray*> F(frames, NULL);	// frames
	mxArray *Frows = NULL;
	mxArray *Fcols = NULL;
	// char names
	std::vector<char*> F_name(frames);	// frame
	std::vector<char*> V_name(frames);	// value of the row/col
	for (int f = 0; f < frames; ++f) {
		F_name[f] = new char[128];
		sprintf(F_name[f], "F%d", f);
		V_name[f] = new char[128];
		sprintf(V_name[f], "V%d", f);
	}
	char Frows_name[128] = "Frows";
	char Fcols_name[128] = "Fcols";
	char X_name[128] = "X";
	char strStorer[128] = "FUNCTIONS";
	// MATLAB Engine pointer
	if (epExtUsing)
		ep = epExt;
	
	// Variables. Array structure needed to deal with MATLAB functions
	int sizeFrame = frameV[0]->rows * frameV[0]->cols;
	int sizeX = frameV[0]->cols;
	if (row < 0)
		sizeX = frameV[0]->rows;
	std::vector<double*> F_arrayD(frames);
	for (int f = 0; f < frames; ++f) {
		F_arrayD[f] = new double[sizeFrame];
		for (int i = 0; i < sizeFrame; ++i)
			F_arrayD[f][i] = frameV[f]->data[i];
	}
	
	// Call engOpen(""). This starts a MATLAB process on the current host using the command "matlab"
	if (!epExtStarted)
		std::cout << "\nOpening MATLAB engine, wait...";
	if (!(ep = engOpen("")))
		fprintf(stderr, "\nCan't start MATLAB engine\n");
	if (!epExtStarted)
		std::cout << "\nMATLAB engine opened\n\n";

	
	// Create MATLAB variables from C++ variables
	engEvalString(ep, "clear;");	// we are not interested on using engEvalString_andStoreInMATLAB(...) here
	engEvalString_andStoreInMATLAB(ep, "disp('Below are shown all the commands executed in matlab for the plotting');", strStorer, true);	// firstTime = true
	for (int f = 0; f < frames; ++f)
		F[f] = mxCreateDoubleMatrix(1, sizeFrame, mxREAL);
	for (int f = 0; f < frames; ++f)
		memcpy((void *)mxGetPr(F[f]), (void *)F_arrayD[f], sizeFrame*sizeof(F_arrayD[f]));
	Frows = mxCreateDoubleScalar(frameV[0]->rows);
	Fcols = mxCreateDoubleScalar(frameV[0]->cols);
	
	// Place MATLAB variables into the MATLAB workspace
	engPutVariable(ep, Frows_name, Frows);
	engPutVariable(ep, Fcols_name, Fcols);
	char charAux[1024];
	for (int f = 0; f < frames; ++f) {
		engPutVariable(ep, F_name[f], F[f]);
		// manage the shape of F
		sprintf(charAux, "%s = [reshape(%s, %s, %s)]';", F_name[f], F_name[f], Fcols_name, Frows_name);
		engEvalString(ep, charAux);
	}
	
	// Fill the X axis
	if (row >= 0) {
		sprintf(charAux, "%s = 0:%d;", X_name, frameV[0]->cols - 1);
		engEvalString_andStoreInMATLAB(ep, charAux, strStorer);
	} else {
		sprintf(charAux, "%s = 0:%d;", X_name, frameV[0]->rows - 1);
		engEvalString_andStoreInMATLAB(ep, charAux, strStorer);
	}
	// Fill the corresponding values of the row/col
	if (avg) {
		if (row >= 0) {	// row, avg
			for (int f = 0; f < frames; ++f) {
				sprintf(charAux, "%s = mean(%s,1);", V_name[f], F_name[f]);
				engEvalString_andStoreInMATLAB(ep, charAux, strStorer);
			}
		} else {		// col, avg
			for (int f = 0; f < frames; ++f) {
				sprintf(charAux, "%s = mean(%s,2)';", V_name[f], F_name[f]);
				engEvalString_andStoreInMATLAB(ep, charAux, strStorer);
			}

	}	} else {
		if (row >= 0) {	// row, non-avg
			for (int f = 0; f < frames; ++f) {
				sprintf(charAux, "%s = %s(%d+1,:);", V_name[f], F_name[f], row);
				engEvalString_andStoreInMATLAB(ep, charAux, strStorer);
			}
			
		} else {		// col, non-avg
			for (int f = 0; f < frames; ++f) {
				sprintf(charAux, "%s = %s(:,%d+1)';", V_name[f], F_name[f], col);
				engEvalString_andStoreInMATLAB(ep, charAux, strStorer);
			}
	}	}
	
	// Plot the results of the figure 1 (00)
	engEvalString_andStoreInMATLAB(ep, "clf;", strStorer);
	engEvalString_andStoreInMATLAB(ep, "fig = figure(1);", strStorer);
	engEvalString_andStoreInMATLAB(ep, "set(fig, 'position', [320, 240, 800, 600]);", strStorer);
	engEvalString_andStoreInMATLAB(ep, "hold on;", strStorer);
	for (int f = 0; f < frames; ++f) {
		sprintf(charAux, "plot(%s, %s, 'color', [%f, %f, %f], 'lineWidth', %f);", X_name, V_name[f], colorV[3*f+0], colorV[3*f+1], colorV[3*f+2], lineWidth);
		engEvalString_andStoreInMATLAB(ep, charAux, strStorer);
	}
	
	char xTxt[1024];
	char titleTxt[1024];
	char titleTxtAppend[1024];
	if (!legend) {
		int titleTxtAppendFirstChar = 0;
		for (int f = 0; f < frames-1; ++f)
			titleTxtAppendFirstChar += sprintf(titleTxtAppend + titleTxtAppendFirstChar, "%s, ", textV[f]);
		titleTxtAppendFirstChar += sprintf(titleTxtAppend + titleTxtAppendFirstChar, "%s", textV[frames-1]);
	} else {
		sprintf(titleTxtAppend, "%s", "");
	}
	if (avg) {
		if (row >= 0) {
			sprintf(titleTxt,"title('Pixel values of the averaged rows. %s');", titleTxtAppend);
			sprintf(xTxt,"xlabel('cols of the averaged rows');");
		} else {
			sprintf(titleTxt,"title('Pixel values of the averaged cols. %s');", titleTxtAppend);
			sprintf(xTxt,"xlabel('rows of the averaged cols');");
	}	} else {
		if (row >= 0) {
			sprintf(titleTxt,"title('Pixel values of each col of the row #%d. %s');", row, titleTxtAppend);
			sprintf(xTxt,"xlabel('cols of the row #%d');", row);
		} else {
			sprintf(titleTxt,"title('Pixel values of each row of the col #%d. %s');", col, titleTxtAppend);
			sprintf(xTxt,"xlabel('rows of the col #%d');", col);
	}	}
	
	engEvalString_andStoreInMATLAB(ep, titleTxt, strStorer);
	engEvalString_andStoreInMATLAB(ep, xTxt, strStorer);
	engEvalString_andStoreInMATLAB(ep, "ylabel('Pixel values');", strStorer);
	char xlimTxt[1024];
	sprintf(xlimTxt, "xlim([0, %d]);", sizeX-1);
	engEvalString_andStoreInMATLAB(ep, xlimTxt, strStorer);
	engEvalString_andStoreInMATLAB(ep, "grid on;", strStorer);

	if (legend) {	//if (!epExtUsing)
		int legendFirstChar = 0;
		char legendTxt[1024] = "";
		legendFirstChar += sprintf(legendTxt + legendFirstChar, "legend(");
		for (int f = 0; f < frames-1; ++f)
			legendFirstChar += sprintf(legendTxt + legendFirstChar, "'%s',", textV[f]);
		legendFirstChar += sprintf(legendTxt + legendFirstChar, "'%s');", textV[frames-1]);
		engEvalString_andStoreInMATLAB(ep, legendTxt, strStorer);		// the legend blinks iterating through an external ep
	}
	engEvalString_andStoreInMATLAB(ep, "hold off;", strStorer);
	
	// use std::cin freeze the plot
	if (freezePlot) {	// if (!epExtUsing) {
		std::cout << "\nWrite any std::string and click ENTER to continue\n";
		std::string answer;
		std::cin >> answer;
	}
	
	// Free memory, close MATLAB figure.
	mxDestroyArray(Frows);
	mxDestroyArray(Fcols);
	for (int f = 0; f < frames; ++f)
		mxDestroyArray(F[f]);
	if (!epExtUsing) {
		engEvalString_andStoreInMATLAB(ep, "close;", strStorer);
		engClose(ep);
		epExtStarted = false;
	} else {
		epExtStarted = true;
	}
	for (int f = 0; f < frames; ++f) {
		delete [] F_name[f];
		delete [] V_name[f];
	}

	// OLD IMPLEMENTATION
	/*
	// MATLAB variables
	Engine *ep;
	mxArray *X = NULL;
	const int frames =  frameV.size();
	std::vector<mxArray*> V(frames, NULL);
	char strStorer[128] = "FUNCTIONS";
	if (epExtUsing)
		ep = epExt;

	// Variables. Array structure needed to deal with MATLAB functions
	int size = frameV[0]->cols;
	if (row < 0)
		size = frameV[0]->rows;
	double* x = new double[size];			// need to convert to double to deal with MATLAB
	std::vector<double*> value(frames);
	for (int f = 0; f < frames; ++f)
		value[f] = new double[size];
	
	// Fill the corresponding values arrays
	if (avg) {
		if (row >= 0) {	// row, avg
			for (int c = 0; c < frameV[0]->cols; ++c) {
				x[c] = (double)c;
				for (int f = 0; f < frames; ++f)
					value[f][c] = 0.0;
				for (int r = 0; r < frameV[0]->rows; ++r) {
					for (int f = 0; f < frames; ++f)
						value[f][c] += (double)(frameV[f]->at(r, c));
				}
				for (int f = 0; f < frames; ++f)
					value[f][c] /= (double)(frameV[f]->rows);
		}	} else {	// col, avg
			for (int r = 0; r < frameV[0]->rows; ++r) {
				x[r] = (double)r;
				for (int f = 0; f < frames; ++f)
					value[f][r] = 0.0;
				for (int c = 0; c < frameV[0]->cols; ++c) {
					for (int f = 0; f < frames; ++f)
						value[f][r] += (double)(frameV[f]->at(r, c));
				}
				for (int f = 0; f < frames; ++f)
					value[f][r] /= (double)(frameV[f]->cols);
	}	}	} else {
		if (row >= 0) {	// row, non-avg
			for (int c = 0; c < frameV[0]->cols; ++c) {
				x[c] = (double)c;
				for (int f = 0; f < frames; ++f)
					value[f][c] = (double)(frameV[f]->at(row, c));
		}	} else {	// col, non-avg
			for (int r = 0; r < frameV[0]->rows; ++r) {
				x[r] = (double)r;
				for (int f = 0; f < frames; ++f)
					value[f][r] = (double)(frameV[f]->at(r, col)); 
	}	}	}
	
	// Call engOpen(""). This starts a MATLAB process on the current host using the command "matlab"
	if (!epExtStarted)
		std::cout << "\nOpening MATLAB engine, wait...";
	if (!(ep = engOpen("")))
		fprintf(stderr, "\nCan't start MATLAB engine\n");
	if (!epExtStarted)
		std::cout << "\nMATLAB engine opened\n\n";
	
	// Create MATLAB variables from C++ variables
	engEvalString(ep, "clear;");	// we are not interested on using engEvalString_andStoreInMATLAB(...) here
	X = mxCreateDoubleMatrix(1, size, mxREAL);
	for (int f = 0; f < frames; ++f)
		V[f] = mxCreateDoubleMatrix(1, size, mxREAL);
	memcpy((void *)mxGetPr(X), (void *)x, size*sizeof(x));
	for (int f = 0; f < frames; ++f)
		memcpy((void *)mxGetPr(V[f]), (void *)value[f], size*sizeof(value[f]));
	
	// Place MATLAB variables into the MATLAB workspace
	engPutVariable(ep, "X", X);
	std::vector<char*> VM(frames);
	for (int f = 0; f < frames; ++f) {
		VM[f] = new char[128];
		sprintf(VM[f], "VM%d", f);
		engPutVariable(ep, VM[f], V[f]);
	}

	// Plot the results of the figure 1 (00)
	engEvalString_andStoreInMATLAB(ep, "figure(1);", strStorer, true);	// firstTime = true
	engEvalString_andStoreInMATLAB(ep, "clf;", strStorer);
	engEvalString_andStoreInMATLAB(ep, "hold on;", strStorer);
	std::vector<char*> plotTxtV(frames);
	for (int f = 0; f < frames; ++f) {
		plotTxtV[f] = new char[1024];
		sprintf(plotTxtV[f], "plot(X, %s, 'color', [%f, %f, %f], 'lineWidth', %f);", VM[f], colorV[3*f+0], colorV[3*f+1], colorV[3*f+2], lineWidth);
		engEvalString_andStoreInMATLAB(ep, plotTxtV[f], strStorer);
	}
	
	char xTxt[1024];
	char titleTxt[1024];
	char titleTxtAppend[1024];
	if (!legend) {
		int titleTxtAppendFirstChar = 0;
		for (int f = 0; f < frames-1; ++f)
			titleTxtAppendFirstChar += sprintf(titleTxtAppend + titleTxtAppendFirstChar, "%s, ", textV[f]);
		titleTxtAppendFirstChar += sprintf(titleTxtAppend + titleTxtAppendFirstChar, "%s", textV[frames-1]);
	} else {
		sprintf(titleTxtAppend, "%s", "");
	}
	if (avg) {
		if (row >= 0) {
			sprintf(titleTxt,"title('Values of the averaged rows. %s');", titleTxtAppend);
			sprintf(xTxt,"xlabel('cols of the averaged rows');");
		} else {
			sprintf(titleTxt,"title('Values of the averaged cols. %s');", titleTxtAppend);
			sprintf(xTxt,"xlabel('rows of the averaged cols');");
	}	} else {
		if (row >= 0) {
			sprintf(titleTxt,"title('Values of each col of the row #%d. %s');", row, titleTxtAppend);
			sprintf(xTxt,"xlabel('cols of the row #%d');", row);
		} else {
			sprintf(titleTxt,"title('Values of each row of the col #%d. %s');", col, titleTxtAppend);
			sprintf(xTxt,"xlabel('rows of the col #%d');", col);
	}	}
	
	engEvalString_andStoreInMATLAB(ep, titleTxt, strStorer);
	engEvalString_andStoreInMATLAB(ep, xTxt, strStorer);
	engEvalString_andStoreInMATLAB(ep, "ylabel('values');", strStorer);
	char xlimTxt[1024];
	sprintf(xlimTxt, "xlim([0, %d]);", size-1);
	engEvalString_andStoreInMATLAB(ep, xlimTxt, strStorer);
	engEvalString_andStoreInMATLAB(ep, "grid on;", strStorer);

	if (legend) {	//if (!epExtUsing)
		int legendFirstChar = 0;
		char legendTxt[1024] = "";
		legendFirstChar += sprintf(legendTxt + legendFirstChar, "legend(");
		for (int f = 0; f < frames-1; ++f)
			legendFirstChar += sprintf(legendTxt + legendFirstChar, "'%s',", textV[f]);
		legendFirstChar += sprintf(legendTxt + legendFirstChar, "'%s');", textV[frames-1]);
		engEvalString_andStoreInMATLAB(ep, legendTxt, strStorer);		// the legend blinks iterating through an external ep
	}
	engEvalString_andStoreInMATLAB(ep, "hold off;", strStorer);
	
	// use std::cin freeze the plot
	if (freezePlot) {	// if (!epExtUsing) {
		std::cout << "\nWrite any std::string and click ENTER to continue\n";
		std::string answer;
		std::cin >> answer;
	}
	
	// Free memory, close MATLAB figure.
	mxDestroyArray(X);
	for (int f = 0; f < frames; ++f)
		mxDestroyArray(V[f]);
	if (!epExtUsing) {
		engEvalString_andStoreInMATLAB(ep, "close;", strStorer);
		engClose(ep);
		epExtStarted = false;
	} else {
		epExtStarted = true;
	}
	delete [] x;
	for (int f = 0; f < frames; ++f) {
		delete [] value[f];
		delete [] VM[f];
		delete [] plotTxtV[f];
	}
	*/
}

// This stores all the function executed in a MATLAB variable strStore. This is like exectuting both:
//   engEvalString(ep, strFunction);				// in C++
//   strStorer = char(strStorer, 'strFunction');	// in MATLAB (taking care of duplicating (')
void engEvalString_andStoreInMATLAB (Engine *ep, const char* strFunction, char* strStorer, bool firstTime) {

	// Execute: engEvalString(ep, strFunction);
	engEvalString(ep, strFunction);

	// Take care of diplicating (')
	char strFunctionFiltered[1024];
	int origIdx = 0, filtIdx = 0;
	while (strFunction[origIdx]) {
		strFunctionFiltered[filtIdx++] = strFunction[origIdx++];
		if (strFunction[origIdx-1] == '\'')
			strFunctionFiltered[filtIdx++] = '\'';
	}
	strFunctionFiltered[filtIdx] = '\0';

	// Execute: strStorer = char(strStorer, 'strFunctionFiltered');
	char strToStore[1024];
	if (firstTime)
		sprintf(strToStore, "%s = char('%s');", strStorer, strFunctionFiltered);
	else
		sprintf(strToStore, "%s = char(%s, '%s');", strStorer, strStorer, strFunctionFiltered);
	engEvalString(ep, strToStore);
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
float maxAbsSigned(std::vector<float> & v) {
	float maxAS = v[0];
	for (size_t i = 1; i < v.size(); i++) {
		if (abs(v[i]) > abs(maxAS))
			maxAS = v[i];
	}
	return maxAS;
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
float avgError(std::vector<float> & v0, std::vector<float> & v1) {
	float avgErr = 0.0f;
	for (size_t i = 0; i < v0.size(); ++i)
		avgErr += abs(v0[i] - v1[i]);
	return avgErr / v0.size();
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
