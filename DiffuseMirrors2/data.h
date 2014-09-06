
#ifndef __DATA_READ_H
#define __DATA_READ_H

#include "global.h"


#include <vector>

#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>	


	// ------------------------------------------------------------------------------------------------------------------------------
	// The measured data is stored in shutters[i].second, while calling:
	//     pmd_capture(hnd, port, shutter, frequencies[fi], delays[di], buffer = shutters[i].second, w, h, numframes);
	// in each:
	//     for(take){ for(distance){ for(freq){ for(shutter){ // here... }}}}
	// each take saves the data in different and independent files

	// Once all the shutters of each
	//     for(take){ for(distance){ for(freq){ // shutters... }}}
	// have been measured, the data of those shutters is stored in the rawdumpfile, while calling:
	//     process_data_to_file(w, h, shutters, measpath, fnprefix, take, rawdumpfile);
	// although it could not look like in the code, it is stored in the order (x=width=column), (y=heigth=row) (phase={0, 90}):
	//     for(phase){ for(shutter){ for(heigth){ for(width){ // dump_data(...) }}}}

	// So, the order of each measured unsigned value of 2 Bytes in the file is:
	//     for(dist) { for(freq) { for(phase) { for(shutter){ for(heigth){ for(width){ // here... }}}}}}

	// The position in that array/file of values of 2 Bytes is given by:
	// int pos = frequencies.size() * phases.size() * shutters.size() * heigth * width * distances_idx   +
	//                                phases.size() * shutters.size() * heigth * width * frequencies_idx +
	//                                                shutters.size() * heigth * width * phases_idx      +
	//                                                                  heigth * width * shutters_idx    +
	//                                                                           width * h               +
	//                                                                                   w;
	// And the position of the corresponding first Byte is:
	//     int pos_Byte = 2 * pos;
	// ------------------------------------------------------------------------------------------------------------------------------




// ----------------------------------------------------------------------------------------------------------------------------------------
// ----- INFO -----------------------------------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------------------

class Info {
public:
	
	// ----- PARAMETERS ------------------------------

	// External Parameters (file names):
	char* dir_name;
	char* file_name;
	char* inf_full_file_name;
	char* raw_full_file_name;
	char* cmx_full_file_name;
	char* cmd_full_file_name;
	std::vector<char*> raw_take_full_file_name;

	// Info Parameters:
	int sizeof_value_raw;	// bytes
	int rows;
	int cols;
	std::vector<float> freqs;
	std::vector<float> dists;
	std::vector<float> shuts;
	std::vector<float> phass;
	int numtakes;
	// Calibration Matrix parameters:
	int sizeof_value_cmx;	// bytes
	float laser_to_cam_offset_x;
	float laser_to_cam_offset_y;
	float laser_to_cam_offset_z;
	float dist_wall_cam;
	int error_code;	// 0=no_error, !=0:error
	

	// ----- CONSTRUCTORS ---------------------------- // Note that each Constructor just contains its corresponding Setter
	
	// Constructor Default
	Info::Info();
	// Constructor Copy
	// pointers are copied "as are", the data pointed is not duplicated. For this use .clone(...) (if implemented)
	Info::Info(Info & info);
	// Constructor All parameters
	Info::Info(	char* dir_name_, char* file_name_, int sizeof_value_raw_, int rows_, int cols_,
				std::vector<float> & freqs_, std::vector<float> & dists_, std::vector<float> & shuts_, std::vector<float> & phass_, int numtakes_, int error_code_ = 0,
				int sizeof_value_cmx_ = -1, float laser_to_cam_offset_x_ = -1.0f, float laser_to_cam_offset_y_ = -1.0f, float laser_to_cam_offset_z_ = -1.0f, float dist_wall_cam_ = -1.0f);

	
	// Constructor from file
	Info::Info(char* dir_name_, char* file_name_);
	

	// ----- SETTERS --------------------------------- // Note that each Constructor just contains its corresponding Setter
	
	// Setter Default
	void Info::set ();
	// Setter Copy
	// pointers are copied "as are", the data pointed is not duplicated. For this use .clone(...) (if implemented)
	void Info::set (Info & info);
	// Setter All parameters
	void Info::set (	char* dir_name_, char* file_name_, int sizeof_value_raw_, int rows_, int cols_,
				std::vector<float> & freqs_, std::vector<float> & dists_, std::vector<float> & shuts_, std::vector<float> & phass_, int numtakes_, int error_code_ = 0,
				int sizeof_value_cmx_ = -1, float laser_to_cam_offset_x_ = -1.0f, float laser_to_cam_offset_y_ = -1.0f, float laser_to_cam_offset_z_ = -1.0f, float dist_wall_cam_ = -1.0f);
	// Setter from file
	void Info::set (char* dir_name_, char* file_name_);
};




// ----------------------------------------------------------------------------------------------------------------------------------------
// ----- RAW DATA -------------------------------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------------------

class RawData {
public:
	
	// ----- PARAMETERS ------------------------------

	// External Parameters
	Info* info;		// Pointer to the info object
	int take;		// number of the raw_numtake file this is referencing to. take = -1 if refers to the normal raw file

	// RawData Parameters
	// data ordereing: for(dist){ for(freq){ for(phase){ for(shutter){ for(r){ for(c){ // here... }}}}}} // r,c NOT Matrix-like, 0-idx
	// inside each frame, data stores all cols, then next row and so on, from down to top, unlike Frame and any Matrix
	unsigned short int* data; // independent of Pixels_storing, the accessing depends on it
	int data_size;
	int error_code;	// 0=no_error, !=0:error
	

	// ----- CONSTRUCTORS ---------------------------- // Note that each Constructor just contains its corresponding Setter
	
	// Constructor Default
	RawData::RawData();
	// Constructor Copy
	// pointers are copied "as are", the data pointed is not duplicated. For this use .clone(...) (if implemented)
	RawData::RawData(RawData & raw_data);
	// Constructor All parameters
	RawData::RawData(Info & info_, unsigned short int* data_, int data_size_, int take_ = -1, int error_code_ = 0);
	// Constructor from Info. It creates a CalibrationMatrix object from the .raw file noted in the info object
	// take: number of the raw_numtake file this is referencing to. take = -1 if refers to the normal raw file
	RawData::RawData(Info & info_, int take_ = -1);
	
	
	
	// ----- SETTERS --------------------------------- // Note that each Constructor just contains its corresponding Setter

	// Setter Default
	void RawData::set ();
	// Setter Copy
	// pointers are copied "as are", the data pointed is not duplicated. For this use .clone(...) (if implemented)
	void RawData::set (RawData & raw_data);
	// Setter All parameters
	void RawData::set (Info & info_, unsigned short int* data_, int data_size_, int take_ = -1, int error_code_ = 0);
	// Setter from Info. It creates a CalibrationMatrix object from the .raw file noted in the info object
	// take: number of the raw_numtake file this is referencing to. take = -1 if refers to the normal raw file
	void RawData::set (Info & info_, int take_ = -1);
	

	// ----- FUNCTIONS -------------------------------

	// Returns the index in data[], corresponding to the parameter indices
	// input r,c are considered like Matrix from 0 indexation. It also takes care on Pixels_storing
	int RawData::data_idx(int freq_idx, int dist_idx, int shut_idx, int phas_idx, int r, int c, Pixels_storing ps = PIXELS_STORING_GLOBAL);
	// Returns the value corresponding to the parameter indices = data[idx_in_data]
	// input r,c are considered like Matrix from 0 indexation. It also takes care on Pixels_storing
	unsigned short int RawData::at(int freq_idx, int dist_idx, int shut_idx, int phas_idx, int r, int c, Pixels_storing ps = PIXELS_STORING_GLOBAL);
};




// ----------------------------------------------------------------------------------------------------------------------------------------
// ----- CALIBRATION MATRIX ---------------------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------------------

class CalibrationMatrix {
public:
	
	// ----- PARAMETERS ------------------------------

	// External Parameters (Info)
	Info* info;		// Pointer to the info object

	// Calibration Matrix Parameters
	// data ordereing: for(freq){ for(dist){ for(r){ for(c){ // here... }}}} // r,c Matrix-like, 0-idx
	// inside each frame, data stores all cols, then next row and so on, from top to down, like Frame and any Matrix
	float* data;					// independent of Pixels_storing, the accessing depends on it
	int data_size;		
	std::vector<float> path_dist_0;	// independent of Pixels_storing, the accessing depends on it. Ordering: for(r){ for(c){ // here... }}}} // r,c Matrix-like, 0-idx
	int error_code;	// 0=no_error, !=0:error
	

	// ----- CONSTRUCTORS ---------------------------- // Note that each Constructor just contains its corresponding Setter
	
	// Constructor Default
	CalibrationMatrix::CalibrationMatrix();
	// Constructor Copy
	// pointers are copied "as are", the data pointed is not duplicated. For this use .clone(...) (if implemented)
	CalibrationMatrix::CalibrationMatrix(CalibrationMatrix & cmx);
	// Constructor All parameters
	CalibrationMatrix::CalibrationMatrix(Info & info_, float* data_, int data_size_, std::vector<float> & path_dist_0_, int error_code_ = 0);
	// Constructor. It creates a CalibrationMatrix object from the .cmx file noted in the info object
 	CalibrationMatrix::CalibrationMatrix(Info & info_);
	
	
	// ----- SETTERS --------------------------------- // Note that each Constructor just contains its corresponding Setter

	// Constructor Default
	void CalibrationMatrix::set ();
	// Constructor Copy
	// pointers are copied "as are", the data pointed is not duplicated. For this use .clone(...) (if implemented)
	void CalibrationMatrix::set (CalibrationMatrix & cmx);
	// Constructor All parameters
	void CalibrationMatrix::set (Info & info_, float* data_, int data_size_, std::vector<float> & path_dist_0_, int error_code_ = 0);
	// Constructor. It creates a CalibrationMatrix object from the .cmx file noted in the info object
 	void CalibrationMatrix::set (Info & info_);
	


	// ----- FUNCTIONS -------------------------------

	// Returns the index in data[], corresponding to the parameter indices.
	// input r,c are considered like Matrix from 0 indexation. It also takes care on Pixels_storing
	int CalibrationMatrix::data_idx (int freq_idx, int dist_idx, int r, int c, Pixels_storing ps = PIXELS_STORING_GLOBAL);
	// Returns the value from the Calibration Matrix data corresponding to the parameter indices = data[idx_in_data]
	// input r,c are considered like Matrix from 0 indexation. It also takes care on Pixels_storing
	float CalibrationMatrix::at (int freq_idx, int dist_idx, int r, int c, Pixels_storing ps = PIXELS_STORING_GLOBAL);
	// Returns the index in path_dist_0, corresponding to the parameter indices.
	// input r,c are considered like Matrix from 0 indexation. It also takes care on Pixels_storing
	int CalibrationMatrix::path_dist_0_idx (int r, int c, Pixels_storing ps = PIXELS_STORING_GLOBAL);
	// Returns the value from the path_dist_0 corresponding to the parameter indices = path_dist_0[path_dist_0_idx]
	// input r,c are considered like Matrix from 0 indexation. It also takes care on Pixels_storing
	float CalibrationMatrix::path_dist_0_at (int r, int c, Pixels_storing ps = PIXELS_STORING_GLOBAL);
	
	// This is the Calibtration Matrix coefficient: c_{\omega}^{r,c}(\tau^{r,c}) in the Master Thesis document
	// Returns the value from the Calibration Matrix data at any path distance interpolating with the closest path distances. Path distances have to be equidistant in vector
	float CalibrationMatrix::c_coef (int freq_idx, int r, int c, float path_dist, Pixels_storing ps = PIXELS_STORING_GLOBAL);
	// This is the Simulation term for the direct vision problem: S_{i\;\omega}^{r,c}(\tau^{r,c}) in the Master Thesis document
	// Returns the value of the Simulation from the Calibration Matrix data at any path distance interpolating with the closest path distances. Path distances have to be equidistant in vector
	// Uses c(...)
	float CalibrationMatrix::S_direct_vision (int freq_idx, int r, int c, Point & r_src, Point & r_x,  Point & r_cam, float relative_albedo = 1.0f, Pixels_storing ps = PIXELS_STORING_GLOBAL);
};




// ----------------------------------------------------------------------------------------------------------------------------------------
// ----- FRAME ----------------------------------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------------------

class Frame {
public:

	// ----- PARAMETERS ------------------------------

	// External Parameters (RawData, Info)
	RawData* RawData_src;	// a pointer to the RawData this Frame comes from (if so)
	int freq_idx;
	int dist_idx;
	int shut_idx;
	int phas_idx;

	// Frame Parameters
	std::vector<float> data;	// Pixels_storing dependent. With all the values of the frame. // r,c Matrix-like, 0-idx. 
	Pixels_storing ps;	// the kind of pixels it can store (PIXELS_TOTAL, PIXELS_VALID, UNKNOWN_PIXELS_STORING). See global.h
	int rows;			// Pixels_storing dependent
	int cols;			// Pixels_storing dependent
	float freq;	// (MHz)
	float dist;	// (m)
	float shut;	// (us)
	float phas;	// (degrees)
	

	// ----- CONSTRUCTORS ---------------------------- // Note that each Constructor just contains its corresponding Setter
	
	// Constructor Default
	Frame::Frame ();
	// Constructor Copy
	// pointers are copied "as are", the data pointed is not duplicated. For this use .clone(...) (if implemented)
	Frame::Frame (Frame & frame);
	// Constructor All parameters
	Frame::Frame (RawData & RawData_src_, int freq_idx_, int dist_idx_, int shut_idx_, int phas_idx_,
				  std::vector<float> & data_, Pixels_storing ps_, int rows_, int cols_, float freq_, float dist_, float shut_, float phas_);
	// Constructor from RawData. RawData oriented
	Frame::Frame (RawData & RawData_src_, int freq_idx_, int dist_idx_, int shut_idx_, int phas_idx_, Pixels_storing ps_ = PIXELS_STORING_GLOBAL);
	// Constructor from vector. Simulation oriented. For any Pixels_storing it considers the vector matrix_vector properly arranged
	// If rows_ and cols_ are > 0, they will be the new rows and cols (interesting while simulating arbitrary rows and cols. Otherwise rows and cols are made from ps_
	Frame::Frame (std::vector<float> & data_, int rows_ = 0, int cols_ = 0, float freq_ = 0.0f, float dist_ = 0.0f, float shut_ = 0.0f, float phas_ = 0.0f, Pixels_storing ps_ = PIXELS_STORING_GLOBAL);
	// Constructor from ushort int*. Real Time capture oriented. rows_ and cols_ must be refered to the sizes of the total frame, regerdingless to the Pixels_storing
	Frame::Frame (unsigned short int* data_, int rows_, int cols_, float freq_, float dist_, float shut_, float phas_, int phas_idx_, Pixels_storing ps_ = PIXELS_STORING_GLOBAL);
	
	
	// ----- SETTERS --------------------------------- // Note that each Constructor just contains its corresponding Setter

	// Setter Default
	void Frame::set ();
	// Setter Copy
	// pointers are copied "as are", the data pointed is not duplicated. For this use .clone(...) (if implemented)
	void Frame::set (Frame & frame);
	// Setter All parameters
	void Frame::set (RawData & RawData_src_, int freq_idx_, int dist_idx_, int shut_idx_, int phas_idx_,
				  std::vector<float> & data_, Pixels_storing ps_, int rows_, int cols_, float freq_, float dist_, float shut_, float phas_);
	// Setter from RawData. RawData oriented
	void Frame::set (RawData & RawData_src_, int freq_idx_, int dist_idx_, int shut_idx_, int phas_idx_, Pixels_storing ps_ = PIXELS_STORING_GLOBAL);
	// Setter from vector. Simulation oriented. For any Pixels_storing it considers the vector matrix_vector properly arranged
	// If rows_ and cols_ are > 0, they will be the new rows and cols (interesting while simulating arbitrary rows and cols. Otherwise rows and cols are made from ps_
	void Frame::set (std::vector<float> & data_, int rows_ = 0, int cols_ = 0, float freq_ = 0.0f, float dist_ = 0.0f, float shut_ = 0.0f, float phas_ = 0.0f, Pixels_storing ps_ = PIXELS_STORING_GLOBAL);
	// Setter from ushort int*. Real Time capture oriented. rows_ and cols_ must be refered to the sizes of the total frame, regerdingless to the Pixels_storing
	void Frame::set (unsigned short int* data_, int rows_, int cols_, float freq_, float dist_, float shut_, float phas_, int phas_idx_, Pixels_storing ps_ = PIXELS_STORING_GLOBAL);
	

	// ----- FUNCTIONS -------------------------------

	// r,c Matrix-like, 0-idx.
	int Frame::data_idx (int r, int c);
	// r,c Matrix-like, 0-idx.
	float Frame::at (int r, int c);

	// Plot frame with opencv
	void Frame::plot(int delay_ms = 1000);
};
// plot frame amplitude with sinusoidal assumption
void plot_frame(Frame & frame_00, Frame & frame_90, int delay_ms = 1000);
// For FoV measurement scene. Plot frame with opencv with syncronization
void plot_frame_fov_measurement(Frame & frame_00, Frame & frame_90, bool loop = false);




// ----------------------------------------------------------------------------------------------------------------------------------------
// ----- OTHER FUNCTIONS ------------------------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------------------

// sets a vector of floats form a char array from a given delimiter
void char_array_to_float_vector_from_delimiter (char* char_array, std::vector<float> & float_vector, char delimiter);

// returns the corresponding position of a vector from r,c considering Matrix-like ordering 0-indexed.
// Takes into account the Pixels_storing
int rc2idx (int r, int c, Pixels_storing ps = PIXELS_STORING_GLOBAL);


#endif

