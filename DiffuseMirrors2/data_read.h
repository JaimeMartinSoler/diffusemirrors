
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
	//     process_data(w, h, shutters, measpath, fnprefix, take, rawdumpfile);
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




// INFO
class Info {
public:

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
	int width;
	int heigth;
	std::vector<float> frequencies;
	std::vector<float> distances;
	std::vector<float> shutters;
	std::vector<float> phases;
	int numtakes;
	// Calibration Matrix parameters:
	int sizeof_value_cmx;	// bytes
	float laser_to_cam_offset_x;
	float laser_to_cam_offset_y;
	float laser_to_cam_offset_z;
	float dist_wall_cam;
	int error_code;	// 0=no_error, !=0:error
	
	// Constructor
	Info::Info(char* dir_name_, char* file_name_);
	// Constructor
	Info::Info(	char* dir_name_, char* file_name_, int sizeof_value_raw_, int width_, int heigth_,
				std::vector<float> & frequencies_, std::vector<float> & distances_, std::vector<float> & shutters_, std::vector<float> & phases_, int numtakes_, int error_code_ = 0,
				int sizeof_value_cmx_ = -1, float laser_to_cam_offset_x_ = -1.0f, float laser_to_cam_offset_y_ = -1.0f, float laser_to_cam_offset_z_ = -1.0f, float dist_wall_cam_ = -1.0f);
	// Constructor Default
	Info::Info();
};




// DATA-PMD 
class RawData {
public:

	// External Parameters
	Info* info;		// Pointer to the info object

	// RawData Parameters
	// data ordereing: for(dist) { for(freq) { for(phase) { for(shutter){ for(heigth){ for(width){ // here... }}}}}}
	// inside each frame, data stores all cols, then next row and so on, from down to top, unlike Frame and any Matrix
	unsigned short int* data; // always takes w=165, h=120 indep of PIXEL_STORING, as long as it's raw data
	int data_size;
	int take;		// number of the raw_numtake file this is referencing to. take = -1 if refers to the normal raw file
	int error_code;	// 0=no_error, !=0:error


	// Constructor. It creates a CalibrationMatrix object from the .raw file noted in the info object
	RawData::RawData(Info* info_, int take_ = -1);
	// Constructor
	RawData::RawData(Info* info_, unsigned short int* data_, int data_size_, int take_ = -1, int error_code_ = 0);
	// Constructor Default
	RawData::RawData();

	// Functions
	// Returns the index in data[], corresponding to the parameter indices
	int RawData::idx_in_data(int distances_idx, int frequencies_idx, int shutters_idx, int w, int h, int phases_idx);
	// Returns the value corresponding to the parameter indices = data[idx_in_data]
	unsigned short int RawData::at(int distances_idx, int frequencies_idx, int shutters_idx, int w, int h, int phases_idx);
};




// CALIBRATION MATRIX
class CalibrationMatrix {
public:
	
	// External Parameters (RawData, Info)
	Info* info;				// Pointer to the info object
	RawData* RawData_src;	// a pointer to the RawData this Frame comes from

	// Calibration Matrix Parameters
	// data ordereing: for(freq) { for(dist) { for(heigth){ for(width){ // here... }}}}
	// inside each frame, data stores all cols, then next row and so on, from top to down, like Frame and any Matrix
	float* data;		// data size is NOT pixels_storing-dependent (always considers w=165, h=120). It is arranged in the accessing
	int data_size;		// data_size is NOT pixels_storing-dependent (always considers w=165, h=120). It is arranged in the accessing
	cv::Mat path_dist_0;			// matrix wih the path distances of the original path src-pix-cam. Dimensions are pixels_storing-dependent
	Pixels_storing pixels_storing;	// the kind of pixels it can store (PIXELS_TOTAL, PIXELS_VALID, UNKNOWN_PIXELS_STORING). See global.h
	int width;			// it is info->width or CAMERA_PIX_X_VALID depending on pixels_storing_
	int heigth;			// it is info->heigth or CAMERA_PIX_Y_VALID depending on pixels_storing_
	int error_code;	// 0=no_error, !=0:error

	// Constructor. It creates a CalibrationMatrix object from the .cmx file noted in the info object
 	CalibrationMatrix::CalibrationMatrix(Info* info_, Pixels_storing pixels_storing_ = PIXELS_VALID);
	// Constructor
	CalibrationMatrix::CalibrationMatrix(Info* info_, RawData* RawData_src_, float* data_, int data_size_, cv::Mat & path_dist_0_, Pixels_storing pixels_storing_, int width_, int heigth_, int error_code_ = 0);
	// Constructor Default
	CalibrationMatrix::CalibrationMatrix();

	// Functions
	// Returns the index in data[], corresponding to the parameter indices. Takes care of the pixels_storing internally
	int CalibrationMatrix::idx_in_data(int frequencies_idx, int distances_idx, int w, int h);
	// Returns the value corresponding to the parameter indices = data[idx_in_data]
	float CalibrationMatrix::at(int frequencies_idx, int distances_idx, int w, int h);
	// Returns the value at any path distance interpolating with the closest path distances. Path distances have to be equidistant
	float CalibrationMatrix::at_any_path_dist(int frequencies_idx, float path_dist, int w, int h);

};




// FRAME
class Frame {
public:

	// External Parameters (RawData, Info)
	Info* info;
	RawData* RawData_src;		// a pointer to the RawData this Frame comes from
	int frequency_idx;
	int distance_idx;
	int shutter_idx;
	int phase_idx;

	// Frame Parameters
	// matrix stores all cols, then next row and so on, from up to down, like any Matrix
	cv::Mat matrix;				// the opencv matrix with the values of the frame
	Pixels_storing pixels_storing;	// the kind of pixels it can store (PIXELS_TOTAL, PIXELS_VALID, UNKNOWN_PIXELS_STORING). See global.h
	int width;			// it is info->width or CAMERA_PIX_X_VALID depending on pixels_storing_
	int heigth;			// it is info->heigth or CAMERA_PIX_Y_VALID depending on pixels_storing_
	float frequency;	// (MHz)
	float distance;		// (m)
	float shutter;		// (us)
	float phase;		// (degrees)

	// Constructor. RawData oriented
	Frame::Frame(Info* info_, RawData* RawData_src_, int distance_idx_, int frequency_idx_, int shutter_idx_, int phase_idx_, Pixels_storing pixels_storing_ = PIXELS_VALID);
	// Constructor from vector. Simulation oriented. For any Pixels_storing it considers the vector matrix_vector properly arranged
	Frame::Frame(std::vector<float> & matrix_vector, int heigth_, int width_, bool rows_up2down = true, float distance_ = 0.0f, float frequency_ = 0.0f, float shutter_ = 0.0f, float phase_ = 0.0f, Pixels_storing pixels_storing_ = PIXELS_VALID);
	// Constructor from vector. Data real time capture oriented. heigth_ and width_ must be refered to the sizes of the total frame, regerdingless to the Pixels_storing
	Frame::Frame(unsigned short int* data_, int heigth_, int width_, float distance_, float frequency_, float shutter_, float phase_, int phase_idx_, Pixels_storing pixels_storing_ = PIXELS_VALID);
	// Constructor Default
	Frame::Frame();

	// Functions
	// first_idx can be 0 or 1, it is the first idx of the row and col we are considering
	float Frame::at(int row, int col, int first_idx = 0);
	// Plot frame with opencv
	void Frame::plot_frame();
};
// plot frame amplitude with sinusoidal assumption
void plot_frame(Frame & frame_00, Frame & frame_90);
// For FoV measurement scene. Plot frame with opencv with syncronization
void plot_frame_fov_measurement(bool loop = false);




// Reads data from a .dat and info.txt file setting the data_read variable
int data_read_main();

// sets a vector of floats form a char array from a given delimiter
void char_array_to_float_vector_from_delimiter (char* char_array, std::vector<float> & float_vector, char delimiter);



#endif

