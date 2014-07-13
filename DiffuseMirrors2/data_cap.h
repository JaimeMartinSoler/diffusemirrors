
#ifndef __DATA_CAP_H
#define __DATA_CAP_H

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

// enum Source
enum Source {DATA_FILE, DATA_REAL_TIME, SIMULATION};

// DATA CAPTURED
class DataCap {
public:

	// Parameters
	short int* data;
	int data_size;	

	std::vector<float> frequencies;
	std::vector<float> distances;
	std::vector<float> shutters;
	std::vector<float> phases;

	int width;
	int heigth;
	int numtakes;
	int bytes_per_value;

	int error_code;	// 0=no_error, !=0:error
	char* file_data_name;
	char* file_info_name;

	Source src;

	// Constructor
	DataCap::DataCap(char* file_data_name_, char* file_info_name_);
	// Constructor
	DataCap::DataCap(short int* data_, int data_size_, std::vector<float> & frequencies_, std::vector<float> & distances_, std::vector<float> & shutters_, std::vector<float> & phases_, int width_, int heigth_, int numtakes_, int bytes_per_value_, int error_code_, char* file_data_name_, char* file_info_name_, Source src_);
	// Constructor Default
	DataCap::DataCap();

	// Functions
	// Returns the index in data[], corresponding to the parameter indices
	int DataCap::idx_in_data(int distances_idx, int frequencies_idx, int shutters_idx, int w, int h, int phases_idx);
	// Returns the value corresponding to the parameter indices = data[idx_in_data]
	short int DataCap::at(int distances_idx, int frequencies_idx, int shutters_idx, int w, int h, int phases_idx);
};




// FRAME
class Frame {
public:

	// Parameters
	// matrix stores all cols, then next row and so on, from up to down
	cv::Mat matrix;				// the opencv matrix with the values of the frame
	DataCap* DataCap_src;		// The DataCap the Frame comes from

	float distance;
	float frequency;
	float shutter;
	float phase;

	int distance_idx;
	int frequency_idx;
	int shutter_idx;
	int phase_idx;

	int width;
	int heigth;
	int numtakes;
	int bytes_per_value;

	Source src;
	
	// Constructor from DataCap
	Frame::Frame(DataCap & DataCap_src_, int distance_idx_, int frequency_idx_, int shutter_idx_, int phase_idx_);
	// Constructor from vector (from simulation usually)
	Frame::Frame(std::vector<float> & matrix_vector, int heigth_, int width_, bool rows_up2down = true, float distance_ = 0.0f, float frequency_ = 0.0f, float shutter_ = 0.0f, float phase_ = 0.0f, Source src_ = SIMULATION);
	// Constructor Default
	Frame::Frame();

	// Functions
	// first_idx can be 0 or 1, it is the first idx of the row and col we are considering
	float Frame::at(int row, int col, int first_idx = 0);
	// Plot frame with opencv
	void Frame::plot_frame();
};




// Reads data from a .dat and info.txt file setting the DATA_CAPTURE variable
int data_read();

// sets a vector of floats form a char array from a given delimiter
void char_array_to_float_vector_from_delimiter (char* char_array, std::vector<float> & float_vector, char delimiter);



#endif

