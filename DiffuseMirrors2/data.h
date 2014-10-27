
#ifndef __DATA_READ_H
#define __DATA_READ_H

#include "global.h"


#include <vector>

#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>	

#include "engine.h"

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
	std::vector<float> freqV;
	std::vector<float> distV;
	std::vector<float> shutV;
	std::vector<float> phasV;
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
				std::vector<float> & freqV_, std::vector<float> & distV_, std::vector<float> & shutV_, std::vector<float> & phasV_, int numtakes_, int error_code_ = 0,
				int sizeof_value_cmx_ = -1, float laser_to_cam_offset_x_ = -1.0f, float laser_to_cam_offset_y_ = -1.0f, float laser_to_cam_offset_z_ = -1.0f, float dist_wall_cam_ = -1.0f);

	// Constructor from file
	Info::Info(char* dir_name_, char* file_name_);
	// Destructor
	Info::~Info();
	

	// ----- SETTERS --------------------------------- // Note that each Constructor just contains its corresponding Setter
	
	// Setter Default
	void Info::set ();
	// Setter Copy
	// pointers are copied "as are", the data pointed is not duplicated. For this use .clone(...) (if implemented)
	void Info::set (Info & info);
	// Setter All parameters
	void Info::set (	char* dir_name_, char* file_name_, int sizeof_value_raw_, int rows_, int cols_,
				std::vector<float> & freqV_, std::vector<float> & distV_, std::vector<float> & shutV_, std::vector<float> & phasV_, int numtakes_, int error_code_ = 0,
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
	unsigned short int* data; // independent of PixStoring, the accessing depends on it
	int data_size;
	int error_code;	// 0=no_error, !=0:error

	// Simulation Parameters
	float* data_sim_PT;
	float* data_sim_PV;
	int data_sim_size;
	int data_sim_rows;
	int data_sim_cols;

	// ----- CONSTRUCTORS ---------------------------- // Note that each Constructor just contains its corresponding Setter
	
	// Constructor Default
	RawData::RawData();
	// Constructor Copy
	// pointers are copied "as are", the data pointed is not duplicated. For this use .clone(...) (if implemented)
	RawData::RawData(RawData & raw_data);
	// Constructor All parameters
	RawData::RawData(Info & info_, unsigned short int* data_, int data_size_, int take_, int error_code_,
		float* data_sim_PT_, float* data_sim_PV_, int data_sim_size_, int data_sim_rows_, int data_sim_cols_);
	// Constructor from Info. It creates a CalibrationMatrix object from the .raw file noted in the info object
	// take: number of the raw_numtake file this is referencing to. take = -1 if refers to the normal raw file
	RawData::RawData(Info & info_, int take_ = -1);
	// Destructor
	RawData::~RawData();
	
	
	
	// ----- SETTERS --------------------------------- // Note that each Constructor just contains its corresponding Setter

	// Setter Default
	void RawData::set ();
	// Setter Copy
	// pointers are copied "as are", the data pointed is not duplicated. For this use .clone(...) (if implemented)
	void RawData::set (RawData & raw_data);
	// Setter All parameters
	void RawData::set(Info & info_, unsigned short int* data_, int data_size_, int take_, int error_code_,
		float* data_sim_PT_, float* data_sim_PV_, int data_sim_size_, int data_sim_rows_, int data_sim_cols_);
	// Setter from Info. It creates a CalibrationMatrix object from the .raw file noted in the info object
	// take: number of the raw_numtake file this is referencing to. take = -1 if refers to the normal raw file
	void RawData::set (Info & info_, int take_ = -1);
	

	// ----- FUNCTIONS -------------------------------

	// Returns the index in data[], corresponding to the parameter indices
	// input r,c are considered like Matrix from 0 indexation. It also takes care on PixStoring
	int RawData::data_idx(int freq_idx, int dist_idx, int shut_idx, int phas_idx, int r, int c, PixStoring ps = PIXELS_STORING_GLOBAL, bool pixSim = false);
	// Returns the value corresponding to the parameter indices = data[idx_in_data]
	// input r,c are considered like Matrix from 0 indexation. It also takes care on PixStoring
	unsigned short int RawData::at(int freq_idx, int dist_idx, int shut_idx, int phas_idx, int r, int c, PixStoring ps = PIXELS_STORING_GLOBAL);
	// Returns (short int)(data[data_idx(...)] - 32768), fixing the 32768 default offset and converting it to (signed) short int
	short int RawData::atSI(int freq_idx, int dist_idx, int shut_idx, int phas_idx, int r, int c, PixStoring ps = PIXELS_STORING_GLOBAL);
	// Returns (int)(data[data_idx(...)] - 32768), fixing the 32768 default offset and converting it to (signed) int
	int RawData::atI(int freq_idx, int dist_idx, int shut_idx, int phas_idx, int r, int c, PixStoring ps = PIXELS_STORING_GLOBAL);
	// Returns (float)(data[data_idx(...)] - 32768), fixing the 32768 default offset and converting it to float. Deals with data_sim_PT, data_sim_PV
	float RawData::atF(int freq_idx, int dist_idx, int shut_idx, int phas_idx, int r, int c, PixStoring ps = PIXELS_STORING_GLOBAL, bool pixSim = false);
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
	// C ordereing: for(freq){ for(dist){ for(phas){ for(r){ for(c){ //here...}}}}} // r,c Matrix-like, 0-idx
	// inside each frame, C stores all cols, then next row and so on, from top to down, like Frame and any Matrix
	float* C;					// independent of PixStoring, the accessing depends on it
	int C_size;		
	std::vector<float> pathDist0;	// independent of PixStoring, the accessing depends on it. Ordering: for(r){ for(c){ // here... }}}} // r,c Matrix-like, 0-idx
	int error_code;	// 0=no_error, !=0:error

	// Simulation Parameters
	float* C_sim_PT;
	float* C_sim_PV;
	int C_sim_size;
	int C_sim_rows;
	int C_sim_cols;
	std::vector<float> pathDist0_sim_PT;
	std::vector<float> pathDist0_sim_PV;


	// ----- CONSTRUCTORS ---------------------------- // Note that each Constructor just contains its corresponding Setter
	
	// Constructor Default
	CalibrationMatrix::CalibrationMatrix();
	// Constructor Copy
	// pointers are copied "as are", the C pointed is not duplicated. For this use .clone(...) (if implemented)
	CalibrationMatrix::CalibrationMatrix(CalibrationMatrix & cmx);
	// Constructor All parameters
	CalibrationMatrix::CalibrationMatrix(Info & info_, float* C_, int C_size_, std::vector<float> & pathDist0_, int error_code_,
		float* C_sim_PT_, float* C_sim_PV_, int C_sim_size_, int C_sim_rows_, int C_sim_cols_, std::vector<float> & pathDist0_sim_PT_, std::vector<float> & pathDist0_sim_PV_);
	// Constructor. It creates a CalibrationMatrix object from the .cmx file noted in the info object
 	CalibrationMatrix::CalibrationMatrix(Info & info_);
	// Destructor
	CalibrationMatrix::~CalibrationMatrix();
	
	
	// ----- SETTERS --------------------------------- // Note that each Constructor just contains its corresponding Setter

	// Constructor Default
	void CalibrationMatrix::set ();
	// Constructor Copy
	// pointers are copied "as are", the C pointed is not duplicated. For this use .clone(...) (if implemented)
	void CalibrationMatrix::set (CalibrationMatrix & cmx);
	// Constructor All parameters
	void CalibrationMatrix::set (Info & info_, float* C_, int C_size_, std::vector<float> & pathDist0_, int error_code_,
		float* C_sim_PT_, float* C_sim_PV_, int C_sim_size_, int C_sim_rows_, int C_sim_cols_, std::vector<float> & pathDist0_sim_PT_, std::vector<float> & pathDist0_sim_PV_);
	// Constructor. It creates a CalibrationMatrix object from the .cmx file noted in the info object
 	void CalibrationMatrix::set (Info & info_);
	


	// ----- FUNCTIONS -------------------------------

	// Returns the index in C[], corresponding to the parameter indices.
	// input r,c are considered like Matrix from 0 indexation. It also takes care on PixStoring
	int CalibrationMatrix::C_idx (int freq_idx, int dist_idx, int phas_idx, int r, int c, PixStoring ps = PIXELS_STORING_GLOBAL, bool pSim = false);
	// Returns C[C.C_idx(...)], the value from the Calibration Matrix C corresponding to the parameters
	// input r,c are considered like Matrix from 0 indexation. It also takes care on PixStoring
	float CalibrationMatrix::C_at (int freq_idx, int dist_idx, int phas_idx, int r, int c, PixStoring ps = PIXELS_STORING_GLOBAL, bool pSim = false);
	// Returns the C[...] interpolating with the closest path distances. Path distances have to be equidistant in vector
	// input r,c are considered like Matrix from 0 indexation. It also takes care on PixStoring
	float CalibrationMatrix::C_atX (int freq_idx, float pathDist, int phas_idx, int r, int c, PixStoring ps = PIXELS_STORING_GLOBAL, bool pSim = false);
	
	// Returns the index in pathDist0, corresponding to the parameter indices.
	// input r,c are considered like Matrix from 0 indexation. It also takes care on PixStoring
	int CalibrationMatrix::pathDist0_idx (int r, int c, PixStoring ps = PIXELS_STORING_GLOBAL, bool pSim = false);
	// Returns the value from the pathDist0 corresponding to the parameter indices = pathDist0[pathDist0_idx]
	// input r,c are considered like Matrix from 0 indexation. It also takes care on PixStoring
	float CalibrationMatrix::pathDist0_at (int r, int c, PixStoring ps = PIXELS_STORING_GLOBAL, bool pSim = false);
	
	// This is the Simulation term for the direct vision problem: S_{i\;\omega}^{r,c}(\tau^{r,c}) in the Master Thesis document
	// Returns the value of the Simulation from the Calibration Matrix C at any path distance interpolating with the closest path distances. Path distances have to be equidistant in vector
	// Uses C_atX(...) for interpolating path distances
	float CalibrationMatrix::S_DirectVision (int freq_idx, int phas_idx, int r, int c, Point & r_src, Point & r_x,  Point & r_cam, float relative_albedo = 1.0f, PixStoring ps = PIXELS_STORING_GLOBAL, bool pSim = false);
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
	std::vector<float> data;	// PixStoring dependent. With all the values of the frame. // r,c Matrix-like, 0-idx. 
	PixStoring ps;	// the kind of pixels it can store (PIXELS_TOTAL, PIXELS_VALID, UNKNOWN_PIS). See global.h
	bool pSim;		// tells if the Frame is storing pixels with the Simulated size or if they are actual size. See global.h
	int rows;			// PixStoring dependent
	int cols;			// PixStoring dependent
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
		std::vector<float> & data_, PixStoring ps_, bool pSim_, int rows_, int cols_, float freq_, float dist_, float shut_, float phas_);
	// Constructor from RawData. RawData oriented
	Frame::Frame(RawData & RawData_src_, int freq_idx_, int dist_idx_, int shut_idx_, int phas_idx_, PixStoring ps_ = PIXELS_STORING_GLOBAL, bool pSim_ = false);
	// Constructor from vector. Simulation oriented. For any PixStoring it considers the vector matrix_vector properly arranged
	// If rows_ and cols_ are > 0, they will be the new rows and cols (interesting while simulating arbitrary rows and cols. Otherwise rows and cols are made from ps_)
	Frame::Frame(std::vector<float> & data_, int rows_ = 0, int cols_ = 0, float freq_ = 0.0f, float dist_ = 0.0f, float shut_ = 0.0f, float phas_ = 0.0f, PixStoring ps_ = PIXELS_STORING_GLOBAL, bool pSim_ = false);
	// Constructor from ushort int*. Real Time capture oriented. rows_ and cols_ must be refered to the sizes of the total frame, regerdingless to the PixStoring
	Frame::Frame(unsigned short int* data_, int rowsPT, int colsPT, float freq_, float dist_, float shut_, float phas_, int phas_idx_, PixStoring ps_ = PIXELS_STORING_GLOBAL, bool pSim_ = false, bool first_iter = true);
	// Constructor from std::vector<Frame>. Real Time capture oriented. Average of the vector of Frames
	Frame::Frame(std::vector<Frame> & Frame_v, int Frame_v_size, bool first_iter = true);
	// Constructor stub from basic parameters
	Frame::Frame(Info & info, PixStoring ps_, bool pSim_, int rows_, int cols_, float freq_, float dist_, float shut_, float phas_);

	// ----- SETTERS --------------------------------- // Note that each Constructor just contains its corresponding Setter

	// Setter Default
	void Frame::set ();
	// Setter Copy
	// pointers are copied "as are", the data pointed is not duplicated. For this use .clone(...) (if implemented)
	void Frame::set (Frame & frame);
	// Setter All parameters
	void Frame::set (RawData & RawData_src_, int freq_idx_, int dist_idx_, int shut_idx_, int phas_idx_,
				  std::vector<float> & data_, PixStoring ps_, bool pSim_, int rows_, int cols_, float freq_, float dist_, float shut_, float phas_);
	// Setter from RawData. RawData oriented
	void Frame::set(RawData & RawData_src_, int freq_idx_, int dist_idx_, int shut_idx_, int phas_idx_, PixStoring ps_ = PIXELS_STORING_GLOBAL, bool pSim_ = false);
	// Setter from vector. Simulation oriented. For any PixStoring it considers the vector matrix_vector properly arranged
	// If rows_ and cols_ are > 0, they will be the new rows and cols (interesting while simulating arbitrary rows and cols. Otherwise rows and cols are made from ps_
	void Frame::set(std::vector<float> & data_, int rows_ = 0, int cols_ = 0, float freq_ = 0.0f, float dist_ = 0.0f, float shut_ = 0.0f, float phas_ = 0.0f, PixStoring ps_ = PIXELS_STORING_GLOBAL, bool pSim_ = false);
	// Setter from ushort int*. Real Time capture oriented. rowsPT and colsPT must be refered to the sizes of the total frame, regerdingless to the PixStoring
	void Frame::set(unsigned short int* data_, int rowsPT, int colsPT, float freq_, float dist_, float shut_, float phas_, int phas_idx_, PixStoring ps_ = PIXELS_STORING_GLOBAL, bool pSim_ = false, bool first_iter = true);
	// Setter from std::vector<Frame>. Real Time capture oriented. Average of the vector of Frames
	void Frame::set(std::vector<Frame> & Frame_v, int Frame_v_size, bool first_iter = true);
	// Setter stub from basic parameters
	void Frame::set(Info & info, PixStoring ps_, bool pSim_, int rows_, int cols_, float freq_, float dist_, float shut_, float phas_);
	

	// ----- FUNCTIONS -------------------------------

	// r,c Matrix-like, 0-idx.
	int Frame::data_idx (int r, int c);
	// r,c Matrix-like, 0-idx.
	float Frame::at (int r, int c);

	// Frame Real to Frame with with Simulated Pixels
	void Frame::toPixSim(int rowsSim = PMD_SIM_ROWS, int colsSim = PMD_SIM_COLS);
	// Frame Simulated to Frame Real (useless, but for testing)
	void Frame::toPixReal();
	// Plot frame with opencv
	void Frame::plot(int delay_ms = 1000, bool destroyWindow_ = false, char* windowName = NULL, float scale = -1.0f);
};
// plot frame amplitude with sinusoidal assumption
void plot_frame(Frame & frame_00, Frame & frame_90, int delay_ms = 1000, bool destroyWindow_ = false, char* windowName = NULL, float scale = -1.0f);
// For FoV measurement scene. Plot frame with opencv with syncronization
void plot_frame_fov_measurement(Frame & frame_00, Frame & frame_90, bool loop = false, bool destroyWindow_ = false, char* windowName = NULL, bool line_center = false, int lines_grid = 0);
// Plots a row (if >=0) or a col (otherwise and if >= 0) of a Frame using MATALAB engine
void plot_rowcol(Frame & frame, char* text, int row, int col, bool & epExtStarted, bool epExtUsing = false, Engine *epExt = NULL);
// Plots a row, a colum, the average of every row or the average every columns of 2 Frames using MATALAB engine
//   (row >= 0) && (col <  0) && (avg == false): Plots the raw
//   (row <  0) && (col >= 0) && (avg == false): Plots the col
//   (row >= 0) && (col <  0) && (avg == true ): Plots the average of every raw
//   (row <  0) && (col >= 0) && (avg == true ): Plots the average of every col
//   else: undefined behavior
void plot_rowcol2(Frame & frame0, Frame & frame1, char* text0, char* text1, int row, int col, bool avg, bool & epExtStarted, bool epExtUsing = false, Engine *epExt = NULL);
// Plots a row, a colum, the average of every row or the average every columns of 4 Frames using MATALAB engine
//   (row >= 0) && (col <  0) && (avg == false): Plots the raw
//   (row <  0) && (col >= 0) && (avg == false): Plots the col
//   (row >= 0) && (col <  0) && (avg == true ): Plots the average of every raw
//   (row <  0) && (col >= 0) && (avg == true ): Plots the average of every col
//   else: undefined behavior
void plot_rowcol4(Frame & frameR00, Frame & frameS00, Frame & frameR90, Frame & frameS90, char* textR00, char* textS00, char* textR90, char* textS90, int row, int col, bool avg, bool & epExtStarted, bool epExtUsing, Engine *epExt);
// Plots a row, a colum, the average of every row or the average every columns of all the Frames of a vector of frames using MATALAB engine
//   (row >= 0) && (col <  0) && (avg == false): Plots the row
//   (row <  0) && (col >= 0) && (avg == false): Plots the col
//   (row >= 0) && (col <  0) && (avg == true ): Plots the average of every row
//   (row <  0) && (col >= 0) && (avg == true ): Plots the average of every col
//   else: undefined behavior
void plot_rowcolV(std::vector<Frame> & frameV, std::vector<char*> & textV, std::vector<float> & colorV, float lineWidth, int row, int col, bool avg, bool legend, bool freezePlot, bool & epExtStarted, bool epExtUsing, Engine *epExt);



// ----------------------------------------------------------------------------------------------------------------------------------------
// ----- OTHER FUNCTIONS ------------------------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------------------

// sets a vector of floats form a char array from a given delimiter
void char_array_to_float_vector_from_delimiter (char* char_array, std::vector<float> & float_vector, char delimiter);

// returns the min, max, sum, mean, variance value or index of the vector
float min(std::vector<float> & v);
float max(std::vector<float> & v);
float maxAbsSigned(std::vector<float> & v);
float min(std::vector<float> & v, int & min_idx);
float max(std::vector<float> & v, int & max_idx);
float sum(std::vector<float> & v);
float mean(std::vector<float> & v);
float var(std::vector<float> & v);
float avgError(std::vector<float> & v0, std::vector<float> & v1);

// operates an element over a vector. Sizes must match, does not resizes
void sumElemToVector(float x, std::vector<float> & vIn, std::vector<float> & vOut);
void subElemToVector(float x, std::vector<float> & vIn, std::vector<float> & vOut);
void subPow2ElemToVector(float x, std::vector<float> & vIn, std::vector<float> & vOut);
void mulElemToVector(float x, std::vector<float> & vIn, std::vector<float> & vOut);
void divElemToVector(float x, std::vector<float> & vIn, std::vector<float> & vOut);
// operates a vector over a vector. Sizes must match, does not resizes
void sumVectorToVector(std::vector<float> & vIn0, std::vector<float> & vIn1, std::vector<float> & vOut);
void subVectorToVector(std::vector<float> & vIn0, std::vector<float> & vIn1, std::vector<float> & vOut);
void subPow2VectorToVector(std::vector<float> & vIn0, std::vector<float> & vIn1, std::vector<float> & vOut);
void mulVectorToVector(std::vector<float> & vIn0, std::vector<float> & vIn1, std::vector<float> & vOut);
void divVectorToVector(std::vector<float> & vIn0, std::vector<float> & vIn1, std::vector<float> & vOut);

// these functions returns the corresponding idx/stuff of a vector from r,c considering Matrix-like ordering 0-indexed.
int rc2idx (int r, int c, PixStoring ps = PIXELS_STORING_GLOBAL, bool pSim = false);
int rc2idxCor(int r, int c, PixStoring ps = PIXELS_STORING_GLOBAL, bool pSim = false);	// see scene.cpp, setPixelPatchesNCor(...)
int rc2idxPT(int r, int c);
int rc2idxPV(int r, int c);
int rc2idxPSIM(int r, int c);
int rc2idxFromPT2PV(int r, int c, bool pSim = false);	// returns -1 if the PT(r,c) is out of PV(r,c) range !!!
int numPix(PixStoring ps = PIXELS_STORING_GLOBAL, bool pSim = false);
int numPixCor(PixStoring ps = PIXELS_STORING_GLOBAL, bool pSim = false);	// see scene.cpp, setPixelPatchesNCor(...)
int numPixPT();
int numPixPV();
int numPixPSIM();
int rows(PixStoring ps = PIXELS_STORING_GLOBAL, bool pSim = false);
int cols(PixStoring ps = PIXELS_STORING_GLOBAL, bool pSim = false);
int rowsCor(PixStoring ps = PIXELS_STORING_GLOBAL, bool pSim = false);
int colsCor(PixStoring ps = PIXELS_STORING_GLOBAL, bool pSim = false);

// returns the corresponding index. Return -1 if no correspondance found
int get_freq_idx (Info & info, float freq, float RelDiffMax = 0.0001);
int get_dist_idx (Info & info, float dist, float RelDiffMax = 0.0001);
int get_shut_idx (Info & info, float shut, float RelDiffMax = 0.0001);
int get_phas_idx (Info & info, float phas, float RelDiffMax = 0.0001);


// other auxiliar functions
bool equalAproxf(float f0, float f1, float RelDiffMax = 0.0001);
void print(std::vector<float> & v, char* prefix = NULL, char* sufix = NULL);
void print(std::vector<int> & v, char* prefix = NULL, char* sufix = NULL);

#endif

