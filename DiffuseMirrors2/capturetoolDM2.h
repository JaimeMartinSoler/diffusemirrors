
#ifndef __CAPTURETOOLDM2_H
#define __CAPTURETOOLDM2_H

#include "global.h"
#include "data_read.h"
#include <vector>


void process_data(int w, int h, std::vector<std::pair<int, unsigned short* > > &shutters, char* path, char* file_prefix, int pass = 0, FILE *rawdumpfile = NULL);

// Author: Jaime Martin (modification of process_data())
void process_data_no_cv(int w, int h, std::vector<std::pair<int, unsigned short* > > &shutters, FILE *rawdumpfile = NULL);

// Author: Jaime Martin (modification of process_data())
void process_data_to_buffer(int w, int h, std::vector<std::pair<int, unsigned short* > > &shutters, unsigned short* ushort_img[2], int pass = 0);

// Author: Jaime Martin (modification of process_data()). It does not plot frames with openCV
void process_data_to_buffer_no_cv(int w, int h, std::vector<std::pair<int, unsigned short* > > &shutters, unsigned short* ushort_img[2]);

bool dir_exists(const std::string& dirName_in);

// Author: Jaime Martin
bool file_exists (const std::string& name);

// Author: Jaime Martin
// str_is_number ()
bool str_is_number (std::string & str);

// Author: Jaime Martin
// char_array_to_float_vector (...)
void char_array_to_float_vector (char* & char_array, std::vector<float> & float_vector, float min, float max);

// Author: Jaime Martin
// parser_main(...)
// return 0:  No errors parsing
// return -1: Errors parsing
int parser_main (int argc, char *argv[], std::vector<float> & frequencies, std::vector<float> & delays, std::vector<float> & shutters_float, char* dir_name, char* file_name, char* comport, int & numtakes);

// Author: Jaime Martin
// check_parameters(...)
// return 0: all parameters OK
// return 1: any parameter out of bounds, modified
// return 2: unproper dimensions or size
int check_parameters (float & frequency_, float & shutter_, char* comport, bool show_text = true, bool ask_to_continue_ = true);

// Author: Jaime Martin
// check_input_data(...)
// return    0, 1:     !continue / continue
int check_input_data(float frequency_, float distance_, float shutter_, char* comport, bool loop, bool show_text, bool ask_to_continue_);

// Author: Jaime Martin
// check_input_data_vectors(...)
// return    0, 1:     !continue / continue
int check_input_data_vectors (std::vector<float> & frequencies, std::vector<float> & delays, std::vector<float> & shutters_float, char* dir_name, char* file_name, char* comport, int & numtakes, bool show_text, bool ask_to_continue_);

// Author: Jaime Martin
// check_parameters_vector(...)
// return 0: all parameters OK
// return 1: any parameter out of bounds, modified
// return 2: unproper dimensions or size
int check_parameters_vector (std::vector<float> & frequencies, std::vector<float> & delays, std::vector<float> & shutters_float, char* comport, int & numtakes, bool show_text = true, bool ask_to_continue_ = true);

// Author: Jaime Martin
// PMD_charArray_to_file
int PMD_charArray_to_file (int argc, char *argv[]);

// Author: Jaime Martin
// ask_to_continue()
// return false / true:    !continue / continue
int ask_to_continue();

// Author: Jaime Martin
// check_overwrite()
// return    0, 1, 2:    file_exists(...)+!continue, file_exists(...)+continue, !file_exists(...)+continue
int check_overwrite (char* file_name, bool show_text = true, bool ask_to_continue_ = true);

// Author: Jaime Martin
// check_frame(...)
// return    0, 1:     !continue / continue
int check_frame(float freq, float dist, float shutter, char* comport, bool show_text = true, bool ask_to_continue_ = true);

// Author: Jaime Martin
// check_file_size(...)
// return    0, 1:     !continue / continue
// if file_size_B_take < 0.0f: file_size_B_take info will not be printed
int check_file_size(float file_size_B_take, int numtakes, bool show_text = true, bool ask_to_continue_ = true);

// Author: Jaime Martin
// check_time(...)
// return    0, 1:     !continue / continue
// if (numtakes > 1) : the text shows file_size_MB_take, file_size_MB_tot, numtakes
// if (numtakes == 1): the text shows file_size_MB_take, numtakes
// if (numtakes <= 0): the text shows file_size_MB_take
int check_time(float time_tot_s, bool show_text = true, bool ask_to_continue_ = true);

// Converts time in seconds to time in hours, minutes and seconds
void seconds_to_hms (float time_tot_s, float & time_h, float & time_m, float & time_s);

// Author: Jaime Martin
// countdown(...)
void countdown(bool ask_number = false, int default_time = 10, bool show_text = true);

// Author: Jaime Martin (modification of previous function)
// PMD_params_to_file
int PMD_params_to_file (std::vector<float> & frequencies, std::vector<float> & delays, std::vector<float> & shutters_float, char* dir_name, char* file_name, char* comport, int & numtakes, bool cmx_info = false, float* cmx_params = NULL);
// there's a weird bug when calling directly to PMD_params_to_file from thread constructor. With this re-calling functtion the bug is avoided
int PMD_params_to_file_anti_bug_thread (std::vector<float> & frequencies, std::vector<float> & delays, std::vector<float> & shutters_float, char* dir_name, char* file_name, char* comport, int & numtakes, bool cmx_info = false, float* cmx_params = NULL);

// creates a the corresponding averaged raw file from the raw takes files
void create_raw_from_raw_takes (Info* info);

// Author: Jaime Martin
// create_cmx_from_raw (...)
void create_cmx_from_raw (Info* info_);
// there's a weird bug when calling directly to PMD_params_to_file from thread constructor. With this re-calling functtion the bug is avoided
void create_cmx_from_raw_anti_bug_thread (Info* info_);

// Author: Jaime Martin
// copy_array (...)
// return 0: no error, in bounds
// return 1: error, out of bounds
int copy_array (unsigned short int* dst, unsigned short int* src, int dst_pos, int dst_size, int src_size);

// Author: Jaime Martin (modification of previous function)
// PMD_params_to_RawData
int PMD_params_to_RawData (RawData & RawData_cap, std::vector<float> & frequencies, std::vector<float> & delays, std::vector<float> & shutters_float, char* comport, int & numtakes, bool loop);

// Author: Jaime Martin (modification of previous function)
// PMD_params_to_Frame
int PMD_params_to_Frame (Frame & Frame_00_cap, Frame & Frame_90_cap, float frequency_, float distance_, float shutter_, char* comport, bool loop);
// there's a weird bug when calling directly to PMD_params_to_Frame from thread constructor. With this re-calling functtion the bug is avoided
int PMD_params_to_Frame_anti_bug_thread (Frame & Frame_00_cap, Frame & Frame_90_cap, float frequency_, float distance_, float shutter_, char* comport, bool loop);

// Author: Jaime Martin 
// MAIN
int capturetoolDM2_main(int argc, char *argv[], bool loop = false);


#endif
