
#ifndef __CAPTURETOOLDM2_H
#define __CAPTURETOOLDM2_H


#include <vector>

// capturetool2_main
int PMD_charArray_to_file (int argc, char *argv[]);

// PMD_params_to_file
int PMD_params_to_file (std::vector<float> & frequencies, std::vector<float> & delays, std::vector<float> & shutters_float, char* dir_name, char* file_name, char* comport_, int & numtakes);

// MAIN
int capturetoolDM2_main(int argc, char *argv[]);


#endif
