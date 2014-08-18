
#include "capturetoolDM2.h"
#include "data_read.h"
#include "global.h"
#include <stdio.h>
#include <stdlib.h>     // atof
#include <pmdsdk2.h>
#include <Windows.h> // For timing calculations
#include <algorithm>
#include <assert.h>
//#include "CImg.h"
#include "SerialPort.h"
// OPENCV INCLUDES
#include <cv.h>
#include <cvaux.h>
#include <highgui.h>

// VARIABLES
char err[128];
FILE *rawdumpfile;
char comport_full_name[128] = "\\\\.\\COM6";

using namespace std;
using namespace cv;


#if /* Communication with USB trigger/control unit */ 1  
// Serial port for function generator/trigger control
SerialPort& control_init(const char *portname) {
	SerialPort *port = new SerialPort(portname, CBR_57600, ONESTOPBIT);
	port->WriteString("Q\n"); // Enter quiet mode
	return *port;
}
void control_setfrequency(SerialPort &port, unsigned int tuningword_32bit) {
	char command[16];
	sprintf_s<16>(command, "F %X\n", tuningword_32bit);
	port.WriteString(command);
}
void control_setphase(SerialPort &port, unsigned int phase_14bit) {
	char command[16];
	phase_14bit = (phase_14bit + 0x100000) & 0x3FFF;
	sprintf_s<16>(command, "P %hX\n", phase_14bit);
	port.WriteString(command);
}
void control_setshutter(SerialPort &port, unsigned int shutter_us) {
	char command[16];
	sprintf_s<16>(command, "S %hX\n", shutter_us);
	port.WriteString(command);
}
#endif

// Sets frequency and returns accurate value
double set_frequency(SerialPort &port, double frequency_MHz) {
	double tuningword = (0x100000000 * (frequency_MHz / SYNTH_CLOCK) + 0.5);
	control_setfrequency(port, (unsigned int) tuningword);
	return tuningword / 0x100000000 * SYNTH_CLOCK;
}

void set_delay(SerialPort &port, double frequency_MHz, double delay_m) {

	double timedelay = delay_m / SPEEDOFLIGHT_AIR;
	unsigned short phase = 0x3FFF & (unsigned short)(frequency_MHz * 1e6 * timedelay * 16384 + 0.5 + 0x200000);
	//cout << "timedelay in ns:" <<(timedelay*1e9)<<" Frequency MHz:" << frequency_MHz << " Phase: "<<phase<<endl;
	control_setphase(port, phase);
}

inline void set_shutter(SerialPort &port, unsigned int shutter_us) {
	control_setshutter(port, shutter_us);
}

#if /* Misc data handling */ 1  
void swap_endian(unsigned short *buffer, size_t n) {
	for (size_t i = 0; i < n; ++i) {
		unsigned char *bytebuf = (unsigned char*) buffer;
		unsigned char tmp = bytebuf[2*i];
		bytebuf[2*i] = bytebuf[2*i+1];
		bytebuf[2*i+1] = tmp;
	}
}
void to_unsigned(unsigned short *buffer, size_t n) {

	for (size_t i = 0; i < n; ++i) {
		short *signedbuf = (short*) buffer;
		buffer[i] = signedbuf[i] + 32768;
	}
}

// PMD helper functions
void pmd_handle_error(PMDHandle &hnd, int res, const char* msg) {
	if (res != PMD_OK)
	{
		pmdGetLastError (0, err, 128);
		fprintf (stderr, "%s: %s\n", msg, err);
		pmdClose (hnd);
		exit(res);
	}
}
#endif

#if /* Timing measurement */ 1
// Timing measurements
double get_cpu_frequency() {
	LARGE_INTEGER frequency;
	if (::QueryPerformanceFrequency(&frequency) == FALSE)
		return 1;
	return frequency.HighPart * 4294967296.0f + frequency.LowPart;
}
double get_cpu_time_cycles() {
	LARGE_INTEGER time;
	if (::QueryPerformanceCounter(&time) == FALSE)
		return 1;
	return time.HighPart * 4294967296.0f + time.LowPart;
}
double get_cpu_time_ms(double cycles, double freq) {
	return cycles/freq * 1e3;
}
double get_cpu_time_us(double cycles, double freq) {
	return cycles/freq * 1e6;
}
double get_cpu_time_s(double cycles, double freq) {
	return cycles/freq;
}

#endif

void save_ppm(const char* filename, void* data, int w, int h, int maxvalue, int channels) {

	FILE *fp = fopen(filename, "wb"); 
	if (channels == 3)
		fprintf(fp, "P6\n%d %d\n%d\n", w, h, maxvalue);
	else
		fprintf(fp, "P5\n%d %d\n%d\n", w, h, maxvalue);
	fwrite(data, (maxvalue > 255) ? 2 : 1, w * h * channels, fp);
	fclose(fp);
}
void save_raw(const char* filename, void* data, int w, int h, int maxvalue, int channels) {

	FILE *fp = fopen(filename, "wb"); 
	fwrite(data, (maxvalue > 255) ? 2 : 1, w * h * channels, fp);
	fclose(fp);
}

void dump_data(FILE *handle, void* data, int w, int h, int maxvalue, int channels) {

	fwrite(data, (maxvalue > 255) ? 2 : 1, w * h * channels, handle);
}


int pmd_capture(PMDHandle &hnd, SerialPort &port, unsigned short shutter, double frequency_MHz, double delay_m, unsigned short *buffer, int &w, int &h, int &numframes) {
	//cout << endl << "Capturing frame" << flush;
	// Make frequency generator settings
	double true_frequency = set_frequency(port, frequency_MHz);
	set_delay(port, true_frequency, delay_m);
	set_shutter(port, shutter);
	//cout << "." << flush;

	int res;
	// Set exposure time on camera
	res = pmdSetIntegrationTime(hnd, 0, shutter+12); // Add a few microseconds overhead 
	pmd_handle_error(hnd, res, "Could not set integration time");
	//cout << "." << flush;

	// Trigger capture!
	res = pmdUpdate (hnd);
	pmd_handle_error(hnd, res, "Could not update");
	//cout << "." << flush;

	// Retrieve data from camera
	PMDDataDescription dd;
	res = pmdGetSourceDataDescription(hnd, &dd);
	pmd_handle_error(hnd, res, "Could not retrieve source data description");
	w = dd.img.numColumns;
	h = dd.img.numRows;
	numframes = dd.img.numSubImages;
	res = pmdGetSourceData(hnd, buffer, w*h*numframes*4);
	pmd_handle_error(hnd, res, "Could not get source data");
	//cout << "." << flush;

	// Data is in big endian format
	swap_endian(buffer, w*h*numframes);

	// Cut the signal generator some slack
	set_frequency(port, 31.0);
	//cout << "." << flush;
	//cout << "done." << endl;
	return 0;
}
void hsv_to_rgb(float H, float S, float V, float &R, float &G, float &B){
	if (S>1) S=1;
	if (S<0) S=0;
	if (V>1) V=1;
	if (V<0) V=0;

	H = fmod(H, 1.f);

    int i = H * 6;
    float f = H * 6 - i;
    float p = V * (1 - S);
    float q = V * (1 - f * S);
    float t = V * (1 - (1 - f) * S);

    switch(i % 6){
        case 0: R = V, G = t, B = p; break;
        case 1: R = q, G = V, B = p; break;
        case 2: R = p, G = V, B = t; break;
        case 3: R = p, G = q, B = V; break;
        case 4: R = t, G = p, B = V; break;
        case 5: R = V, G = p, B = q; break;
    }
}

void process_data(int w, int h, vector<pair<int, unsigned short* > > &shutters, char* path, char* file_prefix, int pass = 0, FILE *rawdumpfile = NULL) {

	int capturecount = shutters.size();
	int* int_img = new int[w*2*h];
	unsigned short* ushort_img[2];
	ushort_img[0] = new unsigned short[w*h*capturecount*2];
	ushort_img[1] = new unsigned short[w*h*capturecount];
	float* float_img = new float[w*2*h];
	unsigned char* jpg_img = new unsigned char[3*w*h*capturecount];
	
	char fname[1024];

	IplImage* img = cvCreateImageHeader(cvSize(w*3,h*capturecount), IPL_DEPTH_8U, 1);

	// Loop through all exposures
	for (size_t i = 0; i < capturecount; ++i)  { 
		int shuttertime = shutters[i].first;
		unsigned short* ubuf = shutters[i].second + w * h; // (discard first subframe)

		// Compute difference images between opposite phases
		for (int x = 0; x < w; ++x) {
			for (int y = 0; y < h; ++y) 
				for (int ph = 0; ph < 2; ++ph) { // 0: 0deg, 1: 90deg

					int diff = (int) ubuf[x + y*w + (2+ph)*w*h] - ubuf[x + y*w +ph*w*h];
					int_img[x + y*w + ph*w*h] = diff;

					float gammacorr = pow((double)(diff * 2+32768)/65536.f, 1/1);
					unsigned char eightbitvalue = 255 * gammacorr;
					jpg_img[((x+ y*3*w + ph*w+3*i*w*h))] 
					//= rgb_img[(3* (x+ y*3*w + ph*w+3*i*w*h))+1] 
					//= rgb_img[(3* (x+ y*3*w + ph*w+3*i*w*h))+2] 
					= eightbitvalue;

					// Bias and clamp to 16-bit range
					diff += 32768;
					ushort_img[0][x + y*w + i*w*h + capturecount*ph*w*h] = (diff > 65535) ? 65535 : ((diff < 0) ? 0  : diff);
				}}

		//if (i == capturecount - 1) 
		{
			// False color phase/amplitude image for last image
			float max_amplitude = 0;
			for (int x = 10; x < w-10; ++x) {
				for (int y = 10; y < h-10; ++y) {
					float ix = int_img[x+y*w];
					float iy = int_img[x+y*w + w*h];
					float amplitude = sqrt(iy*iy+ix*ix);
					if (amplitude > max_amplitude) 
						max_amplitude = amplitude;
				}
			}
			for (int x = 0; x < w; ++x) {
				for (int y = 0; y < h; ++y) {
					float ix = int_img[x+y*w];
					float iy = int_img[x+y*w + w*h];
					jpg_img[((x+ y*3*w + 2*w + 3 * i * w * h))] = pow(sqrt(iy*iy+ix*ix) / max_amplitude, 1.f/2.2f) * 255;
				}
			}
		}	
	}
	cvSetData(img, jpg_img, 3 * w);
	sprintf_s<1024>(fname, "%s\\%s.%s", path, file_prefix,  "jpg");
#ifndef NOJPEG
		cvSaveImage(fname, img);
#endif	
	// 0deg
	sprintf_s<1024> (fname, "%s\\%s" FILENAME_APPEND, path, file_prefix,   0, "dat");
	// UNDO THIS: save_raw(fname, &(ushort_img[0][0]), w, capturecount*h*2, 65535, 1);
	dump_data(rawdumpfile, &(ushort_img[0][0]), w, capturecount*h*2, 65535, 1);

	// 90deg
	//sprintf_s<1024>(fname, "%s\\%s" FILENAME_APPEND, path, file_prefix,  90, "dat");
	//save_raw(fname, &(ushort_img[1][0]), w, capturecount*h, 65535, 1);
	
	if (pass == 0) 
	{cvShowImage( WindowName, img );
	cvWaitKey( 10 );}
	delete jpg_img;
	delete float_img;
	delete ushort_img[0];
	delete ushort_img[1];
	delete int_img;
	cvReleaseImageHeader(&img);
}

// Author: Jaime Martin (modification of process_data())
void process_data_to_buffer(int w, int h, std::vector<std::pair<int, unsigned short* > > &shutters, unsigned short* ushort_img[2], int pass) {	// default: (int pass = 0)

	int capturecount = shutters.size();
	int* int_img = new int[w*2*h];
	//unsigned short* ushort_img[2];
	ushort_img[0] = new unsigned short[w*h*capturecount*2];
	ushort_img[1] = new unsigned short[w*h*capturecount];
	float* float_img = new float[w*2*h];
	unsigned char* jpg_img = new unsigned char[3*w*h*capturecount];
	
	char fname[1024];

	IplImage* img = cvCreateImageHeader(cvSize(w*3,h*capturecount), IPL_DEPTH_8U, 1);

	// Loop through all exposures
	for (size_t i = 0; i < capturecount; ++i)  { 
		int shuttertime = shutters[i].first;
		unsigned short* ubuf = shutters[i].second + w * h; // (discard first subframe)

		// Compute difference images between opposite phases
		for (int x = 0; x < w; ++x) {
			for (int y = 0; y < h; ++y) 
				for (int ph = 0; ph < 2; ++ph) { // 0: 0deg, 1: 90deg

					int diff = (int) ubuf[x + y*w + (2+ph)*w*h] - ubuf[x + y*w +ph*w*h];
					int_img[x + y*w + ph*w*h] = diff;

					float gammacorr = pow((double)(diff * 2+32768)/65536.f, 1/1);
					unsigned char eightbitvalue = 255 * gammacorr;
					jpg_img[((x+ y*3*w + ph*w+3*i*w*h))] 
					//= rgb_img[(3* (x+ y*3*w + ph*w+3*i*w*h))+1] 
					//= rgb_img[(3* (x+ y*3*w + ph*w+3*i*w*h))+2] 
					= eightbitvalue;

					// Bias and clamp to 16-bit range
					diff += 32768;
					ushort_img[0][x + y*w + i*w*h + capturecount*ph*w*h] = (diff > 65535) ? 65535 : ((diff < 0) ? 0  : diff);
				}}

		//if (i == capturecount - 1) 
		{
			// False color phase/amplitude image for last image
			float max_amplitude = 0;
			for (int x = 10; x < w-10; ++x) {
				for (int y = 10; y < h-10; ++y) {
					float ix = int_img[x+y*w];
					float iy = int_img[x+y*w + w*h];
					float amplitude = sqrt(iy*iy+ix*ix);
					if (amplitude > max_amplitude) 
						max_amplitude = amplitude;
				}
			}
			for (int x = 0; x < w; ++x) {
				for (int y = 0; y < h; ++y) {
					float ix = int_img[x+y*w];
					float iy = int_img[x+y*w + w*h];
					jpg_img[((x+ y*3*w + 2*w + 3 * i * w * h))] = pow(sqrt(iy*iy+ix*ix) / max_amplitude, 1.f/2.2f) * 255;
				}
			}
		}	
	}
	cvSetData(img, jpg_img, 3 * w);
	char* path = "f:\tmp";
	char* file_prefix = "DiffuseMirrors2_tmp";
	sprintf_s<1024>(fname, "%s\\%s.%s", path, file_prefix,  "jpg");
#ifndef NOJPEG
		cvSaveImage(fname, img);
#endif
	sprintf_s<1024> (fname, "%s\\%s" FILENAME_APPEND, path, file_prefix, 0, "dat");
	
	if (pass == 0) 
	{cvShowImage( WindowName, img );
	cvWaitKey( 10 );}
	delete jpg_img;
	delete float_img;
	//delete ushort_img[0];
	//delete ushort_img[1];
	delete int_img;
	cvReleaseImageHeader(&img);
}


// Author: Jaime Martin (modification of process_data()). It does not plot frames with openCV
void process_data_to_buffer_no_cv(int w, int h, std::vector<std::pair<int, unsigned short* > > &shutters, unsigned short* ushort_img[2], int pass) {	// default: (int pass = 0)

	int capturecount = shutters.size();
	ushort_img[0] = new unsigned short[w*h*capturecount*2];
	ushort_img[1] = new unsigned short[w*h*capturecount];

	// Loop through all exposures
	for (size_t i = 0; i < capturecount; ++i)  { 
		int shuttertime = shutters[i].first;
		unsigned short* ubuf = shutters[i].second + w * h; // (discard first subframe)

		// Compute difference images between opposite phases
		for (int x = 0; x < w; ++x) {
			for (int y = 0; y < h; ++y) {
				for (int ph = 0; ph < 2; ++ph) { // 0: 0deg, 1: 90deg

					int diff = (int) ubuf[x + y*w + (2+ph)*w*h] - ubuf[x + y*w +ph*w*h];
					float gammacorr = pow((double)(diff * 2+32768)/65536.f, 1/1);
					unsigned char eightbitvalue = 255 * gammacorr;

					// Bias and clamp to 16-bit range
					diff += 32768;
					ushort_img[0][x + y*w + i*w*h + capturecount*ph*w*h] = (diff > 65535) ? 65535 : ((diff < 0) ? 0  : diff);
	}	}	}	}
}




template <typename T>
T ato (const string &text) {
	stringstream ss(text);
	T result;
	return ss >> result ? result : 0;
}

bool dir_exists(const std::string& dirName_in)
{
  DWORD ftyp = GetFileAttributesA(dirName_in.c_str());
  if (ftyp == INVALID_FILE_ATTRIBUTES)
    return false;  //something is wrong with your path!

  if (ftyp & FILE_ATTRIBUTE_DIRECTORY)
    return true;   // this is a directory!

  return false;    // this is not a directory!
}

// Author: Jaime Martin
bool file_exists (const std::string& name) {
    if (FILE *file = fopen(name.c_str(), "r")) {
        fclose(file);
        return true;
    }
    return false;  
}

// Author: Jaime Martin
// str_is_number ()
bool str_is_number (std::string & str) {
	
	int points = 0;
	int size = str.length();
	if (size < 1)
		return false;

	for (int i = 0; i < size; i++) {
		if (((str[i] < '0') || (str[i] > '9')) && (str[i] != '.'))
			return false;
		if (str[i] == '.') {
			points++;
			if ((i==0) || (i==size-1) || (points > 1))
				return false;
		}
	}
	return true;
}

// Author: Jaime Martin
// char_array_to_float_vector (...)
void char_array_to_float_vector (char* & char_array, std::vector<float> & float_vector, float min, float max) {

    //char_array[char_array_Length-1] = '\0';
    float float_value;
    char char_value;
    string float_str = "";
    int idx_float_str = 0;
    while(true) {
        char_value = char_array[idx_float_str++];
        if (char_value == ' ' || char_value == '\0') {
			if (!str_is_number(float_str)) {
				float_vector.clear();
				return;
			}
            float_value = ato<float>(float_str.c_str());
            float_str = "";
            if (float_value < min) float_value = min;
            else if (float_value > max) float_value = max;
            float_vector.push_back(float_value);
            if (char_value == '\0')
                break;
        } else {
            float_str += char_value;
        }
    }  
}


// Author: Jaime Martin
// parser_main(...)
// return 0:  No errors parsing
// return -1: Errors parsing
int parser_main (int argc, char *argv[], std::vector<float> & frequencies, std::vector<float> & delays, std::vector<float> & shutters_float, char* dir_name, char* file_name, char* comport, int & numtakes) {
	
	// Check the number of arguments. Print info and return -1 if it is wrong
	if (argc < 7) {
		std::cout << endl << "TransientPMD capture tool 2. Synopsis: " << endl << endl <<
			"  capturetool2.exe  // name of binary" << endl <<
			"  freq_array       // Freqs (MHz) to measure (ex: \"80 90.5 100.0\")" << endl <<
			"  dist_array       // Distances sweep (m) to measure (ex: \"0 0.5 1\")" << endl <<
			"  shutter_array    // Shutter/Exposures (us) to measure (ex: \"960 1920\")" << endl <<
			"  dir_name         // Target directory (ex: f:\\tmp\\pmdtest2)" << endl <<
			"  file_name        // File name (ex: PMD_measurements)" << endl <<
			"  comport          // USB serial port (ex: COM6)" << endl << 
			"  [numtakes=1]     // Repeat measurement multiple times? (ex: 10)" << endl << endl;
		return -1;
	}
	
	// Frequencies (argv[1])
	char_array_to_float_vector (argv[1], frequencies, FREQUENCY_MIN, FREQUENCY_MAX);
	
	// Delays (argv[2])
	char_array_to_float_vector (argv[2], delays, -FLT_MAX/2.0f, FLT_MAX/2.0f);
	
	// Shutters (argv[3])
	char_array_to_float_vector (argv[3], shutters_float, SHUTTER_MIN, SHUTTER_MAX);
	
	// Directory Name (argv[4])
	sprintf(dir_name,"%s", argv[4]);
	
	// File Name (argv[5])
	sprintf(file_name,"%s", argv[5]);
	
	// Comport (argv[6])
	sprintf(comport, "%s", argv[6]);
	sprintf(comport_full_name, COMPORT_FORMAT, argv[6]);	// configure comport_full_name
	
	// Number of Takes (argv[7)
	if (argc >= 8) {
		numtakes = ato<int> (argv[7]);
        if (numtakes < 1) numtakes = 1;
	}

	// Check the size of the vectors. Return -1 if it is wrong
	if (frequencies.size() < 1)
		std::cout << endl << "Frequencies array (argv[1]) empty or wrong. Quitting.";
	if (delays.size() < 1)
		std::cout << endl << "Delays array (argv[2]) empty or wrong. Quitting.";
	if (shutters_float.size() < 1)
		std::cout << endl << "Shutters/Exposures array (argv[3]) empty or wrong. Quitting.";
	if ((frequencies.size() < 1) || (delays.size() < 1) || (shutters_float.size() < 1)) {
		std::cout << endl  << endl;
		return -1;
	}

	// No errors
	return 0;
}


// Author: Jaime Martin
// check_parameters(...)
// return 0: all parameters OK
// return 1: any parameter out of bounds, modified
// return 2: unproper dimensions or size
int check_parameters (float frequency_, float shutter_, char* comport) {
	
	sprintf(comport_full_name, COMPORT_FORMAT, comport);	// configure comport_full_name

	// checking all parameter in bounds, otherwise modify them
	int return_value = 0;
	if (frequency_ < FREQUENCY_MIN) {
			frequency_ = FREQUENCY_MIN;
			return_value = 1;
	}
	else if (frequency_ > FREQUENCY_MAX) {
			frequency_ = FREQUENCY_MAX;
			return_value = 1;
	}
	if (shutter_ < SHUTTER_MIN) {
			shutter_ = SHUTTER_MIN;
			return_value = 1;
	}
	else if (shutter_ > SHUTTER_MAX) {
			shutter_ = SHUTTER_MAX;
			return_value = 1;
	}

	return return_value;
}



// Author: Jaime Martin
// check_parameters_vector(...)
// return 0: all parameters OK
// return 1: any parameter out of bounds, modified
// return 2: unproper dimensions or size
int check_parameters_vector (std::vector<float> & frequencies, std::vector<float> & delays, std::vector<float> & shutters_float, char* comport, int & numtakes) {
	
	sprintf(comport_full_name, COMPORT_FORMAT, comport);	// configure comport_full_name

	// ckecking unproper dimensions or size
	if (frequencies.size() < 1) {
		std::cout << endl << "Frequencies array empty or wrong. Quitting.";
		return 2;
	}
	if (delays.size() < 1) {
		std::cout << endl << "Delays array empty or wrong. Quitting.";
		return 2;
	}
	if (shutters_float.size() < 1) {
		std::cout << endl << "Shutters/Exposures array empty or wrong. Quitting.";
		return 2;
	}
	if (numtakes < 1) {
		std::cout << endl << "Numtakes (=" << numtakes << ") must be > 0. Quitting.";
		return 2;
	}

	// checking all parameter in bounds, otherwise modify them
	int return_value = 0;
	for (size_t i = 0; i < frequencies.size(); i++) {
		if (frequencies[i] < FREQUENCY_MIN) {
			frequencies[i] = FREQUENCY_MIN;
			return_value = 1;
		}
		else if (frequencies[i] > FREQUENCY_MAX) {
			frequencies[i] = FREQUENCY_MAX;
			return_value = 1;
		}
	}
	for (size_t i = 0; i < shutters_float.size(); i++) {
		if (shutters_float[i] < SHUTTER_MIN) {
			shutters_float[i] = SHUTTER_MIN;
			return_value = 1;
		}
		else if (shutters_float[i] > SHUTTER_MAX) {
			shutters_float[i] = SHUTTER_MAX;
			return_value = 1;
		}
	}

	return return_value;
}



// Author: Jaime Martin
// PMD_charArray_to_file
int PMD_charArray_to_file (int argc, char *argv[]) {

	// Variables
	std::vector<float> frequencies;
	std::vector<float> delays;
	std::vector<float> shutters_float;
	char dir_name[1024];
	char file_name[1024];
	char comport[128];
	int numtakes;
	
	// Parsing the input to the variables
	if (int parser_error = parser_main (argc, argv, frequencies, delays, shutters_float, dir_name, file_name, comport, numtakes) < 0)
		return parser_error;

	// PMD_params_to_file
	return PMD_params_to_file (frequencies, delays, shutters_float, dir_name, file_name, comport, numtakes);
	
}



// Author: Jaime Martin (modification of previous function)
// PMD_params_to_file
int PMD_params_to_file (std::vector<float> & frequencies, std::vector<float> & delays, std::vector<float> & shutters_float, char* dir_name, char* file_name, char* comport, int & numtakes) {
	
	// ------------------------------------------------------------------------------------------------------------------------------
	// DEBUGGING:
	// Properties - Configuration Properties - Debugging - Command Arguments:
	// "80 90 100" "0 1 2 3" "1920" f:\tmp\pmdtest2 PMD_test_meas COM6 1
	//
	// EXECUTING:
	// cd C:\Users\transient\Documents\transient-code\CaptureTool2\x64\Release
	// capturetool2.exe "80 90 100" "0 1 2 3" "1920" f:\tmp\pmdtest2 PMD_test_meas COM6 1
	// ------------------------------------------------------------------------------------------------------------------------------

	int error_checking_parameters = check_parameters_vector (frequencies, delays, shutters_float, comport, numtakes);
	if (error_checking_parameters == 1)
		std::cout << endl << "Some parameters out of bounds modified. Continuing...\n";
	else if (error_checking_parameters == 2)
		return error_checking_parameters;
		
	std::vector<pair<int, unsigned short*>> shutters;
	for (size_t i = 0; i < shutters_float.size(); i++)
		shutters.push_back(pair<int, unsigned short*>((int)shutters_float[i], new unsigned short [PMD_WIDTH*PMD_HEIGTH*10]));

	//char measurement_name[1024] = "default";


	// Init devices

	// Open PMD sensor
	PMDHandle hnd;
	int res;
	int w, h, numframes;

	res = pmdOpenSourcePlugin(&hnd, SOURCE_PLUGIN, SOURCE_PARAM);
	pmd_handle_error(hnd, res, "Could not open device");

	SerialPort port = control_init(comport_full_name);

	// Check if if already exists
	char command[1024];
	char full_file_name[1024];
	char full_file_name_take[1024];
	sprintf(full_file_name,"%s\\%s%s", dir_name, file_name, FILE_DATA_NAME_SUFFIX);
	sprintf(full_file_name_take,"%s\\%s_%03d%s", dir_name, file_name, 0, FILE_DATA_NAME_SUFFIX);
	sprintf(command,"%s", dir_name);
	if (file_exists(full_file_name) || file_exists(full_file_name_take)) {
		cout << "\nWarning: File \"" << full_file_name << "\" already exists and will be deleted. Sure? (y/n) ";
		string answer;
		cin >> answer;
		if (answer[0] != 'y' && answer[0] != 'Y') {
			cout << "Okay - quitting." << endl;
			pmdClose(hnd);
			return -2;
		}
		// these 2 lines would delete all the entire directory (and its content)
		//sprintf(command,"rd /S /Q %s", dir_name);
		//system(command);
	}
	sprintf(command,"md %s", dir_name);
	system(command);

	// Init OpenCV
	cvNamedWindow( WindowName, CV_WINDOW_AUTOSIZE );

	bool firstiter = true;

	for (int take = 0; take < numtakes;++take) {
		char fn[256];
		if (numtakes > 1)
			sprintf(fn,"%s\\%s_nt%03d%s", dir_name, file_name, take, FILE_DATA_NAME_SUFFIX);
		else
			sprintf(fn,"%s\\%s%s", dir_name, file_name, FILE_DATA_NAME_SUFFIX);
		cout << "raw file name: ["<<fn<<"]"<<endl;
		rawdumpfile = fopen(fn,"wb");

	// Capture loop: Loop through delays
	for (size_t di = 0; di < delays.size(); di += 1) {
		cout << "delay = " << delays[di] << " m" << endl;

		// frequencies
		for (size_t fi = 0; fi < frequencies.size(); fi += 1) {
			
			cout << "    freq = " << frequencies[fi] << " MHz" << endl << "        Exposure ";
			char fnprefix[256];
			sprintf_s<256> (fnprefix, FILENAME_FORMAT, take, frequencies[fi], delays[di]);
			// and shutter times
			double timed;
			double freqd;
			for (size_t ci = 0; ci < shutters.size(); ++ci) {
				timed = get_cpu_time_cycles();
				freqd = get_cpu_frequency();

				int shutter = shutters[ci].first;
				unsigned short* buffer = shutters[ci].second;

				cout << " " << shutter << flush;

				// ----- PMD CAPTURE ----------------------------------------------------------------------------------
				pmd_capture(hnd, port, shutter, frequencies[fi], delays[di], buffer, w, h, numframes);

				if (firstiter) {

					char dateStr [256];
					char timeStr [256];
					_strdate( dateStr);
					_strtime( timeStr );
					
					sprintf(command,"%s\\%s%s", dir_name, file_name, FILE_INFO_NAME_SUFFIX);
					FILE *fp = fopen(command, "w"); 

					fprintf(fp, "# Capture date: %s %s\r\n", dateStr, timeStr);
					
					fprintf(fp, "Bytes per measured value: 2\n");

					fprintf(fp, "imagedims: %d %d\r\n", w, h);

					fprintf(fp, "frequencies (MHz) [%d]:", frequencies.size());
					for (size_t i = 0; i < frequencies.size(); ++i) 
						fprintf(fp, " %.3f", frequencies[i]);
					fprintf(fp, "\r\n");

					fprintf(fp, "distances (m) [%d]:", delays.size());
					for (size_t i = 0; i < delays.size(); ++i) 
						fprintf(fp, " %.3f", delays[i]);
					fprintf(fp, "\r\n");

					fprintf(fp, "shutters (us) [%d]:", shutters.size());
					for (size_t i = 0; i < shutters.size(); ++i) 
						fprintf(fp, " %.d", shutters[i].first);
					fprintf(fp, "\r\n");

					fprintf(fp, "phases (degrees) [2]: 0 90\r\n");

					fprintf(fp, "number_of_takes: %d\r\n", numtakes); 

					fclose(fp);

					firstiter = false;
				}

				// On last iteration, process data
				if (ci == shutters.size() - 1) {
					char measpath[1024];
					sprintf(measpath, "%s", dir_name);
					process_data(w, h, shutters, measpath, fnprefix, take, rawdumpfile);
					cout << endl;
				}

				// ABSOLUTELY IMPORTANT for thermal stability: 
				// add delay to ensure a duty cycle below 4%
				
				timed = get_cpu_time_cycles() - timed;
				int ms_elapsed = (int) get_cpu_time_ms(timed, freqd);
				int extra_delay = 4L*DUTYCYCLE_INVERSE_OLD*shutter/1000 - ms_elapsed + 1;
				if (extra_delay > 0)
				Sleep( extra_delay );

			}
			//Sleep(2000);

			int totalfreqs = frequencies.size() * delays.size();
			int currentfreq = frequencies.size() * di + fi;
			cout << "Progress: " << (float)(100 * (currentfreq+1)/totalfreqs) << "% of pass " << take <<endl;
		}
	}
	fclose(rawdumpfile);
	}
	// The aftermath
	for (int i = 0; i < shutters.size(); ++i) {
		delete shutters[i].second;
	}
	pmdClose (hnd);
	cvDestroyWindow( WindowName );

	// Exit program
	//Sleep(2000);
	return 0;
}


// Author: Jaime Martin
// copy_array (...)
// return 0: no error, in bounds
// return 1: error, out of bounds
int copy_array (unsigned short int* dst, unsigned short int* src, int dst_pos, int dst_size, int src_size) {
	if (dst_pos + src_size > dst_size) {
		cout << "\nCopy array out of bounds" << endl;
		return 1;
	}
	for (int i = 0; i < src_size; i++)
		dst[dst_pos + i] = src[i];
	return 0;
}



// Author: Jaime Martin (modification of previous function)
// PMD_params_to_DataPMD
int PMD_params_to_DataPMD (DataPMD & DataPMD_cap, std::vector<float> & frequencies, std::vector<float> & delays, std::vector<float> & shutters_float, char* comport, int & numtakes, bool loop) {

	// Checking errors in parameters
	int error_checking_parameters = check_parameters_vector (frequencies, delays, shutters_float, comport, numtakes);
	if (error_checking_parameters == 1)
		std::cout << endl << "Some parameters out of bounds modified. Continuing...\n";
	else if (error_checking_parameters == 2)
		return error_checking_parameters;
	// Shutters vector of pairs
	std::vector<pair<int, unsigned short*>> shutters;
	for (size_t i = 0; i < shutters_float.size(); i++)
		shutters.push_back(pair<int, unsigned short*>((int)shutters_float[i], new unsigned short [PMD_WIDTH*PMD_HEIGTH*10]));
	std::vector<float> phases(2);	phases[0] = 0.0f;	phases[1] = 90.0f; 
	
	// Buffer of the PMD data
	int data_buffer_PMD_size = frequencies.size() * delays.size() * shutters_float.size() * PMD_WIDTH * PMD_HEIGTH * 2;
	int data_buffer_PMD_pos = 0;
	unsigned short int* data_buffer_PMD = new unsigned short int[data_buffer_PMD_size];

	// Init devices: Open PMD sensor
	PMDHandle hnd;
	int w, h, numframes;
	int res = pmdOpenSourcePlugin(&hnd, SOURCE_PLUGIN, SOURCE_PARAM);
	pmd_handle_error(hnd, res, "Could not open device");
	SerialPort port = control_init(comport_full_name);
	double timed, freqd;
	// Init OpenCV
	cvNamedWindow(WindowName, CV_WINDOW_AUTOSIZE);

	// --- CAPTURE LOOP --------------------------------------------------------------------------------------
	bool first_iter = true;
	while(loop || first_iter) {
		
		if (!PMD_LOOP_ENABLE && !first_iter)
			break;
		first_iter = false;
		data_buffer_PMD_pos = 0;

		// --- numtakes --------------------------------------------------
		// still here but useless, we should consider only numtakes=1, otherwise unexpected behaviour
		for (int take = 0; take < numtakes; take++) {

			// --- delays -------------------------------------------------- 
			for (size_t di = 0; di < delays.size(); di++) {
				//cout << "delay = " << delays[di] << " m" << endl;

				// --- frequencies -------------------------------------------------- 
				for (size_t fi = 0; fi < frequencies.size(); fi++) {
					//cout << "    freq = " << frequencies[fi] << " MHz" << endl << "        Exposure ";
					//char fnprefix[256];
					//sprintf_s<256> (fnprefix, FILENAME_FORMAT, take, frequencies[fi], delays[di]);
					// and shutter times

					// --- shutters -------------------------------------------------- 
					for (size_t ci = 0; ci < shutters.size(); ci++) {
						timed = get_cpu_time_cycles();
						freqd = get_cpu_frequency();
						int shutter = shutters[ci].first;
						unsigned short* buffer = shutters[ci].second;
						//cout << " " << shutter << flush;

						// PMD CAPTURE
						pmd_capture(hnd, port, shutter, frequencies[fi], delays[di], buffer, w, h, numframes);

						// On last iteration, process data
						if (ci == shutters.size() - 1) {
							// unsigned short* ushort_img[0][0] will be size w*h*shutters.size()*2	// 2=num_of_phases
							// will contain all the data captured for those shutters
							int ushort_img_buffer_size = w*h*shutters.size()*2;
							unsigned short* ushort_img[2];
							process_data_to_buffer(w, h, shutters, ushort_img, take);
							if (copy_array (data_buffer_PMD, ushort_img[0], data_buffer_PMD_pos, data_buffer_PMD_size, ushort_img_buffer_size)) {
								// if the copy is out of bounds, copy_array return 1, this closes all, this returns 1, and finishes
								for (int i = 0; i < shutters.size(); ++i)
									delete shutters[i].second;
								pmdClose (hnd);
								cvDestroyWindow(WindowName);
								return 1;	
							}
							data_buffer_PMD_pos += ushort_img_buffer_size;
							//cout << endl;
						}
						// ABSOLUTELY IMPORTANT for thermal stability: 
						// add delay to ensure a duty cycle below 4%
						timed = get_cpu_time_cycles() - timed;
						int ms_elapsed = (int) get_cpu_time_ms(timed, freqd);
						int extra_delay = 4L*DUTYCYCLE_INVERSE_OLD*shutter/1000 - ms_elapsed + 1;
						if (extra_delay > 0)
						Sleep(extra_delay);
					}
					//Sleep(2000);
					//int totalfreqs = frequencies.size() * delays.size();
					//int currentfreq = frequencies.size() * di + fi;
					//cout << "Progress: " << (float)(100 * (currentfreq+1)/totalfreqs) << "% of pass " << take <<endl;
				}
			}
		}
		// Save data_buffer_PMD to the DataPMD instance
		DataPMD_cap = DataPMD(data_buffer_PMD, data_buffer_PMD_size, frequencies, delays, shutters_float, phases, w, h, numtakes, DATA_REAL_TIME);
	}
	// --- END OF CAPTURE LOOP -------------------------------------------------------------------------------
	
	// closing, deleting, the aftermath
	for (int i = 0; i < shutters.size(); ++i)
		delete shutters[i].second;
	pmdClose (hnd);
	cvDestroyWindow(WindowName);

	// Exit program
	//Sleep(2000);
	return 0;
}



// Author: Jaime Martin (modification of previous function)
// PMD_params_to_Frame
int PMD_params_to_Frame (Frame & Frame_00_cap, Frame & Frame_90_cap, float frequency_, float distance_, float shutter_, char* comport, bool loop) {

	// Checking errors in parameters
	int error_checking_parameters = check_parameters (frequency_, shutter_, comport);
	if (error_checking_parameters == 1)
		std::cout << endl << "Some parameters out of bounds modified. Continuing...\n";
	else if (error_checking_parameters == 2)
		return error_checking_parameters;
	std::vector<float> phases(2);	phases[0] = 0.0f;	phases[1] = 90.0f; 
	// Shutters vector of pairs
	std::vector<pair<int, unsigned short*>> shutters;	// process_data_to_buffer(...) deal with vectors of pairs
	shutters.push_back(pair<int, unsigned short*>((int)shutter_, new unsigned short [PMD_WIDTH*PMD_HEIGTH*10]));
	
	// Buffer of the PMD data for the Frame
	int ushort_img_buffer_size = PMD_WIDTH*PMD_HEIGTH*2;
	unsigned short* ushort_img[2];

	// Init devices: Open PMD sensor
	PMDHandle hnd;
	int w, h, numframes;
	int res = pmdOpenSourcePlugin(&hnd, SOURCE_PLUGIN, SOURCE_PARAM);
	pmd_handle_error(hnd, res, "Could not open device");
	SerialPort port = control_init(comport_full_name);
	float ms_time_loop;
	int ms_extra_delay;
	// Init OpenCV
	if (CV_WHILE_CAPTURING)
		cvNamedWindow(WindowName, CV_WINDOW_AUTOSIZE);

	// Syncronization
	std::unique_lock<std::mutex> locker_frame_object;	// Create a defered locker (a locker not locked yet)
	locker_frame_object = std::unique_lock<std::mutex>(mutex_frame_object,std::defer_lock);

	// --- CAPTURE LOOP --------------------------------------------------------------------------------------
	bool first_iter = true;
	while(loop || first_iter) {
		
		const clock_t begin_time_loop = clock();	// begin_time_loop for DUTYCYCLE Sleep

		if (!PMD_LOOP_ENABLE && !first_iter)
			break;
		first_iter = false;

		int shutter = shutters[0].first;
		unsigned short* buffer = shutters[0].second;
		
		// PMD CAPTURE
				//const clock_t begin_time_pmd_capture = clock();
		pmd_capture(hnd, port, shutter, frequency_, distance_, buffer, w, h, numframes);
				//const clock_t end_time_pmd_capture = clock();
				//float ms_time_pmd_capture = 1000.0f * float(end_time_pmd_capture - begin_time_pmd_capture) / (float)CLOCKS_PER_SEC;
				//std::cout << "pmd_capture    : time = " << ms_time_pmd_capture << " ms\n";
		
		// unsigned short* ushort_img[0][0] will be size w*h*shutters.size()*2	// 2 = num_of_phases
		// will contain all the data captured for those shutters
				//const clock_t begin_time_process_data_to_buffer = clock();
		if (CV_WHILE_CAPTURING)
			process_data_to_buffer(w, h, shutters, ushort_img, 0);
		else
			process_data_to_buffer_no_cv(w, h, shutters, ushort_img, 0);
				//const clock_t end_time_process_data_to_buffer = clock();
				//float ms_time_process_data_to_buffer = 1000.0f * float(end_time_process_data_to_buffer - begin_time_process_data_to_buffer) / (float)CLOCKS_PER_SEC;
				//std::cout << "data_to_buffer : time = " << ms_time_process_data_to_buffer << " ms\n";
			
		// Save buffer to Frames. The frames construction takes: < 1 ms. Deals with Syncronization.
				//const clock_t begin_time_buffer_to_frame = clock();
		locker_frame_object.lock();		// Lock mutex_frame_object, any thread which used mutex_frame_object can NOT continue untill unlock()
		while (!UPDATED_NEW_OBJECT) {
			std::cout << "\n\nWaiting in Frame to finish the UPDATED_NEW_OBJECT. This should never happen!\n\n";
			cv_frame_object.wait(locker_frame_object);
		}
		if (&Frame_00_cap != NULL)
			Frame_00_cap = Frame(ushort_img[0], h, w, distance_, frequency_, shutter_, phases[0], 0, DATA_REAL_TIME, PIXELS_STORING_GLOBAL);
		if (&Frame_90_cap != NULL)
			Frame_90_cap = Frame(ushort_img[0], h, w, distance_, frequency_, shutter_, phases[1], 1, DATA_REAL_TIME, PIXELS_STORING_GLOBAL);
		//std::cout << "UPDATED_NEW_FRAME";
		UPDATED_NEW_FRAME = true;
		UPDATED_NEW_OBJECT = false;
		cv_frame_object.notify_all();	// Notify all cv_frame_object. All threads waiting for cv_frame_object will break the wait after waking up
		locker_frame_object.unlock();	// Unlock mutex_frame_object, now threads which used mutex_frame_object can continue
				//const clock_t end_time_buffer_to_frame = clock();
				//float ms_time_buffer_to_frame = 1000.0f * float(end_time_buffer_to_frame - begin_time_buffer_to_frame) / (float)CLOCKS_PER_SEC;
				//std::cout << "buffer_to_frame: time = " << ms_time_buffer_to_frame << " ms\n";

		const clock_t end_time_loop = clock();		// end_time_loop for DUTYCYCLE Sleep
		ms_time_loop = 1000.0f * float(end_time_loop - begin_time_loop) / (float)CLOCKS_PER_SEC;
		ms_extra_delay = ((float)shutter)/(DUTYCYCLE*1000.0f) - ms_time_loop + 1.0f;
		if (ms_extra_delay > 0)
			Sleep(ms_extra_delay);	// suspends the execution of the current thread until the time-out interval elapses
				//std::cout << "ms_time_loop   : " << ms_time_loop << " ms\n";
				//std::cout << "ms_extra_delay : " << ms_extra_delay << " ms\n";

				//const clock_t end_time_loop_total = clock();
				//float ms_time_loop_total = 1000.0f * float(end_time_loop_total - begin_time_loop) / (float)CLOCKS_PER_SEC;
				//float fps_time_loop_total = 1000.0f / ms_time_loop_total;
				//std::cout << "total          : time = " << ms_time_loop_total << " ms,    fps = " << fps_time_loop_total <<  " fps\n\n";
	}
	// --- END OF CAPTURE LOOP -------------------------------------------------------------------------------
	
	// closing, deleting, the aftermath
	delete shutters[0].second;
	pmdClose (hnd);
	cvDestroyWindow(WindowName);

	// Exit program
	//Sleep(2000);
	return 0;
}




// Author: Jaime Martin 
// MAIN
int capturetoolDM2_main(int argc, char *argv[], bool loop) {	// by default: bool = false

	// Capture directly from PMD to Frame (Frame FRAME_00_CAPTURE, Frame FRAME_90_CAPTURE)
	float frequency = 100.0f;
	float distance = 0.0f;
	float shutter = 1920.0f;
	char comport[128] = "COM6";
	Frame * frame_00_null = NULL;	// (*frame_00_null) in PMD_params_to_Frame(...), if we want to avoid this Frame measurement
	Frame * frame_90_null = NULL;	// (*frame_90_null) in PMD_params_to_Frame(...), if we want to avoid this Frame measurement
	// FRAME_00_CAPTURE, FRAME_90_CAPTURE
	return PMD_params_to_Frame (FRAME_00_CAPTURE, FRAME_90_CAPTURE, frequency, distance, shutter, comport, loop);
}


