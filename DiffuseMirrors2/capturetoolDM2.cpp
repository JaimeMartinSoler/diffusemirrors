
#include "capturetoolDM2.h"
#include "data_read.h"
#include "global.h"
#include "shapes.h"
#include "scene.h"
#include <stdio.h>
#include <stdlib.h>     // atof
#include <pmdsdk2.h>
#include <Windows.h>	// For timing calculations
#include <algorithm>
#include <math.h>		// std::round
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

void process_data(int w, int h, vector<pair<int, unsigned short* > > &shutters, char* path, char* file_prefix, int pass, FILE *rawdumpfile) {

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
void process_data_no_cv(int w, int h, vector<pair<int, unsigned short* > > &shutters, FILE *rawdumpfile) {
	
	int capturecount = shutters.size();
	unsigned short* ushort_img[2];
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
					diff += 32768;	// Bias and clamp to 16-bit range
					ushort_img[0][x + y*w + i*w*h + capturecount*ph*w*h] = (diff > 65535) ? 65535 : ((diff < 0) ? 0  : diff);
	}	}	}	}

	// store the data in the raw_file
	dump_data(rawdumpfile, &(ushort_img[0][0]), w, capturecount*h*2, 65535, 1);
	delete ushort_img[0];
	delete ushort_img[1];
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
void process_data_to_buffer_no_cv(int w, int h, std::vector<std::pair<int, unsigned short* > > &shutters, unsigned short* ushort_img[2]) {

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
					diff += 32768;	// Bias and clamp to 16-bit range
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
// check_input_data(...)
// return    0, 1:     !continue / continue
int check_input_data(float frequency_, float distance_, float shutter_, char* comport, bool loop, bool show_text, bool ask_to_continue_) { // by default: show_text = true, ask_to_continue_ = true
	
	if (show_text) {

		cout << "\nThe input parameters are:";
		cout << "\n  Frequency (MHz): " << frequency_;
		cout << "\n  Disatance (m)  : " << distance_;
		cout << "\n  Shutter   (us) : " << shutter_;
		cout << "\n  comport        : " << comport;
		cout << "\n  loop           : " << loop << "\n";
	}
	
	if (ask_to_continue_) {
		if(ask_to_continue() == 0)
			return 0;	// !continue
	}

	return 1; 			// continue
}


// Author: Jaime Martin
// check_input_data_vectors(...)
// return    0, 1:     !continue / continue
int check_input_data_vectors (std::vector<float> & frequencies, std::vector<float> & delays, std::vector<float> & shutters_float, char* dir_name, char* file_name, char* comport, int & numtakes, bool show_text, bool ask_to_continue_) { // by default: show_text = true, ask_to_continue_ = true

	if (show_text) {

		cout << "\nThe input parameters are:";
		// Frequencies
		cout << "\n\n  Frequencies (MHz) (" << frequencies.size() << "):\n    ";
		for (size_t fi = 0; fi < frequencies.size()-1; fi++) {
			cout << frequencies[fi] << ", ";
		}	cout << frequencies[frequencies.size()-1];
		// Delays
		cout << "\n\n  Delays (m) (" << delays.size() << "):\n   ";
		for (size_t di = 0; di < delays.size()-1; di++) {
			cout << delays[di] << ", ";
		}	cout << delays[delays.size()-1];
		// Shutters
		cout << "\n\n  Shutters (us) (" << shutters_float.size() << "):\n   ";
		for (size_t si = 0; si < shutters_float.size()-1; si++) {
			cout << shutters_float[si] << ", ";
		}	cout << shutters_float[shutters_float.size()-1];
		// numtakes
		cout << "\n\n  numtakes  : " << numtakes;
		// comport
		cout << "\n  comport   : " << comport;
		// dir_name
		cout << "\n  dir_name  : " << dir_name;
		// file_name
		cout << "\n  file_name : " << file_name << "\n";
	}
	
	if (ask_to_continue_) {
		if(ask_to_continue() == 0)
			return 0;	// !continue
	}

	return 1; 			// continue

}



// Author: Jaime Martin
// check_parameters(...)
// return 0: any parameter out of bounds, modified, !continue
// return 1: any parameter out of bounds, modified, continue
// return 2: all parameters OK                    , continue
int check_parameters (float & frequency_, float & shutter_, char* comport, bool show_text, bool ask_to_continue_) { // by default: show_text = true, ask_to_continue_ = true
	
	sprintf(comport_full_name, COMPORT_FORMAT, comport);	// configure comport_full_name

	// checking all parameter in bounds, otherwise modify them
	int return_value = 2;	// all parameters OK, continue
	if (frequency_ < FREQUENCY_MIN) {
			frequency_ = FREQUENCY_MIN;
			return_value = 1;	// any parameter out of bounds, modified, continue
	}
	else if (frequency_ > FREQUENCY_MAX) {
			frequency_ = FREQUENCY_MAX;
			return_value = 1;	// any parameter out of bounds, modified, continue
	}
	if (shutter_ < SHUTTER_MIN) {
			shutter_ = SHUTTER_MIN;
			return_value = 1;	// any parameter out of bounds, modified, continue
	}
	else if (shutter_ > SHUTTER_MAX) {
			shutter_ = SHUTTER_MAX;
			return_value = 1;	// any parameter out of bounds, modified, continue
	}
	
	// show text, ask to continue
	if ((show_text) && (return_value == 1)) {
		std::cout << endl << "\nWarning: Some parameters out of bounds have been modified.";
		if (ask_to_continue_) {
			if(ask_to_continue() == 0)
				return 0;	// any parameter out of bounds, modified, !continue
	}	}

	return return_value;
}



// Author: Jaime Martin
// check_parameters_vector(...)
// return 0: any parameter out of bounds, modified, !continue
// return 1: any parameter out of bounds, modified, continue
// return 2: all parameters OK                    , continue
// return 3: unproper dimensions or size          , !continue
int check_parameters_vector (std::vector<float> & frequencies, std::vector<float> & delays, std::vector<float> & shutters_float, char* comport, int & numtakes, bool show_text, bool ask_to_continue_) { // by default: show_text = true, ask_to_continue_ = true
	
	sprintf(comport_full_name, COMPORT_FORMAT, comport);	// configure comport_full_name

	// ckecking unproper dimensions or size
	if (frequencies.size() < 1) {
		std::cout << endl << "\nFrequencies array empty or wrong. Quitting.";
		return 3;	// unproper dimensions or size
	}
	if (delays.size() < 1) {
		std::cout << endl << "\nDelays array empty or wrong. Quitting.";
		return 3;	// unproper dimensions or size
	}
	if (shutters_float.size() < 1) {
		std::cout << endl << "\nShutters/Exposures array empty or wrong. Quitting.";
		return 3;	// unproper dimensions or size
	}
	if (numtakes < 1) {
		std::cout << endl << "\nNumtakes (=" << numtakes << ") must be > 0. Quitting.";
		return 3;	// unproper dimensions or size
	}

	// checking all parameter in bounds, otherwise modify them
	int return_value = 2;	// all parameters OK, continue
	for (size_t i = 0; i < frequencies.size(); i++) {
		if (frequencies[i] < FREQUENCY_MIN) {
			frequencies[i] = FREQUENCY_MIN;
			return_value = 1;	// any parameter out of bounds, modified, continue
		}
		else if (frequencies[i] > FREQUENCY_MAX) {
			frequencies[i] = FREQUENCY_MAX;
			return_value = 1;	// any parameter out of bounds, modified, continue
		}
	}
	for (size_t i = 0; i < shutters_float.size(); i++) {
		if (shutters_float[i] < SHUTTER_MIN) {
			shutters_float[i] = SHUTTER_MIN;
			return_value = 1;	// any parameter out of bounds, modified, continue
		}
		else if (shutters_float[i] > SHUTTER_MAX) {
			shutters_float[i] = SHUTTER_MAX;
			return_value = 1;	// any parameter out of bounds, modified, continue
		}
	}

	// show text, ask to continue
	if ((show_text) && (return_value == 1)) {
		std::cout << endl << "\nWarning: Some parameters out of bounds have been modified.";
		if (ask_to_continue_) {
			if(ask_to_continue() == 0)
				return 0;	// any parameter out of bounds, modified, !continue
	}	}

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


// Author: Jaime Martin
// ask_to_continue()
// return    0, 1:     !continue / continue
int ask_to_continue() {

	std::cout << "\nAre you sure you want to continue? (y/n)\n";
	std::string answer;
	std::cin >> answer;
	if ((answer[0] != 'y') && (answer[0] != 'Y')) {
		std::cout << "\nOkay. Quitting.\n" << endl;
		return 0;	// !continue
	}
	return 1;		// continue
}


// Author: Jaime Martin
// check_overwrite(...)
// return    0, 1, 2:    file_exists(...)+!continue, file_exists(...)+continue, !file_exists(...)+continue
int check_overwrite(char* file_name, bool show_text, bool ask_to_continue_) { // by default: show_text = true, ask_to_continue_ = true

	if (file_exists(file_name)) {	
		if (show_text)
			std::cout << "\nWarning: File \"" << file_name << "\" already exists and will be deleted.";
		if (ask_to_continue_) {
			if(ask_to_continue() == 0)
				return 0;	// file_exists(...)+!continue
		}
		// these 2 lines would delete all the entire directory (and its content):
		//sprintf(command,"rd /S /Q %s", dir_name);
		//system(command);
		return 1;	// file_exists(...)+continue
	} else {
		return 2;	// !file_exists(...)+continue
	}
}


// Author: Jaime Martin
// check_frame(...)
// return    0, 1:     !continue / continue
int check_frame(float freq, float dist, float shutter, char* comport, bool show_text, bool ask_to_continue_) { // by default: show_text = true, ask_to_continue_ = true

	if (show_text)
		std::cout << "\nA frame is being shown to let you check the scene before the measurement.";
	Frame frame_00;
	Frame frame_90;
	PMD_params_to_Frame (frame_00, frame_90, freq, dist, shutter, comport, false);
	plot_frame (frame_00, frame_90);
	if (ask_to_continue_) {
		if(ask_to_continue() == 0)
			return 0;	// !continue
	}
	return 1;			// continue
}


// Author: Jaime Martin
// check_file_size(...)
// return    0, 1:     !continue / continue
// if (numtakes > 1) : the text shows file_size_MB_take, file_size_MB_tot, numtakes
// if (numtakes == 1): the text shows file_size_MB_take, numtakes
// if (numtakes <= 0): the text shows file_size_MB_take
int check_file_size(float file_size_B_take, int numtakes, bool show_text, bool ask_to_continue_) { // by default: show_text = true, ask_to_continue_ = true
	
	float file_size_MB_take = file_size_B_take / 1048576.0f;	// Bytes
	float file_size_MB_tot  = file_size_MB_take * numtakes;		// Bytes
	if (show_text) {
		if (numtakes > 1)
			cout << "\nThe size required will be: " << (int)file_size_MB_take << " MB for each take, " << (int)file_size_MB_tot << " MB in total (" << numtakes << " takes)";
		else if (numtakes == 1)
			cout << "\nThe size required will be: " << (int)file_size_MB_take << " MB for each take, " << (int)file_size_MB_tot << " MB in total (1 take)";
		else
			cout << "\nThe size required will be: " << (int)file_size_MB_take << " MB";
	}
	if (ask_to_continue_) {
		if(ask_to_continue() == 0)
			return 0;	// !continue
	}
	return 1;			// continue
}


// Author: Jaime Martin
// check_time(...)
// return    0, 1:     !continue / continue
int check_time(float time_tot_s, bool show_text, bool ask_to_continue_) { // by default: show_text = true, ask_to_continue_ = true
	
	float time_h, time_m, time_s;
	seconds_to_hms (time_tot_s, time_h, time_m, time_s);

	if (show_text)
		cout << "\nThe measurement will take   : " << time_h << "h " << time_m << "' " << time_s << "''.";
	if (ask_to_continue_) {
		if(ask_to_continue() == 0)
			return 0;	// !continue
	}
	return 1;			// continue
}

// Converts time in seconds to time in hours, minutes and seconds
void seconds_to_hms (float time_tot_s, float & time_h, float & time_m, float & time_s) {
	time_h = std::floor(time_tot_s / 3600.0f);
	time_m = std::floor(((time_tot_s / 3600.0f) - time_h) * 60.0f);
	time_s = std::floor(((((time_tot_s / 3600.0f) - time_h) * 60.0f) - time_m) * 60.0f);
}

// Author: Jaime Martin
// countdown(...)
void countdown(bool ask_number, int default_time, bool show_text) { // by default: ask_number = false, default_time = 10, bool show_text = true
	 
	if (default_time <= 0) {
		if (show_text)
			cout << "\nThe program will start right now.";
		return;
	}
	
	if (show_text)
		cout << "\nThe program will start after the countdown.";
	int countdown_time = default_time;

	if (ask_number) {
		std::string number = "";
		while (!str_is_number(number)) {
			std::cout << "\nEnter a valid number for the countdown (seconds), and press ENTER to start it:\n  ";
			std::cin >> number;
		}
		countdown_time = (int)(atof(number.c_str()));
	}

	for (int i = countdown_time-1; i >= 0; i--) {
		Sleep(1000);
		if (show_text)
			cout << "\n  " << i;
	}
}

// Author: Jaime Martin (modification of previous function)
// PMD_params_to_file
int PMD_params_to_file (std::vector<float> & frequencies, std::vector<float> & delays, std::vector<float> & shutters_float, char* dir_name, char* file_name, char* comport, int & numtakes, bool cmx_info, float* cmx_params) {	// by default: (bool cmx_info = false, float* cmx_params = NULL)
	
	// Checking the input data
	if (check_input_data_vectors (frequencies,delays, shutters_float, dir_name, file_name, comport, numtakes, true, false) == 0)
		return -3;

	// Checking errors in parameters
	int check_parameters_vector_error = check_parameters_vector (frequencies, delays, shutters_float, comport, numtakes, true, true);
	if ((check_parameters_vector_error == 0) || (check_parameters_vector_error == 3))
			return -3;
	// Shutters vector of pairs
	std::vector<pair<int, unsigned short*>> shutters;
	for (size_t i = 0; i < shutters_float.size(); i++)
		shutters.push_back(pair<int, unsigned short*>((int)shutters_float[i], new unsigned short [PMD_WIDTH*PMD_HEIGTH*10]));

	// Check if file already exists and if overwrite
	char fn[256];
	char command[1024];
	char full_file_name[1024];
	char full_file_name_take[1024];
	sprintf(full_file_name,"%s\\%s%s", dir_name, file_name, RAW_FILENAME_SUFFIX);
	sprintf(full_file_name_take,"%s\\%s_%03d%s", dir_name, file_name, 0, RAW_FILENAME_SUFFIX);
	sprintf(command,"%s", dir_name);
	// Check overwrite
	if (check_overwrite(full_file_name, true, true) == 0)
		return -2;	// if did not want to continue
	sprintf(command,"md %s", dir_name);
	system(command);
	
	// Check frames. In order to visualize the setup previously
	//if (check_frame(frequencies[0], delays[0], shutters_float[shutters_float.size()-1], comport, true, false) == 0)
	//	return -2;	// if did not want to continue

	// Check file size
	float file_size_B_take = frequencies.size() * delays.size() * shutters.size() * 2 * PMD_WIDTH * PMD_HEIGTH * sizeof(unsigned short);	// Bytes
	if (check_file_size(file_size_B_take, numtakes, true, false) == 0)
		return -2;	// if did not want to continue

	// Check time
	float time_tot_s, time_h, time_m, time_s;
	float period_shutter_tot = 0.0f;
	for (size_t si = 0; si < shutters.size(); si++)
		period_shutter_tot +=  shutters_float[si] / (1000000.0f * DUTYCYCLE);
	time_tot_s = period_shutter_tot * frequencies.size() * delays.size() * numtakes;
	if (check_time(time_tot_s, true, false) == 0)
		return -2;	// if did not want to continue
	
	// Start countdown
	countdown (false, 0, false);

	// close cv Frame window
	cv::destroyAllWindows();

	// Init devices: Open PMD sensor
	PMDHandle hnd;
	int res;
	int w, h, numframes;
	res = pmdOpenSourcePlugin(&hnd, SOURCE_PLUGIN, SOURCE_PARAM);
	pmd_handle_error(hnd, res, "Could not open device");
	SerialPort port = control_init(comport_full_name);

	// Init OpenCV
	char measpath[1024];	// For temporal files for CV_WHILE_CAPTURING
	char fnprefix[256];		// For temporal files for CV_WHILE_CAPTURING
	if (CV_WHILE_CAPTURING)
		cvNamedWindow(WindowName, CV_WINDOW_AUTOSIZE);
	
	// timing: loop
	float ms_time_loop;
	int ms_extra_delay;
	clock_t begin_time_loop, end_time_loop;
	// timing: timer
	bool set_timer = true;
	float ms_time_timer;
	float ms_time_timer_max = 5000.0f;
	clock_t begin_time_timer, end_time_timer;
	float time_tot_s_rem, time_h_rem, time_m_rem, time_s_rem;
	seconds_to_hms (time_tot_s, time_h, time_m, time_s);
	begin_time_timer = clock();	// begin_time_timer for the timer

	// Loop through takes
	bool firstiter = true;
	for (int take = 0; take < numtakes; take++) {
		if (numtakes > 1)
			sprintf(fn,"%s\\%s%s%03d%s", dir_name, file_name, NUMTAKE_FILENAME_APPEND, take, RAW_FILENAME_SUFFIX);
		else
			sprintf(fn,"%s\\%s%s", dir_name, file_name, RAW_FILENAME_SUFFIX);
		cout << "\nraw file name: ["<<fn<<"]\n\n";
		rawdumpfile = fopen(fn,"wb");

		// Loop through delays
		for (size_t di = 0; di < delays.size(); di++) {
			//cout << "delay = " << delays[di] << " m" << endl;

			// Loop through frequencies
			for (size_t fi = 0; fi < frequencies.size(); fi++) {
				//cout << "    freq = " << frequencies[fi] << " MHz" << endl << "        Exposure ";
				if (CV_WHILE_CAPTURING)
					sprintf_s<256> (fnprefix, FILENAME_FORMAT, take, frequencies[fi], delays[di]);
				
				if (set_timer) {
					end_time_timer = clock();	// end_time_timer for the timer
					ms_time_timer = 1000.0f * float(end_time_timer - begin_time_timer) / (float)CLOCKS_PER_SEC;
					if ((ms_time_timer >= ms_time_timer_max) || (firstiter)) {
						begin_time_timer = clock();	// begin_time_timer for the timer
						time_tot_s_rem = time_tot_s - period_shutter_tot * (take * delays.size() * frequencies.size() + di * frequencies.size() + fi);
						seconds_to_hms (time_tot_s_rem, time_h_rem, time_m_rem, time_s_rem);
						cout << "\nRemaning time: " << time_h_rem << "h " << time_m_rem << "' " << time_s_rem << "'', of a total of : " << time_h << "h " << time_m << "' " << time_s << "''.";
					}
				}

				// Loop through shutters
				for (size_t ci = 0; ci < shutters.size(); ++ci) {
					
					begin_time_loop = clock();	// begin_time_loop for DUTYCYCLE Sleep

					int shutter = shutters[ci].first;
					unsigned short* buffer = shutters[ci].second;

					//cout << " " << shutter << flush;

					// ----- PMD CAPTURE ----------------------------------------------------------------------------------
					pmd_capture(hnd, port, shutter, frequencies[fi], delays[di], buffer, w, h, numframes);

					if (firstiter) {

						char dateStr [256];
						char timeStr [256];
						_strdate( dateStr);
						_strtime( timeStr );
					
						sprintf(command,"%s\\%s%s", dir_name, file_name, INF_FILENAME_SUFFIX);
						FILE *fp = fopen(command, "w"); 
						// line 0
						fprintf(fp, "# Capture date: %s %s\r\n", dateStr, timeStr);
						// line 1
						fprintf(fp, "\r\n");
						// line 2
						fprintf(fp, "# Raw Data measurements:\r\n");
						// line	3
						fprintf(fp, "Bytes per raw value: %d\r\n", sizeof(unsigned short));
						// line	4
						fprintf(fp, "imagedims: %d %d\r\n", w, h);
						// line	5
						fprintf(fp, "frequencies (MHz) [%d]:", frequencies.size());
						for (size_t i = 0; i < frequencies.size(); ++i) 
							fprintf(fp, " %.3f", frequencies[i]);
						fprintf(fp, "\r\n");
						// line	6
						fprintf(fp, "distances (m) [%d]:", delays.size());
						for (size_t i = 0; i < delays.size(); ++i) 
							fprintf(fp, " %.3f", delays[i]);
						fprintf(fp, "\r\n");
						// line	7
						fprintf(fp, "shutters (us) [%d]:", shutters.size());
						for (size_t i = 0; i < shutters.size(); ++i) 
							fprintf(fp, " %.d", shutters[i].first);
						fprintf(fp, "\r\n");
						// line	8
						fprintf(fp, "phases (degrees) [2]: 0 90\r\n");
						// line	9
						fprintf(fp, "number_of_takes: %d\r\n", numtakes); 
						
						if (cmx_info == true) {
							// line 10
							fprintf(fp, "\r\n");
							// line 11
							fprintf(fp, "# Calibration Matrix Data [camera_pos=(0,0,0), camera_n=(0,0,-1)]:\r\n");
							// line	12
							fprintf(fp, "Bytes per calibration matrix value: %d\r\n", sizeof(float));
							// line	13
							float lcx = cmx_params[0];
							float lcy = cmx_params[1];
							float lcz = cmx_params[2];
							fprintf(fp, "Laser position relative to camera (x,y,z): %.3f %.3f %.3f\r\n", lcx, lcy, lcz);
							// line	14
							float wcd = cmx_params[3];
							fprintf(fp, "Wall distance to camera: %.3f\r\n", wcd);
						}

						fclose(fp);

						firstiter = false;
					}

					// On last iteration, process data
					if (ci == shutters.size() - 1) {
						if (CV_WHILE_CAPTURING) {
							sprintf(measpath, "%s", dir_name);
							process_data(w, h, shutters, measpath, fnprefix, take, rawdumpfile);
						} else
							process_data_no_cv(w, h, shutters, rawdumpfile);
						//cout << endl;
					}

					end_time_loop = clock();		// end_time_loop for DUTYCYCLE Sleep
					ms_time_loop = 1000.0f * float(end_time_loop - begin_time_loop) / (float)CLOCKS_PER_SEC;
					ms_extra_delay = ((float)shutter)/(DUTYCYCLE*1000.0f) - ms_time_loop + 1.0f;
					if (ms_extra_delay > 0)
						Sleep(ms_extra_delay);	// suspends the execution of the current thread until the time-out interval elapses
					//std::cout << "ms_time_loop   : " << ms_time_loop << " ms\n";
					//std::cout << "ms_extra_delay : " << ms_extra_delay << " ms\n\n";
				}
			}
		}
		fclose(rawdumpfile);
	}
	// The aftermath
	for (int i = 0; i < shutters.size(); ++i) {
		delete shutters[i].second;
	}
	pmdClose (hnd);
	if (CV_WHILE_CAPTURING)
		cvDestroyWindow(WindowName);

	// Build the averaged raw file if many takes were made
	if (numtakes > 1) {
		Info info (dir_name, file_name);
		create_raw_from_raw_takes(&info);
	}
	else
		cout << "\nRaw Data done in [" << full_file_name << "]\n";

	return 0;
}
// there's a weird bug when calling directly to PMD_params_to_file from thread constructor. With this re-calling functtion the bug is avoided
int PMD_params_to_file_anti_bug_thread (std::vector<float> & frequencies, std::vector<float> & delays, std::vector<float> & shutters_float, char* dir_name, char* file_name, char* comport, int & numtakes, bool cmx_info, float* cmx_params) {	// by default: (bool cmx_info = false, float* cmx_params = NULL)
	return PMD_params_to_file (frequencies, delays, shutters_float, dir_name, file_name, comport, numtakes, cmx_info, cmx_params);
}

// creates a the corresponding averaged raw file from the raw takes files
void create_raw_from_raw_takes (Info* info) {

	int elements = info->frequencies.size() * info->distances.size() * info->shutters.size() * info->phases.size() * info->width * info->heigth;
	float* raw_float_store = new float[elements];	// this will temporally store the sum of the corresponding values. We need float as long as with short int may cause overload
	for (int pos = 0; pos < elements; pos++)
		raw_float_store[pos] = 0.0f;

	// store all the data
	for (int take = 0; take < info->numtakes; take++) {
		RawData raw_data(info, take);	// we load .raw files them one by one for memory efficiency reasons
		if (elements != raw_data.data_size) {
			std::cout << "\nError in: create_raw_from_raw_takes(Info* info), elements = " << elements <<" != raw_data.data_size = " << raw_data.data_size;
			return;
		}
		for (int pos = 0; pos < elements; pos++) {
			if (pos == elements/2)
				cout << "\nFor take " << take << " raw_data.data[" << pos << "] = " << raw_data.data[pos];
			raw_float_store[pos] += (float)raw_data.data[pos];
	}	}

	// average all the data
	float numtakes_float = (float)info->numtakes;
	for (int pos = 0; pos < elements; pos++) {
		raw_float_store[pos] /= numtakes_float;
	}
	cout << "\nAvg value float = raw_float_store.data[" << elements/2 << "] = " << raw_float_store[elements/2];

	// store the data in the raw_store of unsigned short int
	unsigned short int* raw_store_ushort = new unsigned short int[elements];
	for (int pos = 0; pos < elements; pos++) {
		raw_store_ushort[pos] = (unsigned short int)(floor(raw_float_store[pos]+0.5f));	// floor(x+0.5) = round(x)
	}
	cout << "\nAvg value ushort int = raw_store_ushort[" << elements/2 << "] = " << raw_store_ushort[elements/2];

	// fwrite parameters
	size_t raw_bytes_per_value = sizeof(unsigned short int);
	size_t raw_elements_per_write = elements;
	FILE* raw_file = fopen(info->raw_full_file_name,"wb");
	fwrite(raw_store_ushort, raw_bytes_per_value, raw_elements_per_write, raw_file);
	fclose (raw_file);
	
	delete [] raw_float_store;
	delete [] raw_store_ushort;
	cout << "\nRaw Data done in [" << info->raw_full_file_name << "]\n";
}

// Author: Jaime Martin
// create_cmx_from_raw (...)
void create_cmx_from_raw(Info* info_) {
	
	// Check overwrite
	if(check_overwrite(info_->cmx_full_file_name, true, true) == 0)
		return;	// if did not want to overwrite
	// Check file size
	float file_size_B = info_->frequencies.size() * info_->distances.size() * info_->width * info_->heigth * info_->sizeof_value_cmx;	// Bytes
	if (check_file_size(file_size_B, 0, true, false) == 0)
		return;	// if did not want to continue

	RawData raw_data(info_);	// this will load all the data into the memory
	
	// IMPORTANT:    cmx_data_value = c * Em * albedo = H * dist_src_pix_rc,    dist_src_pix_rc = |r_src - r_x0(r,c)|^2,
	float cmx_data_value;
	float raw_data_value;
	int phase_idx = 0;										// always
	int shutter_idx = raw_data.info->shutters.size() - 1;	//always, the calibration matrix is 1-shutter oriented
	// dist_src_pix_rc vector
	std::vector<float> dist_src_pix_pow2_rc, v_aux;
	set_scene_calibration_matrix (info_, PIXELS_TOTAL);	// set the corresponding scene (camera, laser, wall and wall_patches)
	dist_2_pow2_centers( (*(*OBJECT3D_SET[LASER])[0]).c, (*OBJECT3D_SET[WALL_PATCHES]), v_aux);
	clear_scene();										// clear scene
	dist_src_pix_pow2_rc.resize(v_aux.size());	// v_aux is ordered as WALL_PATCHES and it is, by rows, from down to top, we want it from top to down
	for (size_t r = 0; r < raw_data.info->heigth; r++) {		// should be: raw_data.info->heigth = CAMERA_PIX_Y
		for (size_t c = 0; c < raw_data.info->width; c++) {		// should be: raw_data.info->width  = CAMERA_PIX_X
			dist_src_pix_pow2_rc[r*raw_data.info->width + c] = v_aux[(raw_data.info->heigth-1-r)*raw_data.info->width + c];
	}	}
	// fwrite parameters
	float* cmx_data_value_ptr = &cmx_data_value;
	size_t cmx_bytes_per_value = sizeof(float);
	size_t cmx_elements_per_write = 1;
	FILE* cmx_file = fopen(raw_data.info->cmx_full_file_name,"wb");
	
	// in data_read.h (about the .cmx ordering):
	// data ordereing: for(freq) { for(dist) { for(heigth){ for(width){ // here... }}}}
	for (size_t fi = 0; fi < raw_data.info->frequencies.size(); fi++) {
		for (size_t di = 0; di < raw_data.info->distances.size(); di++) {
			//for (phase_idx = 0, always) 
			//for (shutter_idx = shutters.size() - 1, always, the calibration matrix is 1-shutter oriented) 
			for (size_t r = 0; r < raw_data.info->heigth; r++) {
				for (size_t c = 0; c < raw_data.info->width; c++) {

					// Calculate the corresponding value of the Calibration Matrix
					// data is not stored properly. -32768 fixes it thanks to short int over-run
					// by default image is up-down-fliped so heigth_RawData = (heigth_Frame-1-h)
					raw_data_value = (float)(raw_data.at(di, fi, shutter_idx, c, raw_data.info->heigth - 1 - r, phase_idx) - 32768);
					
					//cmx_data_value = c * Em * albedo = H * dist_src_pix_rc,    dist_src_pix_rc = |r_src - r_x0(r,c)|^2,
					cmx_data_value = raw_data_value * dist_src_pix_pow2_rc[r*raw_data.info->width + c];

					// Store cmx_data_value in the Calibration Matrix
					fwrite(cmx_data_value_ptr, cmx_bytes_per_value, cmx_elements_per_write, cmx_file);
	}	}	}	}
	fclose (cmx_file);
	cout << "\nCalibration Matrix done in [" << info_->cmx_full_file_name << "]\n";
}
// there's a weird bug when calling directly to PMD_params_to_file from thread constructor. With this re-calling functtion the bug is avoided
void create_cmx_from_raw_anti_bug_thread (Info* info_) {
	create_cmx_from_raw(info_);
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
// PMD_params_to_RawData
int PMD_params_to_RawData (RawData & RawData_cap, std::vector<float> & frequencies, std::vector<float> & delays, std::vector<float> & shutters_float, char* comport, int & numtakes, bool loop) {

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
		// Save data_buffer_PMD to the RawData instance
		//RawData_cap = RawData(data_buffer_PMD, data_buffer_PMD_size, frequencies, delays, shutters_float, phases, w, h, numtakes);
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

	// Checking input data
	if (check_input_data(frequency_, distance_, shutter_, comport, loop, false, false) == 0)
		return -3;

	// Checking errors in parameters
	if (check_parameters (frequency_, shutter_, comport, true, true) == 0)
		return -3;
	std::vector<float> phases(2);	phases[0] = 0.0f;	phases[1] = 90.0f; 
	// Shutters vector of pairs
	std::vector<pair<int, unsigned short*>> shutters;	// process_data_to_buffer(...) deal with vectors of pairs
	shutters.push_back(pair<int, unsigned short*>((int)shutter_, new unsigned short [PMD_WIDTH*PMD_HEIGTH*10]));
	
	// Buffer of the PMD data
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
	clock_t begin_time_loop, end_time_loop;
	// Init OpenCV
	if (CV_WHILE_CAPTURING)
		cvNamedWindow(WindowName, CV_WINDOW_AUTOSIZE);

	// Syncronization
	std::unique_lock<std::mutex> locker_frame_object;	// Create a defered locker (a locker not locked yet)
	locker_frame_object = std::unique_lock<std::mutex>(mutex_frame_object,std::defer_lock);

	// --- CAPTURE LOOP --------------------------------------------------------------------------------------
	bool first_iter = true;
	while(loop || first_iter) {

		begin_time_loop = clock();	// begin_time_loop for DUTYCYCLE Sleep 

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
			process_data_to_buffer_no_cv(w, h, shutters, ushort_img);
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
			Frame_00_cap = Frame(ushort_img[0], h, w, distance_, frequency_, shutter_, phases[0], 0, PIXELS_STORING_GLOBAL);
		if (&Frame_90_cap != NULL)
			Frame_90_cap = Frame(ushort_img[0], h, w, distance_, frequency_, shutter_, phases[1], 1, PIXELS_STORING_GLOBAL);
		//std::cout << "UPDATED_NEW_FRAME";
		UPDATED_NEW_FRAME = true;
		UPDATED_NEW_OBJECT = false;
		cv_frame_object.notify_all();	// Notify all cv_frame_object. All threads waiting for cv_frame_object will break the wait after waking up
		locker_frame_object.unlock();	// Unlock mutex_frame_object, now threads which used mutex_frame_object can continue
				//const clock_t end_time_buffer_to_frame = clock();
				//float ms_time_buffer_to_frame = 1000.0f * float(end_time_buffer_to_frame - begin_time_buffer_to_frame) / (float)CLOCKS_PER_SEC;
				//std::cout << "buffer_to_frame: time = " << ms_time_buffer_to_frame << " ms\n";

		end_time_loop = clock();		// end_time_loop for DUTYCYCLE Sleep
		ms_time_loop = 1000.0f * float(end_time_loop - begin_time_loop) / (float)CLOCKS_PER_SEC;
		ms_extra_delay = ((float)shutter)/(DUTYCYCLE*1000.0f) - ms_time_loop + 1.0f;
		if (ms_extra_delay > 0)
			Sleep(ms_extra_delay);	// suspends the execution of the current thread until the time-out interval elapses
				//std::cout << "ms_time_loop   : " << ms_time_loop << " ms\n";
				//std::cout << "ms_extra_delay : " << ms_extra_delay << " ms\n\n";

				//const clock_t end_time_loop_total = clock();
				//float ms_time_loop_total = 1000.0f * float(end_time_loop_total - begin_time_loop) / (float)CLOCKS_PER_SEC;
				//float fps_time_loop_total = 1000.0f / ms_time_loop_total;
				//std::cout << "total          : time = " << ms_time_loop_total << " ms,    fps = " << fps_time_loop_total <<  " fps\n\n";
	}
	// --- END OF CAPTURE LOOP -------------------------------------------------------------------------------
	
	// closing, deleting, the aftermath
	delete shutters[0].second;
	pmdClose (hnd);
	if (CV_WHILE_CAPTURING)
		cvDestroyWindow(WindowName);

	// Exit program
	//Sleep(2000);
	return 0;
}
// there's a weird bug when calling directly to PMD_params_to_Frame from thread constructor. With this re-calling functtion the bug is avoided
int PMD_params_to_Frame_anti_bug_thread (Frame & Frame_00_cap, Frame & Frame_90_cap, float frequency_, float distance_, float shutter_, char* comport, bool loop) {
	return PMD_params_to_Frame (FRAME_00_CAPTURE, FRAME_90_CAPTURE, frequency_, distance_, shutter_, comport, loop);
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


