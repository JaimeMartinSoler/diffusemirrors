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

using namespace std;
using namespace cv;

#define SOURCE_PLUGIN "camboardnano"
#define SOURCE_PARAM ""

#define SYNTH_CLOCK 600.0000 // Sample clock of function generator
//#define SPEEDOFLIGHT	 299792458	// (m/s)
#define SPEEDOFLIGHT_AIR 299705000	// (m/s)
#define DUTYCYCLE 10 // 1 us exposure, 9 us delay
#define FILENAME_FORMAT "capture_take%02d_f%06.2f_d%05.2f" // Filename prefix: Frequency, delay in m
#define FILENAME_APPEND "_p%03d.%s"         // Append phase, shutter (=0 for HDR) and suffix to filename

#define FREQUENCY_MIN 1.0	// (MHz)
#define FREQUENCY_MAX 180.0	// (MHz)
#define SHUTTER_MIN 3.0		// (us)
#define SHUTTER_MAX 1920.0	// (us)

char err[128];

FILE *rawdumpfile;

// TRANSIENTPMD SETTINGS
char comport[128] = "\\\\.\\COM6";
#define COMPORT "\\\\.\\%s"

#define WindowName "PMD Image"
#define NOJPEG

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
	cout << endl << "Capturing frame" << flush;
	// Make frequency generator settings
	double true_frequency = set_frequency(port, frequency_MHz);
	set_delay(port, true_frequency, delay_m);
	set_shutter(port, shutter);
	cout << "." << flush;

	int res;
	// Set exposure time on camera
	res = pmdSetIntegrationTime(hnd, 0, shutter+12); // Add a few microseconds overhead 
	pmd_handle_error(hnd, res, "Could not set integration time");
	cout << "." << flush;

	// Trigger capture!
	res = pmdUpdate (hnd);
	pmd_handle_error(hnd, res, "Could not update");
	cout << "." << flush;

	// Retrieve data from camera
	PMDDataDescription dd;
	res = pmdGetSourceDataDescription(hnd, &dd);
	pmd_handle_error(hnd, res, "Could not retrieve source data description");
	w = dd.img.numColumns;
	h = dd.img.numRows;
	numframes = dd.img.numSubImages;
	res = pmdGetSourceData(hnd, buffer, w*h*numframes*4);
	pmd_handle_error(hnd, res, "Could not get source data");
	cout << "." << flush;

	// Data is in big endian format
	swap_endian(buffer, w*h*numframes);

	// Cut the signal generator some slack
	set_frequency(port, 31.0);
	cout << "." << flush;
	cout << "done." << endl;
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
template <typename T>
T ato ( const string &text) {
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


// char_array_to_float_vector (...)
void char_array_to_float_vector (char* & char_array, vector<float> & float_vector, float min, float max) {

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
            if (float_value > max) float_value = max;
            float_vector.push_back(float_value);
            if (char_value == '\0')
                break;
        } else {
            float_str += char_value;
        }
    }  
}




// parser_main(...)
// return 0:  No errors parsing
// return -1: Errors parsing
int parser_main (int argc, char *argv[], vector<float> & frequencies, vector<float> & delays, vector<pair<int, unsigned short*>> & shutters, char* dir_name, char* comport_, int & numtakes) {
	
	// Check the number of arguments. Print info and return -1 if it is wrong
	if (argc < 6) {
		std::cout << endl << "TransientPMD capture tool 2. Synopsis: " << endl << endl <<
			"  capturetool2.exe  // name of binary" << endl <<
			"  freq_array       // Freqs (MHz) to measure (ex: \"80 90.5 100.0\")" << endl <<
			"  dist_array       // Distances sweep (m) to measure (ex: \"0 0.5 1\")" << endl <<
			"  shutter_array    // Shutter/Exposures (us) to measure (ex: \"960 1920\")" << endl <<
			"  dir_name         // Target directory (ex: f:\\tmp\\pmdtest2)" << endl <<
			"  comport          // USB serial port (ex: COM6)" << endl << 
			"  [numtakes=1]     // Repeat measurement multiple times? (ex: 10)" << endl << endl;
		return -1;
	}
	
	// Frequencies (argv[1])
	char_array_to_float_vector (argv[1], frequencies, FREQUENCY_MIN, FREQUENCY_MAX);
	
	// Delays (argv[2])
	char_array_to_float_vector (argv[2], delays, -FLT_MAX/2.0f, FLT_MAX/2.0f);
	
	// Shutters (argv[3])
	vector<float> shutters_float;
	char_array_to_float_vector (argv[3], shutters_float, SHUTTER_MIN, SHUTTER_MAX);
	for (size_t i = 0; i < shutters_float.size(); i++)
		shutters.push_back(pair<int, unsigned short*>((int)shutters_float[i], new unsigned short [165*120*10]));
	
	// Directory Name (argv[4])
	sprintf(dir_name,"%s", argv[4]);
	
	// Comport (argv[5])
	sprintf(comport_, COMPORT, argv[5]);
	
	// Number of Takes (argv[6])
	if (argc >= 7) {
		numtakes = ato<int> (argv[6]);
        if (numtakes < 1) numtakes = 1;
	}

	// Check the size of the vectors. Return -1 if it is wrong
	if (frequencies.size() < 1)
		std::cout << endl << "Frequencies array (argv[1]) empty or wrong. Quitting.";
	if (delays.size() < 1)
		std::cout << endl << "Delays array (argv[2]) empty or wrong. Quitting.";
	if (shutters.size() < 1)
		std::cout << endl << "Shutters/Exposures array (argv[3]) empty or wrong. Quitting.";
	if ((frequencies.size() < 1) || (delays.size() < 1) || (shutters.size() < 1)) {
		std::cout << endl  << endl;
		return -1;
	}

	// No errors
	return 0;
}




// MAIN
int capturetoolDM2_main (int argc, char *argv[]) {
	
	// ------------------------------------------------------------------------------------------------------------------------------
	// DEBUGGING:
	// Properties - Configuration Properties - Debugging - Command Arguments:
	// "80 90 100" "0 1 2 3" "1920" f:\tmp\pmdtest COM6 1
	//
	// EXECUTING:
	// cd C:\Users\transient\Documents\Visual Studio 2012\Projects\DiffuseMirrors2\x64\Release
	// DiffuseMirrors2.exe "80 90 100" "0 1 2 3" "1920" f:\tmp\pmdtest2 COM6 1
	// ------------------------------------------------------------------------------------------------------------------------------
	

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


	// sizeof(short int s) = 2, but sizeof(int i) = 4

	// Variables
	vector<float> frequencies;
	vector<float> delays;
	vector<pair<int, unsigned short*>> shutters;
	char dir_name[1024];
	// char comport[128] = "\\\\.\\COM6";	// is already a global variable
	int numtakes = 1;
	
	// Parsing the input to the variables
	if (int parser_error = parser_main (argc, argv, frequencies, delays, shutters, dir_name, comport, numtakes) < 0)
		return parser_error;

	//char measurement_name[1024] = "default";


	// Init devices

	// Open PMD sensor
	PMDHandle hnd;
	int res;
	int w, h, numframes;

	res = pmdOpenSourcePlugin(&hnd, SOURCE_PLUGIN, SOURCE_PARAM);
	pmd_handle_error(hnd, res, "Could not open device");

	SerialPort port = control_init(comport);

	// Capture parameters
	char command[1024];
	sprintf(command,"%s", dir_name);
	if (dir_exists(command)) {
		cout << "Warning: Measurement directory \"" << command << "\" exists and will be deleted. Sure? (y/n)";
		string answer;
		cin >> answer;
		if (answer[0] != 'y' && answer[0] != 'Y') {
			cout << "Okay - quitting." << endl;
			pmdClose(hnd);
			return -2;
		}
		sprintf(command,"rd /S /Q %s", dir_name);
		system(command);
	}
	sprintf(command,"md %s", dir_name);
	system(command);

	// Init OpenCV
	cvNamedWindow( WindowName, CV_WINDOW_AUTOSIZE );

	bool firstiter = true;


	for (int take = 0; take < numtakes;++take) {
		char fn[256];
		sprintf(fn,"%s\\meas_data_take_%03d.dat", dir_name, take);
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

					sprintf(command,"%s\\info.txt", dir_name);
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
				int extra_delay = 4L*DUTYCYCLE*shutter/1000 - ms_elapsed + 1;
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


