
#include <iostream> 
#include <stdlib.h>     // atof

#include "global.h"
#include "data_read.h"
#include "data_sim.h"
#include "capturetoolDM2.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>	


// Constructor
DataPMD::DataPMD(char* dir_name_, char* file_name_) {

	dir_name = dir_name_;
	file_name = file_name_;
	
	char file_data_full_path_name_[1024];
	char file_info_full_path_name_ [1024];
	sprintf(file_data_full_path_name_,"%s\\%s%s", dir_name_, file_name_, FILE_DATA_NAME_SUFFIX);
	sprintf(file_info_full_path_name_,"%s\\%s%s", dir_name_, file_name_, FILE_INFO_NAME_SUFFIX);
	file_data_full_path_name = file_data_full_path_name_;
	file_info_full_path_name = file_info_full_path_name_;

	src = DATA_FILE;

	// INFO FILE. Open with read permissions
	FILE* file_info = fopen(file_info_full_path_name, "r");
	if (file_info == NULL) {
		std::cout << "\n\nError Reading \""<< file_info_full_path_name << "\"\n\n";
		error_code = 1;
		return;
	}
	int line_number = 0;
	char delimiter = ':';
	char file_info_line[1024];
	
	// Explore line by line, getting the vectors and width, heigth and numtakes
	while (fgets(file_info_line, 1024, file_info)) {
		std::cout << file_info_line;
		
		// bytes_per_value
		if (line_number == 1) {
			std::vector<float> bytes_per_value_vector;
			char_array_to_float_vector_from_delimiter (file_info_line, bytes_per_value_vector, delimiter);
			bytes_per_value = (int)bytes_per_value_vector[0];
		}
		// width and heigth
		else if (line_number == 2) {
			std::vector<float> width_and_heigth;
			char_array_to_float_vector_from_delimiter (file_info_line, width_and_heigth, delimiter);
			width = (int)width_and_heigth[0];
			heigth = (int)width_and_heigth[1];
		}
		// frequencies
		else if (line_number == 3)
			char_array_to_float_vector_from_delimiter (file_info_line, frequencies, delimiter);
		// distances
		else if (line_number == 4)
			char_array_to_float_vector_from_delimiter (file_info_line, distances, delimiter);
		// shutters_
		else if (line_number == 5)
			char_array_to_float_vector_from_delimiter (file_info_line, shutters, delimiter);
		// phases
		else if (line_number == 6)
			char_array_to_float_vector_from_delimiter (file_info_line, phases, delimiter);
		// numtakes
		else if (line_number == 7) {
			std::vector<float> numtakes_vector;
			char_array_to_float_vector_from_delimiter (file_info_line, numtakes_vector, delimiter);
			numtakes = (int)numtakes_vector[0];
		}
		line_number++;
	}
	std::cout << "\n";
	fclose(file_info);
	int file_data_size_expected = frequencies.size() * distances.size() * shutters.size() * phases.size() * width * heigth;

	// DATA FILE. Open with read permissions
	FILE* file_data = fopen(file_data_full_path_name, "rb");	// open in binary/raw mode
	if (file_data == NULL) {
		std::cout << "\n\nError Reading \""<< file_data_full_path_name << "\"\n\n";
		error_code = 1;
		return;
	}
	size_t fread_output_size;

	// get file_data_size
	fseek (file_data , 0 , SEEK_END);
	data_size = ftell (file_data) / bytes_per_value;
	rewind (file_data);
	if (data_size != file_data_size_expected) {
		std::cout << "\n\nSize Incoherence Error while getting size of \""<< file_data_full_path_name << "\"\n\n";
		error_code = 4;
		return;
	}

	// allocate memory to contain the whole file
	data = (unsigned short int*) malloc(sizeof(unsigned short int)*data_size);
	if (data == NULL) {
		std::cout << "\n\nMemory Error while allocating \""<< file_data_full_path_name << "\"\n\n";
		error_code = 2;
		return;
	}

	// copy the file into the buffer:
	fread_output_size = fread (data, bytes_per_value, data_size, file_data);
	if (fread_output_size != data_size) {
		std::cout << "\n\nSize Error while reading \""<< file_data_full_path_name << "\"\n\n";
		error_code = 3;
		return;
	}

	fclose (file_data);
	//free (data_size_);
	error_code = 0;		// no errors

}

// Constructor
DataPMD::DataPMD(unsigned short int* data_, int data_size_, std::vector<float> & frequencies_, std::vector<float> & distances_, std::vector<float> & shutters_, std::vector<float> & phases_, int width_, int heigth_, int numtakes_, Source src_, int bytes_per_value_, int error_code_, char* dir_name_, char* file_name_) {
	
	data = data_;
	data_size = data_size_;	
	
	frequencies = frequencies_;
	distances = distances_;
	shutters = shutters_;
	phases = phases_;
	
	width = width_;
	heigth = heigth_;
	numtakes = numtakes_;
	
	src = src_;

	bytes_per_value = bytes_per_value_;
	error_code = error_code_;
	
	dir_name = dir_name_;
	file_name = file_name_;
	if ((dir_name_ != NULL) && (file_name_ != NULL)) {
		sprintf(file_data_full_path_name,"%s\\%s%s", dir_name_, file_name_, FILE_DATA_NAME_SUFFIX);
		sprintf(file_info_full_path_name,"%s\\%s%s", dir_name_, file_name_, FILE_INFO_NAME_SUFFIX);
	}
	else {
		file_data_full_path_name = NULL;
		file_info_full_path_name = NULL;
	}
}
// Constructor Default
DataPMD::DataPMD() {
	
	data = NULL;
	data_size = 0;	
	
	frequencies = std::vector<float>(0);
	distances = std::vector<float>(0);
	shutters = std::vector<float>(0);
	phases = std::vector<float>(0);
	
	width = 0;
	heigth = 0;
	numtakes = 0;
	
	src = UNKNOWN_SRC;

	bytes_per_value = 0;
	error_code = 0;
	
	dir_name = NULL;
	file_name = NULL;
	file_data_full_path_name = NULL;
	file_info_full_path_name = NULL;
}

// Returns the index in data[] corresponding to the parameter indices
int DataPMD::idx_in_data(int distances_idx, int frequencies_idx, int shutters_idx, int w, int h, int phases_idx) {
	return frequencies.size() * phases.size() * shutters.size() * heigth * width * distances_idx   +
	                            phases.size() * shutters.size() * heigth * width * frequencies_idx +
	                                            shutters.size() * heigth * width * phases_idx      +
	                                                              heigth * width * shutters_idx    +
	                                                                       width * h               +
	                                                                               w;
}

// Returns the value corresponding to the parameter indices = data[idx_in_data]
unsigned short int DataPMD::at(int distances_idx, int frequencies_idx, int shutters_idx, int w, int h, int phases_idx) {
	return data[idx_in_data(distances_idx, frequencies_idx, shutters_idx, w, h, phases_idx)];
}




// Constructor from DataPMD oriented
Frame::Frame(DataPMD & DataPMD_src_, int distance_idx_, int frequency_idx_, int shutter_idx_, int phase_idx_, Pixels_storing pixels_storing_) {

	DataPMD_src = &DataPMD_src_;
	pixels_storing = pixels_storing_;

	distance_idx = distance_idx_;
	frequency_idx = frequency_idx_;
	shutter_idx = shutter_idx_;
	phase_idx = phase_idx_;

	distance = DataPMD_src_.distances[distance_idx_];
	frequency = DataPMD_src_.frequencies[frequency_idx_];
	shutter = DataPMD_src_.shutters[shutter_idx_];
	phase = DataPMD_src_.phases[phase_idx_];

	if (pixels_storing_ == PIXELS_TOTAL) {
		width = DataPMD_src_.width;
		heigth = DataPMD_src_.heigth;
	}
	else if (pixels_storing_ == PIXELS_VALID) {
		width = CAMERA_PIX_X_VALID;
		heigth = CAMERA_PIX_Y_VALID;
	}
	numtakes = DataPMD_src_.numtakes;

	src = DataPMD_src_.src;

	bytes_per_value = DataPMD_src_.bytes_per_value;

	matrix = cv::Mat(heigth, width, cv::DataType<float>::type);
	if (pixels_storing_ == PIXELS_TOTAL) {
		for (int h = 0; h < heigth; h++) {
			for (int w = 0; w < width; w++) {
				// data is not stored properly. -32768 fixes it thanks to short int over-run
				// by default image is up-down-fliped so heigth_DataPMD = (heigth_Frame-1-h)
				matrix.at<float>(h, w) = (float)(DataPMD_src_.at(distance_idx_, frequency_idx_, shutter_idx_, w, heigth - 1 - h, phase_idx_) - 32768);
			}
		}
	} else if (pixels_storing_ == PIXELS_VALID) {
		for (int h = 0; h < heigth; h++) {
			for (int w = 0; w < width; w++) {
				// Also, take into account that DataPMD only stores Pixels Total
				matrix.at<float>(h, w) = (float)(DataPMD_src_.at(distance_idx_, frequency_idx_, shutter_idx_, w + CAMERA_PIX_X_BAD_LEFT, CAMERA_PIX_Y - CAMERA_PIX_Y_BAD_TOP - 1 - h, phase_idx_) - 32768);
			}
		}

	}
}

// Constructor from vector from SIMULATION oriented. For any Pixels_storing it considered the vector matrix_vector properly arranged
Frame::Frame(std::vector<float> & matrix_vector, int heigth_, int width_, bool rows_up2down, float distance_, float frequency_, float shutter_, float phase_, Source src_, Pixels_storing pixels_storing_) {
	
	DataPMD_src = NULL;
	pixels_storing = pixels_storing_;

	distance_idx = 0;
	frequency_idx = 0;
	shutter_idx = 0;
	phase_idx = 0;

	distance = distance_;
	frequency = frequency_;
	shutter = shutter_;
	phase = phase_;

	width = width_;
	heigth = heigth_;
	numtakes = 0;

	src = src_;

	bytes_per_value = 0;


	matrix = cv::Mat(heigth, width, cv::DataType<float>::type);
	if(rows_up2down) {
		for (int h = 0; h < heigth; h++) {
			for (int w = 0; w < width; w++) {
				matrix.at<float>(h, w) = matrix_vector[width * h + w];
	}	}	}
	else {
		for (int h = 0; h < heigth; h++) {
			for (int w = 0; w < width; w++) {
				matrix.at<float>(h, w) = matrix_vector[width * (heigth - 1 - h) + w];
	}	}	}

}

// Constructor from vector from DATA_REAL_TIME oriented. heigth_ and width_ must be refered to the sizes of the total frame, regerdingless to the Pixels_storing
Frame::Frame(unsigned short int* data_, int heigth_, int width_, float distance_, float frequency_, float shutter_, float phase_, int phase_idx_, Source src_, Pixels_storing pixels_storing_) {
	
	DataPMD_src = NULL;
	pixels_storing = pixels_storing_;

	distance_idx = 0;
	frequency_idx = 0;
	shutter_idx = 0;
	phase_idx = phase_idx_;

	distance = distance_;
	frequency = frequency_;
	shutter = shutter_;
	phase = phase_;
	

	if (pixels_storing_ == PIXELS_TOTAL) {
		width = width_;
		heigth = heigth_;
	}
	else if (pixels_storing_ == PIXELS_VALID) {
		width = CAMERA_PIX_X_VALID;
		heigth = CAMERA_PIX_Y_VALID;
	}
	numtakes = 0;

	src = src_;

	bytes_per_value = 0;

	matrix = cv::Mat(heigth, width, cv::DataType<float>::type);
	int idx_in_data;
	if (pixels_storing_ == PIXELS_TOTAL) {
		for (int h = 0; h < heigth; h++) {
			for (int w = 0; w < width; w++) {
				idx_in_data = (heigth_ * width_ * phase_idx_) + (width_ * (heigth_ - 1 - h)) + (w);
				matrix.at<float>(h, w) = (float)(data_[idx_in_data] - 32768);
		}	}
	} else if (pixels_storing_ == PIXELS_VALID) {
		for (int h = 0; h < heigth; h++) {
			for (int w = 0; w < width; w++) {
				idx_in_data = (heigth_ * width_ * phase_idx_) + (width_ * (CAMERA_PIX_Y - CAMERA_PIX_Y_BAD_TOP - 1 - h)) + (w + CAMERA_PIX_X_BAD_LEFT);
				matrix.at<float>(h, w) = (float)(data_[idx_in_data] - 32768);
		}	}
	}
}

// Constructor Default
Frame::Frame() {
	
	matrix = cv::Mat(0, 0, cv::DataType<float>::type);
	DataPMD_src = NULL;

	distance_idx = 0;
	frequency_idx = 0;
	shutter_idx = 0;
	phase_idx = 0;

	distance = 0.0f;
	frequency = 0.0f;
	shutter = 0.0f;
	phase = 0.0f;

	width = 0;
	heigth = 0;
	numtakes = 0;

	src = UNKNOWN_SRC;

	bytes_per_value = 0;
}

// first_idx can be 0 (default) or 1, it is the first idx of the row and col we are considering
float Frame::at(int row, int col, int first_idx) {	// (int first_idx = 0) by default in data_read.h
	if (first_idx == 0)
		return matrix.at<float>(row, col);
	else
		return matrix.at<float>(row-1, col-1);
} 

// Plot frame with opencv
void Frame::plot_frame() {

	if ((width <= 0) || (heigth <= 0))
		return;

	// Get a normalized matrix (min=0.0, max=1.0)
	cv::Mat M_norm = matrix.clone();
	double min, max, new_value;
	cv::minMaxLoc(M_norm, &min, &max);
	max -= min;
	cv::MatIterator_<float> it, end;
	for(it = M_norm.begin<float>(), end = M_norm.end<float>(); it != end; ++it) {
		(*it) = ((*it)-min) / max;
	}
	
	// show the image
	cv::namedWindow("Frame", cv::WINDOW_AUTOSIZE);	// WINDOW_NORMAL, WINDOW_AUTOSIZE
	cv::imshow("Frame", M_norm);
	cv::waitKey(1000);
}

// For FoV measurement scene. Plot frame with opencv with syncronization
void plot_frame_fov_measurement(bool loop) {		// by default: loop = false

	// Syncronization
	std::unique_lock<std::mutex> locker_frame_object;	// Create a defered locker (a locker not locked yet)
	locker_frame_object = std::unique_lock<std::mutex>(mutex_frame_object,std::defer_lock);

	
	// show the image
	cv::Mat M_norm;
	int scale = 10;
	bool first_iter = true;
	cv::namedWindow("Frame", cv::WINDOW_AUTOSIZE);	// WINDOW_NORMAL, WINDOW_AUTOSIZE

	// --- LOOP ------------------------------------------------------------------------------------------------
	while(loop || first_iter) {

		if (!PMD_LOOP_ENABLE && !first_iter)
			break;
		first_iter = false;
		
		// Syncronization
		locker_frame_object.lock();		// Lock mutex_frame_object, any thread which used mutex_frame_object can NOT continue until unlock()
		while (!UPDATED_NEW_FRAME) {
			cv_frame_object.wait(locker_frame_object);
		}

		// this three lines are the only critical zone
		if ((FRAME_90_CAPTURE.width <= 0) || (FRAME_90_CAPTURE.heigth <= 0)) {
			return;
		}
		M_norm = FRAME_90_CAPTURE.matrix.clone();		
		
		// Syncronization
		UPDATED_NEW_FRAME = false;
		UPDATED_NEW_OBJECT = true;
		cv_frame_object.notify_all();	// Notify all cv_frame_object. All threads waiting for cv_frame_object will break the wait after waking up
		locker_frame_object.unlock();	// Unlock mutex_frame_object, now threads which used mutex_frame_object can continue

		// Get a normalized matrix (min=0.0, max=1.0)
		double min, max, new_value;
		cv::minMaxLoc(M_norm, &min, &max);
		max -= min;
		cv::MatIterator_<float> it, end;
		for(it = M_norm.begin<float>(), end = M_norm.end<float>(); it != end; ++it)
			(*it) = ((*it)-min) / max;
		cv::resize(M_norm, M_norm, cv::Size(), scale, scale, cv::INTER_NEAREST);

		// show window
		cv::imshow("Frame", M_norm);
		cv::waitKey(20);
	}
}



// Reads data from a .dat and info.txt file setting the DATAPMD_READ variable
int data_read_main() {

	// FRAME_00_CAPTURE, FRAME_90_CAPTURE
	FRAME_00_CAPTURE.plot_frame();
	FRAME_90_CAPTURE.plot_frame();

	return 0;	// no errors
}



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

