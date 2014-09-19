/*
Mater Thesis
Jaime Martin
*/

//#define _VARIADIC_MAX 10	// To let thread dealing with functions up to 10 variables. Already defined in Preprocessor Definitions

#include <windows.h>  // for MS Windows
#include <GL/glut.h>  // GLUT, include glu.h and gl.h
#include "global.h"
#include "main.h"
#include "render.h"
#include "scene.h"
#include "data_sim.h"
#include "data.h"
#include "test.h"
#include "capturetool2.h"

#include <thread>      

// To measure loop time:
/*
	const clock_t begin_time = clock();
		// code...
	const clock_t end_time = clock();
	float ms_time = 1000.0f * float(end_time - begin_time) / (float)CLOCKS_PER_SEC;
	float fpstime = 1000.0f / ms_time;
	std::cout << "time = " << ms_time << " ms,    fps = " << fpstime <<  " fps\n";
*/




// it allows to externally control when the loops will finish
void control_loop_pause() {
	
	char end_loop[1024];	end_loop[0] = 'n';
	while ((end_loop[0] != 'y') && (end_loop[0] != 'Y') && (end_loop[0] != 'z') && (end_loop[0] != 'Z')) {
		std::cout << "\nDo you want to finish the PMD (and this) loop? (y/n), (z/n)\n  ";
		std::cin >> end_loop;
	}
	PMD_LOOP_ENABLE = false;
	std::cout << "\nOK, finishing loops...\n\n";
}




// Direct Vision problem, Sinusoid f,g assumption. Real Time.
int main_DirectVision_Sinusoid() {

	// capture data directly from PMD to Frame (FRAME_00_CAPTURE, FRAME_90_CAPTURE)
	float frequency = 100.0f;
	float distance = 0.0f;
	float shutter = 1920.0f;
	char comport[128] = "COM6";
	bool loop = true;
	PixStoring ps = PIXELS_STORING_GLOBAL;
	bool pSim = true;
	std::thread thread_PMD_params_to_Frame(PMD_params_to_Frame_anti_bug_thread, std::ref(FRAME_00_CAPTURE), std::ref(FRAME_90_CAPTURE), frequency, distance, shutter, comport, loop, ps, pSim);

	// Set all the object3D of the corresponding scene
	SCENEMAIN.setScene_DirectVision(ps, pSim);
	std::thread thread_updatePixelPatches_Sinusoid(updatePixelPatches_Sinusoid_antiBugThread, std::ref(SCENEMAIN), std::ref(FRAME_00_CAPTURE), std::ref(FRAME_90_CAPTURE), loop, ps, pSim);

	// Render all the object3D of the scene
	int argcStub = 0;
	char** argvStub = NULL;
	std::thread thread_render(render_anti_bug_thread, argcStub, argvStub);
	std::cout << "\n\nDo NOT close the openGL window while the PMD loop is runnung.\n(it wouldn't be the end of the world, but it's better not to)\n\n";

	// pause in main to allow control when the loops will finish
	control_loop_pause();

	// joins
	thread_PMD_params_to_Frame.join();
	thread_updatePixelPatches_Sinusoid.join();
	thread_render.join();

	return 0;
}

// Direct Vision problem, Simulation, uses Calibration Matrix. Real Time.
int main_DirectVision_Simulation(char* dir_name_, char* file_name_) {

	// set the Info
	Info info(dir_name_, file_name_);

	// capture data directly from PMD to Frame (FRAME_00_CAPTURE, FRAME_90_CAPTURE)
	float frequency = 100.0f;
	float distance = 0.0f;
	float shutter = 1920.0f;
	char comport[128] = "COM6";
	bool loop = true;
	PixStoring ps = PIXELS_STORING_GLOBAL;
	bool pSim = true;
	std::thread thread_PMD_params_to_Frame(PMD_params_to_Frame_anti_bug_thread, std::ref(FRAME_00_CAPTURE), std::ref(FRAME_90_CAPTURE), frequency, distance, shutter, comport, loop, ps, pSim);

	// Set all the corresponding scene and start updating
	SCENEMAIN.setScene_DirectVision(ps, pSim);
	std::thread thread_updatePixelPatches_Simulation(updatePixelPatches_Simulation_antiBugThread, std::ref(info), std::ref(SCENEMAIN), std::ref(FRAME_00_CAPTURE), std::ref(FRAME_90_CAPTURE), loop, ps, pSim);

	// Render all the object3D of the scene
	int argcStub = 0;
	char** argvStub = NULL;
	std::thread thread_render(render_anti_bug_thread, argcStub, argvStub);
	std::cout << "\n\nDo NOT close the openGL window while the PMD loop is runnung.\n(it wouldn't be the end of the world, but it's better not to)\n\n";

	// pause in main to allow control when the loops will finish
	control_loop_pause();

	// joins
	thread_PMD_params_to_Frame.join();
	thread_updatePixelPatches_Simulation.join();
	thread_render.join();

	return 0;
}

// Direct Vision problem, Simulation, uses Calibration Matrix. NO Real Time, one only Frame from RawData.
int main_DirectVision_Simulation_Frame(char* dir_name_, char* file_name_) {

	// set the Info
	Info info(dir_name_, file_name_);

	// get a Frame from the RawData
	RawData rawData(info);
	float pathWallOffset = -0.5f;
	PixStoring ps = PIXELS_VALID;
	bool pSim = true;
	int dist_idx = get_dist_idx(info, pathWallOffset);	// returns -1 if no idx correspondance was found
	if (dist_idx < 0) {
		std::cout << "\nWarning: Distance Offset = " << pathWallOffset << " is not a dist in .cmx distV = ";
		print(info.distV);
		return -1;
	}
	Frame frame00(rawData, info.freqV.size() - 1, dist_idx, info.shutV.size() - 1, 0, ps, pSim);
	Frame frame90(rawData, info.freqV.size() - 1, dist_idx, info.shutV.size() - 1, 1, ps, pSim);
	UPDATED_NEW_FRAME = true;
	//frame00.plot();

	// Set all the corresponding scene and start updating
	bool loop = false;
	SCENEMAIN.setScene_DirectVision(ps, pSim);
	updatePixelPatches_Simulation_antiBugThread(info, SCENEMAIN, frame00, frame90, loop, ps, pSim);	// the second Fame is frame90 but it's not used

	// Render all the object3D of the scene
	int argcStub = 0;
	char** argvStub = NULL;
	std::thread thread_render(render_anti_bug_thread, argcStub, argvStub);

	// joins
	thread_render.join();
	//cv::destroyAllWindows();

	return 0;
}

// Occlusion problem. Real Time.
int main_Occlusion(char* dir_name_, char* file_name_) {
	
	// set the Info
	Info info(dir_name_, file_name_);

	// capture data directly from PMD to Frame (FRAME_00_CAPTURE, FRAME_90_CAPTURE)
	float frequency = 100.0f;
	float distance = 0.0f;
	float shutter = 1920.0f;
	char comport[128] = "COM6";
	bool loop = true;
	PixStoring ps = PIXELS_STORING_GLOBAL;
	bool pSim = true;
	std::thread thread_PMD_params_to_Frame(PMD_params_to_Frame_anti_bug_thread, std::ref(FRAME_00_CAPTURE), std::ref(FRAME_90_CAPTURE), frequency, distance, shutter, comport, loop, ps, pSim);

	// Set all the corresponding scene and start updating
	SCENEMAIN.setScene_Occlusion(ps, pSim);
	std::thread thread_updateVolumePatches_Occlusion(updateVolumePatches_Occlusion_antiBugThread, std::ref(info), std::ref(SCENEMAIN), std::ref(FRAME_00_CAPTURE), std::ref(FRAME_90_CAPTURE), loop, ps, pSim);

	// Render all the object3D of the scene
	int argcStub = 0;
	char** argvStub = NULL;
	std::thread thread_render(render_anti_bug_thread, argcStub, argvStub);
	std::cout << "\n\nDo NOT close the openGL window while the PMD loop is runnung.\n(it wouldn't be the end of the world, but it's better not to)\n\n";

	// pause in main to allow control when the loops will finish
	control_loop_pause();
	
	// joins
	thread_PMD_params_to_Frame.join();
	thread_updateVolumePatches_Occlusion.join();
	thread_render.join();

	return 0;
}

// Occlusion problem. NO Real Time, one only Frame from RawData.
int main_Occlusion_Frame(char* dir_name_, char* file_name_) {

	// set the Info
	Info info(dir_name_, file_name_);

	// get a Frame from the RawData
	RawData rawData(info);
	float pathWallOffset = -0.5f;
	PixStoring ps = PIXELS_VALID;
	bool pSim = true;
	int dist_idx = get_dist_idx(info, pathWallOffset);	// returns -1 if no idx correspondance was found
	if (dist_idx < 0) {
		std::cout << "\nWarning: Distance Offset = " << pathWallOffset << " is not a dist in .cmx distV = ";
		print(info.distV);
		return -1;
	}
	Frame frame00(rawData, info.freqV.size() - 1, dist_idx, info.shutV.size() - 1, 0, ps, pSim);
	Frame frame90(rawData, info.freqV.size() - 1, dist_idx, info.shutV.size() - 1, 1, ps, pSim);
	UPDATED_NEW_FRAME = true;
	//frame00.plot();

	// Set all the corresponding scene and start updating
	bool loop = false;
	SCENEMAIN.setScene_Occlusion(ps, pSim);
	std::thread thread_updateVolumePatches_Occlusion(updateVolumePatches_Occlusion_antiBugThread, std::ref(info), std::ref(SCENEMAIN), std::ref(frame00), std::ref(frame90), loop, ps, pSim);

	// Render all the object3D of the scene
	int argcStub = 0;
	char** argvStub = NULL;
	std::thread thread_render(render_anti_bug_thread, argcStub, argvStub);

	// joins
	thread_updateVolumePatches_Occlusion.join();
	thread_render.join();
	//cv::destroyAllWindows();

	return 0;
}




// Raw Data Measurement
int main_RawData(char* dir_name_, char* file_name_) {
	
	// set all the object3D of the corresponding scene (just camera, laser and wall)
	bool cmx_info = true;	// pure raw data is not cmx oriented, but it's a good idea to store the corresponding info anyway
	float laser_to_cam_offset_x = -0.15f;
	float laser_to_cam_offset_y = 0.0f;
	float laser_to_cam_offset_z = 0.0;
	float dist_wall_cam = 2.0f;
	float cmx_params[4] = {laser_to_cam_offset_x, laser_to_cam_offset_y, laser_to_cam_offset_z, dist_wall_cam};

	// create the .raw file, capturing data from the PMD

	// FREQUENCIES
	std::vector<float> frequencies;	// (MHz)
	float freq_res = 50.0f;
	float freq_min = 50.0f;
	float freq_max = 100.0f + freq_res/2.0f;	// (+ freq_res/2.0f) is due to avoid rounding problems
	for (float fi = freq_min; fi <= freq_max; fi += freq_res)
		frequencies.push_back(fi);
	// DISTANCES
	std::vector<float> delays;		// (m)
	float delay_res = 2.0f;
	float delay_min = -3.0f;
	float delay_max = 10.0f + delay_res/2.0f;	// (+ delay_res/2.0f) is due to avoid rounding problems
	for (float di = delay_min; di <= delay_max; di += delay_res)
		delays.push_back(di);
	// SUTTERS
	//std::vector<float> shutters_float(1, 1920.0f);	// (us)
	std::vector<float> shutters_float;		// (m)
	float shutters_float_mul = 2.0f;	if (shutters_float_mul <= 1.0f)	{ shutters_float_mul = 2.0f; }
	float shutters_float_min = SHUTTER_MAX / 1.0f;	// 15(/128), 30(/64), 60(/32), 120(/16), 240(/8), 480(/4), 960(/2), 1920(/1)
	float shutters_float_max = SHUTTER_MAX + 1.0f;	// (+ 1.0f) is due to avoid rounding problems
	for (float si = shutters_float_min; si <= shutters_float_max; si *= shutters_float_mul)
		shutters_float.push_back(si);
	// NUMTAKES
	int numtakes = 10;
	// OTHER
	char comport[128] = "COM6";
	std::thread thread_PMD_params_to_file (PMD_params_to_file_anti_bug_thread, frequencies, delays, shutters_float, dir_name_, file_name_, comport, numtakes, cmx_info, cmx_params);

	// pause in main to allow control when the loops will finish
	//control_loop_pause();

	// joins
	thread_PMD_params_to_file.join();

	return 0;
}

// Field of View Measurement. Image upscaled in Real Time.
int main_FoVmeas() {

	// capture data directly from PMD to Frame (FRAME_00_CAPTURE, FRAME_90_CAPTURE)
	float frequency = 100.0f;
	float distance = 0.0f;
	float shutter = 1920.0f; // 1920.0f;
	char comport[128] = "COM6";
	bool loop = true;
	PixStoring ps = PIXELS_STORING_GLOBAL;
	bool pSim = false;
	std::thread thread_PMD_params_to_Frame(PMD_params_to_Frame_anti_bug_thread, std::ref(FRAME_00_CAPTURE), std::ref(FRAME_90_CAPTURE), frequency, distance, shutter, comport, loop, ps, pSim);
	
	// plot the Frame (combining frame00 and frame90)
	char windowName[128] = "Frame FoV";
	std::thread thread_plot_frame_fov_measurement (plot_frame_fov_measurement, std::ref(FRAME_00_CAPTURE), std::ref(FRAME_90_CAPTURE), loop, false, windowName);//(char*)NULL);

	// pause in main to allow control when the loops will finish
	control_loop_pause();

	// joins
	thread_PMD_params_to_Frame.join();
	thread_plot_frame_fov_measurement.join();

	return 0;
}

// Testing main.
int main_Test(char* dir_name_, char* file_name_) {

	// Set all the object3D of the corresponding scene
	Info info(dir_name_, file_name_);
	bool loop = true;
	PixStoring ps = PIXELS_STORING_GLOBAL;
	bool pSim = true;
	//SCENEMAIN.setScene_DirectVision(ps, pSim);
	//SCENEMAIN.setScene_Occlusion(ps, pSim);
	SCENEMAIN.setScene_CalibrationMatrix(info.laser_to_cam_offset_x, info.laser_to_cam_offset_y, info.laser_to_cam_offset_z, info.dist_wall_cam, PIXELS_TOTAL, pSim);
	SCENEMAIN.o[WALL].clear();

	// Render all the object3D of the scene
	int argcStub = 0;
	char** argvStub = NULL;
	std::thread thread_render(render_anti_bug_thread, argcStub, argvStub);

	// joins
	thread_render.join();

	return 0;
}

// DEPRECATED (now CalibrationMatrix constructor build it from .raw, no need .cmx) .raw to .cmx.
/*
int main_CalibrationMatrix (char* dir_name_, char* file_name_) {
	
	// built the global Info instance INFO
	Info info = Info(dir_name_, file_name_);
	
	// create the .cmx file, dealing with the .inf and .raw files. It internally creates the corresponding scene
	std::thread thread_create_cmx_from_raw (create_cmx_from_raw_anti_bug_thread, std::ref(info));

	// pause in main to allow control when the loops will finish
	//control_loop_pause();
	
	// joins
	thread_create_cmx_from_raw.join();

	return 0;
}
*/



// MAIN
int main(int argc, char** argv) {
	
	// Set RAW_DATA
	SceneType sceneType = DIRECT_VISION_SIMULATION;
	SCENEMAIN.set(sceneType);
	char dir_name[1024] = "F:\\Jaime\\CalibrationMatrix\\cmx_01";
	//char dir_name[1024] = "C:\\Users\\Natalia\\Documents\\Visual Studio 2013\\Projects\\DiffuseMirrors2\\CalibrationMatrix\\cmx_01";
	char file_name[1024] = "PMD";
	
	// Main Switcher
	switch (sceneType) {
		case DIRECT_VISION_SINUSOID:		 main_DirectVision_Sinusoid();								break;
		case DIRECT_VISION_SIMULATION:		 main_DirectVision_Simulation(dir_name, file_name);			break;
		case DIRECT_VISION_SIMULATION_FRAME: main_DirectVision_Simulation_Frame(dir_name, file_name);	break;
		case OCCLUSION:						 main_Occlusion(dir_name, file_name);						break;
		case OCCLUSION_FRAME:				 main_Occlusion_Frame(dir_name, file_name);					break;
		case RAW_DATA:						 main_RawData (dir_name, file_name);						break;
		case FOV_MEASUREMENT:				 main_FoVmeas ();											break;
		//case CALIBRATION_MATRIX:			 main_CalibrationMatrix (dir_name, file_name);				break;
		case TEST:							 main_Test (dir_name, file_name);							break;
		case TEST_TEST:						 test(dir_name, file_name);									break;
	}

	system("pause");
	return 0;
}


