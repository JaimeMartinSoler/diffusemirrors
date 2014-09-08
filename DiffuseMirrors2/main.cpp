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
	while ((end_loop[0] != 'y') && (end_loop[0] != 'Y')) {
		std::cout << "\nDo you want to finish the PMD (and this) loop? (y/n)\n  ";
		std::cin >> end_loop;
	}
	PMD_LOOP_ENABLE = false;
	std::cout << "\nOK, finishing loops...\n\n";
}




// CHECKED OK
int main_DirectVision_Sinusoid(bool loop = true) {

	// capture data directly from PMD to Frame (FRAME_00_CAPTURE, FRAME_90_CAPTURE)
	float frequency = 100.0f;
	float distance = 0.0f;
	float shutter = 1920.0f;
	char comport[128] = "COM6";
	PixStoring ps = PIXELS_STORING_GLOBAL;
	std::thread thread_PMD_params_to_Frame(PMD_params_to_Frame_anti_bug_thread, std::ref(FRAME_00_CAPTURE), std::ref(FRAME_90_CAPTURE), frequency, distance, shutter, comport, loop, ps);

	// Set all the object3D of the corresponding scene
	SCENEMAIN.setScene_DirectVision(ps);
	std::thread thread_updatePixelPatches_Sinusoid(updatePixelPatches_Sinusoid_antiBugThread, std::ref(SCENEMAIN), std::ref(FRAME_00_CAPTURE), std::ref(FRAME_90_CAPTURE), loop, ps);

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

// TO-DO (just TO-CHECK)
int main_DirectVision_Simulation(bool loop = true) {

	// capture data directly from PMD to Frame (FRAME_00_CAPTURE, FRAME_90_CAPTURE)
	float frequency = 100.0f;
	float distance = 0.0f;
	float shutter = 1920.0f;
	char comport[128] = "COM6";
	PixStoring ps = PIXELS_STORING_GLOBAL;
	std::thread thread_PMD_params_to_Frame(PMD_params_to_Frame_anti_bug_thread, std::ref(FRAME_00_CAPTURE), std::ref(FRAME_90_CAPTURE), frequency, distance, shutter, comport, loop, ps);

	// Set all the corresponding scene and start updating
	SCENEMAIN.setScene_DirectVision(ps);
	std::thread thread_updatePixelPatches_Simulation(updatePixelPatches_Simulation_antiBugThread, std::ref(SCENEMAIN), std::ref(FRAME_00_CAPTURE), std::ref(FRAME_90_CAPTURE), loop, ps);

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

// TO-DO (TO-DO related functions inside and TO-CHECK)
int main_Occlusion(bool loop = true) {

	// capture data directly from PMD to Frame (FRAME_00_CAPTURE, FRAME_90_CAPTURE)
	float frequency = 100.0f;
	float distance = 0.0f;
	float shutter = 1920.0f;
	char comport[128] = "COM6";
	PixStoring ps = PIXELS_STORING_GLOBAL;
	std::thread thread_PMD_params_to_Frame(PMD_params_to_Frame_anti_bug_thread, std::ref(FRAME_00_CAPTURE), std::ref(FRAME_90_CAPTURE), frequency, distance, shutter, comport, loop, ps);

	// Set all the corresponding scene and start updating
	SCENEMAIN.setScene_Occlusion(ps);
	std::thread thread_updateVolumePatches_Occlusion(updateVolumePatches_Occlusion_antiBugThread, std::ref(SCENEMAIN), std::ref(FRAME_00_CAPTURE), std::ref(FRAME_90_CAPTURE), loop, ps);

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

// CHECKED OK
int main_FoVmeas(bool loop = true) {
	
	// capture data directly from PMD to Frame (FRAME_00_CAPTURE, FRAME_90_CAPTURE)
	float frequency = 100.0f;
	float distance = 0.0f;
	float shutter = 1920.0f;
	char comport[128] = "COM6";
	PixStoring ps = PIXELS_STORING_GLOBAL;
	std::thread thread_PMD_params_to_Frame(PMD_params_to_Frame_anti_bug_thread, std::ref(FRAME_00_CAPTURE), std::ref(FRAME_90_CAPTURE), frequency, distance, shutter, comport, loop, ps);
	
	// plot the Frame (combining frame00 and frame90)
	std::thread thread_plot_frame_fov_measurement (plot_frame_fov_measurement, std::ref(FRAME_00_CAPTURE), std::ref(FRAME_90_CAPTURE), loop);

	// pause in main to allow control when the loops will finish
	control_loop_pause();

	// joins
	thread_PMD_params_to_Frame.join();
	thread_plot_frame_fov_measurement.join();

	return 0;
}

// CHECKED OK
int main_RawData(char* dir_name_, char* file_name_) {
	
	// set all the object3D of the corresponding scene (just camera, laser and wall)
	bool cmx_info = true;	// pure raw data is not cmx oriented, but it's a good idea to store the corresponding info anyway
	float laser_to_cam_offset_x = -0.15f;
	float laser_to_cam_offset_y = 0.0f;
	float laser_to_cam_offset_z = 0.0;
	float dist_wall_cam = 2.0f;
	float cmx_params[4] = {laser_to_cam_offset_x, laser_to_cam_offset_y, laser_to_cam_offset_z, dist_wall_cam};

	// create the .raw file, capturing data from the PMD
	std::vector<float> frequencies;	// (MHz)
	float freq_res = 20;
	float freq_min = 100.0f;
	float freq_max = 100.0f + freq_res/2.0f;	// (+ freq_res/2.0f) is due to avoid rounding problems
	for (float fi = freq_min; fi <= freq_max; fi += freq_res)
		frequencies.push_back(fi);
	std::vector<float> delays;		// (m)
	float delay_res = 1.0f;
	float delay_min = -2.0f;
	float delay_max = 10.0f + delay_res/2.0f;	// (+ delay_res/2.0f) is due to avoid rounding problems
	for (float di = delay_min; di <= delay_max; di += delay_res)
		delays.push_back(di);
	std::vector<float> shutters_float(1, 1920.0f);	// (us)
	char comport[128] = "COM6";
	int numtakes = 10;
	std::thread thread_PMD_params_to_file (PMD_params_to_file_anti_bug_thread, frequencies, delays, shutters_float, dir_name_, file_name_, comport, numtakes, cmx_info, cmx_params);

	// pause in main to allow control when the loops will finish
	//control_loop_pause();

	// joins
	thread_PMD_params_to_file.join();

	return 0;
}

// CHECKED OK: It builds a .cmx file from a .raw file with the parameters stored in the .inf file
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

int main_Test(bool loop = true) {

	// Set all the object3D of the corresponding scene
	PixStoring ps = PIXELS_STORING_GLOBAL;
	SCENEMAIN.setScene_DirectVision(ps);
	//SCENEMAIN.setScene_Occlusion(ps);
	
	// Render all the object3D of the scene
	int argcStub = 0;
	char** argvStub = NULL;
	std::thread thread_render(render_anti_bug_thread, argcStub, argvStub);

	// joins
	thread_render.join();

	return 0;
}



// MAIN
int main(int argc, char** argv) {
	
	// Set parameteres
	SceneType sceneType = DIRECT_VISION_SINUSOID;
	SCENEMAIN.set(sceneType);
	//char dir_name[1024] = "C:\\Users\\Natalia\\Documents\\Visual Studio 2013\\Projects\\DiffuseMirrors2\\CalibrationMatrix\\test_03";
	//char file_name[1024] = "PMD";
	char dir_name[1024] = "F:\\Jaime\\CalibrationMatrix\\test_05";
	char file_name[1024] = "PMD";
	bool loop = true;
	
	// Main Switcher
	switch (sceneType) {
		case DIRECT_VISION_SINUSOID:	main_DirectVision_Sinusoid (loop);				break;
		case DIRECT_VISION_SIMULATION:	main_DirectVision_Simulation (loop);			break;
		case OCCLUSION:					main_Occlusion (loop);							break;
		case FOV_MEASUREMENT:			main_FoVmeas (loop);							break;
		case RAW_DATA:					main_RawData (dir_name, file_name);				break;
		case CALIBRATION_MATRIX:		main_CalibrationMatrix (dir_name, file_name);	break;
		case TEST:						main_Test (loop);								break;
	}

	system("pause");
	return 0;
}


