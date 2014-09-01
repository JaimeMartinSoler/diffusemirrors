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
#include "data_read.h"
#include "shapes.h"
#include "test.h"
#include "capturetoolDM2.h"

#include <thread>      

// To measure loop time:
/*
	const clock_t begin_time = clock();
		// code...
	const clock_t end_time = clock();
	float ms_time = 1000.0f * float(end_time - begin_time) / (float)CLOCKS_PER_SEC;
	float fps_time = 1000.0f / ms_time;
	std::cout << "time = " << ms_time << " ms,    fps = " << fps_time <<  " fps\n";
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




// main_direct_vision_any(...)
int main_direct_vision_any(int argc, char** argv, bool loop = true, Scene scene = DIRECT_VISION_ANY) {

	// capture data directly from PMD to Frame (FRAME_00_CAPTURE, FRAME_90_CAPTURE)
	float frequency = 100.0f;
	float distance = 0.0f;
	float shutter = 1920.0f;
	char comport[128] = "COM6";
	Frame * frame_00_null = NULL;	// (*frame_00_null) in PMD_params_to_Frame(...), to avoid this Frame meas. FRAME_00_CAPTURE else
	Frame * frame_90_null = NULL;	// (*frame_90_null) in PMD_params_to_Frame(...), to avoid this Frame meas. FRAME_90_CAPTURE else
	std::thread thread_PMD_params_to_Frame (PMD_params_to_Frame_anti_bug_thread, FRAME_00_CAPTURE, FRAME_90_CAPTURE, frequency, distance, shutter, comport, loop);
	
	// Set all the object3D of the corresponding scene
	std::thread thread_set_scene (set_scene_direct_vision_any, loop);
	
	// Render all the object3D of the scene
	std::thread thread_render (render_anti_bug_thread, argc, argv);
	std::cout << "\n\nDo NOT close the openGL window while the PMD loop is runnung.\n(it wouldn't be the end of the world, but it's better not to)\n\n";

	// pause in main to allow control when the loops will finish
	control_loop_pause();

	// joins
	thread_PMD_params_to_Frame.join();
	thread_set_scene.join();
	thread_render.join();

	return 0;
}

// main_direct_vision_wall(...)
int main_direct_vision_wall(int argc, char** argv, bool loop = true, Scene scene = DIRECT_VISION_WALL) {
	
	// capture data directly from PMD to Frame (FRAME_00_CAPTURE, FRAME_90_CAPTURE)
	float frequency = 100.0f;
	float distance = 0.0f;
	float shutter = 1920.0f;
	char comport[128] = "COM6";
	Frame * frame_00_null = NULL;	// (*frame_00_null) in PMD_params_to_Frame(...), to avoid this Frame meas. FRAME_00_CAPTURE else
	Frame * frame_90_null = NULL;	// (*frame_90_null) in PMD_params_to_Frame(...), to avoid this Frame meas. FRAME_90_CAPTURE else
	std::thread thread_PMD_params_to_Frame (PMD_params_to_Frame_anti_bug_thread, FRAME_00_CAPTURE, FRAME_90_CAPTURE, frequency, distance, shutter, comport, loop);
	
	// Set all the object3D of the corresponding scene
	std::thread thread_set_scene (set_scene_direct_vision_wall, loop);
	
	// Render all the object3D of the scene
	std::thread thread_render (render_anti_bug_thread, argc, argv);
	std::cout << "\n\nDo NOT close the openGL window while the PMD loop is runnung.\n(it wouldn't be the end of the world, but it's better not to)\n\n";

	// pause in main to allow control when the loops will finish
	control_loop_pause();

	// joins
	thread_PMD_params_to_Frame.join();
	thread_set_scene.join();
	thread_render.join();

	return 0;
}

// main_diffused_mirror(...)
int main_diffused_mirror(int argc, char** argv, bool loop = true, Scene scene = DIFFUSED_MIRROR) {
	
	// capture data directly from PMD to Frame (FRAME_00_CAPTURE, FRAME_90_CAPTURE)
	/*
	float frequency = 100.0f;
	float distance = 0.0f;
	float shutter = 1920.0f;
	char comport[128] = "COM6";
	Frame * frame_00_null = NULL;	// (*frame_00_null) in PMD_params_to_Frame(...), to avoid this Frame meas. FRAME_00_CAPTURE else
	Frame * frame_90_null = NULL;	// (*frame_90_null) in PMD_params_to_Frame(...), to avoid this Frame meas. FRAME_90_CAPTURE else
	std::thread thread_PMD_params_to_Frame (PMD_params_to_Frame_anti_bug_thread, FRAME_00_CAPTURE, FRAME_90_CAPTURE, frequency, distance, shutter, comport, loop);
	*/
	// Set all the object3D of the corresponding scene
	std::thread thread_set_scene (set_scene_diffused_mirror, false);
	std::cout << "\nHERE 000";
	// Simulation
	get_data_sim_diffused_mirror();
	std::cout << "\nHERE 001";

	// Render all the object3D of the scene
	std::thread thread_render (render_anti_bug_thread, argc, argv);
	std::cout << "\n\nDo NOT close the openGL window while the PMD loop is runnung.\n(it wouldn't be the end of the world, but it's better not to)\n\n";

	// pause in main to allow control when the loops will finish
	control_loop_pause();
	
	// joins
	//thread_PMD_params_to_Frame.join();
	thread_set_scene.join();
	thread_render.join();

	return 0;
}

// main_fov_measurement(...)
int main_fov_measurement(int argc, char** argv, bool loop = true, Scene scene = FOV_MEASUREMENT) {
	
	// capture data directly from PMD to Frame (FRAME_00_CAPTURE, FRAME_90_CAPTURE)
	float frequency = 100.0f;
	float distance = 0.0f;
	float shutter = 1920.0f;
	char comport[128] = "COM6";
	Frame * frame_00_null = NULL;	// (*frame_00_null) in PMD_params_to_Frame(...), to avoid this Frame meas. FRAME_00_CAPTURE else
	Frame * frame_90_null = NULL;	// (*frame_90_null) in PMD_params_to_Frame(...), to avoid this Frame meas. FRAME_90_CAPTURE else
	std::thread thread_PMD_params_to_Frame (PMD_params_to_Frame_anti_bug_thread, FRAME_00_CAPTURE, FRAME_90_CAPTURE, frequency, distance, shutter, comport, loop);
	
	// plot the frame_00
	std::thread thread_plot_frame_fov_measurement (plot_frame_fov_measurement, loop);

	// pause in main to allow control when the loops will finish
	control_loop_pause();

	// joins
	thread_PMD_params_to_Frame.join();
	thread_plot_frame_fov_measurement.join();

	return 0;
}

// main_raw_data(...)
int main_raw_data(int argc, char** argv, char* dir_name_, char* file_name_, Scene scene = RAW_DATA) {
	
	// set all the object3D of the corresponding scene (just camera, laser and wall)
	bool cmx_info = true;
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
	int numtakes = 20;
	std::thread thread_PMD_params_to_file (PMD_params_to_file_anti_bug_thread, frequencies, delays, shutters_float, dir_name_, file_name_, comport, numtakes, cmx_info, cmx_params);

	// pause in main to allow control when the loops will finish
	//control_loop_pause();

	// joins
	thread_PMD_params_to_file.join();

	return 0;
}


// main_calibration_matrix(...)
// It builds a .cmx file from a .raw file with the parameters stored in the .inf file
int main_calibration_matrix (int argc, char** argv, char* dir_name_, char* file_name_, Scene scene = CALIBRATION_MATRIX) {
	
	// built the global Info instance INFO
	Info info = Info(dir_name_, file_name_);

	// create the .cmx file, dealing with the .inf and .raw files. It internally creates the corresponding scene
	std::thread thread_create_cmx_from_raw (create_cmx_from_raw_anti_bug_thread, &info);

	// pause in main to allow control when the loops will finish
	//control_loop_pause();

	// joins
	thread_create_cmx_from_raw.join();

	return 0;
}



// MAIN
// Observation: PMD captures at 9.5 fps with the current set-up (2014-08-05):
//     100 captures (in loop, with pmd always opened) take aprox 10.5 seconds,
//     tested with both PMD_params_to_file(...) and PMD_params_to_Frame(...)
int main(int argc, char** argv) {
	
	// ------------------------------------------------------------------------------------------------------------------------------
	// EXECUTING:
	// cd C:\Users\transient\Documents\Visual Studio 2012\Projects\DiffuseMirrors2\x64\Release
	// DiffuseMirrors2.exe "80 90 100" "0 1 2 3" "1920" f:\tmp\pmdtest2 PMD_test_meas COM6 1
	// ------------------------------------------------------------------------------------------------------------------------------

	Scene scene = RAW_DATA_AND_CALIBRATION_MATRIX;	// DIRECT_VISION_WALL, DIRECT_VISION_ANY, DIRECT_VISION_ANY_SIMULATION, DIFFUSED_MIRROR, FOV_MEASUREMENT, RAW_DATA, CALIBRATION_MATRIX, UNKNOWN_SCENE, TEST, RAW_DATA_AND_CALIBRATION_MATRIX
	char dir_name[1024] = "F:\\Jaime\\CalibrationMatrix\\test_03";
	char file_name[1024] = "PMD";
	bool loop = true;
	
	switch(scene) {
		case DIRECT_VISION_WALL : main_direct_vision_wall (argc, argv, loop, scene);                break;
		case DIRECT_VISION_ANY  : main_direct_vision_any  (argc, argv, loop, scene);                break;
		case DIFFUSED_MIRROR    : main_diffused_mirror    (argc, argv, loop, scene);                break;
		case FOV_MEASUREMENT    : main_fov_measurement    (argc, argv, loop, scene);                break;
		case RAW_DATA           : main_raw_data           (argc, argv, dir_name, file_name, scene); break;
		case CALIBRATION_MATRIX : main_calibration_matrix (argc, argv, dir_name, file_name, scene); break;
		case RAW_DATA_AND_CALIBRATION_MATRIX: main_raw_data (argc, argv, dir_name, file_name, scene);
			                                  main_calibration_matrix (argc, argv, dir_name, file_name, scene); break;
		case TEST				: test();                                                           break;
	}

	system("pause");
	return 0;
}


