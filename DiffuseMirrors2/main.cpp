/*
Mater Thesis
Jaime Martin
*/

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
	std::cout << "OK, finishing loops...\n\n";
}




// main_direct_vision_any(...)
int main_direct_vision_any(int argc, char** argv, bool loop = true, Scene scene = DIRECT_VISION_ANY) {

	// capture data from PMD
	std::thread thread_capturetoolDM2_main (capturetoolDM2_main, argc, argv, loop);
	
	// Set all the object3D of the corresponding scene
	std::thread thread_set_scene (set_scene, scene, loop);
	
	// Render all the object3D of the scene
	std::thread thread_render (render, argc, argv);
	std::cout << "\n\nDo NOT close the openGL window while the PMD loop is runnung.\n(it wouldn't be the end of the world, but it's better not to)\n\n";

	// pause in main to allow control when the loops will finish
	control_loop_pause();

	// joins
	thread_capturetoolDM2_main.join();
	thread_set_scene.join();
	thread_render.join();

	return 0;
}

// main_direct_vision_wall(...)
int main_direct_vision_wall(int argc, char** argv, bool loop = true, Scene scene = DIRECT_VISION_WALL) {

	// capture data from PMD
	std::thread thread_capturetoolDM2_main (capturetoolDM2_main, argc, argv, loop);
	
	// Set all the object3D of the corresponding scene
	std::thread thread_set_scene (set_scene, scene, loop);
	
	// Render all the object3D of the scene
	std::thread thread_render (render, argc, argv);
	std::cout << "\n\nDo NOT close the openGL window while the PMD loop is runnung.\n(it wouldn't be the end of the world, but it's better not to)\n\n";

	// pause in main to allow control when the loops will finish
	control_loop_pause();

	// joins
	thread_capturetoolDM2_main.join();
	thread_set_scene.join();
	thread_render.join();

	return 0;
	
	return 0;
}

// main_diffused_mirror(...)
int main_diffused_mirror(int argc, char** argv, bool loop = true, Scene scene = DIFFUSED_MIRROR) {

	// capture data from PMD
	std::thread thread_capturetoolDM2_main (capturetoolDM2_main, argc, argv, loop);
	
	// Set all the object3D of the corresponding scene
	std::thread thread_set_scene (set_scene, scene, loop);
	
	// Simulation
	get_data_sim_diffused_mirror();

	// Render all the object3D of the scene
	std::thread thread_render (render, argc, argv);
	std::cout << "\n\nDo NOT close the openGL window while the PMD loop is runnung.\n(it wouldn't be the end of the world, but it's better not to)\n\n";

	// pause in main to allow control when the loops will finish
	control_loop_pause();

	return 0;
}

// main_direct_vision_any(...)
int main_fov_measurement(int argc, char** argv, bool loop = true, Scene scene = FOV_MEASUREMENT) {

	// capture data from PMD
	std::thread thread_capturetoolDM2_main (capturetoolDM2_main, argc, argv, loop);
	
	// plot the frame_00
	std::thread thread_plot_frame_fov_measurement (plot_frame_fov_measurement, loop);

	// pause in main to allow control when the loops will finish
	control_loop_pause();

	// joins
	thread_capturetoolDM2_main.join();
	thread_plot_frame_fov_measurement.join();

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

	// tests some functions
	//test();
	//return 0;

	Scene scene = DIRECT_VISION_WALL;	// DIRECT_VISION_WALL,    DIRECT_VISION_ANY,    DIFFUSED_MIRROR,    FOV_MEASUREMENT,    CALIBRATION_MATRIX,    UNKNOWN_SCENE
	bool loop = true;
	
	switch(scene) {
		case DIRECT_VISION_WALL: main_direct_vision_wall(argc, argv, loop, scene); break;
		case DIRECT_VISION_ANY : main_direct_vision_any (argc, argv, loop, scene); break;
		case DIFFUSED_MIRROR   : main_diffused_mirror   (argc, argv, loop, scene); break;
		case FOV_MEASUREMENT   : main_fov_measurement   (argc, argv, loop, scene); break;
	}

	system("pause");
	return 0;
}


