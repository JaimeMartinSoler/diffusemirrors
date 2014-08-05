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

	Scene scene = DIFFUSED_MIRROR;	// DIRECT_VISION_WALL,    DIRECT_VISION_ANY,    DIFFUSED_MIRROR,    UNKNOWN_SCENE

	// tests some functions
	//test();
	//return 0;
	
	// capture data from PMD
	std::thread thread_capturetoolDM2_main (capturetoolDM2_main, argc, argv);
	/*
	if (capturetoolDM2_main(argc, argv) != 0) {
		system("pause");
		return 0;
	}
	*/

	// Read data from file (.dat)
	// This just makes sense if capturetoolDM2_main(...) generated some data to a file,
	// if we are capturing data directly from the PMD into DataPMD or Frame variables there is nothing to do with data_read_main(),
	// unless we want to read from a previous measurement, for example one storing the calibration matrix
	if (data_read_main() != 0) {
		system("pause");
		return 0;
	}
	std::cout << "data_read_main() done\n";

	// Set all the object3D of the corresponding scene
	if (scene == DIRECT_VISION_WALL)
		set_scene_direct_vision_wall();
	else if (scene == DIFFUSED_MIRROR)
		set_scene_diffused_mirror();
	std::cout << "set_scene() done\n";
	
	// Get all the data of the model from the corresponding simulation
	if (scene == DIRECT_VISION_WALL)
		get_data_sim_direct_vision_wall();
	else if (scene == DIFFUSED_MIRROR)
		get_data_sim_diffused_mirror();
	std::cout << "get_data() done\n";

	// Render all the object3D of the scene
	//render(argc, argv);
	std::thread thread_render (render, argc, argv);
	std::cout << "\n\nDo NOT close the openGL window while the PMD loop is runnung.\n(it wouldn't be the end of the world, but it's better not to)\n\n";

	// Pause till the threads have ended
	char end_loop[1024];	end_loop[0] = 'n';
	while ((end_loop[0] != 'y') && (end_loop[0] != 'Y')) {
		std::cout << "\nDo you want to finish the PMD (and this) loop? (y/n)\n";
		std::cin >> end_loop;
	}
	PMD_LOOP_ENABLE = false;
	thread_capturetoolDM2_main.join();
	thread_render.join();

	system("pause");
	return 0;
}


