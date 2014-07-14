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
// http://eigen.tuxfamily.org/dox/group__QuickRefPage.html
#include <Eigen/Dense>
using namespace Eigen;


// MAIN
int main(int argc, char** argv) {
	
	// ------------------------------------------------------------------------------------------------------------------------------
	// EXECUTING:
	// cd C:\Users\transient\Documents\Visual Studio 2012\Projects\DiffuseMirrors2\x64\Release
	// DiffuseMirrors2.exe "80 90 100" "0 1 2 3" "1920" f:\tmp\pmdtest2 PMD_test_meas COM6 1
	// ------------------------------------------------------------------------------------------------------------------------------

	bool scene_simple = false;

	// tests some functions
	//test();
	//return 0;
	
	// capture data from PMD
	if (capturetoolDM2_main(argc, argv) != 0) {
		system("pause");
		return 0;
	}

	// Read data from file
	if (data_read_main() != 0) {
		system("pause");
		return 0;
	}

	// Set all the object3D of the scene
	if (!scene_simple)
		set_scene();
	else
		set_scene_simple();
	
	// Get all the data of the model
	if (!scene_simple)
		get_data_sim();
	else
		get_data_sim_simple();

	// Render all the object3D of the scene
	render(argc, argv);

	system("pause");
	return 0;
}


