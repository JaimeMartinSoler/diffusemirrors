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
	float frequency = 75.0f;
	float distance = 0.0f;
	float shutter = 1920.0f;
	char comport[128] = "COM6";
	bool loop = true;
	PixStoring ps = PIXELS_STORING_GLOBAL;
	bool pSim = false;
	//int* opt = NULL;
	int opt[2];
	opt[0] = 1;		// avg_size: output frame is the average of the last avg_size frames
	opt[1] = 1;		// update_size: output frame is updated each update_size frames
	std::thread thread_PMD_params_to_Frame(PMD_params_to_Frame_anti_bug_thread, std::ref(FRAME_00_CAPTURE), std::ref(FRAME_90_CAPTURE), frequency, distance, shutter, comport, loop, ps, pSim, opt);

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
	
	std::cout << "\nmain_DirectVision_Sinusoid() ended.";

	return 0;
}

// Direct Vision problem, Simulation, uses Calibration Matrix. Real Time.
int main_DirectVision_Simulation(char* dir_name_, char* file_name_) {

	// set the Info
	Info info(dir_name_, file_name_);

	// capture data directly from PMD to Frame (FRAME_00_CAPTURE, FRAME_90_CAPTURE)
	float frequency = 50.0f;
	float distance = 0.0f;
	float shutter = 1920.0f;
	char comport[128] = "COM6";
	bool loop = true;
	PixStoring ps = PIXELS_STORING_GLOBAL;
	bool pSim = true;
	int* opt = NULL;
	//int opt[2];
	//opt[0] = 10;	// avg_size: output frame is the average of the last avg_size frames
	//opt[1] = 1;	// update_size: output frame is updated each update_size frames
	std::thread thread_PMD_params_to_Frame(PMD_params_to_Frame_anti_bug_thread, std::ref(FRAME_00_CAPTURE), std::ref(FRAME_90_CAPTURE), frequency, distance, shutter, comport, loop, ps, pSim, opt);

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
	float pathWallOffset = 1.0f;
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
	updatePixelPatches_Simulation_antiBugThread(std::ref(info), std::ref(SCENEMAIN), std::ref(frame00), std::ref(frame90), loop, ps, pSim);

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
	float frequency = 50.0f;	
	float distance = 0.0f; //* frequency / 50.0f;	// 0.2f works fine for 50.0f MHz
	float shutter = 1920.0f;
	char comport[128] = "COM6";
	bool loop = true;
	PixStoring ps = PIXELS_STORING_GLOBAL;
	bool pSim = true;
	// set frames with parameters and stub data:
	FRAME_00_CAPTURE.set(info, ps, pSim, 0, 0, frequency, distance, shutter, 0.0f);
	FRAME_90_CAPTURE.set(info, ps, pSim, 0, 0, frequency, distance, shutter, 90.0f);
	int opt[2];
	opt[0] = 10;	// avg_size: output frame is the average of the last avg_size frames
	opt[1] = 10;	// update_size: output frame is updated each update_size frames
	std::thread thread_PMD_params_to_Frame(PMD_params_to_Frame_anti_bug_thread, std::ref(FRAME_00_CAPTURE), std::ref(FRAME_90_CAPTURE), frequency, distance, shutter, comport, loop, ps, pSim, opt);

	// Set all the corresponding scene and start updating
	// faces
	const int vop_faces = 6;	// 1:Plane, 6:Box
	std::vector<int> rowsPerFaceV(vop_faces);
	std::vector<int> colsPerFaceV(vop_faces);
	/*
	for (int f = FRONT; f < vop_faces; ++f) {
		rowsPerFaceV[f] = 100;
		colsPerFaceV[f] = 100;
	}
	*/
	// Point vopS(0.584f, 0.505f, 0.399f);	// manual measurement
	int mul = 1;
	rowsPerFaceV[FRONT]  = 5 * mul;	colsPerFaceV[FRONT]  = 6 * mul;
	rowsPerFaceV[BACK]   = 5 * mul;	colsPerFaceV[BACK]   = 6 * mul;
	rowsPerFaceV[RIGHT]  = 5 * mul;	colsPerFaceV[RIGHT]  = 4 * mul;
	rowsPerFaceV[LEFT]   = 5 * mul;	colsPerFaceV[LEFT]   = 4 * mul;
	rowsPerFaceV[BOTTOM] = 4 * mul;	colsPerFaceV[BOTTOM] = 6 * mul;
	rowsPerFaceV[TOP]    = 4 * mul;	colsPerFaceV[TOP]    = 6 * mul;
	/*
	rowsPerFaceV[FRONT]  = mul;	colsPerFaceV[FRONT]  = mul;
	rowsPerFaceV[BACK]   = mul;	colsPerFaceV[BACK]   = mul;
	rowsPerFaceV[RIGHT]  = mul;	colsPerFaceV[RIGHT]  = mul;
	rowsPerFaceV[LEFT]   = mul;	colsPerFaceV[LEFT]   = mul;
	rowsPerFaceV[BOTTOM] = mul;	colsPerFaceV[BOTTOM] = mul;
	rowsPerFaceV[TOP]    = mul;	colsPerFaceV[TOP]    = mul;
	*/
	
	SCENEMAIN.setScene_Occlusion(rowsPerFaceV, colsPerFaceV, ps, pSim);
	std::thread thread_updateVolumePatches_Occlusion(updateVolumePatches_Occlusion_antiBugThread, std::ref(info), std::ref(SCENEMAIN), std::ref(FRAME_00_CAPTURE), std::ref(FRAME_90_CAPTURE), std::ref(rowsPerFaceV), std::ref(colsPerFaceV), loop, ps, pSim);
	//std::thread thread_updateVolumePatches_Occlusion(updateVolumePatches_Occlusion_OLD_antiBugThread, std::ref(info), std::ref(SCENEMAIN), std::ref(FRAME_00_CAPTURE), std::ref(FRAME_90_CAPTURE), loop, ps, pSim);

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

	// get a Frame from the RawData
	PixStoring ps = PIXELS_VALID;
	bool pSim = true;
	float scale = -1.0f;
	if (pSim)
		scale = (int)(CAMERA_PIX_X / (float)PMD_SIM_COLS);
	/*
	RawData rawData(info);
	float pathWallOffset = 0.0f;
	int dist_idx = get_dist_idx(info, pathWallOffset);	// returns -1 if no idx correspondance was found
	if (dist_idx < 0) {
		std::cout << "\nWarning: Distance Offset = " << pathWallOffset << " is not a dist in .cmx distV = ";
		print(info.distV);
		return -1;
	}
	*/
	int dist_idx = 0;
	Info infoVideo("F:\\Jaime\\CalibrationMatrix\\video_occlusion_000", "PMD_video_occlusion");
	RawData rawDataVideo(infoVideo);
	Frame frame00(rawDataVideo, infoVideo.freqV.size() - 5, dist_idx, infoVideo.shutV.size() - 1, 0, ps, pSim);
	Frame frame90(rawDataVideo, infoVideo.freqV.size() - 5, dist_idx, infoVideo.shutV.size() - 1, 1, ps, pSim);
	std::cout << "\nHERE 002";
	UPDATED_NEW_FRAME = true;
	//frame00.plot();

	// Set all the corresponding scene and start updating
	bool loop = false;
	// faces
	const int vop_faces = 6;	// 1:Plane, 6:Box
	std::vector<int> rowsPerFaceV(vop_faces);
	std::vector<int> colsPerFaceV(vop_faces);
	for (int f = FRONT; f < vop_faces; ++f) {
		rowsPerFaceV[f] = 4;
		colsPerFaceV[f] = 4;
	}
	frame00.plot(1, false, "R00", scale);
	frame90.plot(1, false, "R90", scale);
	Info info(dir_name_, file_name_);
	SCENEMAIN.setScene_Occlusion(rowsPerFaceV, colsPerFaceV, ps, pSim);
	std::thread thread_updateVolumePatches_Occlusion(updateVolumePatches_Occlusion_antiBugThread, std::ref(info), std::ref(SCENEMAIN), std::ref(frame00), std::ref(frame90), std::ref(rowsPerFaceV), std::ref(colsPerFaceV), loop, ps, pSim);
	//std::thread thread_updateVolumePatches_Occlusion(updateVolumePatches_Occlusion_OLD_antiBugThread, std::ref(info), std::ref(SCENEMAIN), std::ref(FRAME_00_CAPTURE), std::ref(FRAME_90_CAPTURE), loop, ps, pSim);

	// Render all the object3D of the scene
	int argcStub = 0;
	char** argvStub = NULL;
	std::thread thread_render(render_anti_bug_thread, argcStub, argvStub);
	std::cout << "\n\nDo NOT close the openGL window while the PMD loop is runnung.\n(it wouldn't be the end of the world, but it's better not to)\n\n";

	// pause in main to allow control when the loops will finish
	control_loop_pause();

	// joins
	thread_updateVolumePatches_Occlusion.join();
	thread_render.join();

	return 0;
}





// Raw Data Measurement
int main_RawData(char* dir_name_, char* file_name_) {
	
	// set all the object3D of the corresponding scene (just camera, laser and wall)
	bool cmx_info = true;	// pure raw data is not cmx oriented, but it's a good idea to store the corresponding info anyway
	float laser_to_cam_offset_x = -0.140f;
	float laser_to_cam_offset_y = 0.00f;
	float laser_to_cam_offset_z = -0.05;
	float dist_wall_cam = 1.5f;
	float cmx_params[4] = {laser_to_cam_offset_x, laser_to_cam_offset_y, laser_to_cam_offset_z, dist_wall_cam};

	// create the .raw file, capturing data from the PMD

	// FREQUENCIES
	std::vector<float> frequencies;	// (MHz)
	frequencies.push_back(1.0f);
	frequencies.push_back(5.0f);
	frequencies.push_back(10.0f);
	frequencies.push_back(25.0f);
	frequencies.push_back(50.0f);
	frequencies.push_back(75.0f);
	frequencies.push_back(100.0f);
	/*
	float freq_res = 10.0f;
	float freq_min = 10.0f;
	float freq_max = 100.0f + freq_res/2.0f;	// (+ freq_res/2.0f) is due to avoid rounding problems
	for (float fi = freq_min; fi <= freq_max; fi += freq_res)
		frequencies.push_back(fi);
	*/

	// DISTANCES
	std::vector<float> delays;		// (m)
	float delay_res = 0.1f;
	float delay_min = -2.5f;
	float delay_max = 20.0f + delay_res/2.0f;	// (+ delay_res/2.0f) is due to avoid rounding problems
	for (float di = delay_min; di <= delay_max; di += delay_res)
		delays.push_back(di);

	// SUTTERS	//std::vector<float> shutters_float(1, 1920.0f);	// (us)
	std::vector<float> shutters_float;		// (m)
	float shutters_float_mul = 2.0f;	if (shutters_float_mul <= 1.0f)	{ shutters_float_mul = 2.0f; }
	float shutters_float_min = SHUTTER_MAX / 1.0f;	// 15(/128), 30(/64), 60(/32), 120(/16), 240(/8), 480(/4), 960(/2), 1920(/1)
	float shutters_float_max = SHUTTER_MAX + (SHUTTER_MIN / 2.0f);	// (+ SHUTTER_MIN/2.0f) is due to avoid rounding problems
	for (float si = shutters_float_min; si <= shutters_float_max; si *= shutters_float_mul)
		shutters_float.push_back(si);

	// NUMTAKES
	int numtakes = 20;

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
	float frequency = 25.0f;
	float distance = 0.0f;
	float shutter = 1920.0f / 32.0f; // 1920.0f;
	char comport[128] = "COM6";
	bool loop = true;
	PixStoring ps = PIXELS_TOTAL;
	bool pSim = false;
	int* opt = NULL;
	//int opt[2];
	//opt[0] = 10;	// avg_size: output frame is the average of the last avg_size frames
	//opt[1] = 1;	// update_size: output frame is updated each update_size frames
	std::thread thread_PMD_params_to_Frame(PMD_params_to_Frame_anti_bug_thread, std::ref(FRAME_00_CAPTURE), std::ref(FRAME_90_CAPTURE), frequency, distance, shutter, comport, loop, ps, pSim, opt);
	
	// plot the Frame (combining frame00 and frame90)
	char windowName[128] = "Frame FoV";
	bool line_center = true;
	int lines_grid = 16;
	Frame frameStub;	frameStub.set();
	std::thread thread_plot_frame_fov_measurement (plot_frame_fov_measurement, std::ref(FRAME_00_CAPTURE), std::ref(FRAME_90_CAPTURE), loop, false, windowName, line_center, lines_grid);//(char*)NULL);

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
	SceneType sceneType = RAW_DATA;
	SCENEMAIN.set(sceneType);
	//char dir_name[1024] = "C:\\Users\\Natalia\\Documents\\Visual Studio 2013\\Projects\\DiffuseMirrors2\\CalibrationMatrix\\cmx_01";
	char dir_name[1024] = "F:\\Jaime\\CalibrationMatrix\\cmx_04";
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
	
	std::cout << "\n\n";
	system("pause");
	return 0;
}


