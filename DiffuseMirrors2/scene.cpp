

#include <vector>
#include <algorithm>    // std::min

#include "scene.h"
#include "shapes.h"
#include "global.h"
#include "data_read.h"
// OPENCV INCLUDES
#include <cv.h>
#include <cvaux.h>
#include <highgui.h>
// http://eigen.tuxfamily.org/dox/group__QuickRefPage.html
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;


// sets the scene depending on the Scene passed as argument
void set_scene(Scene scene, bool loop) {	// by default: loop = false

	if (scene == DIRECT_VISION_ANY)
		set_scene_direct_vision_any (loop);
	else if (scene == DIRECT_VISION_WALL)
		set_scene_direct_vision_wall(loop);
	else if (scene == DIFFUSED_MIRROR)
		set_scene_diffused_mirror(loop);
	//else if (scene == CALIBRATION_MATRIX)
	//	set_scene_calibration_matrix();
}


// sets all the Diffused Mirror (occluded wall) scene
void set_scene_diffused_mirror(bool loop) {	// by default: loop = false

	// CAMERA (0)
	Point camera_pos(0.0f, 0.75f, 0.0f);		// pos of the center of the camera
	Point camera_rot(0.0f, 180.0f, 0.0f);		// rot from the center of the camera
	Point camera_size(0.15f, 0.15f, 0.04f);
	Point camera_centre(camera_size.x() / 2.0f, camera_size.y() / 2.0f, 0.0f);	// centre relative to the first point
	set_camera(&camera_pos, &camera_rot, &camera_size, &camera_centre);
	// Get the screen normals:
	// Ordering: 1st row: col, col, col... 2nd row: col, col, col... from left to right, from bottom to top
	std::vector<Point*> screen_patches_corners_normals((CAMERA_PIX_X + 1) * (CAMERA_PIX_Y + 1));	// size is always (CAMERA_PIX_X + 1) * (CAMERA_PIX_Y + 1), regardingless the value of PIXELS_STORING_GLOBAL
	std::vector<Point*> screen_patches_centers_normals(CAMERA_PIX_X * CAMERA_PIX_Y);				// size is always (CAMERA_PIX_X)     * (CAMERA_PIX_Y)    , regardingless the value of PIXELS_STORING_GLOBAL
	set_screen_normals_pixel_patches(screen_patches_corners_normals, screen_patches_centers_normals, &camera_pos, &camera_rot, &camera_centre);

	// LASER (1)
	Point laser_pos(-0.15f, 0.75f, 0.0f);		// pos of the center of the laser
	Point laser_rot(0.0f, 180.0f, 0.0f);		// rot from the center of the laser
	Point laser_size(0.1f, 0.1f, 0.3f);
	Point laser_centre(laser_size.x() / 2.0f, laser_size.y() / 2.0f, 0.0f);	// centre relative to the first point
	set_laser(&laser_pos, &laser_rot, &laser_size, &laser_centre);

	// WALL (2)
	Point wall_pos(-1.0f, 0.0f, -1.5f);			// pos of the center of the wall
	Point wall_rot(0.0f, 0.0f, 0.0f);			// rot from the center of the wall
	Point wall_size(6.0f, 3.0f, 0.2f);
	Point wall_centre(0.0f, 0.0f, 0.0f);		// centre relative to the first point
	float wall_albedo = 0.5f;	// does not matter
	set_box(&wall_pos, &wall_rot, &wall_size, &wall_centre, WALL, wall_albedo);

	// OCCLUDER (3)
	Point occluder_pos(1.5f, 0.0f, 0.7f);		// pos of the center of the occluder
	Point occluder_rot(0.0f, 90.0f, 0.0f);		// rot from the center of the occluder
	Point occluder_size(1.0f, 3.0f, 0.05f);
	Point occluder_centre(0.0f, 0.0f, 0.0f);	// centre relative to the first point
	float occluder_albedo = 0.1f;	// does not matter
	set_box(&occluder_pos, &occluder_rot, &occluder_size, &occluder_centre, OCCLUDER, occluder_albedo);

	// FLOOR (4)
	Point floor_pos(wall_pos.x(), 0.0f, occluder_pos.z());
	Point floor_rot(-90.0f, 0.0f, 0.0f);
	Point floor_size(wall_size.x(), floor_pos.z() - wall_pos.z() + wall_size.z(), wall_size.z());
	Point floor_centre(0.0f, 0.0f, 0.0f);
	float floor_albedo = 0.5f;	// does not matter
	set_box(&floor_pos, &floor_rot, &floor_size, &floor_centre, FLOOR, floor_albedo);

	// VOLUME (5)
	Point volume_pos(3.5f, 0.3f, -0.6f);		// pos of the center of the volume
	Point volume_rot(0.0f, 180.0f, 0.0f);		// rot from the center of the volume
	Point volume_size(1.0f, 1.0f, 1.0f);
	Point volume_centre(0.0f, 0.0f, 0.0f);		// centre relative to the first point
	float volume_albedo = 1.0f;	// does not matter
	set_box(&volume_pos, &volume_rot, &volume_size, &volume_centre, VOLUME, volume_albedo);
	(*(*OBJECT3D_SET[VOLUME])[0]).change_shape(LINE);

	// WALL_PATCHES (6)
	std::vector<float> wall_patches_albedo(CAMERA_PIX_X * CAMERA_PIX_Y);	// size is always (CAMERA_PIX_X) * (CAMERA_PIX_Y), regardingless the value of PIXELS_STORING_GLOBAL
	set_wall_patches_albedo(wall_patches_albedo);
	set_wall_patches(&camera_pos, &camera_rot, &camera_size, &camera_centre, screen_patches_corners_normals, screen_patches_centers_normals, wall_patches_albedo);

	// CAMERA_FOV (7)
	set_camera_fov(&camera_pos, screen_patches_corners_normals);

	// LASER_RAY (8)
	set_laser_ray();

	// VOLUME_PATCHES (9)
	std::vector<float>  volume_patches_albedo(VOLUME_GRID_SIZE_X * VOLUME_GRID_SIZE_Y * VOLUME_GRID_SIZE_Z);
	std::vector<Point*> volume_patches_rot   (VOLUME_GRID_SIZE_X * VOLUME_GRID_SIZE_Y * VOLUME_GRID_SIZE_Z);
	std::vector<bool>   volume_patches_bool(VOLUME_GRID_SIZE_X * VOLUME_GRID_SIZE_Y * VOLUME_GRID_SIZE_Z);
	set_volume_patches_params(volume_patches_albedo, volume_patches_rot, volume_patches_bool);
	set_volume_patches(&volume_pos, &volume_rot, &volume_size, &volume_centre,
					   volume_patches_albedo, volume_patches_rot, volume_patches_bool);

	// PIXEL_PATCHES (10) // empty
	Object3D* pixel_patches_obj3D = new Object3D(0);
	OBJECT3D_SET[PIXEL_PATCHES] = pixel_patches_obj3D;
}


// sets all the Direct Vision Wall scene
void set_scene_direct_vision_wall(bool loop) {	// by default: loop = false

	// CAMERA (0)
	Point camera_pos(0.0f, 0.75f, 0.0f);		// pos of the center of the camera
	Point camera_rot(0.0f, 180.0f, 0.0f);		// rot from the center of the camera
	Point camera_size(0.15f, 0.15f, 0.04f);
	Point camera_centre(camera_size.x() / 2.0f, camera_size.y() / 2.0f, 0.0f);	// centre relative to the first point
	set_camera(&camera_pos, &camera_rot, &camera_size, &camera_centre);
	// Get the screen normals:
	// Ordering: 1st row: col, col, col... 2nd row: col, col, col... from left to right, from bottom to top
	std::vector<Point*> screen_patches_corners_normals((CAMERA_PIX_X + 1) * (CAMERA_PIX_Y + 1));	// size is always (CAMERA_PIX_X + 1) * (CAMERA_PIX_Y + 1), regardingless the value of PIXELS_STORING_GLOBAL
	std::vector<Point*> screen_patches_centers_normals(CAMERA_PIX_X * CAMERA_PIX_Y);				// size is always (CAMERA_PIX_X)     * (CAMERA_PIX_Y)    , regardingless the value of PIXELS_STORING_GLOBAL
	set_screen_normals_pixel_patches(screen_patches_corners_normals, screen_patches_centers_normals, &camera_pos, &camera_rot, &camera_centre);

	// LASER (1)
	Point laser_pos(-0.15f, 0.75f, 0.0f);		// pos of the center of the laser
	Point laser_rot(0.0f, 180.0f, 0.0f);		// rot from the center of the laser
	Point laser_size(0.1f, 0.1f, 0.3f);
	Point laser_centre(laser_size.x() / 2.0f, laser_size.y() / 2.0f, 0.0f);	// centre relative to the first point
	set_laser(&laser_pos, &laser_rot, &laser_size, &laser_centre);

	// WALL (2)
	Point wall_size(2.0f, 1.5f, 0.05f);	// Point wall_size(4.0f, 2.5f, 0.2f);
	Point wall_pos(-1.625f + 2.0f, 0.2f + 0.75f, -2.0f);	// pos of the center of the wall
	Point wall_rot(0.0f, 0.0f, 0.0f);												// rot from the center of the wall
	Point wall_centre(wall_size.x()/2.0f, wall_size.y()/2.0f, 0.0f);		// centre relative to the first point
	float wall_albedo = 0.5f;	// does not matter
	set_box(&wall_pos, &wall_rot, &wall_size, &wall_centre, WALL, wall_albedo);

	// OCCLUDER (3) // empty
	Object3D* occluder_obj3D = new Object3D(0);
	OBJECT3D_SET[OCCLUDER] = occluder_obj3D;

	// FLOOR (4)
	Point floor_pos(-1.625f, 0.0f, 0.7f);
	Point floor_rot(-90.0f, 0.0f, 0.0f);
	Point floor_size(4.0f, 6.0f, 0.2f);
	Point floor_centre(0.0f, 0.0f, 0.0f);
	float floor_albedo = 0.5f;	// does not matter
	set_box(&floor_pos, &floor_rot, &floor_size, &floor_centre, FLOOR, floor_albedo);

	// VOLUME (5) // empty
	Object3D* volume_obj3D = new Object3D(0);
	OBJECT3D_SET[VOLUME] = volume_obj3D;

	// WALL_PATCHES (6)
	//Object3D* wall_patches_obj3D = new Object3D(0);
	//OBJECT3D_SET[WALL_PATCHES] = wall_patches_obj3D;
	std::vector<float> wall_patches_albedo(CAMERA_PIX_X * CAMERA_PIX_Y);	// size is always (CAMERA_PIX_X) * (CAMERA_PIX_Y), regardingless the value of PIXELS_STORING_GLOBAL
	set_wall_patches_albedo(wall_patches_albedo);
	set_wall_patches(&camera_pos, &camera_rot, &camera_size, &camera_centre, screen_patches_corners_normals, screen_patches_centers_normals, wall_patches_albedo);

	// CAMERA_FOV (7)
	set_camera_fov(&camera_pos, screen_patches_corners_normals);

	// LASER_RAY (8) // empty
	Object3D* laser_ray_obj3D = new Object3D(0);
	OBJECT3D_SET[LASER_RAY] = laser_ray_obj3D;

	// VOLUME_PATCHES (9) // empty
	Object3D* volume_patches_obj3D = new Object3D(0);
	OBJECT3D_SET[VOLUME_PATCHES] = volume_patches_obj3D;

	// PIXEL_PATCHES (10) // empty
	Object3D* pixel_patches_obj3D = new Object3D(0);
	OBJECT3D_SET[PIXEL_PATCHES] = pixel_patches_obj3D;

	// PROVISIONAL:
	// second: set_pixel_patches(...) actually
	set_pixel_patches(&camera_pos, &camera_rot, &camera_centre, screen_patches_corners_normals, screen_patches_centers_normals);
	// UPDATE PIXEL PATCHES AND WALL IF LOOP
	int r_div = 4;
	int c_div = 5;
	if (loop) {
		update_wall_and_pixel_patches(&camera_pos, &camera_rot, &camera_centre, screen_patches_corners_normals, screen_patches_centers_normals, r_div, c_div, loop);
	}
}


// sets all the Direct Vision Any scene
void set_scene_direct_vision_any (bool loop) {	// by default: loop = false
	
	// CAMERA (0)
	Point camera_pos(0.0f, 0.75f, 0.0f);		// pos of the center of the camera
	Point camera_rot(0.0f, 180.0f, 0.0f);		// rot from the center of the camera
	Point camera_size(0.15f, 0.15f, 0.04f);
	Point camera_centre(camera_size.x() / 2.0f, camera_size.y() / 2.0f, 0.0f);	// centre relative to the first point
	set_camera(&camera_pos, &camera_rot, &camera_size, &camera_centre);
	// Get the screen normals:
	// Ordering: 1st row: col, col, col... 2nd row: col, col, col... from left to right, from bottom to top
	std::vector<Point*> screen_patches_corners_normals((CAMERA_PIX_X + 1) * (CAMERA_PIX_Y + 1));	// size is always (CAMERA_PIX_X + 1) * (CAMERA_PIX_Y + 1), regardingless the value of PIXELS_STORING_GLOBAL
	std::vector<Point*> screen_patches_centers_normals(CAMERA_PIX_X * CAMERA_PIX_Y);				// size is always (CAMERA_PIX_X)     * (CAMERA_PIX_Y)    , regardingless the value of PIXELS_STORING_GLOBAL
	set_screen_normals_pixel_patches(screen_patches_corners_normals, screen_patches_centers_normals, &camera_pos, &camera_rot, &camera_centre);

	// LASER (1)
	Point laser_pos(-0.15f, 0.75f, 0.0f);		// pos of the center of the laser
	Point laser_rot(0.0f, 180.0f, 0.0f);		// rot from the center of the laser
	Point laser_size(0.1f, 0.1f, 0.3f);
	Point laser_centre(laser_size.x() / 2.0f, laser_size.y() / 2.0f, 0.0f);	// centre relative to the first point
	set_laser(&laser_pos, &laser_rot, &laser_size, &laser_centre);

	// WALL (2)
	Point wall_pos(-1.625f, 0.0f, -1.36f);		// pos of the center of the wall
	Point wall_rot(0.0f, 0.0f, 0.0f);			// rot from the center of the wall
	Point wall_size(4.0f, 0.01f, 0.2f);
	Point wall_centre(0.0f, 0.0f, 0.0f);		// centre relative to the first point
	float wall_albedo = 0.5f;	// does not matter
	set_box(&wall_pos, &wall_rot, &wall_size, &wall_centre, WALL, wall_albedo);

	// OCCLUDER (3) // empty
	Object3D* occluder_obj3D = new Object3D(0);
	OBJECT3D_SET[OCCLUDER] = occluder_obj3D;

	// FLOOR (4)
	Point floor_pos(wall_pos.x(), 0.0f, 0.7f);
	Point floor_rot(-90.0f, 0.0f, 0.0f);
	Point floor_size(wall_size.x(), 6.0f, wall_size.z());
	Point floor_centre(0.0f, 0.0f, 0.0f);
	float floor_albedo = 0.5f;	// does not matter
	set_box(&floor_pos, &floor_rot, &floor_size, &floor_centre, FLOOR, floor_albedo);

	// VOLUME (5) // empty
	Object3D* volume_obj3D = new Object3D(0);
	OBJECT3D_SET[VOLUME] = volume_obj3D;

	// WALL_PATCHES (6) // empty
	Object3D* wall_patches_obj3D = new Object3D(0);
	OBJECT3D_SET[WALL_PATCHES] = wall_patches_obj3D;

	// CAMERA_FOV (7) // empty (CAMERA_FOV uses WALL_PATCHES)
	Object3D* camera_fov_obj3D = new Object3D(0);
	OBJECT3D_SET[CAMERA_FOV] = camera_fov_obj3D;

	// LASER_RAY (8) // empty
	Object3D* laser_ray_obj3D = new Object3D(0);
	OBJECT3D_SET[LASER_RAY] = laser_ray_obj3D;

	// VOLUME_PATCHES (9) // empty
	Object3D* volume_patches_obj3D = new Object3D(0);
	OBJECT3D_SET[VOLUME_PATCHES] = volume_patches_obj3D;
 
	// PIXEL_PATCHES (10)
	// first: empty object to let the render start properly since the first frame
	Object3D* pixel_patches_obj3D = new Object3D(0);
	OBJECT3D_SET[PIXEL_PATCHES] = pixel_patches_obj3D;
	// second: set_pixel_patches(...) actually
	set_pixel_patches(&camera_pos, &camera_rot, &camera_centre, screen_patches_corners_normals, screen_patches_centers_normals);

	// UPDATE PIXEL PATCHES IF LOOP
	if (loop)
		update_pixel_patches(&camera_pos, &camera_rot, &camera_centre, screen_patches_corners_normals, screen_patches_centers_normals, loop);
}

// sets all the Calibration Matrix scene. (Actually just the Camera, Laser, Wall and Wall_Patches).
void set_scene_calibration_matrix (Info* info_, Pixels_storing pixel_storing_) { // by default: pixel_storing_ = PIXELS_VALID

	// CAMERA (0)
	Point camera_pos(0.0f, 0.75f, 0.0f);		// pos of the center of the camera
	Point camera_rot(0.0f, 180.0f, 0.0f);		// rot from the center of the camera
	Point camera_size(0.15f, 0.15f, 0.04f);
	Point camera_centre(camera_size.x() / 2.0f, camera_size.y() / 2.0f, 0.0f);	// centre relative to the first point
	set_camera(&camera_pos, &camera_rot, &camera_size, &camera_centre);
	// Get the screen normals:
	// Ordering: 1st row: col, col, col... 2nd row: col, col, col... from left to right, from bottom to top
	std::vector<Point*> screen_patches_corners_normals((CAMERA_PIX_X + 1) * (CAMERA_PIX_Y + 1));	// size is always (CAMERA_PIX_X + 1) * (CAMERA_PIX_Y + 1), regardingless the value of PIXELS_STORING_GLOBAL
	std::vector<Point*> screen_patches_centers_normals(CAMERA_PIX_X * CAMERA_PIX_Y);				// size is always (CAMERA_PIX_X)     * (CAMERA_PIX_Y)    , regardingless the value of PIXELS_STORING_GLOBAL
	set_screen_normals_pixel_patches(screen_patches_corners_normals, screen_patches_centers_normals, &camera_pos, &camera_rot, &camera_centre);

	// LASER (1)
	Point laser_pos(camera_pos.x() + info_->laser_to_cam_offset_x,
					camera_pos.y() + info_->laser_to_cam_offset_y,
					camera_pos.z() + info_->laser_to_cam_offset_z);		// pos of the center of the laser
	Point laser_rot(0.0f, 180.0f, 0.0f);		// rot from the center of the laser
	Point laser_size(0.1f, 0.1f, 0.3f);
	Point laser_centre(laser_size.x() / 2.0f, laser_size.y() / 2.0f, 0.0f);	// centre relative to the first point
	set_laser(&laser_pos, &laser_rot, &laser_size, &laser_centre);

	// WALL (2)
	Point wall_pos(-1.625f, 0.0f, -info_->dist_wall_cam);	// pos of the center of the wall
	Point wall_rot(0.0f, 0.0f, 0.0f);					// rot from the center of the wall
	Point wall_size(4.0f, 0.01f, 0.2f);
	Point wall_centre(0.0f, 0.0f, 0.0f);				// centre relative to the first point
	float wall_albedo = 0.5f;	// does not matter
	set_box(&wall_pos, &wall_rot, &wall_size, &wall_centre, WALL, wall_albedo);

	// OCCLUDER (3)
	Object3D* occluder_obj3D = new Object3D(0);
	OBJECT3D_SET[OCCLUDER] = occluder_obj3D;

	// FLOOR (4)
	Point floor_pos(wall_pos.x(), 0.0f, 0.7f);
	Point floor_rot(-90.0f, 0.0f, 0.0f);
	Point floor_size(wall_size.x(), 6.0f, wall_size.z());
	Point floor_centre(0.0f, 0.0f, 0.0f);
	float floor_albedo = 0.5f;	// does not matter
	set_box(&floor_pos, &floor_rot, &floor_size, &floor_centre, FLOOR, floor_albedo);

	// VOLUME (5)
	Object3D* volume_obj3D = new Object3D(0);
	OBJECT3D_SET[VOLUME] = volume_obj3D;

	// WALL_PATCHES (6)
	std::vector<float> wall_patches_albedo(CAMERA_PIX_X * CAMERA_PIX_Y);	// size is always (CAMERA_PIX_X) * (CAMERA_PIX_Y), regardingless the value of PIXELS_STORING_GLOBAL
	set_wall_patches_albedo(wall_patches_albedo);
	set_wall_patches(&camera_pos, &camera_rot, &camera_size, &camera_centre, screen_patches_corners_normals, screen_patches_centers_normals, wall_patches_albedo, pixel_storing_);

	// CAMERA_FOV (7)
	Object3D* camera_fov_obj3D = new Object3D(0);
	OBJECT3D_SET[CAMERA_FOV] = camera_fov_obj3D;

	// LASER_RAY (8)
	Object3D* laser_ray_obj3D = new Object3D(0);
	OBJECT3D_SET[LASER_RAY] = laser_ray_obj3D;

	// VOLUME_PATCHES (9)
	Object3D* volume_patches_obj3D = new Object3D(0);
	OBJECT3D_SET[VOLUME_PATCHES] = volume_patches_obj3D;

	// PIXEL_PATCHES (10) // empty
	Object3D* pixel_patches_obj3D = new Object3D(0);
	OBJECT3D_SET[PIXEL_PATCHES] = pixel_patches_obj3D;
}

// clears all the scenes, setting empty Objects
void clear_scene() {
	
	// CAMERA (0)
	Object3D* camera_obj3D = new Object3D(0);
	OBJECT3D_SET[CAMERA] = camera_obj3D;

	// LASER (1)
	Object3D* laser_obj3D = new Object3D(0);
	OBJECT3D_SET[LASER] = laser_obj3D;

	// WALL (2)
	Object3D* wall_obj3D = new Object3D(0);
	OBJECT3D_SET[WALL] = wall_obj3D;

	// OCCLUDER (3)
	Object3D* occluder_obj3D = new Object3D(0);
	OBJECT3D_SET[OCCLUDER] = occluder_obj3D;

	// FLOOR (4)
	Object3D* floor_obj3D = new Object3D(0);
	OBJECT3D_SET[FLOOR] = floor_obj3D;

	// VOLUME (5)
	Object3D* volume_obj3D = new Object3D(0);
	OBJECT3D_SET[VOLUME] = volume_obj3D;

	// WALL_PATCHES (6)
	Object3D* wall_patches_obj3D = new Object3D(0);
	OBJECT3D_SET[WALL_PATCHES] = wall_patches_obj3D;

	// CAMERA_FOV (7)
	Object3D* camera_fov_obj3D = new Object3D(0);
	OBJECT3D_SET[CAMERA_FOV] = camera_fov_obj3D;

	// LASER_RAY (8)
	Object3D* laser_ray_obj3D = new Object3D(0);
	OBJECT3D_SET[LASER_RAY] = laser_ray_obj3D;

	// VOLUME_PATCHES (9)
	Object3D* volume_patches_obj3D = new Object3D(0);
	OBJECT3D_SET[VOLUME_PATCHES] = volume_patches_obj3D;

	// PIXEL_PATCHES (10) // empty
	Object3D* pixel_patches_obj3D = new Object3D(0);
	OBJECT3D_SET[PIXEL_PATCHES] = pixel_patches_obj3D;
}


// sets the Object3D camera
void set_camera(Point* camera_pos_, Point* camera_rot_, Point* camera_size_, Point* camera_centre_) {

	// Main box of the camera
	// PointMesh
	PointMesh* camera_box = new PointMesh(vp_box(camera_size_), RECTANGLE, new Point((*camera_centre_)));

	// Stick box of the camera
	// Parameters
	Point stick_size(0.05f, (*camera_pos_).y() - (*camera_centre_).y(), 0.05f);
	Point stick_centre(stick_size.x() / 2.0f, stick_size.y(), -stick_size.z() / 2.0f);
	// PointMesh
	PointMesh* stick_box = new PointMesh(vp_box(&stick_size), RECTANGLE, new Point(stick_centre));

	// Base box of the camera
	// Parameters
	Point base_size(0.3f, 0.05f, 0.3f);
	Point base_centre(base_size.x() / 2.0f, base_size.y(), -base_size.z() / 2.0f);
	// PointMesh
	PointMesh* base_box = new PointMesh(vp_box(&base_size), RECTANGLE, new Point(base_centre));

	// Lens box of the camera
	// Parameters
	Point lens_size(0.08f, 0.08f, 0.1f);
	Point lens_centre(lens_size.x() / 2.0f, lens_size.y() / 2.0f, -lens_size.z());
	// PointMesh
	PointMesh* lens_box = new PointMesh(vp_box(&lens_size), RECTANGLE, new Point(lens_centre));
	(*lens_box).p.erase(lens_box->p.begin(), lens_box->p.begin() + 4);
	(*lens_box).p.erase(lens_box->p.begin() + 4, lens_box->p.begin() + 8);

	// All PointMesh Transformations
	(*stick_box).tra_center_to(new Point((*camera_size_).x() / 2.0f, 0.0f, -(*camera_size_).z() / 2.0f));
	(*base_box).tra_center_to(new Point((*(stick_box->c)).x(), (*(stick_box->c)).y() - stick_size.y() + base_size.y(), (*(stick_box->c)).z()));
	(*lens_box).tra_center_to(new Point((*camera_centre_)));

	// Object3D
	Object3D* camera = new Object3D(4);
	(*camera)[0] = camera_box;
	(*camera)[1] = stick_box;
	(*camera)[2] = base_box;
	(*camera)[3] = lens_box;
	tra_center_to(camera, camera_pos_);
	rot_from_c(camera, camera_rot_, true);

	// Object3D_Set
	OBJECT3D_SET[CAMERA] = camera;
}


// sets the Object3D laser
void set_laser(Point* laser_pos_, Point* laser_rot_, Point* laser_size_, Point* laser_centre_) {

	// Main box of the laser
	// PointMesh
	PointMesh* laser_box = new PointMesh(vp_box(laser_size_), RECTANGLE, new Point((*laser_centre_)));

	// Stick box of the laser
	// Parameters
	Point stick_size(0.05f, (*laser_pos_).y() - (*laser_centre_).y(), 0.05f);
	Point stick_centre(stick_size.x() / 2.0f, stick_size.y(), -stick_size.z() / 2.0f);
	// PointMesh
	PointMesh* stick_box = new PointMesh(vp_box(&stick_size), RECTANGLE, new Point(stick_centre));

	// Base box of the laser
	// Parameters
	Point base_size(0.3f, 0.05f, 0.3f);
	Point base_centre(base_size.x() / 2.0f, base_size.y(), -base_size.z() / 2.0f);
	// PointMesh
	PointMesh* base_box = new PointMesh(vp_box(&base_size), RECTANGLE, new Point(base_centre));

	// All PointMesh Transformations
	(*stick_box).tra_center_to(new Point((*laser_size_).x() / 2.0f, 0.0f, -(*laser_size_).z() / 2.0f));
	(*base_box).tra_center_to(new Point((*(stick_box->c)).x(), (*(stick_box->c)).y() - stick_size.y() + base_size.y(), (*(stick_box->c)).z()));

	// Object3D
	Object3D* laser = new Object3D(3);
	(*laser)[0] = laser_box;
	(*laser)[1] = stick_box;
	(*laser)[2] = base_box;
	tra_center_to(laser, laser_pos_);
	rot_from_c(laser, laser_rot_, true);

	// Object3D_Set
	OBJECT3D_SET[LASER] = laser;
}


// sets any Object3D single-box-shaped (wall, occluder, floor, ...)
void set_box(Point* box_pos_, Point* box_rot_, Point* box_size_, Point* box_centre_, int Obj3D_idx, float albedo_) {

	// Main box (and only) of the box
	// PointMesh
	PointMesh* box_box = new PointMesh(vp_box(box_size_), RECTANGLE, box_centre_, albedo_);

	// Object3D
	Object3D* box = new Object3D(1);
	(*box)[0] = box_box;
	tra_center_to(box, box_pos_);
	rot_from_c(box, box_rot_, true);

	// Object3D_Set
	OBJECT3D_SET[Obj3D_idx] = box;
}


// sets the Object3D with all the wall patches (wall patch = PointMesh with one rectangle)
void set_wall_patches(Point* camera_pos_, Point* camera_rot_, Point* camera_size_, Point* camera_centre_, std::vector<Point*> & screen_patches_corners_normals_, std::vector<Point*> & screen_patches_centers_normals_, std::vector<float> & wall_patches_albedo_, Pixels_storing pixel_storing_) { // by default: pixel_storing_ = PIXELS_VALID

	// setting the intersections
	std::vector<Point*> wall_patches_corners((CAMERA_PIX_X + 1) * (CAMERA_PIX_Y + 1));
	std::vector<Point*> wall_patches_centers(CAMERA_PIX_X * CAMERA_PIX_Y);
	PointMesh* wall_face = (*OBJECT3D_SET[WALL])[0];						// face of the wall
	for (std::size_t i = 0; i < wall_patches_corners.size(); i++)
		wall_patches_corners[i] = new Point(get_intersection_linePointNormal_pointmesh(camera_pos_, screen_patches_corners_normals_[i], wall_face));
	for (std::size_t i = 0; i < wall_patches_centers.size(); i++)
		wall_patches_centers[i] = new Point(get_intersection_linePointNormal_pointmesh(camera_pos_, screen_patches_centers_normals_[i], wall_face));

	// Object3D
	Object3D* wall_patches;
	int size_Object3D, y_begin, y_end, x_begin, x_end;
	int pos_in_Object3D = 0;
	if (pixel_storing_ == PIXELS_TOTAL) {
		size_Object3D = CAMERA_PIX_X * CAMERA_PIX_Y;
		y_begin = 0;	y_end = CAMERA_PIX_Y;
		x_begin = 0;	x_end = CAMERA_PIX_X;
	} else if (pixel_storing_ == PIXELS_VALID) {
		size_Object3D = CAMERA_PIX_X_VALID * CAMERA_PIX_Y_VALID;
		y_begin = CAMERA_PIX_Y_BAD_BOTTOM;	y_end = CAMERA_PIX_Y - CAMERA_PIX_Y_BAD_TOP;
		x_begin = CAMERA_PIX_X_BAD_LEFT;	x_end = CAMERA_PIX_X - CAMERA_PIX_X_BAD_RIGHT;
	}
	wall_patches = new Object3D(size_Object3D);
	for (int iy = y_begin; iy < y_end; iy++) {
		for (int ix = x_begin; ix < x_end; ix++) {
			Point* p0 = new Point(*(wall_patches_corners[iy*(CAMERA_PIX_X + 1) + ix]));
			Point* p1 = new Point(*(wall_patches_corners[iy*(CAMERA_PIX_X + 1) + ix + 1]));
			Point* p2 = new Point(*(wall_patches_corners[(iy + 1)*(CAMERA_PIX_X + 1) + ix + 1]));
			Point* p3 = new Point(*(wall_patches_corners[(iy + 1)*(CAMERA_PIX_X + 1) + ix]));
			Point* c  = new Point(*(wall_patches_centers[iy*CAMERA_PIX_X + ix]));
			// PointMesh
			// Each wall_patch will be a PointMesh with one rectangle inside the Object3D wall_patches
			PointMesh* wall_patches_pm = new PointMesh(vp_rectangle(p0, p1, p2, p3), RECTANGLE, c, wall_patches_albedo_[iy*CAMERA_PIX_X + ix]);
			//(*wall_patches).push_back(wall_patches_pm);
			(*wall_patches)[pos_in_Object3D++] = wall_patches_pm;	// pos_in_Object3D++
		}
	}

	// Object3D_Set
	OBJECT3D_SET[WALL_PATCHES] = wall_patches;
}

// sets the Object3D with all the pixel patches (pixel patch = PointMesh with one rectangle). Only for Direct-Vision-Any scene
void set_pixel_patches(Point* camera_pos_, Point* camera_rot_, Point* camera_centre_, std::vector<Point*> & screen_patches_corners_normals_, std::vector<Point*> & screen_patches_centers_normals_) {	

	// Ordering: 1st row: col, col, col... 2nd row: col, col, col... from left to right, from top to bottom (matrix ordering)
	cv::Mat depth_map(CAMERA_PIX_Y, CAMERA_PIX_X, cv::DataType<float>::type);
	
	// Syncronization
	std::unique_lock<std::mutex> locker_frame_object;	// Create a defered locker (a locker not locked yet)
	locker_frame_object = std::unique_lock<std::mutex>(mutex_frame_object,std::defer_lock);

	Point* camera_centre_abs = (*(*OBJECT3D_SET[CAMERA])[0]).c;
	float albedo = 0.5f;	// does not matter

	// Syncronization
	locker_frame_object.lock();		// Lock mutex_frame_object, any thread which used mutex_frame_object can NOT continue until unlock()
	while (!UPDATED_NEW_FRAME) {
		//std::cout << "Waiting in Object to finish the UPDATED_NEW_Frame. This is OK!\n";
		cv_frame_object.wait(locker_frame_object);
	}

	// Setting Depth Map
	set_depth_map(depth_map, FRAME_00_CAPTURE, FRAME_90_CAPTURE);

	// Object3D
	Object3D* pixel_patches;
	int size_Object3D, y_begin, y_end, x_begin, x_end;
	int pos_in_Object3D = 0;
	if (PIXELS_STORING_GLOBAL == PIXELS_TOTAL) {
		size_Object3D = CAMERA_PIX_X * CAMERA_PIX_Y;
		y_begin = 0;	y_end = CAMERA_PIX_Y;
		x_begin = 0;	x_end = CAMERA_PIX_X;
	} else if (PIXELS_STORING_GLOBAL == PIXELS_VALID) {
		size_Object3D = CAMERA_PIX_X_VALID * CAMERA_PIX_Y_VALID;
		y_begin = CAMERA_PIX_Y_BAD_BOTTOM;	y_end = CAMERA_PIX_Y - CAMERA_PIX_Y_BAD_TOP;
		x_begin = CAMERA_PIX_X_BAD_LEFT;	x_end = CAMERA_PIX_X - CAMERA_PIX_X_BAD_RIGHT;
	}
	pixel_patches = new Object3D(size_Object3D);
	for (int iy = y_begin; iy < y_end; iy++) {
		for (int ix = x_begin; ix < x_end; ix++) {
			Point* p0 = new Point((*(screen_patches_corners_normals_[iy*(CAMERA_PIX_X + 1) + ix])) * depth_map.at<float>(CAMERA_PIX_Y-iy-1,ix) + (*camera_centre_abs));
			Point* p1 = new Point((*(screen_patches_corners_normals_[iy*(CAMERA_PIX_X + 1) + ix + 1])) * depth_map.at<float>(CAMERA_PIX_Y-iy-1,ix) + (*camera_centre_abs));
			Point* p2 = new Point((*(screen_patches_corners_normals_[(iy + 1)*(CAMERA_PIX_X + 1) + ix + 1])) * depth_map.at<float>(CAMERA_PIX_Y-iy-1,ix) + (*camera_centre_abs));
			Point* p3 = new Point((*(screen_patches_corners_normals_[(iy + 1)*(CAMERA_PIX_X + 1) + ix])) * depth_map.at<float>(CAMERA_PIX_Y-iy-1,ix) + (*camera_centre_abs));
			Point* c  = new Point((*(screen_patches_centers_normals_[iy*CAMERA_PIX_X + ix])) * depth_map.at<float>(CAMERA_PIX_Y-iy-1,ix) + (*camera_centre_abs)); 
			// PointMesh
			// Each wall_patch will be a PointMesh with one rectangle inside the Object3D wall_patches
			PointMesh* pixel_patches_pm = new PointMesh(vp_rectangle(p0, p1, p2, p3), RECTANGLE, c, albedo);
			(*pixel_patches)[pos_in_Object3D++] = pixel_patches_pm;	// pos_in_Object3D++
		}
	}
	// Object3D_Set
	//delete OBJECT3D_SET[PIXEL_PATCHES];	// with this line it crashes when looping
	OBJECT3D_SET[PIXEL_PATCHES] = pixel_patches;
		
	// Syncronization
	//std::cout << ",    UPDATED_NEW_OBJECT\n";
	UPDATED_NEW_FRAME = false;
	UPDATED_NEW_OBJECT = true;
	cv_frame_object.notify_all();	// Notify all cv_frame_object. All threads waiting for cv_frame_object will break the wait after waking up
	locker_frame_object.unlock();	// Unlock mutex_frame_object, now threads which used mutex_frame_object can continue
}

// sets the Object3D with all the pixel patches (pixel patch = PointMesh with one rectangle). Only for Direct-Vision-Any scene
void update_pixel_patches(Point* camera_pos_, Point* camera_rot_, Point* camera_centre_, std::vector<Point*> & screen_patches_corners_normals_, std::vector<Point*> & screen_patches_centers_normals_, bool loop) {	// by default: loop = false

	// Ordering: 1st row: col, col, col... 2nd row: col, col, col... from left to right, from top to bottom (matrix ordering)
	cv::Mat depth_map(CAMERA_PIX_Y, CAMERA_PIX_X, cv::DataType<float>::type);
	
	// Syncronization
	std::unique_lock<std::mutex> locker_frame_object;	// Create a defered locker (a locker not locked yet)
	locker_frame_object = std::unique_lock<std::mutex>(mutex_frame_object,std::defer_lock);

	// Variables
	Point* camera_centre_abs = (*(*OBJECT3D_SET[CAMERA])[0]).c;
	bool first_iter = true;
	int y_begin, y_end, x_begin, x_end;
	int pos_in_Object3D = 0;
	if (PIXELS_STORING_GLOBAL == PIXELS_TOTAL) {
		y_begin = 0;	y_end = CAMERA_PIX_Y;
		x_begin = 0;	x_end = CAMERA_PIX_X;
	} else if (PIXELS_STORING_GLOBAL == PIXELS_VALID) {
		y_begin = CAMERA_PIX_Y_BAD_BOTTOM;	y_end = CAMERA_PIX_Y - CAMERA_PIX_Y_BAD_TOP;
		x_begin = CAMERA_PIX_X_BAD_LEFT;	x_end = CAMERA_PIX_X - CAMERA_PIX_X_BAD_RIGHT;
	}

	// --- LOOP ------------------------------------------------------------------------------------------------
	while(loop || first_iter) {

		//const clock_t begin_time = clock();

		if (!PMD_LOOP_ENABLE && !first_iter)
			break;
		first_iter = false;
		
		// Syncronization
		locker_frame_object.lock();		// Lock mutex_frame_object, any thread which used mutex_frame_object can NOT continue until unlock()
		while (!UPDATED_NEW_FRAME) {
			//std::cout << "Waiting in Object to finish the UPDATED_NEW_Frame. This is OK!\n";
			cv_frame_object.wait(locker_frame_object);
		}
		
		// Setting Depth Map
		set_depth_map(depth_map, FRAME_00_CAPTURE, FRAME_90_CAPTURE);

		// LOOP ITERS (not the first)	// this loop takes 0-1 ms
		pos_in_Object3D = 0;
		for (int iy = y_begin; iy < y_end; iy++) {
			for (int ix = x_begin; ix < x_end; ix++) {
				Point p0 ((*(screen_patches_corners_normals_[iy*(CAMERA_PIX_X + 1) + ix])) * depth_map.at<float>(CAMERA_PIX_Y-iy-1,ix) + (*camera_centre_abs));
				Point p1 ((*(screen_patches_corners_normals_[iy*(CAMERA_PIX_X + 1) + ix + 1])) * depth_map.at<float>(CAMERA_PIX_Y-iy-1,ix) + (*camera_centre_abs));
				Point p2 ((*(screen_patches_corners_normals_[(iy + 1)*(CAMERA_PIX_X + 1) + ix + 1])) * depth_map.at<float>(CAMERA_PIX_Y-iy-1,ix) + (*camera_centre_abs));
				Point p3 ((*(screen_patches_corners_normals_[(iy + 1)*(CAMERA_PIX_X + 1) + ix])) * depth_map.at<float>(CAMERA_PIX_Y-iy-1,ix) + (*camera_centre_abs));
				Point c  ((*(screen_patches_centers_normals_[iy*CAMERA_PIX_X + ix])) * depth_map.at<float>(CAMERA_PIX_Y-iy-1,ix) + (*camera_centre_abs)); 
				// Update points and center
				// WARNING: NORMAL VECTOR OF THE POINTMESH IS NOT BEEING UPDATED (but neither being used)
				(*(*(*OBJECT3D_SET[PIXEL_PATCHES])[pos_in_Object3D]).p[0]) = p0;
				(*(*(*OBJECT3D_SET[PIXEL_PATCHES])[pos_in_Object3D]).p[1]) = p1;
				(*(*(*OBJECT3D_SET[PIXEL_PATCHES])[pos_in_Object3D]).p[2]) = p2;
				(*(*(*OBJECT3D_SET[PIXEL_PATCHES])[pos_in_Object3D]).p[3]) = p3;
				(*(*(*OBJECT3D_SET[PIXEL_PATCHES])[pos_in_Object3D]).c) = c;
				pos_in_Object3D++;
			}
		}
		
		// Syncronization
		//std::cout << ",    UPDATED_NEW_OBJECT\n";
		UPDATED_NEW_FRAME = false;
		UPDATED_NEW_OBJECT = true;
		cv_frame_object.notify_all();	// Notify all cv_frame_object. All threads waiting for cv_frame_object will break the wait after waking up
		locker_frame_object.unlock();	// Unlock mutex_frame_object, now threads which used mutex_frame_object can continue

		//const clock_t end_time = clock();
		//float ms_time = 1000.0f * float(end_time - begin_time) / (float)CLOCKS_PER_SEC;
		//float fps_time = 1000.0f / ms_time;
		//std::cout << "time = " << ms_time << " ms,    fps = " << fps_time <<  " fps\n";
	}
	// --- END OF LOOP -----------------------------------------------------------------------------------------
}

// updates the Object3D with all the pixel patches (pixel patch = PointMesh with one rectangle) and the Object3D with the Wall. Only for Direct-Vision-Wall scene
void update_wall_and_pixel_patches(Point* camera_pos_, Point* camera_rot_, Point* camera_centre_, std::vector<Point*> & screen_patches_corners_normals_, std::vector<Point*> & screen_patches_centers_normals_, int r_div, int c_div, bool loop) {	// by default: loop = false

	// Ordering: 1st row: col, col, col... 2nd row: col, col, col... from left to right, from top to bottom (matrix ordering)
	cv::Mat depth_map(CAMERA_PIX_Y, CAMERA_PIX_X, cv::DataType<float>::type);
	
	// Syncronization
	std::unique_lock<std::mutex> locker_frame_object;	// Create a defered locker (a locker not locked yet)
	locker_frame_object = std::unique_lock<std::mutex>(mutex_frame_object,std::defer_lock);

	// Variables
	Point* camera_centre_abs = (*(*OBJECT3D_SET[CAMERA])[0]).c;
	bool first_iter = true;
	int y_begin, y_end, x_begin, x_end;
	int pos_in_Object3D = 0;
	if (PIXELS_STORING_GLOBAL == PIXELS_TOTAL) {
		y_begin = 0;	y_end = CAMERA_PIX_Y;
		x_begin = 0;	x_end = CAMERA_PIX_X;
	} else if (PIXELS_STORING_GLOBAL == PIXELS_VALID) {
		y_begin = CAMERA_PIX_Y_BAD_BOTTOM;	y_end = CAMERA_PIX_Y - CAMERA_PIX_Y_BAD_TOP;
		x_begin = CAMERA_PIX_X_BAD_LEFT;	x_end = CAMERA_PIX_X - CAMERA_PIX_X_BAD_RIGHT;
	}

	// Array Wall Variables	// Ordering: like a Matrix (row, col, starting with 0)
	std::vector<int> r_idx (r_div);	// this will contain the indices of the rows of the pixels involved to get mean_pt, mean_n
	std::vector<int> c_idx (c_div);	// this will contain the indices of the cols of the pixels involved to get mean_pt, mean_n
	for (size_t r = 0; r < r_idx.size(); r++)
		r_idx[r] = std::min((int)(r*FRAME_00_CAPTURE.heigth/(r_div-1)), FRAME_00_CAPTURE.heigth-1);
	for (size_t c = 0; c < c_idx.size(); c++)
		c_idx[c] = std::min((int)(c*FRAME_00_CAPTURE.width/(c_div-1)), FRAME_00_CAPTURE.width-1);
	std::vector<Point> vector_points (r_div * c_div);					// Ordering: like a Matrix (row, col, starting with 0)
	std::vector<Point> vector_normals (4 * (r_div - 1) * (c_div - 1));	// Ordering: like a Matrix (row, col, starting with 0)
	Point mean_pt, mean_n;	// the mean of the position and normal of the points
	Point * p0, * p1, * p2, * p3;	// Auxiliar pointers to points
	Point scale_axis_rotation = Point(1.0f, 1.0f, 0.0f);	// This is due to we don't want to rotate in z axis, and errors in z axis would persist. rot_from_c_to_normal(...) nornmalizes after scaling



	// --- LOOP ------------------------------------------------------------------------------------------------
	while(loop || first_iter) {

		//const clock_t begin_time = clock();

		if (!PMD_LOOP_ENABLE && !first_iter)
			break;
		first_iter = false;
		
		// Syncronization
		locker_frame_object.lock();		// Lock mutex_frame_object, any thread which used mutex_frame_object can NOT continue until unlock()
		while (!UPDATED_NEW_FRAME) {
			//std::cout << "Waiting in Object to finish the UPDATED_NEW_Frame. This is OK!\n";
			cv_frame_object.wait(locker_frame_object);
		}
		
		// Setting Depth Map
		set_depth_map(depth_map, FRAME_00_CAPTURE, FRAME_90_CAPTURE);

		// LOOP ITERS (not the first)	// this loop takes 0-1 ms
		pos_in_Object3D = 0;
		for (int iy = y_begin; iy < y_end; iy++) {
			for (int ix = x_begin; ix < x_end; ix++) {
				// Update points and center
				// WARNING: NORMAL VECTOR OF THE POINTMESH IS NOT BEEING UPDATED (but neither being used)
				(*(*(*OBJECT3D_SET[PIXEL_PATCHES])[pos_in_Object3D]).p[0]) = Point((*(screen_patches_corners_normals_[iy*(CAMERA_PIX_X + 1) + ix])) * depth_map.at<float>(CAMERA_PIX_Y-iy-1,ix) + (*camera_centre_abs));
				(*(*(*OBJECT3D_SET[PIXEL_PATCHES])[pos_in_Object3D]).p[1]) = Point((*(screen_patches_corners_normals_[iy*(CAMERA_PIX_X + 1) + ix + 1])) * depth_map.at<float>(CAMERA_PIX_Y-iy-1,ix) + (*camera_centre_abs));
				(*(*(*OBJECT3D_SET[PIXEL_PATCHES])[pos_in_Object3D]).p[2]) = Point((*(screen_patches_corners_normals_[(iy + 1)*(CAMERA_PIX_X + 1) + ix + 1])) * depth_map.at<float>(CAMERA_PIX_Y-iy-1,ix) + (*camera_centre_abs));
				(*(*(*OBJECT3D_SET[PIXEL_PATCHES])[pos_in_Object3D]).p[3]) = Point((*(screen_patches_corners_normals_[(iy + 1)*(CAMERA_PIX_X + 1) + ix])) * depth_map.at<float>(CAMERA_PIX_Y-iy-1,ix) + (*camera_centre_abs));
				(*(*(*OBJECT3D_SET[PIXEL_PATCHES])[pos_in_Object3D]).c)	   = Point((*(screen_patches_centers_normals_[iy*CAMERA_PIX_X + ix])) * depth_map.at<float>(CAMERA_PIX_Y-iy-1,ix) + (*camera_centre_abs)); 
				pos_in_Object3D++;
			}
		}
		
		// UPDATING THE WALL
		for (size_t r = 0; r < r_idx.size(); r++) {
			for (size_t c = 0; c < c_idx.size(); c++) {
				p0 = (*(*OBJECT3D_SET[PIXEL_PATCHES])[(FRAME_00_CAPTURE.heigth - r_idx[r] - 1) * FRAME_00_CAPTURE.width + c_idx[c]]).c;
				vector_points[r*c_div + c] = (*p0);
		}	}
		for (size_t r = 1; r < r_idx.size(); r++) {
			for (size_t c = 0; c < c_idx.size() - 1; c++) {
				p0 = (*(*OBJECT3D_SET[PIXEL_PATCHES])[(FRAME_00_CAPTURE.heigth - r_idx[r    ] - 1) * FRAME_00_CAPTURE.width + c_idx[c    ]]).c;
				p1 = (*(*OBJECT3D_SET[PIXEL_PATCHES])[(FRAME_00_CAPTURE.heigth - r_idx[r    ] - 1) * FRAME_00_CAPTURE.width + c_idx[c + 1]]).c;
				p2 = (*(*OBJECT3D_SET[PIXEL_PATCHES])[(FRAME_00_CAPTURE.heigth - r_idx[r - 1] - 1) * FRAME_00_CAPTURE.width + c_idx[c + 1]]).c;
				p3 = (*(*OBJECT3D_SET[PIXEL_PATCHES])[(FRAME_00_CAPTURE.heigth - r_idx[r - 1] - 1) * FRAME_00_CAPTURE.width + c_idx[c    ]]).c;
				get_normal ((*p0), (*p1), (*p2), vector_normals[4*((r-1)*(c_div-1) + c) + 0]);
				get_normal ((*p1), (*p2), (*p3), vector_normals[4*((r-1)*(c_div-1) + c) + 1]);
				get_normal ((*p2), (*p3), (*p0), vector_normals[4*((r-1)*(c_div-1) + c) + 2]);
				get_normal ((*p3), (*p0), (*p1), vector_normals[4*((r-1)*(c_div-1) + c) + 3]);
		}	}
		mean_vector_of_points (vector_points, mean_pt);
		mean_vector_of_points (vector_normals, mean_n);
		tra_center_to(OBJECT3D_SET[WALL], &mean_pt),
		rot_from_c_to_normal(OBJECT3D_SET[WALL], &mean_n, scale_axis_rotation);

		// Syncronization
		//std::cout << ",    UPDATED_NEW_OBJECT\n";
		UPDATED_NEW_FRAME = false;
		UPDATED_NEW_OBJECT = true;
		cv_frame_object.notify_all();	// Notify all cv_frame_object. All threads waiting for cv_frame_object will break the wait after waking up
		locker_frame_object.unlock();	// Unlock mutex_frame_object, now threads which used mutex_frame_object can continue

		//const clock_t end_time = clock();
		//float ms_time = 1000.0f * float(end_time - begin_time) / (float)CLOCKS_PER_SEC;
		//float fps_time = 1000.0f / ms_time;
		//std::cout << "time = " << ms_time << " ms,    fps = " << fps_time <<  " fps\n";
	}
	// --- END OF LOOP -----------------------------------------------------------------------------------------
}

// sets the vector of normals of the centers and corners of the pixel patches from the camera(pixel patch = PointMesh with one rectangle). Only for Direct-Vision-Any scene
void set_screen_normals_pixel_patches(std::vector<Point*> & screen_patches_corners_normals_, std::vector<Point*> & screen_patches_centers_normals_, Point* camera_pos_, Point* camera_rot_, Point* camera_centre_) {

	// setting the screen of the original camera FoV measurement
	std::vector<Point*> screen_patches_corners((CAMERA_PIX_X + 1) * (CAMERA_PIX_Y + 1));
	std::vector<Point*> screen_patches_centers(CAMERA_PIX_X * CAMERA_PIX_Y);
	float pixel_size_x = CAMERA_FOV_X_METERS / ((float)CAMERA_PIX_X);
	float pixel_size_y = CAMERA_FOV_Y_METERS / ((float)CAMERA_PIX_Y);
	float pixel_center0_x = pixel_size_x / 2.0f;
	float pixel_center0_y = pixel_size_y / 2.0f;
	float screen_center_x = CAMERA_FOV_X_METERS / 2.0f;
	float screen_center_y = CAMERA_FOV_Y_METERS / 2.0f;
	for (int iy = 0; iy <= CAMERA_PIX_Y; iy++) {
		for (int ix = 0; ix <= CAMERA_PIX_X; ix++) {
			screen_patches_corners[iy*(CAMERA_PIX_X + 1) + ix] = new Point(((float)ix)*pixel_size_x, ((float)iy)*pixel_size_y, 0.0f);
		}
	}
	for (int iy = 0; iy < CAMERA_PIX_Y; iy++) {
		for (int ix = 0; ix < CAMERA_PIX_X; ix++) {
			screen_patches_centers[iy*CAMERA_PIX_X + ix] = new Point(pixel_center0_x + ((float)ix)*pixel_size_x, pixel_center0_y + ((float)iy)*pixel_size_y, 0.0f);
		}
	}
	PointMesh* screen_patches_corners_pm = new PointMesh(screen_patches_corners, PT, new Point(screen_center_x, screen_center_y, 0.0f));
	PointMesh* screen_patches_centers_pm = new PointMesh(screen_patches_centers, PT, new Point(screen_center_x, screen_center_y, 0.0f));
	// setting the relative-to-the-camera position of the original camera FoV measurement
	(*screen_patches_corners_pm).tra_center_to(camera_centre_);
	(*screen_patches_centers_pm).tra_center_to(camera_centre_);
	(*screen_patches_corners_pm).rot_from_c(new Point(0.0f, 180.0f, 0.0f), true);
	(*screen_patches_centers_pm).rot_from_c(new Point(0.0f, 180.0f, 0.0f), true);
	(*screen_patches_corners_pm).tra_center_to(new Point(0.0f, 0.0f, CAMERA_DIST_FOV_MEAS));
	(*screen_patches_centers_pm).tra_center_to(new Point(0.0f, 0.0f, CAMERA_DIST_FOV_MEAS));
	// moving the screen to the new position of the camera
	(*screen_patches_corners_pm).tra(camera_pos_);
	(*screen_patches_centers_pm).tra(camera_pos_);
	(*screen_patches_corners_pm).rot(camera_rot_, camera_pos_, true);
	(*screen_patches_centers_pm).rot(camera_rot_, camera_pos_, true);

	// get and normalize the vectors
	Point* camera_centre_abs = (*(*OBJECT3D_SET[CAMERA])[0]).c;
	// fill the vectors of Points with the corresponding normals
	for (size_t i = 0; i < (*screen_patches_corners_pm).p.size(); i++) {
		Point* pn = new Point(*(*screen_patches_corners_pm).p[i]);
		(*pn) = (*pn) - (*camera_centre_abs);
		(*pn).normalize();
		screen_patches_corners_normals_[i] = pn;
	}
	for (size_t i = 0; i < (*screen_patches_centers_pm).p.size(); i++) {
		Point* pn = new Point(*(*screen_patches_centers_pm).p[i]);
		(*pn) = (*pn) - (*camera_centre_abs);
		(*pn).normalize();
		screen_patches_centers_normals_[i] = pn;
	}
}

// sets a depth map from the 2 frames
void set_depth_map(cv::Mat & depth_map_, Frame & Frame_00_cap, Frame & Frame_90_cap) {

	// It would be better to do this by accessing to a calibration matrix
	
	//float wave_length = C_LIGHT_AIR / (Frame_00_cap.frequency * 1000000.0f);
	//int range_cycle = 1;
	//float range_min = (wave_length / 2.0f) * range_cycle;
	//float range_max = (wave_length / 2.0f) * (range_cycle + 1);
	//float delay_ns = 6.667.0f;
	//float delay_m  = delay_ns * C_LIGHT_AIR / 1000000000.f;
	
	// There exists an additional delay between the laser and the camera. This delay is freq-dependent (not const in rad or meters),
	// but it does not depend on a constant delay time, so it sould be calibrated for each frequency.
	float delay_m_100MHz = 2.0f;	// delay @ 100 MHz = 2.0m

	float path_dist = 0.0f;
	if (PIXELS_STORING_GLOBAL == PIXELS_TOTAL) {
		for (int r = 0; r < depth_map_.rows; r++) {
			for (int c = 0; c < depth_map_.cols; c++) {
				path_dist = (atan2(-Frame_90_cap.at(r,c,1), Frame_00_cap.at(r,c,1)) + PI) * C_LIGHT_AIR / (2 * PI * Frame_00_cap.frequency * 1000000.0f) + delay_m_100MHz;
				depth_map_.at<float>(r,c) = path_dist / 2.0f;	// this is an approximation that supposes camera and laser close enough
		}	}
	} else if (PIXELS_STORING_GLOBAL == PIXELS_VALID) {
		for (int r = 0; r < depth_map_.rows; r++) {
			for (int c = 0; c < depth_map_.cols; c++) {
				if ((r < CAMERA_PIX_Y_BAD_TOP) || (r >= CAMERA_PIX_Y - CAMERA_PIX_Y_BAD_BOTTOM) || (c < CAMERA_PIX_X_BAD_LEFT) || (c >= CAMERA_PIX_X - CAMERA_PIX_X_BAD_RIGHT))
					depth_map_.at<float>(r,c) = 0.0f;	// will not be considered
				else {
					path_dist = (atan2(-Frame_90_cap.at(r,c,1), Frame_00_cap.at(r,c,1)) + PI) * C_LIGHT_AIR / (2 * PI * Frame_00_cap.frequency * 1000000.0f) + delay_m_100MHz;
					depth_map_.at<float>(r,c) = path_dist / 2.0f;	// this is an approximation that supposes camera and laser close enough
	}	}	}	}
}

// sets the Object3D with the lines representing the camera FoV and its intersection with the wall
void set_camera_fov(Point* camera_pos_, std::vector<Point*> & screen_patches_corners_normals_, Pixels_storing pixel_storing_) { // by default: pixel_storing_ = PIXELS_VALID

	// face of the wall
	PointMesh* wall_face = (*OBJECT3D_SET[WALL])[0];
	
	// wall patch points
	int p0_pos, p1_pos, p2_pos, p3_pos;
	if (pixel_storing_ == PIXELS_TOTAL) {
		p0_pos = 0;
		p1_pos = CAMERA_PIX_X;
		p2_pos = (CAMERA_PIX_X+1) * (CAMERA_PIX_Y+1) - 1;
		p3_pos = (CAMERA_PIX_X+1) * CAMERA_PIX_Y;
	} else if (pixel_storing_ == PIXELS_VALID) {
		p0_pos = (CAMERA_PIX_X+1) * CAMERA_PIX_Y_BAD_BOTTOM + CAMERA_PIX_X_BAD_LEFT;
		p1_pos = (CAMERA_PIX_X+1) * (CAMERA_PIX_Y_BAD_BOTTOM+1) - CAMERA_PIX_X_BAD_RIGHT - 1;
		p2_pos = (CAMERA_PIX_X+1) * (CAMERA_PIX_Y+1-CAMERA_PIX_Y_BAD_TOP) - CAMERA_PIX_X_BAD_RIGHT - 1;
		p3_pos = (CAMERA_PIX_X+1) * (CAMERA_PIX_Y+1-CAMERA_PIX_Y_BAD_TOP-1) + CAMERA_PIX_X_BAD_LEFT;
	}
	Point* p0 = new Point(get_intersection_linePointNormal_pointmesh(camera_pos_, screen_patches_corners_normals_[p0_pos], wall_face));
	Point* p1 = new Point(get_intersection_linePointNormal_pointmesh(camera_pos_, screen_patches_corners_normals_[p1_pos], wall_face));
	Point* p2 = new Point(get_intersection_linePointNormal_pointmesh(camera_pos_, screen_patches_corners_normals_[p2_pos], wall_face));
	Point* p3 = new Point(get_intersection_linePointNormal_pointmesh(camera_pos_, screen_patches_corners_normals_[p3_pos], wall_face));
	// camera center point
	Point* pc = new Point((*(*(*OBJECT3D_SET[CAMERA])[0]).c));
	std::vector<Point*> lines(16);
	// wall_patch FoV lines
	lines[0] = new Point((*p0)); lines[1] = new Point((*p1));
	lines[2] = new Point((*p1)); lines[3] = new Point((*p2));
	lines[4] = new Point((*p2)); lines[5] = new Point((*p3));
	lines[6] = new Point((*p3)); lines[7] = new Point((*p0));
	// camera to wall patch corners lines
	lines[8]  = new Point((*pc)); lines[9] = new Point((*p0));
	lines[10] = new Point((*pc)); lines[11] = new Point((*p1));
	lines[12] = new Point((*pc)); lines[13] = new Point((*p2));
	lines[14] = new Point((*pc)); lines[15] = new Point((*p3));

	// PointMesh
	PointMesh* lines_pm = new PointMesh(lines, LINE, pc);

	// Object3D
	Object3D* lines_obj = new Object3D(1);
	(*lines_obj)[0] = lines_pm;

	// Object3D_Set
	OBJECT3D_SET[CAMERA_FOV] = lines_obj;
}

// sets the Object3D with a line representing the laser ray and its intersection with the wall
void set_laser_ray() {

	// Points of the Line
	Point* laser_ray_from = new Point((*(*(*OBJECT3D_SET[LASER])[0]).c));	// center of laser
	Point* laser_ray_normal = (*(*OBJECT3D_SET[LASER])[0]).n[0];			// normal of laser
	PointMesh* wall_face = (*OBJECT3D_SET[WALL])[0];						// face of the wall
	Point* laser_ray_to = new Point(get_intersection_linePointNormal_pointmesh(laser_ray_from, laser_ray_normal, wall_face));
	std::vector<Point*> ray(2);
	ray[0] = laser_ray_from;
	ray[1] = laser_ray_to;

	// PointMesh
	PointMesh* ray_pm = new PointMesh(ray, LINE, laser_ray_from);

	// Object3D
	Object3D* ray_obj = new Object3D(1);
	(*ray_obj)[0] = ray_pm;

	// Object3D_Set
	OBJECT3D_SET[LASER_RAY] = ray_obj;
}

// sets the Object3D with all the volume patches (volume patch = PointMesh with one rectangle)
void set_volume_patches(Point* volume_pos_, Point* volume_rot_, Point* volume_size_, Point* volume_centre_,
	std::vector<float> & volume_patches_albedo_, std::vector<Point*> & volume_patches_rot_, std::vector<bool> & volume_patches_bool_) {

	float const grid_size_x = (*volume_size_).x() / ((float)VOLUME_GRID_SIZE_X);
	float const grid_size_y = (*volume_size_).y() / ((float)VOLUME_GRID_SIZE_Y);
	float const grid_size_z = (*volume_size_).z() / ((float)VOLUME_GRID_SIZE_Z);
	float const p0_x = 0.0f;
	float const p0_y = 0.0f;
	//float const p0_z = -grid_size_z / 2.0f;	// patches centered in grid
	float const p0_z = 0.0f;					// patches centered in the front face of grid

	// Object3D
	Object3D* patch_obj = new Object3D;	// we still do not know the size, have to push_back()

	for (int iz = 0; iz < VOLUME_GRID_SIZE_Z; iz++) {
		for (int iy = 0; iy < VOLUME_GRID_SIZE_Y; iy++) {
			for (int ix = 0; ix < VOLUME_GRID_SIZE_X; ix++) {
				int pos = iz * VOLUME_GRID_SIZE_Y * VOLUME_GRID_SIZE_X + iy * VOLUME_GRID_SIZE_X + ix;

				// if this patch bool is false, continue
				if (!volume_patches_bool_[pos])
					continue;

				// Points of the patch
				Point* p0 = new Point(p0_x + ix*grid_size_x, p0_y + iy*grid_size_y, p0_z - iz*grid_size_z);
				Point* p1 = new Point((*p0).x() + grid_size_x, (*p0).y(), (*p0).z());
				Point* p2 = new Point((*p0).x() + grid_size_x, (*p0).y() + grid_size_y, (*p0).z());
				Point* p3 = new Point((*p0).x(), (*p0).y() + grid_size_y, (*p0).z());
				Point* c  = new Point((*p0).x() + grid_size_x / 2.0f, (*p0).y() + grid_size_y / 2.0f, (*p0).z());
				std::vector<Point*> patch(4);
				patch[0] = p0;
				patch[1] = p1;
				patch[2] = p2;
				patch[3] = p3;

				// PointMesh
				PointMesh* patch_pm = new PointMesh(patch, RECTANGLE, c, volume_patches_albedo_[pos]);

				// Rotation Transformations of the own PointMesh
				(*patch_pm).rot_from_c(volume_patches_rot_[pos], true);

				// Rotantion and Translation Transformations to fit the Volume
				(*patch_pm).rot(volume_rot_, true);
				Point* translation = new Point((*volume_pos_) - (*volume_centre_));
				(*patch_pm).tra(translation);

				// Object3D
				(*patch_obj).push_back(patch_pm);	// we still do not know the size, have to push_back()

			}
		}
	}
	// Object3D_Set
	OBJECT3D_SET[VOLUME_PATCHES] = patch_obj;
}



// sets the albedo value for each wall patch
void set_wall_patches_albedo(std::vector<float> & wall_patches_albedo_) {
	for (int iy = 0; iy < CAMERA_PIX_Y; iy++) {
		for (int ix = 0; ix < CAMERA_PIX_X; ix++) {
			int pos = iy*CAMERA_PIX_X + ix;
			// here we can set the albedo of each patch of the wall
			wall_patches_albedo_[pos] = 0.5;
		}
	}
}

// sets the albedo, rotation and bool values for each volume patch
void set_volume_patches_params(std::vector<float> & volume_patches_albedo_, std::vector<Point*> & volume_patches_rot_, std::vector<bool> & volume_patches_bool_) {
	for (int iz = 0; iz < VOLUME_GRID_SIZE_Z; iz++) {
		for (int iy = 0; iy < VOLUME_GRID_SIZE_Y; iy++) {
			for (int ix = 0; ix < VOLUME_GRID_SIZE_X; ix++) {
				int pos = iz * VOLUME_GRID_SIZE_Y * VOLUME_GRID_SIZE_X + iy * VOLUME_GRID_SIZE_X + ix;
				// here we can set the albedo of each patch of the volume
				volume_patches_albedo_[pos] = 0.4f;
				// here we can set the rotat  of each patch of the volume
				volume_patches_rot_[pos] = new Point (0.0f, 0.0f, 0.0f);
				// here we can set the bool   of each patch of the volume (true=create)
				if (iz == 0)
					volume_patches_bool_[pos] = true;
				else
					volume_patches_bool_[pos] = false;
			}
		}
	}
}

// sets the mean point of a vector of points floats
void mean_vector_of_points (std::vector<Point> & vec, Point & pt) {
	pt = Point(0.0f, 0.0f, 0.0f);
	for (size_t i = 0; i < vec.size(); i++) 
		pt += vec[i];
	pt /= vec.size();
}





