
#ifndef __SCENE_H
#define __SCENE_H

#include "shapes.h"

// sets all the Object3D (in OBJECT3D_SET) of the scene
void set_scene_diffused_mirror();

// sets all the scene with the simple configuration
void set_scene_direct_vision_wall();

// sets the Object3D camera
void set_camera(Point* camera_pos_, Point* camera_rot_, Point* camera_size_, Point* camera_centre_);

// sets the Object3D laser
void set_laser(Point* laser_pos_, Point* laser_rot_, Point* laser_size_, Point* laser_centre_);

// sets any Object3D single-box-shaped (wall, occluder, floor, etc)
void set_box(Point* box_pos_, Point* box_rot_, Point* box_size_, Point* box_centre_, int Obj3D_idx, float albedo);

// sets the Object3D with all the wall patches (wall patch = PointMesh with one rectangle)
void set_wall_patches(Point* camera_pos_, Point* camera_rot_, Point* camera_size_, Point* camera_centre_, std::vector<float> & wall_patches_albedo_);

// sets the Object3D with the lines representing the camera FoV and its intersection with the wall
void set_camera_fov();

// sets the Object3D with a line representing the laser ray and its intersection with the wall
void set_laser_ray();

// sets the Object3D with all the volume patches (volume patch = PointMesh with one rectangle)
void set_volume_patches(Point* volume_pos_, Point* volume_rot_, Point* volume_size_, Point* volume_centre_,
	std::vector<float> & volume_patches_albedo_, std::vector<Point*> & volume_patches_rot_, std::vector<bool> & volume_patches_bool_);



// sets the albedo value for each wall patch
void set_wall_patches_albedo(std::vector<float> & wall_patches_albedo_);

// sets the albedo, rotation and bool values for each volume patch
void set_volume_patches_params(std::vector<float> & volume_patches_albedo_, std::vector<Point*> & volume_patches_rot_, std::vector<bool> & volume_patches_bool_);

#endif

