
#ifndef __SCENE_H
#define __SCENE_H


#include <vector>

#include "global.h";




// ----------------------------------------------------------------------------------------------------------------------------------------
// ----- POINT ----------------------------------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------------------
class Point {
public:

	// ----- PARAMETERS ------------------------------

	float x, y, z;


	// ----- CONSTRUCTORS ---------------------------- // Note that Constructors just call their corresponding Setter

	// Constructor Default
	Point::Point();
	// Constructor Copy
	Point::Point(Point const & p0);
	// Constructor All Parameters
	Point::Point(float x_, float y_, float z_);
	
	// ----- SETTERS --------------------------------- // Note that Constructors just call their corresponding Setter
	
	// Setter Default
	void Point::set();
	// Setter Copy
	void Point::set(Point const & p0);
	// Setter All Parameters
	void Point::set(float x_, float y_, float z_);


	// ----- OPERTATORS ------------------------------

	Point & Point::operator= (Point const & rhs);
	Point Point::operator+ (Point const & rhs);
	Point Point::operator- (Point const & rhs);
	Point Point::operator* (Point const & rhs);
	Point Point::operator* (float rhs);
	Point Point::operator/ (Point const & rhs);
	Point Point::operator/ (float rhs);
	Point & Point::operator+= (Point const & rhs);
	Point & Point::operator-= (Point const & rhs);
	Point & Point::operator*= (Point const & rhs);
	Point & Point::operator*= (float rhs);
	Point & Point::operator/= (Point const & rhs);
	Point & Point::operator/= (float rhs);
	Point & Point::operator++();			// prefix
	Point & Point::operator++(int unused);	// postfix
	Point & Point::operator--();			// prefix
	Point & Point::operator--(int unused);	// postfix
	bool Point::operator== (Point const & rhs);
	bool Point::operator!= (Point const & rhs);
	float Point::operator[] (int idx);


	// ----- FUNCITONS -------------------------------

	// print
	void Point::print();
	void Point::print(char* txt_left, char* txt_right);
	// mod, modPow2
	float Point::mod();
	float Point::modPow2();
	// normal (get, set)
	Point Point::normal();	
	void Point::normalize();
	// dot product, cross prduct (getters)
	float Point::dot(Point const & pr);
	Point Point::cross(Point const & pr);
	// translation (setter)
	void Point::tra(Point const & pr);
	// rotation optimal (cosT and sinT already calculated) absolute (from (0,0,0)), radians (setters). All rot functions (from any class) call this 4 functions internally
	void Point::rotOpt(	float const & r11, float const & r12, float const & r13,
						float const & r21, float const & r22, float const & r23,
						float const & r31, float const & r32, float const & r33);
	void Point::rotxOpt(float cosT, float sinT);
	void Point::rotyOpt(float cosT, float sinT);
	void Point::rotzOpt(float cosT, float sinT);
	// rotation absolute (from (0,0,0)), radians (setters)
	void Point::rot(Point & axisN, float rad);
	void Point::rotx(float rad);
	void Point::roty(float rad);
	void Point::rotz(float rad);
	// rotation absolute (from (0,0,0)), degrees (setters)
	void Point::rotDeg(Point & axisN, float deg);
	void Point::rotxDeg(float deg);
	void Point::rotyDeg(float deg);
	void Point::rotzDeg(float deg);
	// rotation relative (from pr), radians (setters)
	void Point::rotFromP(Point & axisN, float rad, Point & pr);
	void Point::rotxFromP(float rad, Point & pr);
	void Point::rotyFromP(float rad, Point & pr);
	void Point::rotzFromP(float rad, Point & pr);
	// rotation relative (from pr), degrees (setters)
	void Point::rotDegFromP(Point & axisN, float deg, Point & pr);
	void Point::rotxDegFromP(float deg, Point & pr);
	void Point::rotyDegFromP(float deg, Point & pr);
	void Point::rotzDegFromP(float deg, Point & pr);
};
// ----- NON-MEMBER FUNCITONS ------------------------

// normal to 3 points
Point normalTo3P(Point & p0, Point & p1, Point & p2);
// Intersection Line-Plane. [P=Point, N=Normal]
Point int_linePN_planePN(Point & lP, Point & lN, Point & pP, Point & pN);
Point int_linePN_planePPP(Point & lP, Point & lN, Point & pP0, Point & pP1, Point & pP2);
Point int_linePP_planePN(Point & lP0, Point & lP1, Point & pP, Point & pN);	
Point int_linePP_planePPP(Point & lP0, Point & lP1, Point & pP0, Point & pP1, Point & pP2);



// ----------------------------------------------------------------------------------------------------------------------------------------
// ----- SHAPE ----------------------------------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------------------
class Shape {
public:

	// ----- PARAMETERS ------------------------------

	std::vector<Point> p;	// Points of the shape
	Point c;				// Center of the shape
	float albedo;			// albedo: [0.0f to 1.0f]
	float R;				// color Red channel
	float G;				// color Green channel
	float B;				// color Blue channel
	float A;				// color Alpha channel
	ShapeType st;			// st (UNKNOWN, PT, LINE, TRIANGLE, QUAD)

	
	// ----- CONSTRUCTORS ---------------------------- // Note that Constructors just call their corresponding Setter

	// Constructor Default
	Shape::Shape();
	// Constructor Copy
	Shape::Shape(Shape const & s);
	// Constructor All Parameters. shapeType = UNKNOWN, PT, LINE, TRIANGLE, QUAD
	Shape::Shape(std::vector<Point> & p_, Point const & c_,
		float albedo_ = 1.0f, float R_ = 1.0f, float G_ = 1.0f, float B_ = 1.0f, float A_ = 1.0f, ShapeType st_ = UNKNOWN_SHA);
	Shape::Shape(Point const & p0, Point const & c_,
		float albedo_ = 1.0f, float R_ = 1.0f, float G_ = 1.0f, float B_ = 1.0f, float A_ = 1.0f, ShapeType st_ = PT);
	Shape::Shape(Point const & p0, Point const & p1, Point const & c_,
		float albedo_ = 1.0f, float R_ = 1.0f, float G_ = 1.0f, float B_ = 1.0f, float A_ = 1.0f, ShapeType st_ = LINE);
	Shape::Shape(Point const & p0, Point const & p1, Point const & p2, Point const & c_,
		float albedo_ = 1.0f, float R_ = 1.0f, float G_ = 1.0f, float B_ = 1.0f, float A_ = 1.0f, ShapeType st_ = TRIANGLE);
	Shape::Shape(Point const & p0, Point const & p1, Point const & p2, Point const & p3, Point const & c_,
		float albedo_ = 1.0f, float R_ = 1.0f, float G_ = 1.0f, float B_ = 1.0f, float A_ = 1.0f, ShapeType st_ = QUAD);
	
	// ----- SETTERS --------------------------------- // Note that Constructors just call their corresponding Setter
	
	// Setter Default
	void Shape::set();
	// Setter Copy
	void Shape::set(Shape const & s);
	// Setter All Parameters. shapeType = PT, LINE, TRIANGLE, QUAD
	void Shape::set(std::vector<Point> & p_, Point const & c_);
	void Shape::set(Point const & p0, Point const & c_);
	void Shape::set(Point const & p0, Point const & p1, Point const & c_);
	void Shape::set(Point const & p0, Point const & p1, Point const & p2, Point const & c_);
	void Shape::set(Point const & p0, Point const & p1, Point const & p2, Point const & p3, Point const & c_);
	// Setter albedo, R, G, B, A, st
	void Shape::set(float albedo_, float R_, float G_ , float B_ , float A_, ShapeType st_);


	// ----- FUNCITONS -------------------------------

	// normal
	Point Shape::normalPT();		// normal of point: p[0].normal()
	Point Shape::normalLINE();		// normal of line : (p[1]-p[0]).normal()
	Point Shape::normalTRIANGLE();	// normal to tri  : normalTo3P(p[0],p[1],p[2])
	Point Shape::normalQUAD();		// normal to quad : normalTo3P(p[0],p[1],p[2])
	// translation (setter)
	void Shape::tra(Point const & pr);
	void Shape::traCto(Point & Cto);
	// rotation absolute (from (0,0,0)), radians (setters)
	void Shape::rot(Point & axisN, float rad);
	void Shape::rotx(float rad);
	void Shape::roty(float rad);
	void Shape::rotz(float rad);
	// rotation absolute (from (0,0,0)), degrees (setters)
	void Shape::rotDeg(Point & axisN, float deg);
	void Shape::rotxDeg(float deg);
	void Shape::rotyDeg(float deg);
	void Shape::rotzDeg(float deg);
	// rotation relative (from pr), radians (setters)
	void Shape::rotFromP(Point & axisN, float rad, Point & pr);
	void Shape::rotxFromP(float rad, Point & pr);
	void Shape::rotyFromP(float rad, Point & pr);
	void Shape::rotzFromP(float rad, Point & pr);
	// rotation relative (from pr), degrees (setters)
	void Shape::rotDegFromP(Point & axisN, float deg, Point & pr);
	void Shape::rotxDegFromP(float deg, Point & pr);
	void Shape::rotyDegFromP(float deg, Point & pr);
	void Shape::rotzDegFromP(float deg, Point & pr);
	// rotation relative (from c), radians (setters)
	void Shape::rotFromC(Point & axisN, float rad);
	void Shape::rotxFromC(float rad);
	void Shape::rotyFromC(float rad);
	void Shape::rotzFromC(float rad);
	// rotation relative (from c), degrees (setters)
	void Shape::rotDegFromC(Point & axisN, float deg);
	void Shape::rotxDegFromC(float deg);
	void Shape::rotyDegFromC(float deg);
	void Shape::rotzDegFromC(float deg);

	// clear, add Point to Shape
	void Shape::clear(bool clearAll = true);
	void Shape::add(Point & p0, bool updateST = true);

};
// ----- NON-MEMBER FUNCITONS ------------------------

// normal to Shape (TRIANGLE or QUAD)
Point normalToShape(Shape & s0);
// Intersection Line-Shape. [P=Point, N=Normal]
Point int_linePN_shape(Point & lP, Point & lN, Shape & s0);
Point int_linePP_shape(Point & lP0, Point & lP1, Shape & s0);




// ----------------------------------------------------------------------------------------------------------------------------------------
// ----- OBJECT3D -------------------------------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------------------
class Object3D {
public:

	// ----- PARAMETERS ------------------------------

	std::vector<Shape> s;	// Shapes of the Object3D
	Object3DType ot;		// ot = CAMERA, LASER, ..., UNKNWN
	PixStoring ps;		// Some Object3D' like PIXEL_PATCHES depend on the Pixel_storing

	
	// ----- CONSTRUCTORS ---------------------------- // Note that Constructors just call their corresponding Setter

	// Constructor Default
	Object3D::Object3D();
	// Constructor Copy
	Object3D::Object3D(Object3D const & o0);
	// Constructor All Parameters.
	Object3D::Object3D(std::vector<Shape> & s_, Object3DType ot_, PixStoring ps_ = UNKNOWN_PIS);
	// Constructor Object3DType and PixStoring.
	Object3D::Object3D(Object3DType ot_, PixStoring ps_ = UNKNOWN_PIS);
	// Constructor Box
	Object3D::Object3D(Point & boxPC, Point & boxRaxisN, float deg, Point & boxS, Point & boxC_relToP0);
	Object3D::Object3D(Point & boxPC, Point & boxRaxisN, float deg, Point & boxS, Point & boxC_relToP0,
		std::vector<float> & albedoV, std::vector<float> & RV, std::vector<float> & GV, std::vector<float> & BV, std::vector<float> & AV);


	// ----- SETTERS --------------------------------- // Note that Constructors just call their corresponding Setter
	
	// Setter Default
	void Object3D::set();
	// Setter Copy
	void Object3D::set(Object3D const & o0);
	// Setter All Parameters.
	void Object3D::set(std::vector<Shape> & s_, Object3DType ot_);
	// Setter Object3DType.
	void Object3D::set(Object3DType ot_);
	// Setter Object3DType PixStoring.
	void Object3D::set(PixStoring ps_);
	
	// Setter Box
	void Object3D::setBox(Point & boxPC, Point & boxRaxisN, float deg, Point & boxS, Point & boxC_relToP0);
	void Object3D::setBox(Point & boxPC, Point & boxRaxisN, float deg, Point & boxS, Point & boxC_relToP0,
		std::vector<float> & albedoV, std::vector<float> & RV, std::vector<float> & GV, std::vector<float> & BV, std::vector<float> & AV);
	
	// Setter Screen FoV measurement. Here each Point is an actual Point
	void Object3D::setScreenFoVmeasP(Point & camC, Point & camN, PixStoring ps_ = PIXELS_STORING_GLOBAL);
	// Setter Screen FoV measurement. Here each Point is the Normal to the corresponding Point of setScreenFoVmeasP
	void Object3D::setScreenFoVmeasN(Point & camC, Point & camN, PixStoring ps_ = PIXELS_STORING_GLOBAL);
	// Setter Screen FoV measurement. Here each Point is a close to Normal vector to the corresponding Point of setScreenFoVmeasP,
	// compensating the "spherical FoV", setting a constant ScreenPlane-CameraPlane distance
	void Object3D::setScreenFoVmeasNs(Point & camC, Point & camN, PixStoring ps_ = PIXELS_STORING_GLOBAL);
	
	// Setter Camera(0)
	void Object3D::setCamera(Point & posC, Point & axisN, float deg, Point & size, Point & c_relToP0);
	void Object3D::setCamera(Point & posC, Point & axisN, float deg, Point & size, Point & c_relToP0,
		std::vector<std::vector<float>> & albedoVV, std::vector<std::vector<float>> & RVV, std::vector<std::vector<float>> & GVV, std::vector<std::vector<float>> & BVV, std::vector<std::vector<float>> & AVV);
	// Setter Laser	(1)
	void Object3D::setLaser(Point & posC, Point & axisN, float deg, Point & size, Point & c_relToP0);
	void Object3D::setLaser(Point & posC, Point & axisN, float deg, Point & size, Point & c_relToP0,
		std::vector<std::vector<float>> & albedoVV, std::vector<std::vector<float>> & RVV, std::vector<std::vector<float>> & GVV, std::vector<std::vector<float>> & BVV, std::vector<std::vector<float>> & AVV);
	// Setter Wall Patches (6)
	void Object3D::setWallPatches(Scene & scene, PixStoring ps_ = PIXELS_STORING_GLOBAL);
	// Setter Camera FoV (7)
	void Object3D::setCameraFoV(Scene & scene, float R_ = 0.0f, float G_ = 0.0f, float B_ = 1.0f, float A_ = 1.0f, PixStoring ps_ = PIXELS_STORING_GLOBAL, float distDefault = 5.0f);
	// Setter Laser Ray (8)
	void Object3D::setLaserRay(Scene & scene, float R_ = 0.0f, float G_ = 0.0f, float B_ = 1.0f, float A_ = 1.0f, PixStoring ps_ = PIXELS_STORING_GLOBAL);
	// Setter, Updater Volume Patches (9)
	void Object3D::setVolumePatches();
	void Object3D::updateVolumePatches_Occlusion(Info & info, Scene & scene, Frame & frame00, Frame & frame90, bool loop = true, PixStoring ps_ = PIXELS_STORING_GLOBAL);
	// Setter, Updater Pixel Patches (10)
	void Object3D::setPixelPatches(Scene & scene, float distDefault = 2.0f, PixStoring ps_ = PIXELS_STORING_GLOBAL);
	void Object3D::setPixelPatches(Scene & scene, Frame & frame00, Frame & frame90, PixStoring ps_ = PIXELS_STORING_GLOBAL);
	void Object3D::updatePixelPatches_Sinusoid(Scene & scene, Frame & frame00, Frame & frame90, bool loop = true, PixStoring ps_ = PIXELS_STORING_GLOBAL);
	void Object3D::updatePixelPatches_Simulation(Info & info, Scene & scene, Frame & frame00, Frame & frame90, bool loop = true, PixStoring ps_ = PIXELS_STORING_GLOBAL);

	// ----- FUNCITONS -------------------------------

	// normal
	Point Object3D::normalPT();			// normal of point: s[0].p[0].normal()
	Point Object3D::normalLINE();		// normal of line : (s[0].p[1]-s[0].p[0]).normal()
	Point Object3D::normalTRIANGLE();	// normal to tri  : normalTo3P(s[0].p[0],s[0].p[1],s[0].p[2])
	Point Object3D::normalQUAD();		// normal to quad : normalTo3P(s[0].p[0],s[0].p[1],s[0].p[2])
	// translation (setter)
	void Object3D::tra(Point const & pr);
	void Object3D::traCto(Point & Cto);
	// rotation absolute (from (0,0,0)), radians (setters)
	void Object3D::rot(Point & axisN, float rad);
	void Object3D::rotx(float rad);
	void Object3D::roty(float rad);
	void Object3D::rotz(float rad);
	// rotation absolute (from (0,0,0)), degrees (setters)
	void Object3D::rotDeg(Point & axisN, float deg);
	void Object3D::rotxDeg(float deg);
	void Object3D::rotyDeg(float deg);
	void Object3D::rotzDeg(float deg);
	// rotation relative (from pr), radians (setters)
	void Object3D::rotFromP(Point & axisN, float rad, Point & pr);
	void Object3D::rotxFromP(float rad, Point & pr);
	void Object3D::rotyFromP(float rad, Point & pr);
	void Object3D::rotzFromP(float rad, Point & pr);
	// rotation relative (from pr), degrees (setters)
	void Object3D::rotDegFromP(Point & axisN, float deg, Point & pr);
	void Object3D::rotxDegFromP(float deg, Point & pr);
	void Object3D::rotyDegFromP(float deg, Point & pr);
	void Object3D::rotzDegFromP(float deg, Point & pr);
	// rotation relative (from c), radians (setters)
	void Object3D::rotFromC(Point & axisN, float rad);
	void Object3D::rotxFromC(float rad);
	void Object3D::rotyFromC(float rad);
	void Object3D::rotzFromC(float rad);
	// rotation relative (from c), degrees (setters)
	void Object3D::rotDegFromC(Point & axisN, float deg);
	void Object3D::rotxDegFromC(float deg);
	void Object3D::rotyDegFromC(float deg);
	void Object3D::rotzDegFromC(float deg);
	
	// clear, add Shape to Object3D
	void Object3D::clear(bool clearAll = true);
	void Object3D::add(Shape & s0);
	void Object3D::add(Object3D & o0);
};
// ----- NON-MEMBER FUNCITONS ------------------------

// normal to Object3D (TRIANGLE or QUAD)
Point normalToObject3D(Object3D & o0);
// Intersection Line-Object3D. [P=Point, N=Normal]
Point int_linePN_object3D(Point & lP, Point & lN, Object3D & o0);
Point int_linePP_object3D(Point & lP0, Point & lP1, Object3D & o0);




// ----------------------------------------------------------------------------------------------------------------------------------------
// ----- SCENE ----------------------------------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------------------
class Scene {
public:

	// ----- PARAMETERS ------------------------------

	std::vector<Object3D> o;	// Object3D's of the Scene
	SceneType sceneType;
	
	
	// ----- CONSTRUCTORS ---------------------------- // Note that Constructors just call their corresponding Setter

	// Constructor Default
	Scene::Scene(SceneType sceneType_ = UNKNOWN_SCT);
	// Constructor Copy
	Scene::Scene(Scene & scene);

	
	// ----- SETTERS --------------------------------- // Note that Constructors just call their corresponding Setter
	
	// Setter Default
	void Scene::set(SceneType sceneType_ = UNKNOWN_SCT);
	// Setter Copy
	void Scene::set(Scene & scene);
	
	// ----- FUNCITONS -------------------------------
	
	// clear (auto resizes to SCENE_SIZE), add Object3D to Scene
	void Scene::clear();
	void Scene::add(Object3D & o0);

	// Setter Scene Direct Vision
	void Scene::setScene_DirectVision(PixStoring ps = PIXELS_STORING_GLOBAL);	// the same for both Sinusoid and Simulation
	// Setter Scene Occlusion
	void Scene::setScene_Occlusion(PixStoring ps = PIXELS_STORING_GLOBAL);
	// Setter Scene Calibration Matrix
	void Scene::setScene_CalibrationMatrix(float laser_to_cam_offset_x, float laser_to_cam_offset_y, float laser_to_cam_offset_z, float dist_wall_cam);

};
// ----- NON-MEMBER FUNCITONS ------------------------
void updatePixelPatches_Sinusoid_antiBugThread(Scene & scene, Frame & frame00, Frame & frame90, bool loop = true, PixStoring ps_ = PIXELS_STORING_GLOBAL);
void updatePixelPatches_Simulation_antiBugThread(Info & info, Scene & scene, Frame & frame00, Frame & frame90, bool loop = true, PixStoring ps_ = PIXELS_STORING_GLOBAL);
void updateVolumePatches_Occlusion_antiBugThread(Info & info, Scene & scene, Frame & frame00, Frame & frame90, bool loop = true, PixStoring ps_ = PIXELS_STORING_GLOBAL);



// ----------------------------------------------------------------------------------------------------------------------------------------
// ----- OTHER NON-MEMBER FUNCTIONS -------------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------------------

// mean
float mean(std::vector<float> & vf);
Point mean(std::vector<Point> & vp);
// Rotation Matrix
void setRotationMatrix(float & r11, float & r12, float & r13,
	float & r21, float & r22, float & r23,
	float & r31, float & r32, float & r33,
	float const & wx, float const & wy, float const & wz, float const & rad);
// dist
float dist(Point & p0, Point & p1);
float distPow2(Point & p0, Point & p1);
// normal / degrees
Point crossN(Point & p0N, Point & p1N);
float radN(Point & p0N, Point & p1N);
float degN(Point & p0N, Point & p1N);
Point cross(Point & p0, Point & p1);
float rad(Point & p0, Point & p1);
float deg(Point & p0, Point & p1);
// set depth map
void setDepthMap(std::vector<float> & depthMap, Frame & frame00, Frame & frame90);


// MAIN OPERATORS
int main_scene(int argc, char**argv);





// TO-DO: CREATE ALL OF THIS (SOME ACTUALLY):

void set_scene_diffused_mirror(bool loop = false);
void set_scene_direct_vision_wall(bool loop = false);
void set_scene_direct_vision_any(bool loop = false);
void set_scene_calibration_matrix (Info* info_, PixStoring pixel_storing_ = PIXELS_VALID);
void set_scene_vision_simulation(float dist_cam_wall);
void clear_scene();
void set_camera(Point* camera_pos_, Point* camera_rot_, Point* camera_size_, Point* camera_centre_);
void set_laser(Point* laser_pos_, Point* laser_rot_, Point* laser_size_, Point* laser_centre_);
void set_box(Point* box_pos_, Point* box_rot_, Point* box_size_, Point* box_centre_, int Obj3D_idx, float albedo);
void set_wall_patches(Point* camera_pos_, Point* camera_rot_, Point* camera_size_, Point* camera_centre_, std::vector<Point*> & screen_patches_corners_normals_, std::vector<Point*> & screen_patches_centers_normals_, std::vector<float> & wall_patches_albedo_, PixStoring pixel_storing_ = PIXELS_VALID);
void set_camera_fov(Point* camera_pos_, std::vector<Point*> & screen_patches_corners_normals_, PixStoring pixel_storing_ = PIXELS_VALID);
void set_laser_ray();
void set_volume_patches(Point* volume_pos_, Point* volume_rot_, Point* volume_size_, Point* volume_centre_,
	std::vector<float> & volume_patches_albedo_, std::vector<Point*> & volume_patches_rot_, std::vector<bool> & volume_patches_bool_);
void set_wall_patches_albedo(std::vector<float> & wall_patches_albedo_);
void set_pixel_patches(Point* camera_pos_, Point* camera_rot_, Point* camera_centre_, std::vector<Point*> & screen_patches_corners_normals_, std::vector<Point*> & screen_patches_centers_normals_);
void update_pixel_patches(Point* camera_pos_, Point* camera_rot_, Point* camera_centre_, std::vector<Point*> & screen_patches_corners_normals_, std::vector<Point*> & screen_patches_centers_normals_, bool loop = false);
void update_wall_and_pixel_patches(Point* camera_pos_, Point* camera_rot_, Point* camera_centre_, std::vector<Point*> & screen_patches_corners_normals_, std::vector<Point*> & screen_patches_centers_normals_, int r_div, int c_div, bool loop = false);
void set_screen_normals_pixel_patches(std::vector<Point*> & screen_patches_corners_normals_, std::vector<Point*> & screen_patches_centers_normals_, Point* camera_pos_, Point* camera_rot_, Point* camera_centre_);
void set_depth_map(std::vector<float> & depth_map_, Frame & Frame_00_cap, Frame & Frame_90_cap);
void set_volume_patches_params(std::vector<float> & volume_patches_albedo_, std::vector<Point*> & volume_patches_rot_, std::vector<bool> & volume_patches_bool_);
void mean_vector_of_points (std::vector<Point> & vec, Point & pt);
void set_point_to_vector_distances (Point* p, std::vector<Point*> & vp, std::vector<float> & vd);

#endif









