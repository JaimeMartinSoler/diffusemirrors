
#ifndef __SCENE_H
#define __SCENE_H


#include <vector>

#include "global.h"




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
	void Point::tra(Point const & pr, Point & src);
	// rotation optimal (cosT and sinT already calculated) absolute (from (0,0,0)), radians (setters). All rot functions (from any class) call this 4 functions internally
	void Point::rotOpt(	float const & r11, float const & r12, float const & r13,
						float const & r21, float const & r22, float const & r23,
						float const & r31, float const & r32, float const & r33);
	void Point::rotOpt(	float const & r11, float const & r12, float const & r13, 
					float const & r21, float const & r22, float const & r23, 
					float const & r31, float const & r32, float const & r33, Point & src);
	void Point::rotXYZopt(float cosX, float sinX, float cosY, float sinY, float cosZ, float sinZ);
	void Point::rotXYZoptInverted(float cosX, float sinX, float cosY, float sinY, float cosZ, float sinZ);
	void Point::rotXYZopt(float cosX, float sinX, float cosY, float sinY, float cosZ, float sinZ, Point & src);
	void Point::rotXopt(float cosT, float sinT);
	void Point::rotYopt(float cosT, float sinT);
	void Point::rotZopt(float cosT, float sinT);
	// rotation absolute (from (0,0,0)), radians (setters)
	void Point::rot(Point & axisN, float rad);
	void Point::rotXYZ(float radX, float radY, float radZ);
	void Point::rotX(float rad);
	void Point::rotY(float rad);
	void Point::rotZ(float rad);
	// rotation absolute (from (0,0,0)), degrees (setters)
	void Point::rotDeg(Point & axisN, float deg);
	void Point::rotXYZdeg(float degX, float degY, float degZ);
	void Point::rotXdeg(float deg);
	void Point::rotYdeg(float deg);
	void Point::rotZdeg(float deg);
	// rotation relative (from pr), radians (setters)
	void Point::rotFromP(Point & axisN, float rad, Point & pr);
	void Point::rotXYZfromP(float radX, float radY, float radZ, Point & pr);
	void Point::rotXfromP(float rad, Point & pr);
	void Point::rotYfromP(float rad, Point & pr);
	void Point::rotZfromP(float rad, Point & pr);
	// rotation relative (from pr), degrees (setters)
	void Point::rotDegFromP(Point & axisN, float deg, Point & pr);
	void Point::rotXYZdegFromP(float degX, float degY, float degZ, Point & pr);
	void Point::rotXdegFromP(float deg, Point & pr);
	void Point::rotYdegFromP(float deg, Point & pr);
	void Point::rotZdegFromP(float deg, Point & pr);
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
	// area
	float Shape::areaRECTANGLE();
	// translation (setter)
	void Shape::tra(Point const & pr);
	void Shape::traCto(Point & Cto);
	// rotation absolute (from (0,0,0)), radians (setters)
	void Shape::rot(Point & axisN, float rad);
	void Shape::rotXYZ(float radX, float radY, float radZ, bool inverted = false);
	void Shape::rotX(float rad);
	void Shape::rotY(float rad);
	void Shape::rotZ(float rad);
	// rotation absolute (from (0,0,0)), degrees (setters)
	void Shape::rotDeg(Point & axisN, float deg);
	void Shape::rotXYZdeg(float degX, float degY, float degZ, bool inverted = false);
	void Shape::rotXdeg(float deg);
	void Shape::rotYdeg(float deg);
	void Shape::rotZdeg(float deg);
	// rotation relative (from pr), radians (setters)
	void Shape::rotFromP(Point & axisN, float rad, Point & pr);
	void Shape::rotXYZfromP(float radX, float radY, float radZ, Point & pr, bool inverted = false);
	void Shape::rotXfromP(float rad, Point & pr);
	void Shape::rotYfromP(float rad, Point & pr);
	void Shape::rotZfromP(float rad, Point & pr);
	// rotation relative (from pr), degrees (setters)
	void Shape::rotDegFromP(Point & axisN, float deg, Point & pr);
	void Shape::rotXYZdegFromP(float degX, float degY, float degZ, Point & pr, bool inverted = false);
	void Shape::rotXdegFromP(float deg, Point & pr);
	void Shape::rotYdegFromP(float deg, Point & pr);
	void Shape::rotZdegFromP(float deg, Point & pr);
	// rotation relative (from c), radians (setters)
	void Shape::rotFromC(Point & axisN, float rad);
	void Shape::rotXYZfromC(float radX, float radY, float radZ, bool inverted = false);
	void Shape::rotXfromC(float rad);
	void Shape::rotYfromC(float rad);
	void Shape::rotzFromC(float rad);
	// rotation relative (from c), degrees (setters)
	void Shape::rotDegFromC(Point & axisN, float deg);
	void Shape::rotXYZdegFromC(float degX, float degY, float degZ, bool inverted = false);
	void Shape::rotXdegFromC(float deg);
	void Shape::rotYdegFromC(float deg);
	void Shape::rotZdegFromC(float deg);

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
	void Object3D::setScreenFoVmeasP(Point & camC, Point & camN, PixStoring ps_ = PIXELS_STORING_GLOBAL, bool pSim_ = false);
	// Setter Screen FoV measurement. Here each Point is the Normal to the corresponding Point of setScreenFoVmeasP
	void Object3D::setScreenFoVmeasN(Point & camC, Point & camN, PixStoring ps_ = PIXELS_STORING_GLOBAL, bool pSim_ = false);
	// Setter Screen FoV measurement. Here each Point is a close to Normal vector to the corresponding Point of setScreenFoVmeasP,
	// compensating the "spherical FoV", setting a constant ScreenPlane-CameraPlane distance
	void Object3D::setScreenFoVmeasNs(Point & camC, Point & camN, PixStoring ps_ = PIXELS_STORING_GLOBAL, bool pSim_ = false);
	
	// Setter Camera(0)
	void Object3D::setCamera(Point & posC, float degPhi, float degTheta, float degRoll, Point & size, Point & c_relToP0);
	void Object3D::setCamera(Point & posC, float degPhi, float degTheta, float degRoll, Point & size, Point & c_relToP0,
		std::vector<std::vector<float>> & albedoVV, std::vector<std::vector<float>> & RVV, std::vector<std::vector<float>> & GVV, std::vector<std::vector<float>> & BVV, std::vector<std::vector<float>> & AVV);
	// Setter Laser	(1)
	void Object3D::setLaser(Point & posC, float degPhi, float degTheta, float degRoll, Point & size, Point & c_relToP0);
	void Object3D::setLaser(Point & posC, float degPhi, float degTheta, float degRoll, Point & size, Point & c_relToP0,
		std::vector<std::vector<float>> & albedoVV, std::vector<std::vector<float>> & RVV, std::vector<std::vector<float>> & GVV, std::vector<std::vector<float>> & BVV, std::vector<std::vector<float>> & AVV);
	// Setter Wall Patches (6)
	void Object3D::setWallPatches(Scene & scene, PixStoring ps_ = PIXELS_STORING_GLOBAL, bool pSim_ = false);
	// Setter Camera FoV (7)
	void Object3D::setCameraFoV(Scene & scene, float R_ = 0.0f, float G_ = 0.0f, float B_ = 1.0f, float A_ = 1.0f, PixStoring ps_ = PIXELS_STORING_GLOBAL, float distDefault = 5.0f);
	// Setter Laser Ray (8)
	void Object3D::setLaserRay(Scene & scene, float R_ = 0.0f, float G_ = 0.0f, float B_ = 1.0f, float A_ = 1.0f, PixStoring ps_ = PIXELS_STORING_GLOBAL);
	// Setter, Updater Volume Patches (9)
	void Object3D::setVolumePatchesBox(Point & boxPC, Point & boxRaxisN, float deg, Point & boxS, std::vector<int> & rowsPerFaceV, std::vector<int> & colsPerFaceV,
		std::vector<std::vector<float>> & albedoVV, std::vector<std::vector<float>> & RVV, std::vector<std::vector<float>> & GVV, std::vector<std::vector<float>> & BVV, std::vector<std::vector<float>> & AVV);
	void Object3D::updateVolumePatches_Occlusion(Info & info, Scene & scene, Frame & frame00, Frame & frame90, std::vector<int> & rowsPerFaceV, std::vector<int> & colsPerFaceV, bool loop = true, PixStoring ps_ = PIXELS_STORING_GLOBAL, bool pSim_ = false);
	void Object3D::setVolumePatches_OLD();
	void Object3D::updateVolumePatches_Occlusion_OLD(Info & info, Scene & scene, Frame & frame00, Frame & frame90, bool loop = true, PixStoring ps_ = PIXELS_STORING_GLOBAL, bool pSim_ = false);
	// Setter, Updater Pixel Patches (10)
	void Object3D::updatePatchesColor(Frame & frameMod);
	void Object3D::updatePatchesColor(Frame & frame00, Frame & frame90);
	void Object3D::setPixelPatches(Scene & scene, float distDefault = 2.0f, PixStoring ps_ = PIXELS_STORING_GLOBAL, bool pSim_ = false);
	void Object3D::setPixelPatches(Scene & scene, Frame & frame00, Frame & frame90, PixStoring ps_ = PIXELS_STORING_GLOBAL, bool pSim_ = false);
	void Object3D::updatePixelPatches_Sinusoid(Scene & scene, Frame & frame00, Frame & frame90, bool loop = true, PixStoring ps_ = PIXELS_STORING_GLOBAL, bool pSim_ = false);
	void Object3D::updatePixelPatches_Simulation(Info & info, Scene & scene, Frame & frame00, Frame & frame90, bool loop = true, PixStoring ps_ = PIXELS_STORING_GLOBAL, bool pSim_ = false);
	

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
	void Object3D::rotXYZ(float radX, float radY, float radZ, bool inverted = false);
	void Object3D::rotX(float rad);
	void Object3D::rotY(float rad);
	void Object3D::rotZ(float rad);
	// rotation absolute (from (0,0,0)), degrees (setters)
	void Object3D::rotDeg(Point & axisN, float deg);
	void Object3D::rotXYZdeg(float degX, float degY, float degZ, bool inverted = false);
	void Object3D::rotXdeg(float deg);
	void Object3D::rotYdeg(float deg);
	void Object3D::rotZdeg(float deg);
	// rotation relative (from pr), radians (setters)
	void Object3D::rotFromP(Point & axisN, float rad, Point & pr);
	void Object3D::rotXYZfromP(float radX, float radY, float radZ, Point & pr, bool inverted = false);
	void Object3D::rotXfromP(float rad, Point & pr);
	void Object3D::rotYfromP(float rad, Point & pr);
	void Object3D::rotZfromP(float rad, Point & pr);
	// rotation relative (from pr), degrees (setters)
	void Object3D::rotDegFromP(Point & axisN, float deg, Point & pr);
	void Object3D::rotXYZdegFromP(float degX, float degY, float degZ, Point & pr, bool inverted = false);
	void Object3D::rotXdegFromP(float deg, Point & pr);
	void Object3D::rotYdegFromP(float deg, Point & pr);
	void Object3D::rotZdegFromP(float deg, Point & pr);
	// rotation relative (from c), radians (setters)
	void Object3D::rotFromC(Point & axisN, float rad);
	void Object3D::rotXYZfromC(float radX, float radY, float radZ, bool inverted = false);
	void Object3D::rotXfromC(float rad);
	void Object3D::rotYfromC(float rad);
	void Object3D::rotzFromC(float rad);
	// rotation relative (from c), degrees (setters)
	void Object3D::rotDegFromC(Point & axisN, float deg);
	void Object3D::rotXYZdegFromC(float degX, float degY, float degZ, bool inverted = false);
	void Object3D::rotXdegFromC(float deg);
	void Object3D::rotYdegFromC(float deg);
	void Object3D::rotZdegFromC(float deg);
	
	// clear, add Shape to Object3D
	void Object3D::clear(bool clearAll = true);
	void Object3D::add(Shape & s0);
	void Object3D::add(Object3D & o0);
};
// ----- NON-MEMBER FUNCITONS ------------------------

void updateVolumePatches_Occlusion_antiBugThread(Info & info, Scene & scene, Frame & frame00, Frame & frame90, std::vector<int> & rowsPerFaceV, std::vector<int> & colsPerFaceV, bool loop, PixStoring ps_ = PIXELS_STORING_GLOBAL, bool pSim_ = false);
void updateVolumePatches_Occlusion_OLD_antiBugThread(Info & info, Scene & scene, Frame & frame00, Frame & frame90, bool loop = true, PixStoring ps_ = PIXELS_STORING_GLOBAL, bool pSim_ = false);
void updatePixelPatches_Sinusoid_antiBugThread(Scene & scene, Frame & frame00, Frame & frame90, bool loop = true, PixStoring ps_ = PIXELS_STORING_GLOBAL, bool pSim_ = false);
void updatePixelPatches_Simulation_antiBugThread(Info & info, Scene & scene, Frame & frame00, Frame & frame90, bool loop = true, PixStoring ps_ = PIXELS_STORING_GLOBAL, bool pSim_ = false);

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
	void Scene::setScene_DirectVision(PixStoring ps = PIXELS_STORING_GLOBAL, bool pSim_ = false);	// the same for both Sinusoid and Simulation
	// Setter Scene Occlusion
	void Scene::setScene_Occlusion(std::vector<int> & rowsPerFaceV, std::vector<int> & colsPerFaceV, PixStoring ps = PIXELS_STORING_GLOBAL, bool pSim_ = false);
	// Setter Scene Calibration Matrix
	void Scene::setScene_CalibrationMatrix(float laser_to_cam_offset_x, float laser_to_cam_offset_y, float laser_to_cam_offset_z, float dist_wall_cam,
		PixStoring ps = PIXELS_STORING_GLOBAL, bool pSim_ = false);

};


// ----------------------------------------------------------------------------------------------------------------------------------------
// ----- OTHER NON-MEMBER FUNCTIONS -------------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------------------

// mean
//float mean(std::vector<float> & vf);	// already defined in data.h/.cpp
Point mean(std::vector<Point> & vp);
// Rotation Matrix
void setRotationMatrix(float & r11, float & r12, float & r13,
	float & r21, float & r22, float & r23,
	float & r31, float & r32, float & r33,
	float const & wx, float const & wy, float const & wz, float const & rad);
// dist
float dist(Point & v0, Point & v1);
float distPow2(Point & v0, Point & v1);
// dot, cross
Point cross(Point & v0, Point & v1);
float dot(Point & v0, Point & v1);
// cos / rad / deg (rad, deg are dependent of the corresponding cos functions)
float cosVV(Point & v0, Point & v1);
float cosVN(Point & v, Point & n);
float cosNN(Point & n0, Point & n1);
float radVV(Point & v0, Point & v1);
float radVN(Point & v, Point & n);
float radNN(Point & n0, Point & n1);
float degV(Point & v0, Point & v1);
float degVN(Point & v, Point & n);
float degNN(Point & n0, Point & n1);
Point axisNVV(Point & v0, Point & v1);
// cos / rad / deg, with the XYZ/PhiTheta rotation system. (rad, deg are dependent of the corresponding cos functions)
// Note that a rotation in Phi and Theta has the same effect as a rotation in rotX(-Theta) and rotY(Phi) IN THIS ORDER.
// Phi (P) is rotation around axis OY, CW starting at OZ.
// Theta (T) is roation around axis OX', which is the axis OX rotated Phi previously. CW starting at OZ'
float cosP(Point & v);
float cosPN(Point & vN);
float cosT(Point & v);
float cosTN(Point & vN);
float radP(Point & v);
float radPN(Point & vN);
float radT(Point & v);
float radTN(Point & vN);
float degP(Point & v);
float degPN(Point & vN);
float degT(Point & v);
float degTN(Point & vN);
// set depth map
void setDepthMap(std::vector<float> & depthMap, Frame & frame00, Frame & frame90);
// simulation.cpp uses
float geometryTerm(Point & p0, Point & n0, Point & p1, Point & n1);
// distPath
float distPath2(Point & p0, Point & p1);
float distPath3(Point & p0, Point & p1, Point & p2);
float distPath4(Point & p0, Point & p1, Point & p2, Point & p3);
float distPath5(Point & p0, Point & p1, Point & p2, Point & p3, Point & p4);
// ad-hoc function to deal with p and pAll (see void Object3D::updateVolumePatches_Occlusion(...))
float getPiAll(int iAll, float* p, float* pAll, bool* pUse, int* idxOfpInpAll);
float getPiAll(int iAll, float* p, struct OCCLUSION_ADATA* ad);
#endif

