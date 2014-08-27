#ifndef __SHAPES_H
#define __SHAPES_H

#include <vector>
// http://eigen.tuxfamily.org/dox/group__QuickRefPage.html
#include <Eigen/Dense>
using namespace Eigen;


// All types of shapes
enum Shape {PT = 1, LINE = 2, TRIANGLE = 3, RECTANGLE = 4};


// POINT (sometimes also understood as a 3D VECTOR)
typedef Matrix<float, 3, 1> Point;

void rot(Point* p, Point* r_, bool degrees);				// rotates from (0.0f,0.0f,0.0f)
Point get_point_rot(Point* p, Point* r_, bool degrees);		// rotates from (0.0f,0.0f,0.0f)

// Functions Dealing with vectors
float get_cos_vectors(Point* v0, Point* v1);
float get_cos_vector_normal(Point* v0, Point* n1);
float get_cos_normals(Point* n0, Point* n1);
float get_geometry_term(Point* p0, Point* n0, Point* p1, Point* n1);
float dist_2(Point* p0, Point* p1);
float dist_3(Point* p0, Point* p1, Point* p2);
float dist_4(Point* p0, Point* p1, Point* p2, Point* p3);
float dist_5(Point* p0, Point* p1, Point* p2, Point* p3, Point* p4);
float dist_2_pow2(Point* p0, Point* p1);
void dist_2_pow2_vec (Point* p, std::vector<Point*> & vp, std::vector<float> & vd);
void get_normal(Point p0, Point p1, Point p2, Point & p_out);

// Functions to be used in the constructor:
// PointMesh(std::vector<Point*> & p_, Point* c_, Shape shape_);

// Represents a Point: returns a vector of 1 Point from 1 Point
std::vector<Point*> vp_point(Point* p);

// Represents a Line: returns a vector of 2 Points from 2 Points
std::vector<Point*> vp_line(Point* p0, Point* p1);	

// Represents a Triangle: returns a vector of 3 Points from 3 Points
std::vector<Point*> vp_triangle(Point* p0, Point* p1, Point* p2);

// Represents a Rectangle: returns a vector of 4 Points from 4 Points
std::vector<Point*> vp_rectangle(Point* p0, Point* p1, Point* p2, Point* p3);
// Represents a Rectangle: returns a vector of 4 Points from the size in x and y
// this rectangle will be in the XY plane and the first point will be (0.0f,0.0f,0.0f)
std::vector<Point*> vp_rectangle(float sx, float sy);
std::vector<Point*> vp_rectangle(Point* size);	// Point(2) will be ignored

// Represents a Box (6 rectangles): returns a vector of 24 Points (6 rectangles) from 8 Points
std::vector<Point*> vp_box(Point* p0, Point* p1, Point* p2, Point* p3, Point* p4, Point* p5, Point* p6, Point* p7);
// Represents a Box (6 rectangles): returns a vector of 24 Points (6 rectangles) from the size in x, y and z
// this fist rectangle of this box will be in the XY plane and the first point will be (0.0f,0.0f,0.0f)
std::vector<Point*> vp_box(float sx, float sy, float sz);
std::vector<Point*> vp_box(Point* size);


// POINT MESH
class PointMesh {
public:
	// Parameters
	std::vector<Point*> p;
	Shape shape;
	Point* c;	// centre of the Point Mesh (for camera or laser)
	std::vector<Point*> n;	// normal vector (gotten from the std::vector<Point*> p)
	float albedo;
	// Constructor
	PointMesh(std::vector<Point*> & p_, Shape shape_, Point* c_, float albedo_);
	PointMesh(std::vector<Point*> & p_, Shape shape_, Point* c_);		// albedo = 1.0f by default
	PointMesh(std::vector<Point*> & p_, Shape shape_, float albedo_);	// c = (0.0f,0.0f,0.0f) by default
	PointMesh(std::vector<Point*> & p_, Shape shape_);	// c = (0.0f,0.0f,0.0f), albedo = 1.0f by default
	//PointMesh(PointMesh* pm_);
	// We can create a PointMesh from a vector of PointMesh
	// for example: the 6 rectangles of a cube
	PointMesh(std::vector<PointMesh*> & pm_);
	// Functions
	// Set Normal
	void set_normal();
	// Translation
	void tra(Point* t_);
	void tra_center_to(Point* t_c_to_);
	// Rotation
	void rot(Point* r_, Point* cr_, bool degrees);
	void rot(Point* r_, bool degrees);	// rotates from (0.0f,0.0f,0.0f)
	void rot_from_c(Point* r_, bool degrees);
	void rot_to_normal(Point* n_, Point* cr_, Point scale_axis);
	void rot_to_normal(Point* n_, Point scale_axis);
	void rot_from_c_to_normal(Point* r_, Point scale_axis);
	// Transformations of the PointMesh shape
	void change_shape(Shape shape_);
	// Get area of the PointMesh (first Triangle or Rectangle)
	float get_area();
};


// OBJECT3D class
// The center will be the center of the first PointMesh
typedef std::vector<PointMesh*> Object3D;
// Functions
// Translation
void tra(Object3D* obj, Point* t_);
void tra_center_to(Object3D* obj, Point* t_c_to_);
// Rotation
void rot(Object3D* obj, Point* r_, Point* cr_, bool degrees);
void rot(Object3D* obj, Point* r_, bool degrees);	// rotates from (0.0f,0.0f,0.0f)
void rot_from_c(Object3D* obj, Point* r_, bool degrees);
void rot_to_normal(Object3D* obj, Point* n_, Point* cr_, Point scale_axis);
void rot_to_normal(Object3D* obj, Point* n_, Point scale_axis);
void rot_from_c_to_normal(Object3D* obj, Point* n_, Point scale_axis);

void dist_2_centers (Point* p, Object3D & obj3D, std::vector<float> & vd);
void dist_2_pow2_centers (Point* p, Object3D & obj3D, std::vector<float> & vd);


// OBJECT3D_SET class
typedef std::vector<Object3D*> Object3D_Set;

// returns the intersection point between a line given by 2 points (p0 and p1)
// and the Object3D obj
// all = true: checks all the PointMesh itersections and returns the closest to p0
// all = flase: only checks the first shape of the first PointMesh
// TO-DO
Point get_intersection_lineThrough_obj3D(Point* p0, Point* p1, Object3D* obj, bool all);

// returns the intersection point between a line given by 2 points (p0 and p1)
// and the PoinMesh pm_ (rectangle or triangle)
Point get_intersection_lineThrough_pointmesh(Point* p0, Point* p1, PointMesh* pm_);


// returns the intersection point between a line given by 1 points and its normal direction (p0 and n0)
// and the PoinMesh pm_ (rectangle or triangle)
Point get_intersection_linePointNormal_pointmesh(Point* p0, Point* n0, PointMesh* pm_);


#endif


