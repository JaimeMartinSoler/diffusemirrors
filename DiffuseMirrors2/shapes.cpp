
#include "math.h"
#include "shapes.h"
// http://eigen.tuxfamily.org/dox/group__QuickRefPage.html
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;

#define PI 3.14159265359

// POINT Functions

void rot(Point* p, Point* r_, bool degrees) {	// rotates from (0.0f,0.0f,0.0f)
	float rx_rad = (*r_)[0];
	float ry_rad = (*r_)[1];
	float rz_rad = (*r_)[2];
	if (degrees) {
		rx_rad *= PI / 180.0f;
		ry_rad *= PI / 180.0f;
		rz_rad *= PI / 180.0f;
	}
	AngleAxis<float> rx(rx_rad, Vector3f(1.0f, 0.0f, 0.0f));
	AngleAxis<float> ry(ry_rad, Vector3f(0.0f, 1.0f, 0.0f));
	AngleAxis<float> rz(rz_rad, Vector3f(0.0f, 0.0f, 1.0f));
	(*p) = rx * ry * rz * (*p);
}

Point get_point_rot(Point* p, Point* r_, bool degrees) {	// rotates from (0.0f,0.0f,0.0f)
	Point* p_rot = new Point((*p));
	rot(p_rot, r_, degrees);
	return (*p_rot);
}

// Functions Dealing with vectors
float get_cos_vectors(Point* v0, Point* v1) {
	return (*v0).dot((*v1)) / (((*v0).norm())*((*v1).norm()));
}
float get_cos_vector_normal(Point* v0, Point* n1) {
	return (*v0).dot((*n1)) / ((*v0).norm());
}
float get_cos_normals(Point* n0, Point* n1) {
	return (*n0).dot((*n1));
}
float get_geometry_term(Point* p0, Point* n0, Point* p1, Point* n1) {
	Point p1_p0((*p1)-(*p0));
	Point p0_p1(-p1_p0);
	return get_cos_vector_normal(&p1_p0, n0) * get_cos_vector_normal(&p0_p1, n1) / p1_p0.dot(p1_p0);
}
float dist_2(Point* p0, Point* p1) {
	Point p_diff = (*p0) - (*p1);
	return sqrt(p_diff.dot(p_diff));
}
float dist_3(Point* p0, Point* p1, Point* p2) {
	return dist_2(p0, p1) + dist_2(p1, p2);
}
float dist_4(Point* p0, Point* p1, Point* p2, Point* p3) {
	return dist_2(p0, p1) + dist_2(p1, p2) + dist_2(p2, p3);
}
float dist_5(Point* p0, Point* p1, Point* p2, Point* p3, Point* p4) {
	return dist_2(p0, p1) + dist_2(p1, p2) + dist_2(p2, p3) + dist_2(p3, p4);
}

// returns the intersection point between a line given by 2 points (p0 and p1)
// and the Object3D obj
// all = true: checks all the PointMesh itersections and returns the closest to p0
// all = flase: only checks the first shape of the first PointMesh
// TO-DO
Point get_intersection_lineThrough_obj3D(Point* p0, Point* p1, Object3D* obj, bool all) {
	ParametrizedLine<float, 3> line = ParametrizedLine<float, 3>::Through((*p0), (*p1));
	Hyperplane<float, 3> plane = Hyperplane<float, 3>::Through((*(*(*obj)[0]).p[0]), (*(*(*obj)[0]).p[1]), (*(*(*obj)[0]).p[2]));
	Point intersection = line.intersectionPoint(plane);
	return intersection;
}

// returns the intersection point between a line given by 2 points (p0 and p1)
// and the PoinMesh pm_ (rectangle or triangle)
Point get_intersection_lineThrough_pointmesh(Point* p0, Point* p1, PointMesh* pm_) {
	ParametrizedLine<float, 3> line = ParametrizedLine<float, 3>::Through((*p0), (*p1));
	Hyperplane<float, 3> plane = Hyperplane<float, 3>::Through((*(*pm_).p[0]), (*(*pm_).p[1]), (*(*pm_).p[2]));
	return line.intersectionPoint(plane);
}

// returns the intersection point between a line given by 1 points and its normal direction (p0 and n0)
// and the PoinMesh pm_ (rectangle or triangle)
Point get_intersection_linePointNormal_pointmesh(Point* p0, Point* n0, PointMesh* pm_) {
	ParametrizedLine<float, 3> line = ParametrizedLine<float, 3>::ParametrizedLine((*p0), (*n0));
	Hyperplane<float, 3> plane = Hyperplane<float, 3>::Through((*(*pm_).p[0]), (*(*pm_).p[1]), (*(*pm_).p[2]));
	return line.intersectionPoint(plane);
}



// Represents a Point: returns a vector of 1 Point from 1 Point
std::vector<Point*> vp_point(Point* p) {
	std::vector<Point*> vp(1);
	vp[0] = new Point((*p));
	return vp;
}

// Represents a Line: returns a vector of 2 Points from 2 Points
std::vector<Point*> vp_line(Point* p0, Point* p1) {
	std::vector<Point*> vp(2);
	vp[0] = new Point((*p0));
	vp[1] = new Point((*p1));
	return vp;
}

// Represents a Triangle: returns a vector of 3 Points from 3 Points
std::vector<Point*> vp_triangle(Point* p0, Point* p1, Point* p2) {
	std::vector<Point*> vp(3);
	vp[0] = new Point((*p0));
	vp[1] = new Point((*p1));
	vp[2] = new Point((*p2));
	return vp;
}

// Represents a Rectangle: returns a vector of 4 Points from 4 Points
std::vector<Point*> vp_rectangle(Point* p0, Point* p1, Point* p2, Point* p3) {
	std::vector<Point*> vp(4);
	vp[0] = new Point((*p0));
	vp[1] = new Point((*p1));
	vp[2] = new Point((*p2));
	vp[3] = new Point((*p3));
	return vp;
}
// Represents a Rectangle: returns a vector of 4 Points from the size in x and y
// this rectangle will be in the XY plane and the first point will be (0.0f,0.0f,0.0f)
std::vector<Point*> vp_rectangle(float sx, float sy) {
	return vp_rectangle(new Point(0.0f     , 0.0f     , 0.0f),
						new Point(0.0f + sx, 0.0f     , 0.0f),
						new Point(0.0f + sx, 0.0f + sy, 0.0f),
						new Point(0.0f     , 0.0f + sy, 0.0f));
}
std::vector<Point*> vp_rectangle(Point* size) {
	return vp_rectangle((*size).x(), (*size).y());
}

// Represents a Box (6 rectangles): returns a vector of 24 Points (6 rectangles) from 8 Points
std::vector<Point*> vp_box(Point* p0, Point* p1, Point* p2, Point* p3, Point* p4, Point* p5, Point* p6, Point* p7) {
	std::vector<Point*> vp(24);
	// rectangle 0
	vp[0] = new Point((*p0));
	vp[1] = new Point((*p1));
	vp[2] = new Point((*p2));
	vp[3] = new Point((*p3));
	// rectangle 1
	vp[4] = new Point((*p1));
	vp[5] = new Point((*p4));
	vp[6] = new Point((*p7));
	vp[7] = new Point((*p2));
	// rectangle 2
	vp[8]  = new Point((*p4));
	vp[9]  = new Point((*p5));
	vp[10] = new Point((*p6));
	vp[11] = new Point((*p7));
	// rectangle 3
	vp[12] = new Point((*p5));
	vp[13] = new Point((*p0));
	vp[14] = new Point((*p3));
	vp[15] = new Point((*p6));
	// rectangle 4
	vp[16] = new Point((*p5));
	vp[17] = new Point((*p4));
	vp[18] = new Point((*p1));
	vp[19] = new Point((*p0));
	// rectangle 5
	vp[20] = new Point((*p3));
	vp[21] = new Point((*p2));
	vp[22] = new Point((*p7));
	vp[23] = new Point((*p6));
	return vp;
}
// Represents a Box (6 rectangles): returns a vector of 24 Points (6 rectangles) from the size in x, y and z
// this fist rectangle of this box will be in the XY plane and the first point will be (0.0f,0.0f,0.0f)
std::vector<Point*> vp_box(float sx, float sy, float sz) {
	return vp_box(	new Point(0.0f     , 0.0f     , 0.0f     ),
					new Point(0.0f + sx, 0.0f     , 0.0f     ),
					new Point(0.0f + sx, 0.0f + sy, 0.0f     ),
					new Point(0.0f     , 0.0f + sy, 0.0f     ),
					new Point(0.0f + sx, 0.0f     , 0.0f - sz),
					new Point(0.0f     , 0.0f     , 0.0f - sz),
					new Point(0.0f     , 0.0f + sy, 0.0f - sz),
					new Point(0.0f + sx, 0.0f + sy, 0.0f - sz));
}
std::vector<Point*> vp_box(Point* size) {
	return vp_box((*size).x(), (*size).y(), (*size).z());
}


// POINT MESH Constructor
PointMesh::PointMesh(std::vector<Point*> & p_, Shape shape_, Point* c_, float albedo_) {
	p = p_;
	shape = shape_;
	c = new Point((*c_));
	set_normal();
	albedo = albedo_;
}

// POINT MESH Constructor
// albedo = 1.0f by default
PointMesh::PointMesh(std::vector<Point*> & p_, Shape shape_, Point* c_) {
	p = p_;
	shape = shape_;
	c = new Point((*c_));
	set_normal();
	albedo = 1.0f;	// albedo = 1.0f by default
}

// POINT MESH Constructor
// c = (0.0f, 0.0f, 0.0f) by default
PointMesh::PointMesh(std::vector<Point*> & p_, Shape shape_, float albedo_) {
	p = p_;
	shape = shape_;
	c = new Point(0.0f, 0.0f, 0.0f);	// c = (0.0f, 0.0f, 0.0f) by default
	set_normal();
	albedo = albedo_;
}

// POINT MESH Constructor
// c = (0.0f, 0.0f, 0.0f) by default
// albedo = 1.0f by default
PointMesh::PointMesh(std::vector<Point*> & p_, Shape shape_) {
	p = p_;
	shape = shape_;
	c = new Point(0.0f, 0.0f, 0.0f);	// c = (0.0f, 0.0f, 0.0f) by default
	set_normal();
	albedo = 1.0f;	// albedo = 1.0f by default
}

/*
// POINT MESH Constructor (copy Constructor)
// It's not a real copy, it just copies the references...
PointMesh::PointMesh(PointMesh* pm_) {
	p = pm_->p;
	shape = pm_->shape;
	c = pm_->c;
	set_normal();
}
*/

// POINT MESH Constructor
// We can create a PointMesh from a vector of PointMesh
// for example: the 6 rectangles of a cube
PointMesh::PointMesh(std::vector<PointMesh*> & pm_) {
	std::vector<Point*> p_;
	int size = 0;
	for (int i_pm = 0; i_pm < pm_.size(); i_pm++)
		size += pm_[i_pm]->p.size();
	p.reserve(size);	// preallocate memory
	for (int i_pm = 0; i_pm < pm_.size(); i_pm++)
		p.insert(p.end(), pm_[i_pm]->p.begin(), pm_[i_pm]->p.end());
	shape = pm_[0]->shape;
	c = new Point((*pm_[0]->c));
	set_normal();
}


// POINT MESH Functions

// Set Normal
void PointMesh::set_normal() {
	n = (*new std::vector<Point*>(p.size()/shape));
	if (shape == RECTANGLE) {	// normal vector of the surface
		for (int i = 0; i < n.size(); i++) {
			Point u = (*p[i*RECTANGLE + 1]) - (*p[i*RECTANGLE]);
			Point v = (*p[i*RECTANGLE + 2]) - (*p[i*RECTANGLE + 1]);
			n[i] = new Point(u.cross(v));
			(*n[i]).normalize();
		}
	}
	if (shape == TRIANGLE) {	// normal vector of the surface
		for (int i = 0; i < n.size(); i++) {
			Point u = (*p[i*TRIANGLE + 1]) - (*p[i*TRIANGLE]);
			Point v = (*p[i*TRIANGLE + 2]) - (*p[i*TRIANGLE + 1]);
			n[i] = new Point(u.cross(v));
			(*n[i]).normalize();
		}
	}
	if (shape == LINE) {		// normal vector of line (diff of points normalized)
		for (int i = 0; i < n.size(); i++) {
			n[i] = new Point((*p[i*LINE + 1]) - (*p[i*LINE]));
			(*n[i]).normalize();
		}
	}
	if (shape == PT) {			// normal vector, point normalized
		for (int i = 0; i < n.size(); i++) {
			n[i] = new Point((*p[i]));
			(*n[i]).normalize();
		}
	}
}

// Translation
void PointMesh::tra(Point* t_) {
	for (int i_p = 0; i_p < p.size(); i_p++)
		(*p[i_p]) += (*t_);
	(*c) += (*t_);
	if (shape == PT)
		set_normal();
}

void PointMesh::tra_center_to(Point* t_c_to_) {
	Point t = (*t_c_to_) - (*c);
	tra(&t);
}

// Rotation
void PointMesh::rot(Point* r_, Point* cr_, bool degrees) {
	float rx_rad = (*r_)[0];
	float ry_rad = (*r_)[1];
	float rz_rad = (*r_)[2];
	if (degrees) {
		rx_rad *= PI / 180.0f;
		ry_rad *= PI / 180.0f;
		rz_rad *= PI / 180.0f;
	}
	Translation<float, 3> t_n(-(*cr_));
	Translation<float, 3> t_p((*cr_));
	AngleAxis<float> rx(rx_rad, Vector3f(1.0f, 0.0f, 0.0f));
	AngleAxis<float> ry(ry_rad, Vector3f(0.0f, 1.0f, 0.0f));
	AngleAxis<float> rz(rz_rad, Vector3f(0.0f, 0.0f, 1.0f));
	for (int i_p = 0; i_p < p.size(); i_p++)
		(*p[i_p]) = t_p * rx * ry * rz * t_n * (*p[i_p]);
	(*c) = t_p * rx * ry * rz * t_n * (*c);
	set_normal();
}
void PointMesh::rot(Point* r_, bool degrees) {
	rot(r_, new Point(0.0f, 0.0f, 0.0f), degrees);
}
void PointMesh::rot_from_c(Point* r_, bool degrees) {
	rot(r_, c, degrees);
}

// Transformations of the PointMesh shape
void PointMesh::change_shape(Shape shape_) {
	if (shape == RECTANGLE) {
		if (shape_ == RECTANGLE) return;	// does not change
		if (shape_ == TRIANGLE) return;		// not compatible
		if (shape_ == LINE) {
			std::vector<Point*> p_(p.size() * 2);
			for (int i_r = 0; i_r < p.size(); i_r += RECTANGLE) {
				int i_l = i_r * 2;
				p_[i_l    ] = new Point((*p[i_r    ]));
				p_[i_l + 1] = new Point((*p[i_r + 1]));
				p_[i_l + 2] = new Point((*p[i_r + 1]));
				p_[i_l + 3] = new Point((*p[i_r + 2]));
				p_[i_l + 4] = new Point((*p[i_r + 2]));
				p_[i_l + 5] = new Point((*p[i_r + 3]));
				p_[i_l + 6] = new Point((*p[i_r + 3]));
				p_[i_l + 7] = new Point((*p[i_r    ]));
			}
			p = p_;
			shape = shape_;
			set_normal();
			return;
		}
		if (shape_ == PT) {
			shape = shape_;
			set_normal();
			return;
		}
	}
	if (shape == TRIANGLE) {
		if (shape_ == RECTANGLE) return;	// not compatible
		if (shape_ == TRIANGLE) return;		// does not change
		if (shape_ == LINE) {
			std::vector<Point*> p_(p.size() * 2);
			for (int i_t = 0; i_t < p.size(); i_t += TRIANGLE) {
				int i_l = i_t * 2;
				p_[i_l] = new Point((*p[i_t]));
				p_[i_l + 1] = new Point((*p[i_t + 1]));
				p_[i_l + 2] = new Point((*p[i_t + 1]));
				p_[i_l + 3] = new Point((*p[i_t + 2]));
				p_[i_l + 4] = new Point((*p[i_t + 2]));
				p_[i_l + 5] = new Point((*p[i_t]));
			}
			p = p_;
			shape = shape_;
			set_normal();
			return;
		}
		if (shape_ == PT) {
			shape = shape_;
			set_normal();
			return;
		}
	}
	if (shape == LINE) {
		if (shape_ == RECTANGLE) return;	// not compatible
		if (shape_ == TRIANGLE) return;		// not compatible
		if (shape_ == LINE) return;			// does not change
		if (shape_ == PT) {
			shape = shape_;
			set_normal();
			return;
		}
	}
	if (shape == PT) return;	// not compatible / does not change
}


// Get area of the PointMesh (first Triangle or Rectangle)
float PointMesh::get_area() {
	if (shape != TRIANGLE && shape != RECTANGLE) {
		return 0.0f;
	} else {
		Point u = (*p[1]) - (*p[0]);
		Point v = (*p[0]) - (*p[2]);
		Point cp = u.cross(v);
		float area_rect = sqrt(cp.dot(cp));
		if (shape == RECTANGLE)
			return area_rect;
		else // if (shape == TRIANGLE)
			return area_rect / 2.0f;
	}
}

// OBJECT3D Functions

// Translation
void tra(Object3D* obj, Point* t_) {
	for (int i = 0; i < (*obj).size(); i++)
		(*obj)[i]->tra(t_);
}

void tra_center_to(Object3D* obj, Point* t_c_to_) {
	Point t = (*t_c_to_) - (*(*obj)[0]->c);
	tra(obj, &t);
}

// Rotation
void rot(Object3D* obj, Point* r_, Point* cr_, bool degrees) {
	for (int i = 0; i < (*obj).size(); i++)
		(*obj)[i]->rot(r_, cr_, degrees);
}

void rot(Object3D* obj, Point* r_, bool degrees) {
	rot(obj, r_, new Point(0.0f, 0.0f, 0.0f), degrees);
}
// The center will be the center of the first PointMesh
void rot_from_c(Object3D* obj, Point* r_, bool degrees) {
	rot(obj, r_, (*obj)[0]->c, degrees);
}


