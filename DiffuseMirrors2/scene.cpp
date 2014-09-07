

#include <iostream>
#include <vector>

#include "global.h"
#include "scene.h"




// ----------------------------------------------------------------------------------------------------------------------------------------
// ----- POINT ----------------------------------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------------------

	// ----- CONSTRUCTORS ---------------------------- // Note that Constructors just call their corresponding Setter

// Constructor Default
Point::Point() {
	set();
}
// Constructor Copy
Point::Point(Point const & p0) {
	set(p0);
}
// Constructor All Parameters
Point::Point(float x_, float y_, float z_) {
	set(x_,y_,z_);
}

// ----- SETTERS --------------------------------- // Note that Constructors just call their corresponding Setter
	
// Setter Default
void Point::set() {
	x = 0.0f; y = 0.0f; z = 0.0f;
}
// Setter Copy
void Point::set(Point const & p0) {
	x = p0.x; y = p0.y; z = p0.z;
}
// Setter All Parameters
void Point::set(float x_, float y_, float z_) {
	x = x_; y = y_; z = z_;
}

// ----- OPERTATORS ------------------------------

// =
Point & Point::operator= (Point const & rhs) {
	x = rhs.x; y = rhs.y; z = rhs.z;
	return *this;
}

// +, -, *, /
Point Point::operator+ (Point const & rhs) {
	return Point(x + rhs.x, y + rhs.y, z + rhs.z);
}
Point Point::operator- (Point const & rhs) {
	return Point(x - rhs.x, y - rhs.y, z - rhs.z);
}
Point Point::operator* (Point const & rhs) {
	return Point(x * rhs.x, y * rhs.y, z * rhs.z);
}
Point Point::operator* (float rhs) {
	return Point(x * rhs, y * rhs, z * rhs);
}
Point Point::operator/ (Point const & rhs) {
	return Point(x / rhs.x, y / rhs.y, z / rhs.z);
}
Point Point::operator/ (float rhs) {
	return Point(x / rhs, y / rhs, z / rhs);
}

// +=, -=, *=, /=
Point & Point::operator+= (Point const & rhs){
	x += rhs.x; y += rhs.y; z += rhs.z;
	return *this;
}
Point & Point::operator-= (Point const & rhs){
	x -= rhs.x; y -= rhs.y; z -= rhs.z;
	return *this;
}
Point & Point::operator*= (Point const & rhs){
	x *= rhs.x; y *= rhs.y; z *= rhs.z;
	return *this;
}
Point & Point::operator*= (float rhs){
	x *= rhs; y *= rhs; z *= rhs;
	return *this;
}
Point & Point::operator/= (Point const & rhs){
	x /= rhs.x; y /= rhs.y; z /= rhs.z;
	return *this;
}
Point & Point::operator/= (float rhs){
	x /= rhs; y /= rhs; z /= rhs;
	return *this;
}

// ++, --
Point & Point::operator++() { // prefix
	++x; ++y; ++z;
	return *this;
}
Point & Point::operator++(int unused) { // postfix
	Point p_this = *this;
	++x; ++y; ++z;
	return p_this;
}
Point & Point::operator--() { // prefix
	--x; --y; --z;
	return *this;
}
Point & Point::operator--(int unused) { // postfix
	Point p_this = *this;
	--x; --y; --z;
	return p_this;
}

// ==, !=
bool Point::operator== (Point const & rhs) {
	return (x == rhs.x) && (y == rhs.y) && (z == rhs.z);
}
bool Point::operator!= (Point const & rhs) {
	return !((x == rhs.x) && (y == rhs.y) && (z == rhs.z));
}

// []
float Point::operator[] (int idx){
	switch (idx) {
	case 0: return x; break;
	case 1: return y; break;
	case 2: return z; break;
	}
}


// ----- FUNCITONS -------------------------------

// print
void Point::print() {
	std::cout << "(" << x << ", " << y << ", " << z << ")";
}
void Point::print(char* txt_left, char* txt_right) {
	std::cout << txt_left;
	print();
	std::cout << txt_right;
}
// mod
float Point::mod() {
	return sqrt(x*x + y*y + z*z);
}
// normal (get, set)
Point Point::normal() { 
	return (*this) / mod();
}
void Point::normalize() {
	(*this) /= mod();
}
// dot product, cross prduct (getters)
float Point::dot(Point const & pr) {
	return x*x + y*y + z*z;
}
Point Point::cross(Point const & pr) {
	return Point(y*pr.z - z*pr.y, z*pr.x - x*pr.z, x*pr.y - y*pr.x);
}
// translation (setter)
void Point::tra(Point const & pr) {
	x += pr.x; y += pr.y; z += pr.z;
}
// rotation optimal (cosT and sinT already calculated) absolute (from (0,0,0)), radians (setters). All rot functions call this functions internally
void Point::rotOpt(	float const & r11, float const & r12, float const & r13, 
					float const & r21, float const & r22, float const & r23, 
					float const & r31, float const & r32, float const & r33) {
	// Rodrigues' Rotation Formula:
	// http://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
	//(*this) = (*this) * cosT + axis_norm.cross(*this) * sinT + axis_norm * (axis_norm.dot(*this)) * (1 - cosT);
	float xc = x;
	float yc = y;
	x = r11*xc + r12*yc + r13*z;
	y = r21*xc + r22*yc + r23*z;
	z = r31*xc + r32*yc + r33*z;
}
void Point::rotxOpt(float cosT, float sinT) {
	float y_copy = y;
	y = y * cosT - z * sinT;
	z = y_copy * sinT + z * cosT;
}
void Point::rotyOpt(float cosT, float sinT) {
	float x_copy = x;
	x = x * cosT + z * sinT;
	z = -x_copy * sinT + z * cosT;
}
void Point::rotzOpt(float cosT, float sinT) {
	float x_copy = x;
	x = x * cosT - y * sinT;
	y = x_copy * sinT + y * cosT;
}
// rotation absolute (from (0,0,0)), radians (setters)
void Point::rot(Point & axis_norm, float rad) {
	float r11, r12, r13, r21, r22, r23, r31, r32, r33;
	setRodriguesMatrix(r11, r12, r13, r21, r22, r23, r31, r32, r33, axis_norm.x, axis_norm.y, axis_norm.z, rad);
	rotOpt(r11, r12, r13, r21, r22, r23, r31, r32, r33);
}
void Point::rotx(float rad) {
	rotxOpt(cos(rad), sin(rad));
}
void Point::roty(float rad) {
	rotyOpt(cos(rad), sin(rad));
}
void Point::rotz(float rad) {
	rotzOpt(cos(rad), sin(rad));
}
// rotation absolute (from (0,0,0)), degrees (setters)
void Point::rotDeg(Point & axis_norm, float deg) {
	rot(axis_norm, deg * PI / 180.0f);
}
void Point::rotxDeg(float deg) {
	float rad = deg * PI / 180.0f;
	rotxOpt(cos(rad), sin(rad));
}
void Point::rotyDeg(float deg) {
	float rad = deg * PI / 180.0f;
	rotyOpt(cos(rad), sin(rad));
}
void Point::rotzDeg(float deg) {
	float rad = deg * PI / 180.0f;
	rotzOpt(cos(rad), sin(rad));
}
// rotation relative (from pr), radians (setters)
void Point::rotFromP(Point & axis_norm, float rad, Point & pr) {
	float r11, r12, r13, r21, r22, r23, r31, r32, r33;
	setRodriguesMatrix(r11, r12, r13, r21, r22, r23, r31, r32, r33, axis_norm.x, axis_norm.y, axis_norm.z, rad);
	x -= pr.x; y -= pr.y; z -= pr.z;
	rotOpt(r11, r12, r13, r21, r22, r23, r31, r32, r33);
	x += pr.x; y += pr.y; z += pr.z;
}
void Point::rotxFromP(float rad, Point & pr) {
	x -= pr.x; y -= pr.y; z -= pr.z;
	rotxOpt(cos(rad), sin(rad));
	x += pr.x; y += pr.y; z += pr.z;
}
void Point::rotyFromP(float rad, Point & pr) {
	x -= pr.x; y -= pr.y; z -= pr.z;
	rotyOpt(cos(rad), sin(rad));
	x += pr.x; y += pr.y; z += pr.z;
}
void Point::rotzFromP(float rad, Point & pr) {
	x -= pr.x; y -= pr.y; z -= pr.z;
	rotzOpt(cos(rad), sin(rad));
	x += pr.x; y += pr.y; z += pr.z;
}
// rotation relative (from pr), degrees (setters)
void Point::rotDegFromP(Point & axis_norm, float deg, Point & pr) {
	rotFromP(axis_norm, deg * PI / 180.0f, pr);
}
void Point::rotxDegFromP(float deg, Point & pr) {
	rotxFromP(deg * PI / 180.0f, pr);
}
void Point::rotyDegFromP(float deg, Point & pr) {
	rotyFromP(deg * PI / 180.0f, pr);
}
void Point::rotzDegFromP(float deg, Point & pr) {
	rotzFromP(deg * PI / 180.0f, pr);
}


// ----- NON-MEMBER FUNCITONS --------------------

// normal to 3 points
Point normalTo3P(Point & p0, Point & p1, Point & p2) {
	return ((p1 - p0).cross(p2 - p1)).normal();
}
// Intersection Line-Plane. [P=Point, N=Normal]
Point int_linePN_planePN(Point & lP, Point & lN, Point & pP, Point & pN) {
	// http://en.wikipedia.org/wiki/Line%E2%80%93plane_intersection#Algebraic_form
	return lP + lN * ((pP - lP).dot(pN) / pN.dot(pN));
}
Point int_linePN_planePPP(Point & lP, Point & lN, Point & pP0, Point & pP1, Point & pP2) {
	return int_linePN_planePN(lP, lN, pP0, normalTo3P(pP0, pP1, pP2));
}
Point int_linePP_planePN(Point & lP0, Point & lP1, Point & pP, Point & pN) {
	return int_linePN_planePN(lP0, (lP1 - lP0).normal(), pP, pN);
}
Point int_linePP_planePPP(Point & lP0, Point & lP1, Point & pP0, Point & pP1, Point & pP2) {
	return int_linePN_planePN(lP0, (lP1 - lP0).normal(), pP0, normalTo3P(pP0, pP1, pP2));
}
// dist
float dist (Point & p0, Point & p1) {
	Point dif = p1 - p0;
	return dif.mod();
}
float distPow2 (Point & p0, Point & p1) {
	Point dif = p1 - p0;
	return dif.dot(dif);
}



// ----------------------------------------------------------------------------------------------------------------------------------------
// ----- SHAPES ---------------------------------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------------------

// ----- CONSTRUCTORS ---------------------------- // Note that Constructors just call their corresponding Setter

// Constructor Default
Shape::Shape() {
	set();
}
// Constructor Copy
Shape::Shape(Shape const & s) {
	set(s);
}
// Constructor All Parameters. shapeType = PT, LINE, TRIANGLE, QUAD
Shape::Shape(Point const & p0, Point const & c_,
	float albedo_, float R_, float G_, float B_, float A_, ShapeType st_) {	// params of this line has default (see scene.h)
	set(albedo_, R_, G_, B_, A_, st_);
}
Shape::Shape(Point const & p0, Point const & p1, Point const & c_,
	float albedo_, float R_, float G_, float B_, float A_, ShapeType st_) {	// params of this line has default (see scene.h)
	set(albedo_, R_, G_, B_, A_, st_);
}
Shape::Shape(Point const & p0, Point const & p1, Point const & p2, Point const & c_,
	float albedo_, float R_, float G_, float B_, float A_, ShapeType st_) {	// params of this line has default (see scene.h)
	set(albedo_, R_, G_, B_, A_, st_);
}
Shape::Shape(Point const & p0, Point const & p1, Point const & p2, Point const & p3, Point const & c_,
	float albedo_, float R_, float G_, float B_, float A_, ShapeType st_) {	// params of this line has default (see scene.h)
	set(albedo_, R_, G_, B_, A_, st_);
}


// ----- SETTERS --------------------------------- // Note that Constructors just call their corresponding Setter

// Setter Default
void Shape::set() {
	p.resize(0);
	c = Point(0.0f, 0.0f, 0.0f);
	albedo = 0.0f;
	R = 0.0f; G = 0.0f; B = 0.0f; A = 0.0f;
	st = UNKNOWN;
}
// Setter Copy
void Shape::set(Shape const & s) {
	p = s.p;
	c = s.c;
	albedo = s.albedo;
	R = s.R; G = s.G; B = s.B; A = s.A;
	st = s.st;
}
// Setter All Parameters. shapeType = PT, LINE, TRIANGLE, QUAD
void Shape::set(Point const & p0, Point const & c_) {	// params of this line has default (see scene.h)
	p.resize(1); p[0] = p0;	// C++11: p = {p0}
	c = c_;
}
void Shape::set(Point const & p0, Point const & p1, Point const & c_) {	// params of this line has default (see scene.h)
	p.resize(2); p[0] = p0;	p[1] = p1;	// C++11: p = {p0,p1}
	c = c_;
}
void Shape::set(Point const & p0, Point const & p1, Point const & p2, Point const & c_) {	// params of this line has default (see scene.h)
	p.resize(3); p[0] = p0;	p[1] = p1;	p[2] = p2;	// C++11: p = {p0,p1,p2}
	c = c_;
}
void Shape::set(Point const & p0, Point const & p1, Point const & p2, Point const & p3, Point const & c_) {	// params of this line has default (see scene.h)
	p.resize(4); p[0] = p0;	p[1] = p1;	p[2] = p2;	p[3] = p3;	// C++11: p = {p0,p1,p2,p3}
	c = c_;
}
// Setter albedo, R, G, B, A, st
void Shape::set(float albedo_, float R_, float G_ , float B_ , float A_, ShapeType st_) {
	albedo = albedo_;
	R = R_; G = G_; B = B_; A = A_;
	st = st_;
}
	
// ----- FUNCITONS -------------------------------

// normal
Point Shape::normalPT() {
	return p[0].normal();
}
Point Shape::normalLINE() {
	return (p[1] - p[0]).normal();
}
Point Shape::normalTRIANGLE() {
	normalTo3P(p[0], p[1], p[2]);
}
Point Shape::normalQUAD() {
	normalTo3P(p[0], p[1], p[2]);
}
// translation (setter)
void Shape::tra(Point const & pr) {
	for (size_t i = 0; i < p.size(); i++)
		p[i].tra(pr);
	c.tra(pr);
}
void Shape::traCto(Point & Cto) {
	Point pr = Cto - c;
	tra(pr);
}
// rotation absolute (from (0,0,0)), radians (setters)
void Shape::rot(Point & axis_norm, float rad) {
	float r11, r12, r13, r21, r22, r23, r31, r32, r33;
	setRodriguesMatrix(r11, r12, r13, r21, r22, r23, r31, r32, r33, axis_norm.x, axis_norm.y, axis_norm.z, rad);
	for (size_t i = 0; i < p.size(); i++)
		p[i].rotOpt(r11, r12, r13, r21, r22, r23, r31, r32, r33);
	c.rotOpt(r11, r12, r13, r21, r22, r23, r31, r32, r33);
}
void Shape::rotx(float rad) {
	float cosT = cos(rad);
	float sinT = sin(rad);
	for (size_t i = 0; i < p.size(); i++)
		p[i].rotxOpt(cosT, sinT);
	c.rotxOpt(cosT, sinT);
}
void Shape::roty(float rad) {
	float cosT = cos(rad);
	float sinT = sin(rad);
	for (size_t i = 0; i < p.size(); i++)
		p[i].rotyOpt(cosT, sinT);
	c.rotyOpt(cosT, sinT);
}
void Shape::rotz(float rad) {
	float cosT = cos(rad);
	float sinT = sin(rad);
	for (size_t i = 0; i < p.size(); i++)
		p[i].rotzOpt(cosT, sinT);
	c.rotzOpt(cosT, sinT);
}
// rotation absolute (from (0,0,0)), degrees (setters)
void Shape::rotDeg(Point & axis_norm, float deg) {
	rot(axis_norm, deg * PI / 180.0f);
}
void Shape::rotxDeg(float deg) {
	rotx(deg * PI / 180.0f);
}
void Shape::rotyDeg(float deg) {
	roty(deg * PI / 180.0f);
}
void Shape::rotzDeg(float deg) {
	rotz(deg * PI / 180.0f);
}
// rotation relative (from pr), radians (setters)
void Shape::rotFromP(Point & axis_norm, float rad, Point & pr) {
	float r11, r12, r13, r21, r22, r23, r31, r32, r33;
	setRodriguesMatrix(r11, r12, r13, r21, r22, r23, r31, r32, r33, axis_norm.x, axis_norm.y, axis_norm.z, rad);
	for (size_t i = 0; i < p.size(); i++) {
		p[i].x -= pr.x; p[i].y -= pr.y; p[i].z -= pr.z;
		p[i].rotOpt(r11, r12, r13, r21, r22, r23, r31, r32, r33);
		p[i].x += pr.x; p[i].y += pr.y; p[i].z += pr.z;
	}
	c.x -= pr.x; c.y -= pr.y; c.z -= pr.z;
	c.rotOpt(r11, r12, r13, r21, r22, r23, r31, r32, r33);
	c.x += pr.x; c.y += pr.y; c.z += pr.z;
}
void Shape::rotxFromP(float rad, Point & pr) {
	float cosT = cos(rad);
	float sinT = sin(rad);
	for (size_t i = 0; i < p.size(); i++) {
		p[i].x -= pr.x; p[i].y -= pr.y; p[i].z -= pr.z;
		p[i].rotxOpt(cosT, sinT);
		p[i].x += pr.x; p[i].y += pr.y; p[i].z += pr.z;
	}
	c.x -= pr.x; c.y -= pr.y; c.z -= pr.z;
	c.rotxOpt(cosT, sinT);
	c.x += pr.x; c.y += pr.y; c.z += pr.z;
}
void Shape::rotyFromP(float rad, Point & pr) {
	float cosT = cos(rad);
	float sinT = sin(rad);
	for (size_t i = 0; i < p.size(); i++) {
		p[i].x -= pr.x; p[i].y -= pr.y; p[i].z -= pr.z;
		p[i].rotyOpt(cosT, sinT);
		p[i].x += pr.x; p[i].y += pr.y; p[i].z += pr.z;
	}
	c.x -= pr.x; c.y -= pr.y; c.z -= pr.z;
	c.rotyOpt(cosT, sinT);
	c.x += pr.x; c.y += pr.y; c.z += pr.z;
}
void Shape::rotzFromP(float rad, Point & pr) {
	float cosT = cos(rad);
	float sinT = sin(rad);
	for (size_t i = 0; i < p.size(); i++) {
		p[i].x -= pr.x; p[i].y -= pr.y; p[i].z -= pr.z;
		p[i].rotzOpt(cosT, sinT);
		p[i].x += pr.x; p[i].y += pr.y; p[i].z += pr.z;
	}
	c.x -= pr.x; c.y -= pr.y; c.z -= pr.z;
	c.rotzOpt(cosT, sinT);
	c.x += pr.x; c.y += pr.y; c.z += pr.z;
}
// rotation relative (from pr), degrees (setters)
void Shape::rotDegFromP(Point & axis_norm, float deg, Point & pr) {
	rotFromP(axis_norm, deg * PI / 180.0f, pr);
}
void Shape::rotxDegFromP(float deg, Point & pr) {
	rotxFromP(deg * PI / 180.0f, pr);
}
void Shape::rotyDegFromP(float deg, Point & pr) {
	rotyFromP(deg * PI / 180.0f, pr);
}
void Shape::rotzDegFromP(float deg, Point & pr) {
	rotzFromP(deg * PI / 180.0f, pr);
}
// rotation relative (from c), radians (setters)
void Shape::rotFromC(Point & axis_norm, float rad) {
	float r11, r12, r13, r21, r22, r23, r31, r32, r33;
	setRodriguesMatrix(r11, r12, r13, r21, r22, r23, r31, r32, r33, axis_norm.x, axis_norm.y, axis_norm.z, rad);
	for (size_t i = 0; i < p.size(); i++) {
		p[i].x -= c.x; p[i].y -= c.y; p[i].z -= c.z;
		p[i].rotOpt(r11, r12, r13, r21, r22, r23, r31, r32, r33);
		p[i].x += c.x; p[i].y += c.y; p[i].z += c.z;
	}
}
void Shape::rotxFromC(float rad) {
	float cosT = cos(rad);
	float sinT = sin(rad);
	for (size_t i = 0; i < p.size(); i++) {
		p[i].x -= c.x; p[i].y -= c.y; p[i].z -= c.z;
		p[i].rotxOpt(cosT, sinT);
		p[i].x += c.x; p[i].y += c.y; p[i].z += c.z;
	}
}
void Shape::rotyFromC(float rad) {
	float cosT = cos(rad);
	float sinT = sin(rad);
	for (size_t i = 0; i < p.size(); i++) {
		p[i].x -= c.x; p[i].y -= c.y; p[i].z -= c.z;
		p[i].rotyOpt(cosT, sinT);
		p[i].x += c.x; p[i].y += c.y; p[i].z += c.z;
	}
}
void Shape::rotzFromC(float rad) {
	float cosT = cos(rad);
	float sinT = sin(rad);
	for (size_t i = 0; i < p.size(); i++) {
		p[i].x -= c.x; p[i].y -= c.y; p[i].z -= c.z;
		p[i].rotzOpt(cosT, sinT);
		p[i].x += c.x; p[i].y += c.y; p[i].z += c.z;
	}
}
// rotation relative (from c), degrees (setters)
void Shape::rotDegFromC(Point & axis_norm, float deg) {
	rotFromC(axis_norm, deg * PI / 180.0f);
}
void Shape::rotxDegFromC(float deg) {
	rotxFromC(deg * PI / 180.0f);
}
void Shape::rotyDegFromC(float deg) {
	rotyFromC(deg * PI / 180.0f);
}
void Shape::rotzDegFromC(float deg) {
	rotzFromC(deg * PI / 180.0f);
}

// add, clear Point to Shape
void Shape::clear(bool clearAll){
	p.clear();
	if (clearAll) {
		c = Point(0.0f, 0.0f, 0.0f);
		albedo = 0.0f;
		R = 0.0f; G = 0.0f; B = 0.0f; A = 0.0f;
		st = UNKNOWN;
	}
}
void Shape::add(Point & p0, bool updateST) {
	p.push_back(p0);
	if (updateST)
		st = (ShapeType)((int)(st + 1));
}

// ----- NON-MEMBER FUNCITONS ----------------------------

// normal to Shape (TRIANGLE or QUAD)
Point normalToShape(Shape & s0) {
	normalTo3P(s0.p[0], s0.p[1], s0.p[2]);
}
// Intersection Line-Plane. [P=Point, N=Normal]
Point int_linePN_shape(Point & lP, Point & lN, Shape & s0) {
	return int_linePN_planePN(lP, lN, s0.p[0], normalTo3P(s0.p[0], s0.p[1], s0.p[2]));
}
Point int_linePP_shape(Point & lP0, Point & lP1, Shape & s0) {
	return int_linePN_planePN(lP0, (lP1 - lP0).normal(), s0.p[0], normalTo3P(s0.p[0], s0.p[1], s0.p[2]));
}




// ----------------------------------------------------------------------------------------------------------------------------------------
// ----- OBJECT3D -------------------------------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------------------


// ----- CONSTRUCTORS ---------------------------- // Note that Constructors just call their corresponding Setter

// Constructor Default
Object3D::Object3D() {
	set();
}
// Constructor Copy
Object3D::Object3D(Object3D const & o0) {
	set(o0);
}
// Constructor All Parameters.
Object3D::Object3D(std::vector<Shape> & s_, Object3DType ot_, Pixels_storing ps_) { // default: ps_ = UNKNOWN_PIXELS_STORING
	set(s_, ot_);
	set(ps_);
}
// Constructor Object3DType and Pixels_storing, and s.resize(0);
Object3D::Object3D(Object3DType ot_, Pixels_storing ps_) { // default: ps_ = UNKNOWN_PIXELS_STORING
	s.resize(0);
	set(ot_);
	set(ps_);
}
// Constructor Box
Object3D::Object3D(Point & boxPC, Point & boxRaxisN, float deg, Point & boxS, Point & boxC_relToP0) {
	setBox (boxPC, boxRaxisN, deg, boxS, boxC_relToP0);
}
Object3D::Object3D(Point & boxPC, Point & boxRaxisN, float deg, Point & boxS, Point & boxC_relToP0,
	std::vector<float> & albedoV, std::vector<float> & RV, std::vector<float> & GV, std::vector<float> & BV, std::vector<float> & AV) {
	setBox (boxPC, boxRaxisN, deg, boxS, boxC_relToP0, albedoV, RV, GV, BV, AV);
}


// ----- SETTERS --------------------------------- // Note that Constructors just call their corresponding Setter
	
// Setter Default
void Object3D::set() {
	s.resize(0);
	ot = UNKOWN;
	ps = UNKNOWN_PIXELS_STORING;
}
// Setter Copy
void Object3D::set(Object3D const & o0) {
	s = o0.s;
	ot = o0.ot;
	ps = o0.ps;
}
// Setter All Parameters.
void Object3D::set(std::vector<Shape> & s_, Object3DType ot_) {
	s = s_;
	ot = ot_;
}
// Setter Object3DType and Pixels_storing.
void Object3D::set(Object3DType ot_) {
	ot = ot_;
}
// Setter Object3DType Pixels_storing.
void Object3D::set(Pixels_storing ps_) {
	ps = ps_;
}
// Setter Box
void Object3D::setBox(Point & boxPC, Point & boxRaxisN, float deg, Point & boxS, Point & boxC_relToP0) {
	std::vector<float> albedoV_stub;
	std::vector<float> RV_stub;
	std::vector<float> GV_stub;
	std::vector<float> BV_stub;
	std::vector<float> AV_stub;
	setBox(boxPC, boxRaxisN, deg, boxS, boxC_relToP0, albedoV_stub, RV_stub, GV_stub, BV_stub, AV_stub);

}
void Object3D::setBox(Point & boxPC, Point & boxRaxisN, float deg, Point & boxS, Point & boxC_relToP0,
		std::vector<float> & albedoV, std::vector<float> & RV, std::vector<float> & GV, std::vector<float> & BV, std::vector<float> & AV) {
	
	int s_size = 6;
	s.clear();
	s.resize(s_size);
	Point p0, p1, p2, p3;

	int albedo_default = 1.0f;
	int R_default = 1.0f;
	int G_default = 1.0f;
	int B_default = 1.0f;
	int A_default = 1.0f;

	for (size_t ai = albedoV.size(); ai < s_size; ai++)
		albedoV.push_back(albedo_default);
	for (size_t Ri = RV.size(); Ri < s_size; Ri++)
		RV.push_back(R_default);
	for (size_t Gi = GV.size(); Gi < s_size; Gi++)
		GV.push_back(G_default);
	for (size_t Bi = BV.size(); Bi < s_size; Bi++)
		BV.push_back(B_default);
	for (size_t Ai = AV.size(); Ai < s_size; Ai++)
		AV.push_back(A_default);

	// rectangle 0: FRONT
	p0 = Point(0.0f, 0.0f, 0.0f);
	p1 = Point(0.0f + boxS.x, 0.0f, 0.0f);
	p2 = Point(0.0f + boxS.x, 0.0f + boxS.y, 0.0f);
	p3 = Point(0.0f, 0.0f + boxS.y, 0.0f);
	s[FRONT].set(p0, p1, p2, p3, boxC_relToP0);
	s[FRONT].set(albedoV[FRONT], RV[FRONT], GV[FRONT], BV[FRONT], AV[FRONT], QUAD);
	// rectangle 1: RIGHT
	p0 = Point(0.0f + boxS.x, 0.0f, 0.0f);
	p1 = Point(0.0f + boxS.x, 0.0f, 0.0f - boxS.z);
	p2 = Point(0.0f + boxS.x, 0.0f + boxS.y, 0.0f - boxS.z);
	p3 = Point(0.0f + boxS.x, 0.0f + boxS.y, 0.0f);
	s[RIGHT].set(p0, p1, p2, p3, p0);
	s[RIGHT].set(albedoV[RIGHT], RV[RIGHT], GV[RIGHT], BV[RIGHT], AV[RIGHT], QUAD);
	// rectangle 2: BACK
	p0 = Point(0.0f + boxS.x, 0.0f, 0.0f - boxS.z);
	p1 = Point(0.0f, 0.0f, 0.0f - boxS.z);
	p2 = Point(0.0f, 0.0f + boxS.y, 0.0f - boxS.z);
	p3 = Point(0.0f + boxS.x, 0.0f + boxS.y, 0.0f - boxS.z);
	s[BACK].set(p0, p1, p2, p3, p0);
	s[BACK].set(albedoV[BACK], RV[BACK], GV[BACK], BV[BACK], AV[BACK], QUAD);
	// rectangle 3: LEFT
	p0 = Point(0.0f, 0.0f, 0.0f - boxS.z);
	p1 = Point(0.0f, 0.0f, 0.0f);
	p2 = Point(0.0f, 0.0f + boxS.y, 0.0f);
	p3 = Point(0.0f, 0.0f + boxS.y, 0.0f - boxS.z);
	s[LEFT].set(p0, p1, p2, p3, p0);
	s[LEFT].set(albedoV[LEFT], RV[LEFT], GV[LEFT], BV[LEFT], AV[LEFT], QUAD);
	// rectangle 4: BOTTOM
	p0 = Point(0.0f, 0.0f, 0.0f - boxS.z);
	p1 = Point(0.0f + boxS.x, 0.0f, 0.0f - boxS.z);
	p2 = Point(0.0f + boxS.x, 0.0f, 0.0f);
	p3 = Point(0.0f, 0.0f, 0.0f);
	s[BOTTOM].set(p0, p1, p2, p3, p0);
	s[BOTTOM].set(albedoV[BOTTOM], RV[BOTTOM], GV[BOTTOM], BV[BOTTOM], AV[BOTTOM], QUAD);
	// rectangle 5: TOP
	p0 = Point(0.0f, 0.0f + boxS.y, 0.0f);
	p1 = Point(0.0f + boxS.x, 0.0f + boxS.y, 0.0f);
	p2 = Point(0.0f + boxS.x, 0.0f + boxS.y, 0.0f - boxS.z);
	p3 = Point(0.0f, 0.0f + boxS.y, 0.0f - boxS.z);
	s[TOP].set(p0, p1, p2, p3, p0);
	s[TOP].set(albedoV[TOP], RV[TOP], GV[TOP], BV[TOP], AV[TOP], QUAD);
	
	// Transformaitons
	traCto(boxPC);
	rotDegFromC(boxRaxisN,deg);
}

// ----- FUNCITONS -------------------------------

// normal
Point Object3D::normalPT() {
	return s[0].p[0].normal();
}
Point Object3D::normalLINE() {
	return (s[0].p[1] - s[0].p[0]).normal();
}
Point Object3D::normalTRIANGLE() {
	normalTo3P(s[0].p[0], s[0].p[1], s[0].p[2]);
}
Point Object3D::normalQUAD() {
	normalTo3P(s[0].p[0], s[0].p[1], s[0].p[2]);
}
// translation (setter)
void Object3D::tra(Point const & pr) {
	for (size_t j = 0; j < s.size(); j++) {
		for (size_t i = 0; i < s[j].p.size(); i++)
			s[j].p[i].tra(pr);
		s[j].c.tra(pr);
	}
}
void Object3D::traCto(Point & Cto) {
	Point pr = Cto - s[0].c;
	tra(pr);
}
// rotation absolute (from (0,0,0)), radians (setters)
void Object3D::rot(Point & axis_norm, float rad) {
	float r11, r12, r13, r21, r22, r23, r31, r32, r33;
	setRodriguesMatrix(r11, r12, r13, r21, r22, r23, r31, r32, r33, axis_norm.x, axis_norm.y, axis_norm.z, rad);
	for (size_t j = 0; j < s.size(); j++) {
		for (size_t i = 0; i < s[j].p.size(); i++)
			s[j].p[i].rotOpt(r11, r12, r13, r21, r22, r23, r31, r32, r33);
		s[j].c.rotOpt(r11, r12, r13, r21, r22, r23, r31, r32, r33);
	}
}
void Object3D::rotx(float rad) {
	float cosT = cos(rad);
	float sinT = sin(rad);
	for (size_t j = 0; j < s.size(); j++) {
		for (size_t i = 0; i < s[j].p.size(); i++)
			s[j].p[i].rotxOpt(cosT, sinT);
		s[j].c.rotxOpt(cosT, sinT);
	}
}
void Object3D::roty(float rad) {
	float cosT = cos(rad);
	float sinT = sin(rad);
	for (size_t j = 0; j < s.size(); j++) {
		for (size_t i = 0; i < s[j].p.size(); i++)
			s[j].p[i].rotyOpt(cosT, sinT);
		s[j].c.rotyOpt(cosT, sinT);
	}
}
void Object3D::rotz(float rad) {
	float cosT = cos(rad);
	float sinT = sin(rad);
	for (size_t j = 0; j < s.size(); j++) {
		for (size_t i = 0; i < s[j].p.size(); i++)
			s[j].p[i].rotzOpt(cosT, sinT);
		s[j].c.rotzOpt(cosT, sinT);
	}
}
// rotation absolute (from (0,0,0)), degrees (setters)
void Object3D::rotDeg(Point & axis_norm, float deg) {
	rot(axis_norm, deg * PI / 180.0f);
}
void Object3D::rotxDeg(float deg) {
	rotx(deg * PI / 180.0f);
}
void Object3D::rotyDeg(float deg) {
	roty(deg * PI / 180.0f);
}
void Object3D::rotzDeg(float deg) {
	rotz(deg * PI / 180.0f);
}
// rotation relative (from pr), radians (setters)
void Object3D::rotFromP(Point & axis_norm, float rad, Point & pr) {
	float r11, r12, r13, r21, r22, r23, r31, r32, r33;
	setRodriguesMatrix(r11, r12, r13, r21, r22, r23, r31, r32, r33, axis_norm.x, axis_norm.y, axis_norm.z, rad);
	for (size_t j = 0; j < s.size(); j++) {
		for (size_t i = 0; i < s[j].p.size(); i++) {
			s[j].p[i].x -= pr.x; s[j].p[i].y -= pr.y; s[j].p[i].z -= pr.z;
			s[j].p[i].rotOpt(r11, r12, r13, r21, r22, r23, r31, r32, r33);
			s[j].p[i].x += pr.x; s[j].p[i].y += pr.y; s[j].p[i].z += pr.z;
		}
		s[j].c.x -= pr.x; s[j].c.y -= pr.y; s[j].c.z -= pr.z;
		s[j].c.rotOpt(r11, r12, r13, r21, r22, r23, r31, r32, r33);
		s[j].c.x += pr.x; s[j].c.y += pr.y; s[j].c.z += pr.z;
	}
}
void Object3D::rotxFromP(float rad, Point & pr) {
	float cosT = cos(rad);
	float sinT = sin(rad);
	for (size_t j = 0; j < s.size(); j++) {
		for (size_t i = 0; i < s[j].p.size(); i++) {
			s[j].p[i].x -= pr.x; s[j].p[i].y -= pr.y; s[j].p[i].z -= pr.z;
			s[j].p[i].rotxOpt(cosT, sinT);
			s[j].p[i].x += pr.x; s[j].p[i].y += pr.y; s[j].p[i].z += pr.z;
		}
		s[j].c.x -= pr.x; s[j].c.y -= pr.y; s[j].c.z -= pr.z;
		s[j].c.rotxOpt(cosT, sinT);
		s[j].c.x += pr.x; s[j].c.y += pr.y; s[j].c.z += pr.z;
	}
}
void Object3D::rotyFromP(float rad, Point & pr) {
	float cosT = cos(rad);
	float sinT = sin(rad);
	for (size_t j = 0; j < s.size(); j++) {
		for (size_t i = 0; i < s[j].p.size(); i++) {
			s[j].p[i].x -= pr.x; s[j].p[i].y -= pr.y; s[j].p[i].z -= pr.z;
			s[j].p[i].rotyOpt(cosT, sinT);
			s[j].p[i].x += pr.x; s[j].p[i].y += pr.y; s[j].p[i].z += pr.z;
		}
		s[j].c.x -= pr.x; s[j].c.y -= pr.y; s[j].c.z -= pr.z;
		s[j].c.rotyOpt(cosT, sinT);
		s[j].c.x += pr.x; s[j].c.y += pr.y; s[j].c.z += pr.z;
	}
}
void Object3D::rotzFromP(float rad, Point & pr) {
	float cosT = cos(rad);
	float sinT = sin(rad);
	for (size_t j = 0; j < s.size(); j++) {
		for (size_t i = 0; i < s[j].p.size(); i++) {
			s[j].p[i].x -= pr.x; s[j].p[i].y -= pr.y; s[j].p[i].z -= pr.z;
			s[j].p[i].rotzOpt(cosT, sinT);
			s[j].p[i].x += pr.x; s[j].p[i].y += pr.y; s[j].p[i].z += pr.z;
		}
		s[j].c.x -= pr.x; s[j].c.y -= pr.y; s[j].c.z -= pr.z;
		s[j].c.rotzOpt(cosT, sinT);
		s[j].c.x += pr.x; s[j].c.y += pr.y; s[j].c.z += pr.z;
	}
}
// rotation relative (from pr), degrees (setters)
void Object3D::rotDegFromP(Point & axis_norm, float deg, Point & pr) {
	rotFromP(axis_norm, deg * PI / 180.0f, pr);
}
void Object3D::rotxDegFromP(float deg, Point & pr) {
	rotxFromP(deg * PI / 180.0f, pr);
}
void Object3D::rotyDegFromP(float deg, Point & pr) {
	rotyFromP(deg * PI / 180.0f, pr);
}
void Object3D::rotzDegFromP(float deg, Point & pr) {
	rotzFromP(deg * PI / 180.0f, pr);
}
// rotation relative (from c), radians (setters)
void Object3D::rotFromC(Point & axis_norm, float rad) {
	float r11, r12, r13, r21, r22, r23, r31, r32, r33;
	setRodriguesMatrix(r11, r12, r13, r21, r22, r23, r31, r32, r33, axis_norm.x, axis_norm.y, axis_norm.z, rad);
	for (size_t j = 0; j < s.size(); j++) {
		for (size_t i = 0; i < s[j].p.size(); i++) {
			s[j].p[i].x -= s[0].c.x; s[j].p[i].y -= s[0].c.y; s[j].p[i].z -= s[0].c.z;
			s[j].p[i].rotOpt(r11, r12, r13, r21, r22, r23, r31, r32, r33);
			s[j].p[i].x += s[0].c.x; s[j].p[i].y += s[0].c.y; s[j].p[i].z += s[0].c.z;
		}
		if (j >= 1) {
			s[j].c.x -= s[0].c.x; s[j].c.y -= s[0].c.y; s[j].c.z -= s[0].c.z;
			s[j].c.rotOpt(r11, r12, r13, r21, r22, r23, r31, r32, r33);
			s[j].c.x += s[0].c.x; s[j].c.y += s[0].c.y; s[j].c.z += s[0].c.z;
		}
	}
}
void Object3D::rotxFromC(float rad) {
	float cosT = cos(rad);
	float sinT = sin(rad);
	for (size_t j = 0; j < s.size(); j++) {
		for (size_t i = 0; i < s[j].p.size(); i++) {
			s[j].p[i].x -= s[0].c.x; s[j].p[i].y -= s[0].c.y; s[j].p[i].z -= s[0].c.z;
			s[j].p[i].rotxOpt(cosT, sinT);
			s[j].p[i].x += s[0].c.x; s[j].p[i].y += s[0].c.y; s[j].p[i].z += s[0].c.z;
		}
		if (j >= 1) {
			s[j].c.x -= s[0].c.x; s[j].c.y -= s[0].c.y; s[j].c.z -= s[0].c.z;
			s[j].c.rotxOpt(cosT, sinT);
			s[j].c.x += s[0].c.x; s[j].c.y += s[0].c.y; s[j].c.z += s[0].c.z;
		}
	}
}
void Object3D::rotyFromC(float rad) {
	float cosT = cos(rad);
	float sinT = sin(rad);
	for (size_t j = 0; j < s.size(); j++) {
		for (size_t i = 0; i < s[j].p.size(); i++) {
			s[j].p[i].x -= s[0].c.x; s[j].p[i].y -= s[0].c.y; s[j].p[i].z -= s[0].c.z;
			s[j].p[i].rotyOpt(cosT, sinT);
			s[j].p[i].x += s[0].c.x; s[j].p[i].y += s[0].c.y; s[j].p[i].z += s[0].c.z;
		}
		if (j >= 1) {
			s[j].c.x -= s[0].c.x; s[j].c.y -= s[0].c.y; s[j].c.z -= s[0].c.z;
			s[j].c.rotyOpt(cosT, sinT);
			s[j].c.x += s[0].c.x; s[j].c.y += s[0].c.y; s[j].c.z += s[0].c.z;
		}
	}
}
void Object3D::rotzFromC(float rad) {
	float cosT = cos(rad);
	float sinT = sin(rad);
	for (size_t j = 0; j < s.size(); j++) {
		for (size_t i = 0; i < s[j].p.size(); i++) {
			s[j].p[i].x -= s[0].c.x; s[j].p[i].y -= s[0].c.y; s[j].p[i].z -= s[0].c.z;
			s[j].p[i].rotzOpt(cosT, sinT);
			s[j].p[i].x += s[0].c.x; s[j].p[i].y += s[0].c.y; s[j].p[i].z += s[0].c.z;
		}
		if (j >= 1) {
			s[j].c.x -= s[0].c.x; s[j].c.y -= s[0].c.y; s[j].c.z -= s[0].c.z;
			s[j].c.rotzOpt(cosT, sinT);
			s[j].c.x += s[0].c.x; s[j].c.y += s[0].c.y; s[j].c.z += s[0].c.z;
		}
	}
}
// rotation relative (from c), degrees (setters)
void Object3D::rotDegFromC(Point & axis_norm, float deg) {
	rotFromC(axis_norm, deg * PI / 180.0f);
}
void Object3D::rotxDegFromC(float deg) {
	rotxFromC(deg * PI / 180.0f);
}
void Object3D::rotyDegFromC(float deg) {
	rotyFromC(deg * PI / 180.0f);
}
void Object3D::rotzDegFromC(float deg) {
	rotzFromC(deg * PI / 180.0f);
}

// clear, add Shape to Object3D
void Object3D::clear(bool clearAll) {
	s.clear();
	if (clearAll) {
		ot = UNKOWN;
		ps = UNKNOWN_PIXELS_STORING;
	}
}
void Object3D::add(Shape & s0) {
	s.push_back(s0);
}
void Object3D::add(Object3D & s0) {
	int s_size = s.size();
	int s0_size = s0.s.size();
	s.resize(s_size + s0_size);
	for (size_t i = 0; i < s0_size; i++)
		s[s_size+i] = s0.s[i];
}


// ----- NON-MEMBER FUNCITONS ----------------------------

// normal to Shape (TRIANGLE or QUAD)
Point normalToObject3D(Object3D & o0) {
	normalTo3P(o0.s[0].p[0], o0.s[0].p[1], o0.s[0].p[2]);
}
// Intersection Line-Plane. [P=Point, N=Normal]
Point int_linePN_object3D(Point & lP, Point & lN, Object3D & o0) {
	return int_linePN_planePN(lP, lN, o0.s[0].p[0], normalTo3P(o0.s[0].p[0], o0.s[0].p[1], o0.s[0].p[2]));
}
Point int_linePP_object3D(Point & lP0, Point & lP1, Object3D & o0) {
	return int_linePN_planePN(lP0, (lP1 - lP0).normal(), o0.s[0].p[0], normalTo3P(o0.s[0].p[0], o0.s[0].p[1], o0.s[0].p[2]));
}




// ----------------------------------------------------------------------------------------------------------------------------------------
// ----- SCENE ----------------------------------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------------------

// ----- CONSTRUCTORS ---------------------------- // Note that Constructors just call their corresponding Setter

// Constructor Default
Scene::Scene() {
	set();
}


// ----- SETTERS --------------------------------- // Note that Constructors just call their corresponding Setter
	
// Setter Default
void Scene::set() {
	o.resize(UNKOWN);
}
	

// ----- FUNCITONS -------------------------------

// clear, add Object3D to Scene
void Scene::clear() {
	o.clear();
}
void Scene::add(Object3D & o0) {
	if (o0.ot == UNKOWN) {
		if (o.size() < UNKOWN + 1) {
			o.resize(UNKOWN + 1);
			o[UNKOWN] = o0;
		} else 
			o.push_back(o0);
		return; 
	}
	else {
		if (o.size() < o0.ot + 1)
			o.resize(o0.ot + 1);
		o[o0.ot] = o0;
	}
}


// Setter Camera
void Scene::setCamera(Point & posC, Point & axisN, float deg, Point & size, Point & c_relToP0) {
	std::vector<std::vector<float>> albedoVV_stub;
	std::vector<std::vector<float>> RVV_stub;
	std::vector<std::vector<float>> GVV_stub;
	std::vector<std::vector<float>> BVV_stub;
	std::vector<std::vector<float>> AVV_stub;
	setCamera(posC, axisN, deg, size, c_relToP0, albedoVV_stub, RVV_stub, GVV_stub, BVV_stub, AVV_stub);
}
void Scene::setCamera(Point & posC, Point & axisN, float deg, Point & size, Point & c_relToP0,
	std::vector<std::vector<float>> & albedoVV, std::vector<std::vector<float>> & RVV, std::vector<std::vector<float>> & GVV, std::vector<std::vector<float>> & BVV, std::vector<std::vector<float>> & AVV) {
	
	o[CAMERA].clear();
	o[CAMERA].ot = CAMERA;
	o[CAMERA].ps = UNKNOWN_PIXELS_STORING;
	int s_per_box = 6;
	int boxes = 4;
	int s_size = boxes * s_per_box;
	
	int albedo_default = 1.0f;
	int R_default = 1.0f;
	int G_default = 1.0f;
	int B_default = 1.0f;
	int A_default = 1.0f;
	albedoVV.resize(boxes);
	RVV.resize(boxes);
	GVV.resize(boxes);
	BVV.resize(boxes);
	AVV.resize(boxes);
	Point stub0 (0.0f,0.0f,0.0f);

	for (int bi = 0; bi < boxes; bi++) {
		for (size_t ai = albedoVV[bi].size(); ai < s_size; ai++)
			albedoVV[bi].push_back(albedo_default);
		for (size_t Ri = RVV[bi].size(); Ri < s_size; Ri++)
			RVV[bi].push_back(R_default);
		for (size_t Gi = GVV[bi].size(); Gi < s_size; Gi++)
			GVV[bi].push_back(G_default);
		for (size_t Bi = BVV[bi].size(); Bi < s_size; Bi++)
			BVV[bi].push_back(B_default);
		for (size_t Ai = AVV[bi].size(); Ai < s_size; Ai++)
			AVV[bi].push_back(A_default);
	}
	
	// Main box of the camera
	Object3D mainBox(c_relToP0, stub0, 0, size, c_relToP0, albedoVV[0], RVV[0], GVV[0], BVV[0], AVV[0]);
	o[CAMERA].add(mainBox);

	// Stick box of the camera
	float base_height = 0.02f;
	Point stickPC = mean(mainBox.s[BOTTOM].p);
	Point stickS (0.025f, posC.y - base_height - c_relToP0.y, 0.025f);
	Point stickC (stickS.x/2.0f, stickS.y, -stickS.z/2.0f);
	Object3D stickBox(stickPC, stub0, 0.0f, stickS, stickC, albedoVV[1], RVV[1], GVV[1], BVV[1], AVV[1]);
	o[CAMERA].add(stickBox);

	// Base box of the camera
	Point basePC = mean(stickBox.s[BOTTOM].p);
	Point baseS (0.14f, base_height, 0.14f);
	Point baseC(baseS.x / 2.0f, baseS.y, -baseS.z / 2.0f);
	Object3D baseBox (basePC, stub0, 0.0f, baseS, baseC, albedoVV[2], RVV[2], GVV[2], BVV[2], AVV[2]);
	o[CAMERA].add(baseBox);

	// Lens box of the camera
	Point lensPC = mean(mainBox.s[FRONT].p);
	Point lensS (0.08f, 0.08f, 0.1f);
	Point lensC(lensS.x / 2.0f, lensS.y / 2.0f, -lensS.z);
	Object3D lensBox (lensPC, stub0, 0.0f, lensS, lensC, albedoVV[3], RVV[3], GVV[3], BVV[3], AVV[3]);
	lensBox.s.erase(lensBox.s.begin() + FRONT);
	o[CAMERA].add(lensBox);
	
	// Transformaitons
	o[CAMERA].traCto(posC);
	o[CAMERA].rotDegFromC(axisN, deg);
}
// Setter Laser
void Scene::setLaser(Point & posC, Point & axisN, float deg, Point & size, Point & c_relToP0) {
	std::vector<std::vector<float>> albedoVV_stub;
	std::vector<std::vector<float>> RVV_stub;
	std::vector<std::vector<float>> GVV_stub;
	std::vector<std::vector<float>> BVV_stub;
	std::vector<std::vector<float>> AVV_stub;
	setLaser(posC, axisN, deg, size, c_relToP0, albedoVV_stub, RVV_stub, GVV_stub, BVV_stub, AVV_stub);
}
void Scene::setLaser(Point & posC, Point & axisN, float deg, Point & size, Point & c_relToP0,
	std::vector<std::vector<float>> & albedoVV, std::vector<std::vector<float>> & RVV, std::vector<std::vector<float>> & GVV, std::vector<std::vector<float>> & BVV, std::vector<std::vector<float>> & AVV) {

	o[LASER].clear();
	o[LASER].ot = LASER;
	o[LASER].ps = UNKNOWN_PIXELS_STORING;
	int s_per_box = 6;
	int boxes = 3;
	int s_size = boxes * s_per_box;

	int albedo_default = 1.0f;
	int R_default = 1.0f;
	int G_default = 1.0f;
	int B_default = 1.0f;
	int A_default = 1.0f;
	albedoVV.resize(boxes);
	RVV.resize(boxes);
	GVV.resize(boxes);
	BVV.resize(boxes);
	AVV.resize(boxes);
	Point stub0(0.0f, 0.0f, 0.0f);

	for (int bi = 0; bi < boxes; bi++) {
		for (size_t ai = albedoVV[bi].size(); ai < s_size; ai++)
			albedoVV[bi].push_back(albedo_default);
		for (size_t Ri = RVV[bi].size(); Ri < s_size; Ri++)
			RVV[bi].push_back(R_default);
		for (size_t Gi = GVV[bi].size(); Gi < s_size; Gi++)
			GVV[bi].push_back(G_default);
		for (size_t Bi = BVV[bi].size(); Bi < s_size; Bi++)
			BVV[bi].push_back(B_default);
		for (size_t Ai = AVV[bi].size(); Ai < s_size; Ai++)
			AVV[bi].push_back(A_default);
	}

	// Main box of the laser
	Object3D mainBox(c_relToP0, stub0, 0, size, c_relToP0, albedoVV[0], RVV[0], GVV[0], BVV[0], AVV[0]);
	o[LASER].add(mainBox);

	// Stick box of the laser
	float base_height = 0.02f;
	Point stickPC = mean(mainBox.s[BOTTOM].p);
	Point stickS(0.025f, posC.y - base_height - c_relToP0.y, 0.025f);
	Point stickC(stickS.x / 2.0f, stickS.y, -stickS.z / 2.0f);
	Object3D stickBox(stickPC, stub0, 0.0f, stickS, stickC, albedoVV[1], RVV[1], GVV[1], BVV[1], AVV[1]);
	o[LASER].add(stickBox);

	// Base box of the laser
	Point basePC = mean(stickBox.s[BOTTOM].p);
	Point baseS(0.14f, base_height, 0.14f);
	Point baseC(baseS.x / 2.0f, baseS.y, -baseS.z / 2.0f);
	Object3D baseBox(basePC, stub0, 0.0f, baseS, baseC, albedoVV[2], RVV[2], GVV[2], BVV[2], AVV[2]);
	o[LASER].add(baseBox);

	// Transformaitons
	o[LASER].traCto(posC);
	o[LASER].rotDegFromC(axisN, deg);
}

// Setter Scene Direct Vision
void Scene::setScene_DirectVision() {

	// CAMERA (0)
	Point camPosC(0.0f, 0.75f, 0.0f);	// pos of the center of the camera
	Point camAxisN(0.0f, 1.0f, 0.0f);	// normal axis of rotation from the center of the camera (axis Y in this case)
	float camDeg = 180.0f;				// degrees of rotation of the camera around axis normal axis of rotaiton
	Point camS(0.15f, 0.15f, 0.04f);	// size of the main box of the camera
	Point camC_relToP0(camS.x / 2.0f, camS.y / 2.0f, 0.0f);	// pos of the centre, relative to the first point of the main box before transformations
	//std::vector<std::vector<float>> camAlbedoVV(0), camRVV(0), camGVV(0), camBVV(0), camAVV(0);
	int cam_s_per_box = 6;
	int cam_boxes = 4;
	std::vector<std::vector<float>> camAlbedoVV(cam_boxes, std::vector<float>(cam_s_per_box));
	std::vector<std::vector<float>> camRVV(cam_boxes, std::vector<float>(cam_s_per_box));
	std::vector<std::vector<float>> camGVV(cam_boxes, std::vector<float>(cam_s_per_box));
	std::vector<std::vector<float>> camBVV(cam_boxes, std::vector<float>(cam_s_per_box));
	std::vector<std::vector<float>> camAVV(cam_boxes, std::vector<float>(cam_s_per_box));
	for (int b = 0; b < cam_boxes; b++) {
		camRVV[b][FRONT] = 0.5f;  camRVV[b][RIGHT] = 0.8f;  camRVV[b][BACK] = 0.4f;  camRVV[b][LEFT] = 0.3f;  camRVV[b][BOTTOM] = 0.25f;  camRVV[b][TOP] = 0.65f;
		camGVV[b][FRONT] = 0.5f;  camGVV[b][RIGHT] = 0.8f;  camGVV[b][BACK] = 0.4f;  camGVV[b][LEFT] = 0.3f;  camGVV[b][BOTTOM] = 0.25f;  camGVV[b][TOP] = 0.65f;
		camBVV[b][FRONT] = 0.5f;  camBVV[b][RIGHT] = 0.8f;  camBVV[b][BACK] = 0.4f;  camBVV[b][LEFT] = 0.3f;  camBVV[b][BOTTOM] = 0.25f;  camBVV[b][TOP] = 0.65f;
		for (int s = FRONT; s <= TOP; s++) {
			camAlbedoVV[b][s] = 1.0f;
			camAVV[b][s] = 1.0f;
	}	}
	setCamera(camPosC, camAxisN, camDeg, camS, camC_relToP0, camAlbedoVV, camRVV, camGVV, camBVV, camAVV);

	// LASER (1)
	Point lasPosC(-0.15f, 0.75f, 0.0f);
	Point lasAxisN(0.0f, 1.0f, 0.0f);
	float lasDeg = 180.0f;
	Point lasS(0.1f, 0.1f, 0.15f);
	Point lasC_relToP0(lasS.x / 2.0f, lasS.y / 2.0f, 0.0f);
	//std::vector<std::vector<float>> lasAlbedoVV(0), lasRVV(0), lasGVV(0), lasBVV(0), lasAVV(0);
	int las_s_per_box = 6;
	int las_boxes = 4;
	std::vector<std::vector<float>> lasAlbedoVV(las_boxes, std::vector<float>(las_s_per_box));
	std::vector<std::vector<float>> lasRVV(las_boxes, std::vector<float>(las_s_per_box));
	std::vector<std::vector<float>> lasGVV(las_boxes, std::vector<float>(las_s_per_box));
	std::vector<std::vector<float>> lasBVV(las_boxes, std::vector<float>(las_s_per_box));
	std::vector<std::vector<float>> lasAVV(las_boxes, std::vector<float>(las_s_per_box));
	for (int b = 0; b < las_boxes; b++) {
		lasRVV[b][FRONT] = 0.5f;  lasRVV[b][RIGHT] = 0.8f;  lasRVV[b][BACK] = 0.4f;  lasRVV[b][LEFT] = 0.3f;  lasRVV[b][BOTTOM] = 0.25f;  lasRVV[b][TOP] = 0.65f;
		lasGVV[b][FRONT] = 0.5f;  lasGVV[b][RIGHT] = 0.8f;  lasGVV[b][BACK] = 0.4f;  lasGVV[b][LEFT] = 0.3f;  lasGVV[b][BOTTOM] = 0.25f;  lasGVV[b][TOP] = 0.65f;
		lasBVV[b][FRONT] = 0.5f;  lasBVV[b][RIGHT] = 0.8f;  lasBVV[b][BACK] = 0.4f;  lasBVV[b][LEFT] = 0.3f;  lasBVV[b][BOTTOM] = 0.25f;  lasBVV[b][TOP] = 0.65f;
		for (int s = FRONT; s <= TOP; s++) {
			lasAlbedoVV[b][s] = 1.0f;
			lasAVV[b][s] = 1.0f;
	}	}
	setLaser(lasPosC, lasAxisN, lasDeg, lasS, lasC_relToP0, lasAlbedoVV, lasRVV, lasGVV, lasBVV, lasAVV);

	// WALL (2)
	Point wallPosC(-1.625f + 2.0f, 0.2f + 0.75f, -2.0f);
	Point walS(2.0f, 1.5f, 0.05f);
	Point walAxisN(0.0f, 1.0f, 0.0f);
	float walDeg = 0.0f;
	Point walC_relToP0(walS.x / 2.0f, walS.y / 2.0f, 0.0f);
	//std::vector<float> walAlbedoV(0), walRVV(0), walGV(0), walBV(0), walAV(0);
	int wal_s_per_box = 6;
	std::vector<float> walAlbedoV(wal_s_per_box);
	std::vector<float> walRV(wal_s_per_box);
	std::vector<float> walGV(wal_s_per_box);
	std::vector<float> walBV(wal_s_per_box);
	std::vector<float> walAV(wal_s_per_box);
	walRV[FRONT] = 0.85f;  walRV[RIGHT] = 0.3f;  walRV[BACK] = 0.5f;  walRV[LEFT] = 0.8f;  walRV[BOTTOM] = 0.25f;  walRV[TOP] = 0.65f;
	walGV[FRONT] = 0.85f;  walGV[RIGHT] = 0.3f;  walGV[BACK] = 0.5f;  walGV[LEFT] = 0.8f;  walGV[BOTTOM] = 0.25f;  walGV[TOP] = 0.65f;
	walBV[FRONT] = 0.85f;  walBV[RIGHT] = 0.3f;  walBV[BACK] = 0.5f;  walBV[LEFT] = 0.8f;  walBV[BOTTOM] = 0.25f;  walBV[TOP] = 0.65f;
	for (int s = FRONT; s <= TOP; s++) {
		walAlbedoV[s] = 1.0f;
		walAV[s] = 1.0f;
	}
	o[WALL].setBox(wallPosC, walAxisN, walDeg, walS, walC_relToP0, walAlbedoV, walRV, walGV, walBV, walAV);

	// FLOOR (4)
	Point flolPosC(-1.625f, 0.0f, 0.7f);
	Point floS(4.0f, 6.0f, 0.2f);
	Point floAxisN(1.0f, 0.0f, 0.0f);
	float floDeg = -90.0f;
	Point floC_relToP0(0.0f, 0.0f, 0.0f);
	//std::vector<float> floAlbedoV(0), floRVV(0), floGV(0), floBV(0), floAV(0);
	int flo_s_per_box = 6;
	std::vector<float> floAlbedoV(flo_s_per_box);
	std::vector<float> floRV(flo_s_per_box);
	std::vector<float> floGV(flo_s_per_box);
	std::vector<float> floBV(flo_s_per_box);
	std::vector<float> floAV(flo_s_per_box);
	floRV[FRONT] = 0.45f;  floRV[RIGHT] = 0.3f;  floRV[BACK] = 0.25f;  floRV[LEFT] = 0.8f;  floRV[BOTTOM] = 0.4f;  floRV[TOP] = 0.5f;
	floGV[FRONT] = 0.45f;  floGV[RIGHT] = 0.3f;  floGV[BACK] = 0.25f;  floGV[LEFT] = 0.8f;  floGV[BOTTOM] = 0.4f;  floGV[TOP] = 0.5f;
	floBV[FRONT] = 0.45f;  floBV[RIGHT] = 0.3f;  floBV[BACK] = 0.25f;  floBV[LEFT] = 0.8f;  floBV[BOTTOM] = 0.4f;  floBV[TOP] = 0.5f;
	for (int s = FRONT; s <= TOP; s++) {
		floAlbedoV[s] = 1.0f;
		floAV[s] = 1.0f;
	}
	o[FLOOR].setBox(flolPosC, floAxisN, floDeg, floS, floC_relToP0, floAlbedoV, floRV, floGV, floBV, floAV);
}


// ----------------------------------------------------------------------------------------------------------------------------------------
// ----- OTHER NON-MEMBER FUNCTIONS -------------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------------------
float mean(std::vector<float> & vf) {
	float mean = 0.0f;
	for (size_t i = 0; i < vf.size(); i++)
		mean += vf[i];
	return mean / vf.size();
}
Point mean(std::vector<Point> & vp) {
	Point mean (0.0f,0.0f,0.0f);
	for (size_t i = 0; i < vp.size(); i++)
		mean += vp[i];
	return mean / vp.size();
}
void setRodriguesMatrix(float & r11, float & r12, float & r13,
	float & r21, float & r22, float & r23,
	float & r31, float & r32, float & r33,
	float const & wx, float const & wy, float const & wz, float const & rad) {
	// constants
	float cosT = cos(rad);
	float l_cosT = 1.0f - cosT;
	float sinT = sin(rad);
	// Rodrigues' Rotation Matrix: http://mathworld.wolfram.com/RodriguesRotationFormula.html
	r11 = cosT + wx * wx * l_cosT;			r12 = wx * wy * l_cosT - wz * sinT;		r13 = wy * sinT + wx * wz * l_cosT;
	r21 = wz * sinT + wx * wy * l_cosT;		r22 = cosT + wy * wy * l_cosT;			r23 = -wx * sinT + wy * wz * l_cosT;
	r31 = -wy * sinT + wx * wz * l_cosT;	r32 = wx * sinT + wy * wz * l_cosT;		r33 = cosT + wz * wz * l_cosT;
}

// MAIN SCENE
int main_scene(int argc, char**argv) {

	std::cout << "\nSome Point operations will be shown:\n";

	Point p0;
	Point p1(1.0f, 1.0f, 1.0f);
	Point p2(0.3f, -1.6f, 3.2f);
	Point p3 = p2;

	p0.print("\n  p0 = ", "");
	p1.print("\n  p1 = ", "");
	p2.print("\n  p2 = ", "");
	p3.print("\n  p3 = ", "");

	p0 += p1;	std::cout << "\n\np0 += p1";
	p1 += p2;	std::cout << "\np1 += p2";
	p3 *= 2.0f;	std::cout << "\np3 *= 2.0f";
	p0.print("\n  p0 = ", "");
	p1.print("\n  p1 = ", "");
	p2.print("\n  p2 = ", "");
	p3.print("\n  p3 = ", "");

	std::cout << "\np3[1] = " << p3[1];


	std::cout << "\n\n\n";
	system("pause");
	return 0;
}


// TO-DO: CREATE ALL OF THIS (SOME ACTUALLY):

void set_scene_diffused_mirror(bool loop) {}
void set_scene_direct_vision_wall(bool loop) {}
void set_scene_direct_vision_any(bool loop) {}
void set_scene_calibration_matrix (Info* info_, Pixels_storing pixel_storing_) {}
void set_scene_vision_simulation(float dist_cam_wall) {}
void clear_scene() {}
void set_camera(Point* camera_pos_, Point* camera_rot_, Point* camera_size_, Point* camera_centre_) {}
void set_laser(Point* laser_pos_, Point* laser_rot_, Point* laser_size_, Point* laser_centre_) {}
void set_box(Point* box_pos_, Point* box_rot_, Point* box_size_, Point* box_centre_, int Obj3D_idx, float albedo) {}
void set_wall_patches(Point* camera_pos_, Point* camera_rot_, Point* camera_size_, Point* camera_centre_, std::vector<Point*> & screen_patches_corners_normals_, std::vector<Point*> & screen_patches_centers_normals_, std::vector<float> & wall_patches_albedo_, Pixels_storing pixel_storing_) {}
void set_camera_fov(Point* camera_pos_, std::vector<Point*> & screen_patches_corners_normals_, Pixels_storing pixel_storing_) {}
void set_laser_ray() {}
void set_volume_patches(Point* volume_pos_, Point* volume_rot_, Point* volume_size_, Point* volume_centre_,
	std::vector<float> & volume_patches_albedo_, std::vector<Point*> & volume_patches_rot_, std::vector<bool> & volume_patches_bool_) {}
void set_wall_patches_albedo(std::vector<float> & wall_patches_albedo_) {}
void set_pixel_patches(Point* camera_pos_, Point* camera_rot_, Point* camera_centre_, std::vector<Point*> & screen_patches_corners_normals_, std::vector<Point*> & screen_patches_centers_normals_) {}
void update_pixel_patches(Point* camera_pos_, Point* camera_rot_, Point* camera_centre_, std::vector<Point*> & screen_patches_corners_normals_, std::vector<Point*> & screen_patches_centers_normals_, bool loop) {}
void update_wall_and_pixel_patches(Point* camera_pos_, Point* camera_rot_, Point* camera_centre_, std::vector<Point*> & screen_patches_corners_normals_, std::vector<Point*> & screen_patches_centers_normals_, int r_div, int c_div, bool loop) {}
void set_screen_normals_pixel_patches(std::vector<Point*> & screen_patches_corners_normals_, std::vector<Point*> & screen_patches_centers_normals_, Point* camera_pos_, Point* camera_rot_, Point* camera_centre_) {}
void set_depth_map(std::vector<float> & depth_map_, Frame & Frame_00_cap, Frame & Frame_90_cap) {}
void set_volume_patches_params(std::vector<float> & volume_patches_albedo_, std::vector<Point*> & volume_patches_rot_, std::vector<bool> & volume_patches_bool_) {}
void mean_vector_of_points (std::vector<Point> & vec, Point & pt) {}
void set_point_to_vector_distances (Point* p, std::vector<Point*> & vp, std::vector<float> & vd) {}


