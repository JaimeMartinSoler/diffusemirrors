


#include "global.h"
#include "data.h"
#include "data_sim.h"
#include "scene.h"

#include <iostream>
#include <vector>
#include <levmar.h>



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
// mod, modPow2
float Point::mod() {
	return sqrt(x*x + y*y + z*z);
}
float Point::modPow2() {
	return x*x + y*y + z*z;
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
	return x*pr.x + y*pr.y + z*pr.z;
}
Point Point::cross(Point const & pr) {
	return Point(y*pr.z - z*pr.y, z*pr.x - x*pr.z, x*pr.y - y*pr.x);
}
// translation (setter)
void Point::tra(Point const & pr) {
	x += pr.x;
	y += pr.y;
	z += pr.z;
}
void Point::tra(Point const & pr, Point & src) {
	x = src.x + pr.x;
	y = src.y + pr.y;
	z = src.z + pr.z;
}
// rotation optimal (cosT and sinT already calculated) absolute (from (0,0,0)), radians (setters). All rot functions call this functions internally
void Point::rotOpt(	float const & r11, float const & r12, float const & r13, 
					float const & r21, float const & r22, float const & r23, 
					float const & r31, float const & r32, float const & r33) {
	// Rodrigues' Rotation Formula:
	// http://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
	float xc = x;
	float yc = y;
	x = r11*xc + r12*yc + r13*z;
	y = r21*xc + r22*yc + r23*z;
	z = r31*xc + r32*yc + r33*z;
}
void Point::rotOpt(	float const & r11, float const & r12, float const & r13, 
					float const & r21, float const & r22, float const & r23, 
					float const & r31, float const & r32, float const & r33, Point & src) {
	// The same as the rotOpt(r11,...,r33) function but applied from an input src point to another point
	// Rodrigues' Rotation Formula:
	// http://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
	x = r11*src.x + r12*src.y + r13*src.z;
	y = r21*src.x + r22*src.y + r23*src.z;
	z = r31*src.x + r32*src.y + r33*src.z;
}
void Point::rotXYZopt(float cosX, float sinX, float cosY, float sinY, float cosZ, float sinZ) {
	// the rotation order matters and is: z, x, y
	// z rotation
	float x_copy = x; 
	x = x * cosZ - y * sinZ;
	y = x_copy * sinZ + y * cosZ;
	// x rotation
	float y_copy = y;
	y = y * cosX - z * sinX;
	z = y_copy * sinX + z * cosX;
	// y rotation
	x_copy = x;
	x = x * cosY + z * sinY;
	z = -x_copy * sinY + z * cosY;
}
void Point::rotXYZoptInverted(float cosX, float sinX, float cosY, float sinY, float cosZ, float sinZ) {
	// the rotation order matters and is: y, x, z
	// y rotation
	float x_copy = x;
	x = x * cosY + z * sinY;
	z = -x_copy * sinY + z * cosY;
	// x rotation
	float y_copy = y;
	y = y * cosX - z * sinX;
	z = y_copy * sinX + z * cosX;
	// z rotation
	x_copy = x; 
	x = x * cosZ - y * sinZ;
	y = x_copy * sinZ + y * cosZ;
}
void Point::rotXYZopt(float cosX, float sinX, float cosY, float sinY, float cosZ, float sinZ, Point & src) {
	// The same as the rotXYZopt(cosX,...,sinZ) function but applied from an input src point to another point
	// the rotation order matters and is: z, x, y
	// z rotation
	x = src.x * cosZ - src.y * sinZ;
	y = src.x * sinZ + src.y * cosZ;
	// x rotation
	float y_copy = y;
	y = y_copy * cosX - src.z * sinX;
	z = y_copy * sinX + src.z * cosX;
	// y rotation
	float x_copy = x;
	x = x * cosY + z * sinY;
	z = -x_copy * sinY + z * cosY;
}
void Point::rotXopt(float cosT, float sinT) {
	float y_copy = y;
	y = y * cosT - z * sinT;
	z = y_copy * sinT + z * cosT;
}
void Point::rotYopt(float cosT, float sinT) {
	float x_copy = x;
	x = x * cosT + z * sinT;
	z = -x_copy * sinT + z * cosT;
}
void Point::rotZopt(float cosT, float sinT) {
	float x_copy = x;
	x = x * cosT - y * sinT;
	y = x_copy * sinT + y * cosT;
}
// rotation absolute (from (0,0,0)), radians (setters)
void Point::rot(Point & axisN, float rad) {
	float r11, r12, r13, r21, r22, r23, r31, r32, r33;
	setRotationMatrix(r11, r12, r13, r21, r22, r23, r31, r32, r33, axisN.x, axisN.y, axisN.z, rad);
	rotOpt(r11, r12, r13, r21, r22, r23, r31, r32, r33);
}
void Point::rotXYZ(float radX, float radY, float radZ) {
	rotXYZopt(cos(radX), sin(radX), cos(radY), sin(radY), cos(radZ), sin(radZ));
}
void Point::rotX(float rad) {
	rotXopt(cos(rad), sin(rad));
}
void Point::rotY(float rad) {
	rotYopt(cos(rad), sin(rad));
}
void Point::rotZ(float rad) {
	rotZopt(cos(rad), sin(rad));
}
// rotation absolute (from (0,0,0)), degrees (setters)
void Point::rotDeg(Point & axisN, float deg) {
	rot(axisN, deg * PI / 180.0f);
}
void Point::rotXYZdeg(float degX, float degY, float degZ) {
	float radX = degX * PI / 180.0f;
	float radY = degY * PI / 180.0f;
	float radZ = degZ * PI / 180.0f;
	rotXYZopt(cos(radX), sin(radX), cos(radY), sin(radY), cos(radZ), sin(radZ));
}
void Point::rotXdeg(float deg) {
	float rad = deg * PI / 180.0f;
	rotXopt(cos(rad), sin(rad));
}
void Point::rotYdeg(float deg) {
	float rad = deg * PI / 180.0f;
	rotYopt(cos(rad), sin(rad));
}
void Point::rotZdeg(float deg) {
	float rad = deg * PI / 180.0f;
	rotZopt(cos(rad), sin(rad));
}
// rotation relative (from pr), radians (setters)
void Point::rotFromP(Point & axisN, float rad, Point & pr) {
	float r11, r12, r13, r21, r22, r23, r31, r32, r33;
	setRotationMatrix(r11, r12, r13, r21, r22, r23, r31, r32, r33, axisN.x, axisN.y, axisN.z, rad);
	x -= pr.x; y -= pr.y; z -= pr.z;
	rotOpt(r11, r12, r13, r21, r22, r23, r31, r32, r33);
	x += pr.x; y += pr.y; z += pr.z;
}
void Point::rotXYZfromP(float radX, float radY, float radZ, Point & pr) {
	x -= pr.x; y -= pr.y; z -= pr.z;
	rotXYZopt(cos(radX), sin(radX), cos(radY), sin(radY), cos(radZ), sin(radZ));
	x += pr.x; y += pr.y; z += pr.z;
}
void Point::rotXfromP(float rad, Point & pr) {
	x -= pr.x; y -= pr.y; z -= pr.z;
	rotXopt(cos(rad), sin(rad));
	x += pr.x; y += pr.y; z += pr.z;
}
void Point::rotYfromP(float rad, Point & pr) {
	x -= pr.x; y -= pr.y; z -= pr.z;
	rotYopt(cos(rad), sin(rad));
	x += pr.x; y += pr.y; z += pr.z;
}
void Point::rotZfromP(float rad, Point & pr) {
	x -= pr.x; y -= pr.y; z -= pr.z;
	rotZopt(cos(rad), sin(rad));
	x += pr.x; y += pr.y; z += pr.z;
}
// rotation relative (from pr), degrees (setters)
void Point::rotDegFromP(Point & axisN, float deg, Point & pr) {
	rotFromP(axisN, deg * PI / 180.0f, pr);
}
void Point::rotXYZdegFromP(float degX, float degY, float degZ, Point & pr) {
	rotXYZfromP(degX * PI / 180.0f, degY * PI / 180.0f, degZ * PI / 180.0f, pr);
}
void Point::rotXdegFromP(float deg, Point & pr) {
	rotXfromP(deg * PI / 180.0f, pr);
}
void Point::rotYdegFromP(float deg, Point & pr) {
	rotYfromP(deg * PI / 180.0f, pr);
}
void Point::rotZdegFromP(float deg, Point & pr) {
	rotZfromP(deg * PI / 180.0f, pr);
}


// ----- NON-MEMBER FUNCITONS --------------------

// normal to 3 points
Point normalTo3P(Point & p0, Point & p1, Point & p2) {
	return ((p1 - p0).cross(p2 - p1)).normal();
}
// Intersection Line-Plane. [P=Point, N=Normal]
Point int_linePN_planePN(Point & lP, Point & lN, Point & pP, Point & vN) {
	// http://en.wikipedia.org/wiki/Line%E2%80%93plane_intersection#Algebraic_form
	return lP + lN * ((pP - lP).dot(vN) / lN.dot(vN));
}
Point int_linePN_planePPP(Point & lP, Point & lN, Point & pP0, Point & pP1, Point & pP2) {
	return int_linePN_planePN(lP, lN, pP0, normalTo3P(pP0, pP1, pP2));
}
Point int_linePP_planePN(Point & lP0, Point & lP1, Point & pP, Point & vN) {
	return int_linePN_planePN(lP0, (lP1 - lP0).normal(), pP, vN);
}
Point int_linePP_planePPP(Point & lP0, Point & lP1, Point & pP0, Point & pP1, Point & pP2) {
	return int_linePN_planePN(lP0, (lP1 - lP0).normal(), pP0, normalTo3P(pP0, pP1, pP2));
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
Shape::Shape(std::vector<Point> & p_, Point const & c_,
	float albedo_, float R_, float G_, float B_, float A_, ShapeType st_) {	// params of this line has default (see scene.h)
	set(p_, c_);
	set(albedo_, R_, G_, B_, A_, st_);
}
Shape::Shape(Point const & p0, Point const & c_,
	float albedo_, float R_, float G_, float B_, float A_, ShapeType st_) {	// params of this line has default (see scene.h)
	set(p0, c_);
	set(albedo_, R_, G_, B_, A_, st_);
}
Shape::Shape(Point const & p0, Point const & p1, Point const & c_,
	float albedo_, float R_, float G_, float B_, float A_, ShapeType st_) {	// params of this line has default (see scene.h)
	set(p0, p1, c_);
	set(albedo_, R_, G_, B_, A_, st_);
}
Shape::Shape(Point const & p0, Point const & p1, Point const & p2, Point const & c_,
	float albedo_, float R_, float G_, float B_, float A_, ShapeType st_) {	// params of this line has default (see scene.h)
	set(p0, p1, p2, c_);
	set(albedo_, R_, G_, B_, A_, st_);
}
Shape::Shape(Point const & p0, Point const & p1, Point const & p2, Point const & p3, Point const & c_,
	float albedo_, float R_, float G_, float B_, float A_, ShapeType st_) {	// params of this line has default (see scene.h)
	set(p0, p1, p2, p3, c_);
	set(albedo_, R_, G_, B_, A_, st_);
}


// ----- SETTERS --------------------------------- // Note that Constructors just call their corresponding Setter

// Setter Default
void Shape::set() {
	p.resize(0);
	c = Point(0.0f, 0.0f, 0.0f);
	albedo = 1.0f;
	R = 1.0f; G = 1.0f; B = 1.0f; A = 1.0f;
	st = UNKNOWN_SHA;
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
void Shape::set(std::vector<Point> & p_, Point const & c_) {
	p = p_;
	c = c_;
}
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
	return normalTo3P(p[0], p[1], p[2]);
}
Point Shape::normalQUAD() {
	return normalTo3P(p[0], p[1], p[2]);
}
// area
float Shape::areaRECTANGLE() {
	return dist(p[0], p[1]) * dist(p[1], p[2]);
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
void Shape::rot(Point & axisN, float rad) {
	float r11, r12, r13, r21, r22, r23, r31, r32, r33;
	setRotationMatrix(r11, r12, r13, r21, r22, r23, r31, r32, r33, axisN.x, axisN.y, axisN.z, rad);
	for (size_t i = 0; i < p.size(); i++)
		p[i].rotOpt(r11, r12, r13, r21, r22, r23, r31, r32, r33);
	c.rotOpt(r11, r12, r13, r21, r22, r23, r31, r32, r33);
}
void Shape::rotXYZ(float radX, float radY, float radZ, bool inverted) {
	float cosX = cos(radX); float sinX = sin(radX);
	float cosY = cos(radY); float sinY = sin(radY);
	float cosZ = cos(radZ); float sinZ = sin(radZ);
	if (inverted) {
		for (size_t i = 0; i < p.size(); i++)
			p[i].rotXYZoptInverted(cosX, sinX, cosY, sinY, cosZ, sinZ);
		c.rotXYZoptInverted(cosX, sinX, cosY, sinY, cosZ, sinZ);
	} else {
		for (size_t i = 0; i < p.size(); i++)
			p[i].rotXYZopt(cosX, sinX, cosY, sinY, cosZ, sinZ);
		c.rotXYZopt(cosX, sinX, cosY, sinY, cosZ, sinZ);
	}
}
void Shape::rotX(float rad) {
	float cosT = cos(rad);
	float sinT = sin(rad);
	for (size_t i = 0; i < p.size(); i++)
		p[i].rotXopt(cosT, sinT);
	c.rotXopt(cosT, sinT);
}
void Shape::rotY(float rad) {
	float cosT = cos(rad);
	float sinT = sin(rad);
	for (size_t i = 0; i < p.size(); i++)
		p[i].rotYopt(cosT, sinT);
	c.rotYopt(cosT, sinT);
}
void Shape::rotZ(float rad) {
	float cosT = cos(rad);
	float sinT = sin(rad);
	for (size_t i = 0; i < p.size(); i++)
		p[i].rotZopt(cosT, sinT);
	c.rotZopt(cosT, sinT);
}
// rotation absolute (from (0,0,0)), degrees (setters)
void Shape::rotDeg(Point & axisN, float deg) {
	rot(axisN, deg * PI / 180.0f);
}
void Shape::rotXYZdeg(float degX, float degY, float degZ, bool inverted) {
	rotXYZ(degX * PI / 180.0f, degY * PI / 180.0f, degZ * PI / 180.0f, inverted);
}
void Shape::rotXdeg(float deg) {
	rotX(deg * PI / 180.0f);
}
void Shape::rotYdeg(float deg) {
	rotY(deg * PI / 180.0f);
}
void Shape::rotZdeg(float deg) {
	rotZ(deg * PI / 180.0f);
}
// rotation relative (from pr), radians (setters)
void Shape::rotFromP(Point & axisN, float rad, Point & pr) {
	float r11, r12, r13, r21, r22, r23, r31, r32, r33;
	setRotationMatrix(r11, r12, r13, r21, r22, r23, r31, r32, r33, axisN.x, axisN.y, axisN.z, rad);
	for (size_t i = 0; i < p.size(); i++) {
		p[i].x -= pr.x; p[i].y -= pr.y; p[i].z -= pr.z;
		p[i].rotOpt(r11, r12, r13, r21, r22, r23, r31, r32, r33);
		p[i].x += pr.x; p[i].y += pr.y; p[i].z += pr.z;
	}
	c.x -= pr.x; c.y -= pr.y; c.z -= pr.z;
	c.rotOpt(r11, r12, r13, r21, r22, r23, r31, r32, r33);
	c.x += pr.x; c.y += pr.y; c.z += pr.z;
}
void Shape::rotXYZfromP(float radX, float radY, float radZ, Point & pr, bool inverted) {
	float cosX = cos(radX); float sinX = sin(radX);
	float cosY = cos(radY); float sinY = sin(radY);
	float cosZ = cos(radZ); float sinZ = sin(radZ);
	if (inverted) {
		for (size_t i = 0; i < p.size(); i++) {
			p[i].x -= pr.x; p[i].y -= pr.y; p[i].z -= pr.z;
			p[i].rotXYZoptInverted(cosX, sinX, cosY, sinY, cosZ, sinZ);
			p[i].x += pr.x; p[i].y += pr.y; p[i].z += pr.z;
		}
		c.x -= pr.x; c.y -= pr.y; c.z -= pr.z;
		c.rotXYZoptInverted(cosX, sinX, cosY, sinY, cosZ, sinZ);
		c.x += pr.x; c.y += pr.y; c.z += pr.z;
	} else {
		for (size_t i = 0; i < p.size(); i++) {
			p[i].x -= pr.x; p[i].y -= pr.y; p[i].z -= pr.z;
			p[i].rotXYZopt(cosX, sinX, cosY, sinY, cosZ, sinZ);
			p[i].x += pr.x; p[i].y += pr.y; p[i].z += pr.z;
		}
		c.x -= pr.x; c.y -= pr.y; c.z -= pr.z;
		c.rotXYZopt(cosX, sinX, cosY, sinY, cosZ, sinZ);
		c.x += pr.x; c.y += pr.y; c.z += pr.z;
	}

}
void Shape::rotXfromP(float rad, Point & pr) {
	float cosT = cos(rad);
	float sinT = sin(rad);
	for (size_t i = 0; i < p.size(); i++) {
		p[i].x -= pr.x; p[i].y -= pr.y; p[i].z -= pr.z;
		p[i].rotXopt(cosT, sinT);
		p[i].x += pr.x; p[i].y += pr.y; p[i].z += pr.z;
	}
	c.x -= pr.x; c.y -= pr.y; c.z -= pr.z;
	c.rotXopt(cosT, sinT);
	c.x += pr.x; c.y += pr.y; c.z += pr.z;
}
void Shape::rotYfromP(float rad, Point & pr) {
	float cosT = cos(rad);
	float sinT = sin(rad);
	for (size_t i = 0; i < p.size(); i++) {
		p[i].x -= pr.x; p[i].y -= pr.y; p[i].z -= pr.z;
		p[i].rotYopt(cosT, sinT);
		p[i].x += pr.x; p[i].y += pr.y; p[i].z += pr.z;
	}
	c.x -= pr.x; c.y -= pr.y; c.z -= pr.z;
	c.rotYopt(cosT, sinT);
	c.x += pr.x; c.y += pr.y; c.z += pr.z;
}
void Shape::rotZfromP(float rad, Point & pr) {
	float cosT = cos(rad);
	float sinT = sin(rad);
	for (size_t i = 0; i < p.size(); i++) {
		p[i].x -= pr.x; p[i].y -= pr.y; p[i].z -= pr.z;
		p[i].rotZopt(cosT, sinT);
		p[i].x += pr.x; p[i].y += pr.y; p[i].z += pr.z;
	}
	c.x -= pr.x; c.y -= pr.y; c.z -= pr.z;
	c.rotZopt(cosT, sinT);
	c.x += pr.x; c.y += pr.y; c.z += pr.z;
}
// rotation relative (from pr), degrees (setters)
void Shape::rotDegFromP(Point & axisN, float deg, Point & pr) {
	rotFromP(axisN, deg * PI / 180.0f, pr);
}
void Shape::rotXYZdegFromP(float degX, float degY, float degZ, Point & pr, bool inverted) {
	rotXYZfromP(degX * PI / 180.0f, degY * PI / 180.0f, degZ * PI / 180.0f, pr, inverted);
}
void Shape::rotXdegFromP(float deg, Point & pr) {
	rotXfromP(deg * PI / 180.0f, pr);
}
void Shape::rotYdegFromP(float deg, Point & pr) {
	rotYfromP(deg * PI / 180.0f, pr);
}
void Shape::rotZdegFromP(float deg, Point & pr) {
	rotZfromP(deg * PI / 180.0f, pr);
}
// rotation relative (from c), radians (setters)
void Shape::rotFromC(Point & axisN, float rad) {
	float r11, r12, r13, r21, r22, r23, r31, r32, r33;
	setRotationMatrix(r11, r12, r13, r21, r22, r23, r31, r32, r33, axisN.x, axisN.y, axisN.z, rad);
	for (size_t i = 0; i < p.size(); i++) {
		p[i].x -= c.x; p[i].y -= c.y; p[i].z -= c.z;
		p[i].rotOpt(r11, r12, r13, r21, r22, r23, r31, r32, r33);
		p[i].x += c.x; p[i].y += c.y; p[i].z += c.z;
	}
}
void Shape::rotXYZfromC(float radX, float radY, float radZ, bool inverted) {
	float cosX = cos(radX); float sinX = sin(radX);
	float cosY = cos(radY); float sinY = sin(radY);
	float cosZ = cos(radZ); float sinZ = sin(radZ);
	if (inverted) {
		for (size_t i = 0; i < p.size(); i++) {
			p[i].x -= c.x; p[i].y -= c.y; p[i].z -= c.z;
			p[i].rotXYZoptInverted(cosX, sinX, cosY, sinY, cosZ, sinZ);
			p[i].x += c.x; p[i].y += c.y; p[i].z += c.z;
		}
	} else {
		for (size_t i = 0; i < p.size(); i++) {
			p[i].x -= c.x; p[i].y -= c.y; p[i].z -= c.z;
			p[i].rotXYZopt(cosX, sinX, cosY, sinY, cosZ, sinZ);
			p[i].x += c.x; p[i].y += c.y; p[i].z += c.z;
		}
	}
}
void Shape::rotXfromC(float rad) {
	float cosT = cos(rad);
	float sinT = sin(rad);
	for (size_t i = 0; i < p.size(); i++) {
		p[i].x -= c.x; p[i].y -= c.y; p[i].z -= c.z;
		p[i].rotXopt(cosT, sinT);
		p[i].x += c.x; p[i].y += c.y; p[i].z += c.z;
	}
}
void Shape::rotYfromC(float rad) {
	float cosT = cos(rad);
	float sinT = sin(rad);
	for (size_t i = 0; i < p.size(); i++) {
		p[i].x -= c.x; p[i].y -= c.y; p[i].z -= c.z;
		p[i].rotYopt(cosT, sinT);
		p[i].x += c.x; p[i].y += c.y; p[i].z += c.z;
	}
}
void Shape::rotzFromC(float rad) {
	float cosT = cos(rad);
	float sinT = sin(rad);
	for (size_t i = 0; i < p.size(); i++) {
		p[i].x -= c.x; p[i].y -= c.y; p[i].z -= c.z;
		p[i].rotZopt(cosT, sinT);
		p[i].x += c.x; p[i].y += c.y; p[i].z += c.z;
	}
}
// rotation relative (from c), degrees (setters)
void Shape::rotDegFromC(Point & axisN, float deg) {
	rotFromC(axisN, deg * PI / 180.0f);
}
void Shape::rotXYZdegFromC(float degX, float degY, float degZ, bool inverted) {
	rotXYZfromC(degX * PI / 180.0f, degY * PI / 180.0f, degZ * PI / 180.0f, inverted);
}
void Shape::rotXdegFromC(float deg) {
	rotXfromC(deg * PI / 180.0f);
}
void Shape::rotYdegFromC(float deg) {
	rotYfromC(deg * PI / 180.0f);
}
void Shape::rotZdegFromC(float deg) {
	rotzFromC(deg * PI / 180.0f);
}

// add, clear Point to Shape
void Shape::clear(bool clearAll){
	p.clear();
	if (clearAll) {
		c = Point(0.0f, 0.0f, 0.0f);
		albedo = 0.0f;
		R = 0.0f; G = 0.0f; B = 0.0f; A = 0.0f;
		st = UNKNOWN_SHA;
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
Object3D::Object3D(std::vector<Shape> & s_, Object3DType ot_, PixStoring ps_) { // default: ps_ = UNKNOWN_PIS
	set(s_, ot_);
	set(ps_);
}
// Constructor Object3DType and PixStoring, and s.resize(0);
Object3D::Object3D(Object3DType ot_, PixStoring ps_) { // default: ps_ = UNKNOWN_PIS
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
	ot = UNKOWN_OBT;
	ps = UNKNOWN_PIS;
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
// Setter Object3DType and PixStoring.
void Object3D::set(Object3DType ot_) {
	ot = ot_;
}
// Setter Object3DType PixStoring.
void Object3D::set(PixStoring ps_) {
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

// Setter Screen FoV measurement. Here each Point is an actual Point
void Object3D::setScreenFoVmeasP(Point & camC, Point & camN, PixStoring ps_, bool pSim_) {

	// setting the Object3D screen of the original camera FoV measurement
	s.resize(numPix(ps_, pSim_));
	ot = UNKOWN_OBT;
	ps = ps_;
	
	// setting pixels sizes
	float pixSizeRealR = CAMERA_FOV_Y_METERS / ((float)CAMERA_PIX_Y);
	float pixSizeRealC = CAMERA_FOV_X_METERS / ((float)CAMERA_PIX_X);
	float pixSizeR, pixSizeC, offsetR, offsetC;
	if (pSim_) {
		if (ps_ == PIXELS_TOTAL) {
			pixSizeR = CAMERA_FOV_Y_METERS / ((float)PMD_SIM_ROWS);
			pixSizeC = CAMERA_FOV_X_METERS / ((float)PMD_SIM_COLS);
			offsetR = 0.0f;
			offsetC = 0.0f;
		} else if (ps_ == PIXELS_VALID) {
			pixSizeR = CAMERA_FOV_Y_METERS * (((float)CAMERA_PIX_Y_VALID) / ((float)CAMERA_PIX_Y)) / ((float)PMD_SIM_ROWS);
			pixSizeC = CAMERA_FOV_X_METERS * (((float)CAMERA_PIX_X_VALID) / ((float)CAMERA_PIX_X)) / ((float)PMD_SIM_COLS);
			offsetR = - (float)CAMERA_PIX_Y_BAD_TOP * pixSizeRealR;
			offsetC = (float)CAMERA_PIX_X_BAD_LEFT * pixSizeRealC;
		}
	} else {
		pixSizeR = pixSizeRealR;
		pixSizeC = pixSizeRealC;
		if (ps_ == PIXELS_TOTAL) {
			offsetR = 0.0f;
			offsetC = 0.0f;
		} else if (ps_ == PIXELS_VALID) {
			offsetR = - (float)CAMERA_PIX_Y_BAD_TOP * pixSizeRealR;
			offsetC = (float)CAMERA_PIX_X_BAD_LEFT * pixSizeRealC;
	}	}

	// fill the patches
	int s_idx;
	for (int r = 0; r < rows(ps_, pSim_); r++) {
		for (int c = 0; c < cols(ps_, pSim_); c++) {
			s_idx = rc2idx(r, c, ps_, pSim_);
			s[s_idx].p.resize(QUAD);
			//s[s_idx].set(1.0f, 1.0f, 1.0f, 1.0f, 1.0f, QUAD);	// just for testing
			s[s_idx].p[0].set(offsetC + c * pixSizeC, offsetR - (r + 1) * pixSizeR, 0.0f);
			s[s_idx].p[1].set(offsetC + (c + 1) * pixSizeC, offsetR - (r + 1) * pixSizeR, 0.0f);
			s[s_idx].p[2].set(offsetC + (c + 1) * pixSizeC, offsetR - r * pixSizeR, 0.0f);
			s[s_idx].p[3].set(offsetC + c * pixSizeC, offsetR - r * pixSizeR, 0.0f);
			s[s_idx].c.set(offsetC + (c + 0.5f) * pixSizeC, offsetR - (r + 0.5f) * pixSizeR, 0.0f);
	}	}

	// setting the relative to the camera position of the screen as the original camera FoV meas
	Point sreenC(CAMERA_FOV_X_METERS / 2.0f, -CAMERA_FOV_Y_METERS / 2.0f, 0.0f);	// actual center of screen, indep of ps, pSim
	Point screenN(0.0f, 0.0f, 1.0f);
	tra(camC - sreenC);
	rotFromP(axisNVV(screenN,camN), radNN(screenN, camN) + PI, camC);	// note +PI we need to add 180 deg
	tra(camN * CAMERA_DIST_FOV_MEAS);
	//if (SCENEMAIN.o.size() <= SCENE_SIZE)	// just for testing
	//	SCENEMAIN.o.push_back((*this));		// just for testing
}
// Setter Screen FoV measurement. Here each Point is the Normal to the corresponding Point of setScreenFoVmeasP
void Object3D::setScreenFoVmeasN(Point & camC, Point & camN, PixStoring ps_, bool pSim_) {
	// get the setScreenFoVmeasP
	setScreenFoVmeasP(camC, camN, ps_, pSim_);
	// get and normalize the vector, getting setScreenFoVmeasN
	for (size_t i = 0; i < s.size(); i++) {
		s[i].p[0] -= camC;	s[i].p[0].normalize();
		s[i].p[1] -= camC;	s[i].p[1].normalize();
		s[i].p[2] -= camC;	s[i].p[2].normalize();
		s[i].p[3] -= camC;	s[i].p[3].normalize();
		s[i].c    -= camC;	s[i].c.normalize();
	}
}
// Setter Screen FoV measurement. Here each Point is the Normal to the corresponding Point of setScreenFoVmeasP
void Object3D::setScreenFoVmeasNs(Point & camC, Point & camN, PixStoring ps_, bool pSim_) {
	// get the setScreenFoVmeasP
	setScreenFoVmeasP(camC, camN, ps_, pSim_);
	// get and normalize the vector, getting setScreenFoVmeasN
	float distCamScreen = dist(camC, int_linePN_object3D(camC, camN, *this));
	for (size_t i = 0; i < s.size(); i++) {
		s[i].p[0] -= camC;	s[i].p[0] /= distCamScreen;
		s[i].p[1] -= camC;	s[i].p[1] /= distCamScreen;
		s[i].p[2] -= camC;	s[i].p[2] /= distCamScreen;
		s[i].p[3] -= camC;	s[i].p[3] /= distCamScreen;
		s[i].c    -= camC;	s[i].c /= distCamScreen;
	}
}

// Setter Camera(0)
void Object3D::setCamera(Point & posC, float degPhi, float degTheta, float degRoll, Point & size, Point & c_relToP0) {
	std::vector<std::vector<float>> albedoVV_stub;
	std::vector<std::vector<float>> RVV_stub;
	std::vector<std::vector<float>> GVV_stub;
	std::vector<std::vector<float>> BVV_stub;
	std::vector<std::vector<float>> AVV_stub;
	setCamera(posC, degPhi, degTheta, degRoll, size, c_relToP0, albedoVV_stub, RVV_stub, GVV_stub, BVV_stub, AVV_stub);
}
void Object3D::setCamera(Point & posC, float degPhi, float degTheta, float degRoll, Point & size, Point & c_relToP0,
	std::vector<std::vector<float>> & albedoVV, std::vector<std::vector<float>> & RVV, std::vector<std::vector<float>> & GVV, std::vector<std::vector<float>> & BVV, std::vector<std::vector<float>> & AVV) {
	
	clear();
	ot = CAMERA;
	ps = UNKNOWN_PIS;
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
	add(mainBox);

	// Stick box of the camera
	float base_height = 0.02f;
	Point stickPC =  mean(mainBox.s[BOTTOM].p);;
	Point stickS (0.020f, posC.y - base_height - c_relToP0.y + (size.y / 2.0f), 0.020f);
	Point stickC (stickS.x/2.0f, stickS.y -  + (size.y / 2.0f), -stickS.z/2.0f);
	Object3D stickBox(stickPC, stub0, 0.0f, stickS, stickC, albedoVV[1], RVV[1], GVV[1], BVV[1], AVV[1]);

	// Base box of the camera
	Point basePC = mean(stickBox.s[BOTTOM].p);
	Point baseS (0.12f, base_height, 0.12f);
	Point baseC(baseS.x / 2.0f, baseS.y, -baseS.z / 2.0f);
	Object3D baseBox (basePC, stub0, 0.0f, baseS, baseC, albedoVV[2], RVV[2], GVV[2], BVV[2], AVV[2]);

	// Lens box of the camera
	Point lensPC = mean(mainBox.s[FRONT].p);
	Point lensS (0.045f, 0.045f, 0.075f);
	Point lensC(lensS.x / 2.0f, lensS.y / 2.0f, -lensS.z);
	Object3D lensBox (lensPC, stub0, 0.0f, lensS, lensC, albedoVV[3], RVV[3], GVV[3], BVV[3], AVV[3]);
	lensBox.s.erase(lensBox.s.begin() + BACK);
	lensBox.s.erase(lensBox.s.begin() + FRONT);
	add(lensBox);

	// Main box and Lens box phi, roll rotation
	Point mainBoxCenter = (mainBox.s[FRONT].p[0] + mainBox.s[BACK].p[3]) / 2.0f;
	rotXYZdegFromP(-degTheta, 0.0f, degRoll, mainBoxCenter);
	
	// Adding remaning boxes
	add(stickBox);
	add(baseBox);

	// Transformaitons
	rotYdegFromC(degPhi);
	traCto(posC);
}
// Setter Laser (1)
void Object3D::setLaser(Point & posC, float degPhi, float degTheta, float degRoll, Point & size, Point & c_relToP0) {
	std::vector<std::vector<float>> albedoVV_stub;
	std::vector<std::vector<float>> RVV_stub;
	std::vector<std::vector<float>> GVV_stub;
	std::vector<std::vector<float>> BVV_stub;
	std::vector<std::vector<float>> AVV_stub;
	setLaser(posC, degPhi, degTheta, degRoll, size, c_relToP0, albedoVV_stub, RVV_stub, GVV_stub, BVV_stub, AVV_stub);
}
void Object3D::setLaser(Point & posC, float degPhi, float degTheta, float degRoll, Point & size, Point & c_relToP0,
	std::vector<std::vector<float>> & albedoVV, std::vector<std::vector<float>> & RVV, std::vector<std::vector<float>> & GVV, std::vector<std::vector<float>> & BVV, std::vector<std::vector<float>> & AVV) {

	clear();
	ot = LASER;
	ps = UNKNOWN_PIS;
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
	add(mainBox);

	// Stick box of the laser
	float base_height = 0.02f;
	Point stickPC = mean(mainBox.s[BOTTOM].p);;
	Point stickS(0.020f, posC.y - base_height - c_relToP0.y + (size.y / 2.0f), 0.020f);
	Point stickC(stickS.x / 2.0f, stickS.y - (size.y / 2.0f), -stickS.z / 2.0f);
	Object3D stickBox(stickPC, stub0, 0.0f, stickS, stickC, albedoVV[1], RVV[1], GVV[1], BVV[1], AVV[1]);

	// Base box of the laser
	Point basePC = mean(stickBox.s[BOTTOM].p);
	Point baseS(0.12f, base_height, 0.12f);
	Point baseC(baseS.x / 2.0f, baseS.y, -baseS.z / 2.0f);
	Object3D baseBox(basePC, stub0, 0.0f, baseS, baseC, albedoVV[2], RVV[2], GVV[2], BVV[2], AVV[2]);
	
	// Main box phi, roll rotation
	Point mainBoxCenter = (mainBox.s[FRONT].p[0] + mainBox.s[BACK].p[3]) / 2.0f;
	rotXYZdegFromP(-degTheta, 0.0f, degRoll, mainBoxCenter);

	// Adding remaning boxes
	add(stickBox);
	add(baseBox);

	// Transformaitons
	rotYdegFromC(degPhi);
	traCto(posC);
}
// Setter Wall Patches (6)
void Object3D::setWallPatches(Scene & scene, PixStoring ps_, bool pSim_) {

	// Set the Object3D
	s.resize(numPix(ps_, pSim_));
	ot = WALL_PATCHES;
	ps = ps_;

	// Setting constant elements
	Object3D screenFoVmeasN;
	screenFoVmeasN.setScreenFoVmeasN(scene.o[CAMERA].s[0].c, scene.o[CAMERA].normalQUAD(), ps_, pSim_);

	// Update pixel patches
	for (int i = 0; i < s.size(); i++) {
		s[i].p.resize(QUAD);
		s[i].set(1.0f, 1.0f, 1.0f, 1.0f, 1.0f, QUAD);
		s[i].p[0].set(int_linePN_object3D(scene.o[CAMERA].s[0].c, screenFoVmeasN.s[i].p[0], scene.o[WALL]));
		s[i].p[1].set(int_linePN_object3D(scene.o[CAMERA].s[0].c, screenFoVmeasN.s[i].p[1], scene.o[WALL]));
		s[i].p[2].set(int_linePN_object3D(scene.o[CAMERA].s[0].c, screenFoVmeasN.s[i].p[2], scene.o[WALL]));
		s[i].p[3].set(int_linePN_object3D(scene.o[CAMERA].s[0].c, screenFoVmeasN.s[i].p[3], scene.o[WALL]));
		s[i].c.set   (int_linePN_object3D(scene.o[CAMERA].s[0].c, screenFoVmeasN.s[i].c   , scene.o[WALL]));
	}
}
// Setter Camera FoV (7)
void Object3D::setCameraFoV(Scene & scene, float R_, float G_, float B_, float A_, PixStoring ps_, float distDefault) {

	// Set the Object3D
	s.resize(8);
	ot = CAMERA_FOV;
	ps = ps_;

	// Setting constant elements
	bool pSim_ = true;	// doesn't matter true or false here as the concern is the edges, but true takes less calculations
	Object3D screenFoVmeasN;
	screenFoVmeasN.setScreenFoVmeasN(scene.o[CAMERA].s[0].c, scene.o[CAMERA].normalQUAD(), ps_, pSim_);	

	// Get intersections
	Point p0, p1, p2, p3;
	if (scene.o[WALL].s.size() > 0) {
		p0.set(int_linePN_object3D(scene.o[CAMERA].s[0].c, screenFoVmeasN.s[rc2idx(rows(ps, pSim_) - 1, 0, ps, pSim_)].p[0], scene.o[WALL]));
		p1.set(int_linePN_object3D(scene.o[CAMERA].s[0].c, screenFoVmeasN.s[rc2idx(rows(ps, pSim_) - 1, cols(ps, pSim_) - 1, ps, pSim_)].p[1], scene.o[WALL]));
		p2.set(int_linePN_object3D(scene.o[CAMERA].s[0].c, screenFoVmeasN.s[rc2idx(0, cols(ps, pSim_) - 1, ps, pSim_)].p[2], scene.o[WALL]));
		p3.set(int_linePN_object3D(scene.o[CAMERA].s[0].c, screenFoVmeasN.s[rc2idx(0, 0, ps, pSim_)].p[3], scene.o[WALL]));
	} else {
		p0.set(scene.o[CAMERA].s[0].c + screenFoVmeasN.s[rc2idx(rows(ps, pSim_) - 1, 0, ps, pSim_)].p[0] * distDefault);
		p1.set(scene.o[CAMERA].s[0].c + screenFoVmeasN.s[rc2idx(rows(ps, pSim_) - 1, cols(ps, pSim_) - 1, ps, pSim_)].p[1] * distDefault);
		p2.set(scene.o[CAMERA].s[0].c + screenFoVmeasN.s[rc2idx(0, cols(ps, pSim_) - 1, ps, pSim_)].p[2] * distDefault);
		p3.set(scene.o[CAMERA].s[0].c + screenFoVmeasN.s[rc2idx(0, 0, ps, pSim_)].p[3] * distDefault);
	}
	// Set the lines
	s[0].set(scene.o[CAMERA].s[0].c, p0, scene.o[CAMERA].s[0].c);		s[0].set(1.0f, R_, G_, B_, A_, LINE);
	s[1].set(scene.o[CAMERA].s[0].c, p1, scene.o[CAMERA].s[0].c);		s[1].set(1.0f, R_, G_, B_, A_, LINE);
	s[2].set(scene.o[CAMERA].s[0].c, p2, scene.o[CAMERA].s[0].c);		s[2].set(1.0f, R_, G_, B_, A_, LINE);
	s[3].set(scene.o[CAMERA].s[0].c, p3, scene.o[CAMERA].s[0].c);		s[3].set(1.0f, R_, G_, B_, A_, LINE);
	s[4].set(p0, p1, p0);									s[4].set(1.0f, R_, G_, B_, A_, LINE);
	s[5].set(p1, p2, p1);									s[5].set(1.0f, R_, G_, B_, A_, LINE);
	s[6].set(p2, p3, p2);									s[6].set(1.0f, R_, G_, B_, A_, LINE);
	s[7].set(p3, p0, p3);									s[7].set(1.0f, R_, G_, B_, A_, LINE);
}
// Setter Laser Ray (8)
void Object3D::setLaserRay(Scene & scene, float R_, float G_, float B_, float A_, PixStoring ps_) {

	// Set the Object3D
	s.resize(1);
	ot = LASER_RAY;
	ps = ps_;

	// Get intersection
	Point p1 = int_linePN_object3D(scene.o[LASER].s[0].c, scene.o[LASER].normalQUAD(), scene.o[WALL]);

	// Set the line
	s[0].set(scene.o[LASER].s[0].c, p1, s[0].c);	s[0].set(1.0f, R_, G_, B_, A_, LINE);
}
// Setter, Updater Volume Patches (9)
void Object3D::setVolumePatchesBox(Point & boxPC, Point & boxRaxisN, float deg, Point & boxS, std::vector<int> & rowsPerFaceV, std::vector<int> & colsPerFaceV,
	std::vector<std::vector<float>> & albedoVV, std::vector<std::vector<float>> & RVV, std::vector<std::vector<float>> & GVV, std::vector<std::vector<float>> & BVV, std::vector<std::vector<float>> & AVV) {

	// Set the Object3D
	int numFaces = (int)rowsPerFaceV.size();
	int numShapes = 0;
	for (size_t i = FRONT; i < numFaces; ++i)
		numShapes += rowsPerFaceV[i] * colsPerFaceV[i];
	s.resize(numShapes);
	ot = VOLUME_PATCHES;
	ps = UNKNOWN_PIS;

	// Create the convex hull box
	Point boxC_relToP0 = boxS / 2.0f;	// Point(0.0f, 0.0f, 0.0f);
	Object3D convexHullBox;
	convexHullBox.setBox(boxPC, boxRaxisN, deg, boxS, boxC_relToP0);

	// Place the patches from the convexHullBox and the box center
	Point p0, p1, p2, p3, c;
	Point rVec, cVec;
	int ri, ci, rowsPerFacei, colsPerFacei;
	float rf, cf, rowsPerFacef, colsPerFacef;
	int sIdx = 0;
	int oIdx = 0;
	for (int f = FRONT; f < numFaces; ++f) {
		oIdx = 0;
		rowsPerFacei = rowsPerFaceV[f];
		colsPerFacei = colsPerFaceV[f];
		rowsPerFacef = (float)rowsPerFacei;
		colsPerFacef = (float)colsPerFacei;
		rVec = convexHullBox.s[f].p[0] - convexHullBox.s[f].p[3];
		cVec = convexHullBox.s[f].p[1] - convexHullBox.s[f].p[0];
		for (ri = 0; ri < rowsPerFacei; ++ri) {
			rf = (float)ri;
			for (ci = 0; ci < colsPerFacei; ++ci) {
				cf = (float)ci;
				p0 = convexHullBox.s[f].p[3] + rVec * ((rf+1.0f)/rowsPerFacef) + cVec * ( cf      /colsPerFacef);
				p1 = convexHullBox.s[f].p[3] + rVec * ((rf+1.0f)/rowsPerFacef) + cVec * ((cf+1.0f)/colsPerFacef);
				p2 = convexHullBox.s[f].p[3] + rVec * ( rf      /rowsPerFacef) + cVec * ((cf+1.0f)/colsPerFacef);
				p3 = convexHullBox.s[f].p[3] + rVec * ( rf      /rowsPerFacef) + cVec * ( cf      /colsPerFacef);
				c  = convexHullBox.s[f].p[3] + rVec * ((rf+0.5f)/rowsPerFacef) + cVec * ((cf+0.5f)/colsPerFacef);
				s[sIdx].set(p0, p1, p2, p3, c);
				s[sIdx++].set(albedoVV[f][oIdx], RVV[f][oIdx], GVV[f][oIdx], BVV[f][oIdx], AVV[f][oIdx++], QUAD);
	}	}	}
}
void Object3D::updateVolumePatches_Occlusion(Info & info, Scene & scene, Frame & frame00, Frame & frame90, std::vector<int> & rowsPerFaceV, std::vector<int> & colsPerFaceV, bool loop, PixStoring ps_, bool pSim_) {
	
	// Syncronization
	std::unique_lock<std::mutex> locker_frame_object;	// Create a defered locker (a locker not locked yet)
	locker_frame_object = std::unique_lock<std::mutex>(mutex_frame_object, std::defer_lock);

	// OCCLUSION_ADATA constant parameters
	CalibrationMatrix cmx(info);
	Object3D volPatchesRef(scene.o[VOLUME_PATCHES]);
	int numFaces = (int)rowsPerFaceV.size();
	int numShapes = scene.o[VOLUME_PATCHES].s.size();
		Point vopC(0.0f, 0.0f, 0.0f);
		for (int si = 0; si < numShapes; ++si)
			vopC += scene.o[VOLUME_PATCHES].s[si].c;
		vopC /= numShapes;
		volPatchesRef.tra(vopC * (-1.0f));	// set the reference centered in the origin to optimize rotation
	
	// new parameters
	std::vector<int> numShapesInFace(numFaces);
	std::vector<int> idxS0ofF(numFaces + 1);
		int siAccum = 0;
		for (int fi = FRONT; fi < numFaces; ++fi) {
			numShapesInFace[fi] = rowsPerFaceV[fi] * colsPerFaceV[fi];
			idxS0ofF[fi] = siAccum;
			siAccum += rowsPerFaceV[fi] * colsPerFaceV[fi];
		}
		idxS0ofF[numFaces] = numShapes;
	std::vector<bool> facingWLFace(numFaces, false);
	std::vector<float> attTermV(numShapes, 0.0f);
	
		
	std::vector<Point> faceNRef(numFaces);
		for (int fi = FRONT; fi < numFaces; ++fi)
			faceNRef[fi] = volPatchesRef.s[idxS0ofF[fi]].normalQUAD();
	std::vector<Point*> shapeN(numShapes);
		int fi = -1;
		for (int si = 0; si < numShapes; ++si) {
			if (si >= idxS0ofF[fi])
				fi++;
			shapeN[si] = &(faceNRef[fi]);
		}
	std::vector<float> area(numShapes);
		for (int si = 0; si < numShapes; ++si)
			area[si] = volPatchesRef.s[si].areaRECTANGLE();
	Point walN = scene.o[WALL].normalQUAD();
	Point walL = int_linePN_object3D(scene.o[LASER].s[0].c, scene.o[LASER].normalQUAD(), scene.o[WALL]);
	int freq_idx = get_freq_idx(info, frame00.freq);	// returns -1 if no idx correspondance was found
		if (freq_idx < 0) {
			std::cout << "\nWarning (in updateVolumePatches_Occlusion):\n  Frame freq = " << frame00.freq << " is not a freq in .cmx freqV = ";
			print(info.freqV);
			std::cout << "\nending...";
			return;
		}
	int numPix = frame00.data.size();
	int sizeofFrameData = numPix*sizeof(float);

	// OCCLUSION_ADATA variable parameters
	Scene sceneCopy(scene);
	Frame frameSim00;
	Frame frameSim90;
		std::vector<float> frameSim00_data(numPix, 0.0f);
		std::vector<float> frameSim90_data(numPix, 0.0f);
		frameSim00.set(frameSim00_data, frame00.rows,  frame00.cols, info.freqV[freq_idx], 0.0f, info.shutV[info.shutV.size()-1], info.phasV[0], ps_, pSim_);
		frameSim90.set(frameSim90_data, frame90.rows,  frame90.cols, info.freqV[freq_idx], 0.0f, info.shutV[info.shutV.size()-1], info.phasV[1], ps_, pSim_);
	std::vector<Point> faceN(numFaces);

	// old parameters
	/*
	int facesFacing_size = 0;
	std::vector<int> facesFacingIdx(numFaces, 0);
	std::vector<int> firstShapeIdx_in_volPatchesRadiance_of_facesFacingIdx(numFaces, 0);
	int volPatchesRadiance_size = 0;
	std::vector<int> volPatchesRadianceIdx (numShapes, 0);
	std::vector<float> volPatchesRadiance (numShapes, 0.0f);
	*/

	std::vector<int> transientImagePix_size (numPix, 0);
	std::vector<std::vector<float>> transientImageDist (numPix, std::vector<float>(numShapes));
	std::vector<std::vector<float>> transientImageAttTerm (numPix, std::vector<float>(numShapes));
	Point traV(0.0f, 0.0f, 0.0f);

	// OCCLUSION_ADATA storing additional data in struct
	struct OCCLUSION_ADATA adata;
	adata.info = &info;
	adata.cmx = &cmx;
	adata.volPatchesRef = &volPatchesRef;		// this 2nd copy is the reference volPatches centered in the origin and should not be modified
	adata.numFaces = numFaces;
	adata.numShapes = numShapes;
	
	// new parameters
	adata.numShapesInFace = &numShapesInFace;
	adata.idxS0ofF = &idxS0ofF;
	adata.facingWLFace = &facingWLFace;
	adata.attTermV = &attTermV;

	adata.faceNRef = &faceNRef;
	adata.shapeN = &shapeN;						// unnused
	adata.area = &area;
	adata.walN = &walN;
	adata.walL = &walL;
	adata.freq_idx = freq_idx;
	adata.ps_ = ps_;
	adata.pSim_ = pSim_;
	adata.numPix = numPix;						// rows*cols
	adata.sizeofFrameData = sizeofFrameData;	// rows*cols*sizeof(float) = rows*cols*4
	adata.sceneCopy = &sceneCopy;				// semi-constant: sceneCopy.o[VOLUME_PATHCES] is modified in each iteration of levmar
	//adata.sceneCopy = &scene;					// original scene, for debugging
	adata.frame00 = &frame00;
	adata.frame90 = &frame90;
	adata.frameSim00 = &frameSim00;
	adata.frameSim90 = &frameSim90;
	adata.faceN = &faceN;
	
	// old parameters
	/*
	adata.facesFacing_size = facesFacing_size;
	adata.facesFacingIdx = &facesFacingIdx;
	adata.firstShapeIdx_in_volPatchesRadiance_of_facesFacingIdx = &firstShapeIdx_in_volPatchesRadiance_of_facesFacingIdx;
	adata.volPatchesRadiance_size = volPatchesRadiance_size;
	adata.volPatchesRadianceIdx = &volPatchesRadianceIdx;
	adata.volPatchesRadiance = &volPatchesRadiance;
	*/

	adata.transientImagePix_size = &transientImagePix_size;
	adata.transientImageDist = &transientImageDist;
	adata.transientImageAttTerm = &transientImageAttTerm;
	adata.traV = &traV;
	
	// LEVMAR function parameters

	// set the initial parameters (p). Implemented to let the user choose which parameters are we going to use. Just modify the pAll[iAll] and the  pUse[iAll]
	// pAll[]
	const int pAll_size = 7;			// x, y, z, phi[-PI,+PI], theta[-PI/2,+PI/2], roll[-PI,+PI], kTS
	float* pAll = new float[pAll_size];		// p[0],p[1],p[2],p[3],p[4],p[5],p[6] = x,y,z,phi,theta,roll,kTS
	pAll[0] = 1.40f;				// initial parameters estimate (x,y,z) (p[0] = 1.00f; p[1] = 0.85f; p[2] = -0.28f;)
	pAll[1] = 0.4f;
	pAll[2] = -0.7f;				
	//p[0] = (scene.o[CAMERA].s[0].c.x + scene.o[LASER].s[0].c.x) / 2.0f;	// for testing...
	//p[1] = (scene.o[CAMERA].s[0].c.y + scene.o[LASER].s[0].c.y) / 2.0f;
	//p[2] = (scene.o[CAMERA].s[0].c.z + scene.o[LASER].s[0].c.z) / 2.0f + 3.0f;
	pAll[3] = 0.0f * PI / 180.0f;	// initial parameters estimate (phi,theta,roll) (in radians)
	pAll[4] = 0.0f;
	pAll[5] = 0.0f;	
	pAll[6] = 0.63f;				// initial parameters estimate kTS
	// pUse[]
	bool* pUse = new bool[pAll_size];		// pUse[iAll] = true: pAll[iAll] will be used in p[i].
	pUse[0] = true;		// x
	pUse[1] = true;		// y
	pUse[2] = true;		// z
	pUse[3] = true;		// phi
	pUse[4] = true;		// theta
	pUse[5] = true;		// roll
	pUse[6] = true;		// kTS
	// pAllL[], pAllU[] (pAll bounds)
	const float aso = 1.01f;		// angle scale offset to the angular limits, to avoid problems around the bounds of these limits
	const float boxSizeHalf = 0.3f;
	std::vector<float> pAllL(pAll_size);
	pAllL[0] = scene.o[LASER].s[0].c.x + 0.1f + boxSizeHalf;
	pAllL[1] = 0.0f + boxSizeHalf;
	pAllL[2] = scene.o[WALL].s[0].c.z + boxSizeHalf;
	pAllL[3] = -aso*PI;
	pAllL[4] = -aso*PI/2.0f;
	pAllL[5] = -aso*PI;
	pAllL[6] = pAll[6] * 0.001f;	
	std::vector<float> pAllU(pAll_size);
	pAllU[0] = scene.o[WALL].s[0].p[1].x - boxSizeHalf;
	pAllU[1] = scene.o[WALL].s[0].p[2].y - boxSizeHalf;
	if (scene.o[FLOOR].s.size() > 0)
		pAllU[2] = scene.o[FLOOR].s[0].c.z - boxSizeHalf;
	else
		pAllU[2] = scene.o[LASER].s[0].c.z + 0.75f - boxSizeHalf;
	pAllU[3] = aso*PI;
	pAllU[4] = aso*PI/2.0f;
	pAllU[5] = aso*PI;
	pAllU[6] = pAll[6] * 1000.0f;
	// get p_size
	int p_size = 0;
	for (int iAll = 0; iAll < pAll_size; ++iAll) {
		if (pUse[iAll]) {
			p_size++;
	}	}
	// p[], pL[], pU[], idxOfpInpAll[iAll], idxOfpAllInp[i]
	float* p = new float[p_size];	// this will store the parameters we are going to use, this is the actual parameter vector
	std::vector<float> pL(p_size);
	std::vector<float> pU(p_size);
	int* idxOfpInpAll = new int[pAll_size];
	int* idxOfpAllInp = new int[p_size];
	int i = 0;
	for (int iAll = 0; iAll < pAll_size; ++iAll) {
		if (pUse[iAll]) {
			p[i]  = pAll[iAll];
			pL[i] = pAllL[iAll];
			pU[i] = pAllU[iAll];
			idxOfpAllInp[i]    = iAll;
			idxOfpInpAll[iAll] = i;
			i++;
		} else {
			idxOfpInpAll[iAll] = -1;
	}	}

	// OCCLUSION_ADATA variable parameters p bounds
	adata.pAll_size = pAll_size;
	adata.pAll = pAll;
	adata.pUse = pUse;
	adata.idxOfpInpAll = idxOfpInpAll;
	adata.idxOfpAllInp = idxOfpAllInp;
	adata.pAllL = &pAllL;
	adata.pAllU = &pAllU;
	adata.pL = &pL;
	adata.pU = &pU;

	bool resetP = true;					// resets p in each iteration to its initial value
	
	// set the initial values (x)
	const int x_size = numPix * info.phasV.size();	// rows*cols*phases = rows*cols*2
	float* x = new float[x_size];					// x[i]: value of simulated pixel i
	// optimization control parameters; passing to levmar NULL instead of opts reverts to defaults
	float opts[LM_OPTS_SZ];
	//float* optsNULL = NULL;	// in case we decide to pass NULL
	opts[0] = LM_INIT_MU;		// scale factor for initial \mu (def=LM_INIT_MU=1E-03)
	opts[1] = 1E-06;			// stopping thresholds for ||J^T e||_inf (def=1E-15)
	opts[2] = 1E-06;			// stopping thresholds for ||Dp||_2 (def=1E-15)
	opts[3] = 1E-06;			// stopping thresholds for ||e||_2 (def=1E-20)
	opts[4] = 1E-04;			// step used in difference approximation to the Jacobian (def=LM_DIFF_DELTA=1E-06)
	// information parameters. Output of the optimization function about internal parameters such as number of iterations (inf[5]) etc
	float inf[LM_INFO_SZ];
	// other unused parameters (work, covar)
	float* work = NULL;
	float* covar = NULL;
	// invoke the optimization function (returns the number of iterations, -1 if failed)
	int maxIters = 5000;
	int numIters = 0;
	int numCaptures = 0;
	
	
	// printing actual parameters
	const bool print_p_info = true;		// enables the parameter printing
	const bool actual_p_known = false;	// enables the actual and erro parameter printing (if print_p_info is also enabled)
	float* pA = new float[pAll_size];
	pA[0] = 2.5f;	pA[1] = 0.75f;	pA[2] = -0.5f;	pA[3] = 0.0f;	pA[4] = 0.0;	pA[5] = 0.0f;	pA[6] = 0.01f;
	if (print_p_info && actual_p_known) {
		std::cout << "\n\nActual parameters : pA"
			"\npos = (x,y,z)    = (" << pA[0] << ", " << pA[1] << ", " << pA[2] << ") (meters)"
			"\n(phi,theta,roll) = (" << pA[3] * 180.0f / PI << ", " << pA[4] * 180.0f / PI << ", " << pA[5] * 180.0f / PI << ") (degrees)"
			"\nkTs              = " << pA[6];
	}
	// printing initial parameters
	float* p0 = new float[pAll_size];
	p0[0] = getPiAll(0,p,&adata);	p0[1] = getPiAll(1,p,&adata);	p0[2] = getPiAll(2,p,&adata);
	p0[3] = getPiAll(3,p,&adata);	p0[4] = getPiAll(4,p,&adata);	p0[5] = getPiAll(5,p,&adata);	p0[6] = getPiAll(6,p,&adata);
	if (print_p_info) {
		std::cout << "\n\nInitial parameters : p0"
			"\npos = (x,y,z)    = (" << p0[0] << ", " << p0[1] << ", " << p0[2] << ") (meters)"
			"\n(phi,theta,roll) = (" << p0[3] * 180.0f / PI << ", " << p0[4] * 180.0f / PI << ", " << p0[5] * 180.0f / PI << ") (degrees)"
			"\nkTs              = " << p0[6];
	}

	// openCV, plotting
	float scale = -1.0f;
	if (pSim_)
		scale = (int)(CAMERA_PIX_X / (float)PMD_SIM_COLS);
	bool epExtStarted = false;
	bool epExtUsing = true;
	Engine *epExt;

	// Timing
	clock_t begin_time, end_time;
	float ms_time, fps_time;

	// --- LOOP ------------------------------------------------------------------------------------------------
	bool first_iter = true;
	while (loop || first_iter) {

		// External control
		if (!PMD_LOOP_ENABLE && !first_iter)
			break;
		first_iter = false;

		// Syncronization
		locker_frame_object.lock();	// Lock mutex_frame_object, any thread which used mutex_frame_object can NOT continue until unlock()
		while (!UPDATED_NEW_FRAME)	//std::cout << "Waiting in Object to finish the UPDATED_NEW_Frame. This is OK!\n";
			cv_frame_object.wait(locker_frame_object);
		
		// Timing
		begin_time = clock();

		// ----- Update pixel patches, setting the Best Fit --------------------------------------------------------------------------------
		memcpy(x, frame00.data.data(), sizeofFrameData);			// actual measurement values to be fitted with the model
		memcpy(x + numPix, frame90.data.data(), sizeofFrameData);
		// http://users.ics.forth.gr/~lourakis/levmar/
		numIters = slevmar_dif(set_Occlusion_Simulation_Frame_Optim, p, x, p_size, x_size, maxIters, opts, inf, work, covar, (void*)&adata); // withOUT analytic Jacobian
		scene.o[VOLUME_PATCHES].set(sceneCopy.o[VOLUME_PATCHES]);	// we could also modify the original scene with the final parameters

		// plotting simulated frames
		frameSim00.plot(1, false, "S00", scale);
		frameSim90.plot(1, false, "S90", scale);
		// plotting rows values. This takes around 30ms
		plot_rowcol4(frame00, frameSim00, frame90, frameSim90, "Real00", "Sim00", "Real90", "Sim90", 0, -1, true, epExtStarted, epExtUsing, epExt);
			//plot_rowcol2(frame00, frameSim00, "Real", "Sim", 8, -1, false, epExtStarted, epExtUsing, epExt);
			//plot_rowcol(frame00, "Real Frame", frame00.rows/2, -1, epExtStarted, epExtUsing, epExt);
			//plot_rowcol(frameSim00, "Simulated Frame", frameSim00.rows/2, -1, epExtStarted, epExtUsing, epExt);

		// Syncronization	//std::cout << ",    UPDATED_NEW_SCENE\n";
		UPDATED_NEW_FRAME = false;
		UPDATED_NEW_SCENE = true;
		cv_frame_object.notify_all();	// Notify all cv_frame_object. All threads waiting for cv_frame_object will break the wait after waking up
		locker_frame_object.unlock();	// Unlock mutex_frame_object, now threads which used mutex_frame_object can continue

		// Timing and info
		end_time = clock();
		ms_time = 1000.0f * float(end_time - begin_time) / (float)CLOCKS_PER_SEC;
		fps_time = 1000.0f / ms_time;
		std::cout << "\n\n\nCapture#: " << numCaptures++ << ",    levmarIters: " << numIters << ",    time: " << ms_time << " ms,    fps = " << fps_time;
		
		// printing final parameters
		if (print_p_info) {
			std::cout << "\n\nFinal parameters : pF"
				"\npos = (x,y,z)    = (" << getPiAll(0,p,&adata) << ", " << getPiAll(1,p,&adata) << ", " << getPiAll(2,p,&adata) << ") (meters)"
				"\n(phi,theta,roll) = (" << getPiAll(3,p,&adata) * 180.0f / PI << ", " << getPiAll(4,p,&adata) * 180.0f / PI << ", " << getPiAll(5,p,&adata) * 180.0f / PI << ") (degrees)"
				"\nkTs              = " << getPiAll(6,p,&adata);
		}
		// printing error parameters
		if (print_p_info && actual_p_known) {
			std::cout << "\n\nError parameters : pE"
				"\npos = (x,y,z)    = (" << getPiAll(0,p,&adata) - pA[0] << ", " << getPiAll(1,p,&adata) - pA[1] << ", " << getPiAll(2,p,&adata) - pA[2] << ") (meters)"
				"\n(phi,theta,roll) = (" << (getPiAll(3,p,&adata) - pA[3]) * 180.0f / PI << ", " << (getPiAll(4,p,&adata) - pA[4]) * 180.0f / PI << ", " << (getPiAll(5,p,&adata) - pA[5]) * 180.0f / PI << ") (degrees)"
				"\nkTs              = " << getPiAll(6,p,&adata) - pA[6];
		}

		// reset p
		if (resetP) {
			for (int i = 0; i < p_size; i++)
				p[i] = p0[idxOfpAllInp[i]];
		}
	}
	// --- END OF LOOP -----------------------------------------------------------------------------------------
	
	delete[] p;
	delete[] pA;
	delete[] p0;
	delete[] x;
}
void updateVolumePatches_Occlusion_antiBugThread(Info & info, Scene & scene, Frame & frame00, Frame & frame90, std::vector<int> & rowsPerFaceV, std::vector<int> & colsPerFaceV, bool loop, PixStoring ps_, bool pSim_) {
	scene.o[VOLUME_PATCHES].updateVolumePatches_Occlusion(info, scene, frame00, frame90, rowsPerFaceV, colsPerFaceV, loop, ps_, pSim_);
}
void Object3D::setVolumePatches_OLD() {

	// Set the Object3D
	s.resize(0);
	ot = VOLUME_PATCHES;
	ps = UNKNOWN_PIS;

	// Set reference from which the Volume Patches are made (at the end the corresponded transformation will be applied)
	Point refC(3.0f, 0.0f, 0.0f);
	Point refN(0.0f, 0.0f, -1.0f);
	Point originC(0.0f, 0.0f, 0.0f);
	Point originN(0.0f, 0.0f, 1.0f);

	// Set variables
	float minX = 0.0f;	float maxX = 1.0f;	float stepX = (maxX - minX) / (float)VOLUME_GRID_SIZE_X;
	float minY = 0.25f;	float maxY = 1.25f;	float stepY = (maxY - minY) / (float)VOLUME_GRID_SIZE_Y;
	float minZ = 0.0f;	float maxZ = 1.0f;	float stepZ = (maxZ - minZ) / (float)VOLUME_GRID_SIZE_Z;
	Shape sX;

	// Loop filling the Object3D
	float z = 0.0f;
	for (float x = minX; x < maxX - stepX / 2.0f; x += stepX) {
		for (float y = minY; y < maxY - stepY / 2.0f; y += stepY) {
			sX.clear();
			sX.p.resize(QUAD);
			sX.set(1.0f, 1.0f, 1.0f, 1.0f, 1.0f, QUAD);
			sX.p[0].set(x, y, z);
			sX.p[1].set(x + stepX, y, z);
			sX.p[2].set(x + stepX, y + stepY, z);
			sX.p[3].set(x, y + stepY, z);
			sX.c.set   (x + stepX / 2.0f, y + stepY / 2.0f, z);
			s.push_back(sX);
	}	}

	// Apply transformations
	rotYfromP(radNN(originN, refN), originC);
	tra(refC);
}
void Object3D::updateVolumePatches_Occlusion_OLD(Info & info, Scene & scene, Frame & frame00, Frame & frame90, bool loop, PixStoring ps_, bool pSim_) {

	// Setting constant elements
	CalibrationMatrix cmx(info);
	Scene sceneCopy(scene);
	Object3D volPatchesRef(scene.o[VOLUME_PATCHES]);
	Frame frameSim00, frameSim90;
	Point camC = scene.o[CAMERA].s[0].c;
	Point camN = scene.o[CAMERA].normalQUAD();
	Object3D screenFoVmeasNs;
	screenFoVmeasNs.setScreenFoVmeasNs(camC, camN, ps_, pSim_);
	Point walN = scene.o[WALL].normalQUAD();
	Point vovN = scene.o[VOLUME_PATCHES].s[0].normalQUAD();
	Point _vovN = vovN * (-1.0f);
	float dRes = 0.05f;;
	//float dRes = dist(scene.o[VOLUME_PATCHES].s[0].p[0], scene.o[VOLUME_PATCHES].s[0].p[1]);

	// Syncronization
	std::unique_lock<std::mutex> locker_frame_object;	// Create a defered locker (a locker not locked yet)
	locker_frame_object = std::unique_lock<std::mutex>(mutex_frame_object, std::defer_lock);

	// --- LOOP ------------------------------------------------------------------------------------------------
	bool first_iter = true;
	while (loop || first_iter) {

		const clock_t begin_time = clock();

		if (!PMD_LOOP_ENABLE && !first_iter)
			break;
		first_iter = false;

		// Syncronization
		locker_frame_object.lock();		// Lock mutex_frame_object, any thread which used mutex_frame_object can NOT continue until unlock()
		while (!UPDATED_NEW_FRAME)	//std::cout << "Waiting in Object to finish the UPDATED_NEW_Frame. This is OK!\n";
			cv_frame_object.wait(locker_frame_object);

		// Update pixel patches, setting the Best Fit
		updateVolumePatches_Occlusion_OLD_BestFit(cmx, sceneCopy, volPatchesRef, frameSim00, frameSim90, frame00, frame90, walN, _vovN, dRes, ps_, pSim_);
		scene.o[VOLUME_PATCHES] = sceneCopy.o[VOLUME_PATCHES];

		// Syncronization
		//std::cout << ",    UPDATED_NEW_SCENE\n";
		UPDATED_NEW_FRAME = false;
		UPDATED_NEW_SCENE = true;
		cv_frame_object.notify_all();	// Notify all cv_frame_object. All threads waiting for cv_frame_object will break the wait after waking up
		locker_frame_object.unlock();	// Unlock mutex_frame_object, now threads which used mutex_frame_object can continue

		const clock_t end_time = clock();
		float ms_time = 1000.0f * float(end_time - begin_time) / (float)CLOCKS_PER_SEC;
		float fps_time = 1000.0f / ms_time;
		std::cout << "\ntime = " << ms_time << " ms,    fps = " << fps_time << " fps";
	}
	// --- END OF LOOP -----------------------------------------------------------------------------------------
}
void updateVolumePatches_Occlusion_OLD_antiBugThread(Info & info, Scene & scene, Frame & frame00, Frame & frame90, bool loop, PixStoring ps_, bool pSim_) {
	scene.o[VOLUME_PATCHES].updateVolumePatches_Occlusion_OLD(info, scene, frame00, frame90, loop, ps_, pSim_);
}
// Setter, Updater Pixel Patches (10)
void Object3D::setPixelPatches(Scene & scene, float distDefault, PixStoring ps_, bool pSim_) {

	// Set the Object3D
	s.resize(numPix(ps_, pSim_));
	ot = PIXEL_PATCHES;
	ps = ps_;

	// Setting constant elements
	Object3D screenFoVmeasN;
	screenFoVmeasN.setScreenFoVmeasN(scene.o[CAMERA].s[0].c, scene.o[CAMERA].normalQUAD(), ps_, pSim_);

	// Update pixel patches
	for (int i = 0; i < s.size(); i++) {
		s[i].p.resize(QUAD);
		s[i].set(1.0f, 1.0f, 1.0f, 1.0f, 1.0f, QUAD);
		s[i].p[0].set(scene.o[CAMERA].s[0].c + screenFoVmeasN.s[i].p[0] * distDefault);
		s[i].p[1].set(scene.o[CAMERA].s[0].c + screenFoVmeasN.s[i].p[1] * distDefault);
		s[i].p[2].set(scene.o[CAMERA].s[0].c + screenFoVmeasN.s[i].p[2] * distDefault);
		s[i].p[3].set(scene.o[CAMERA].s[0].c + screenFoVmeasN.s[i].p[3] * distDefault);
		s[i].c.set   (scene.o[CAMERA].s[0].c + screenFoVmeasN.s[i].c    * distDefault);
	}
}
void Object3D::setPixelPatches(Scene & scene, Frame & frame00, Frame & frame90, PixStoring ps_, bool pSim_) {

	// Set the Object3D
	s.resize(numPix(ps_, pSim_));
	ot = PIXEL_PATCHES;
	ps = ps_;

	// Setting constant elements
	std::vector<float> depthMap(numPix(ps_, pSim_));	// depthMap stores the relative to the camera distance per pixel
	Object3D screenFoVmeasN;
	screenFoVmeasN.setScreenFoVmeasN(scene.o[CAMERA].s[0].c, scene.o[CAMERA].normalQUAD(), ps_, pSim_);

	// Setting Depth Map from the FRAMES captured
	setDepthMap(depthMap, frame00, frame90);

	// Update pixel patches
	for (int i = 0; i < s.size(); i++) {
		s[i].p.resize(QUAD);
		s[i].set(1.0f, 1.0f, 1.0f, 1.0f, 1.0f, QUAD);
		s[i].p[0].set(scene.o[CAMERA].s[0].c + screenFoVmeasN.s[i].p[0] * depthMap[i]);
		s[i].p[1].set(scene.o[CAMERA].s[0].c + screenFoVmeasN.s[i].p[1] * depthMap[i]);
		s[i].p[2].set(scene.o[CAMERA].s[0].c + screenFoVmeasN.s[i].p[2] * depthMap[i]);
		s[i].p[3].set(scene.o[CAMERA].s[0].c + screenFoVmeasN.s[i].p[3] * depthMap[i]);
		s[i].c.set(scene.o[CAMERA].s[0].c    + screenFoVmeasN.s[i].c    * depthMap[i]);
	}
}
void Object3D::updatePixelPatches_Sinusoid(Scene & scene, Frame & frame00, Frame & frame90, bool loop, PixStoring ps_, bool pSim_) {

	// Setting constant elements
	std::vector<float> depthMap(numPix(ps_, pSim_));	// depthMap stores the relative to the camera distance per pixel
	Object3D screenFoVmeasN;
	screenFoVmeasN.setScreenFoVmeasN(scene.o[CAMERA].s[0].c, scene.o[CAMERA].normalQUAD(), ps_, pSim_);

	// Syncronization
	std::unique_lock<std::mutex> locker_frame_object;	// Create a defered locker (a locker not locked yet)
	locker_frame_object = std::unique_lock<std::mutex>(mutex_frame_object, std::defer_lock);

	// Matlab plotting
	Frame frameMod;
		const int frames = 3;
	std::vector<Frame*> frameV(frames);
		frameV[0] = &frame00;
		frameV[1] = &frame90;
		frameV[2] = &frameMod;
		//frameV[2] = &frameMod;
	std::vector<char*> textV(frames);
		textV[0] = "Frame \\phi=0";
		textV[1] = "Frame \\phi=\\pi/2";
		textV[2] = "Frame Module";
		//textV[2] = "Frame Module";
	std::vector<float> colorV(frames*3);
		colorV[0] = 0.0; colorV[1] = 0.0; colorV[2] = 1.0;	
		colorV[3] = 1.0; colorV[4] = 0.4; colorV[5] = 0.4;
		colorV[6] = 0.0; colorV[7] = 0.0; colorV[8] = 0.0;
		//colorV[6] = 1.0; colorV[7] = 1.0; colorV[8] = 1.0;
	float lineWidth = 1.0f;
	int rowPlot = 40; //frame00.rows/3;
	int colPlot = -1;
	bool avgPlot = false;
	bool legend = true;
	bool freezePlot = false;
	bool epExtStarted = false;
	bool epExtUsing = true;
	Engine *epExt;

	// --- LOOP ------------------------------------------------------------------------------------------------
	bool first_iter = true;
	while (loop || first_iter) {

		//const clock_t begin_time = clock();

		if (!PMD_LOOP_ENABLE && !first_iter)
			break;

		// Syncronization
		locker_frame_object.lock();		// Lock mutex_frame_object, any thread which used mutex_frame_object can NOT continue until unlock()
		while (!UPDATED_NEW_FRAME)	//std::cout << "Waiting in Object to finish the UPDATED_NEW_Frame. This is OK!\n";
			cv_frame_object.wait(locker_frame_object);

		// Setting Depth Map from the FRAMES captured
		setDepthMap(depthMap, frame00, frame90);

		// Update pixel patches
		for (int i = 0; i < s.size(); i++) {
			s[i].p[0].set(scene.o[CAMERA].s[0].c + screenFoVmeasN.s[i].p[0] * depthMap[i]);
			s[i].p[1].set(scene.o[CAMERA].s[0].c + screenFoVmeasN.s[i].p[1] * depthMap[i]);
			s[i].p[2].set(scene.o[CAMERA].s[0].c + screenFoVmeasN.s[i].p[2] * depthMap[i]);
			s[i].p[3].set(scene.o[CAMERA].s[0].c + screenFoVmeasN.s[i].p[3] * depthMap[i]);
			s[i].c.set   (scene.o[CAMERA].s[0].c + screenFoVmeasN.s[i].c    * depthMap[i]);
		}
		
		// plotting rows values. This takes around 30ms
		//plot_rowcol2(frame00, frame90, "R00", "R90", frame00.rows/2, -1, false, epExtStarted, epExtUsing, epExt);
		//plot_rowcol4(frame00, frame90, frame00, frame90, "Ra00", "Ra90", "Rb00", "Rb90", rowPlot, colPlot, avgPlot, epExtStarted, epExtUsing, epExt);
		frameMod.set(frame00, frame90, first_iter);
		plot_rowcolV(frameV, textV, colorV, lineWidth, rowPlot, colPlot, avgPlot, legend, freezePlot,epExtStarted, epExtUsing, epExt);

		// Syncronization
		//std::cout << ",    UPDATED_NEW_SCENE\n";
		UPDATED_NEW_FRAME = false;
		UPDATED_NEW_SCENE = true;
		cv_frame_object.notify_all();	// Notify all cv_frame_object. All threads waiting for cv_frame_object will break the wait after waking up
		locker_frame_object.unlock();	// Unlock mutex_frame_object, now threads which used mutex_frame_object can continue

		//const clock_t end_time = clock();
		//float ms_time = 1000.0f * float(end_time - begin_time) / (float)CLOCKS_PER_SEC;
		//float fps_time = 1000.0f / ms_time;
		//std::cout << "time = " << ms_time << " ms,    fps = " << fps_time <<  " fps\n";
		
		first_iter = false;
	}
	// --- END OF LOOP -----------------------------------------------------------------------------------------
}
void updatePixelPatches_Sinusoid_antiBugThread(Scene & scene, Frame & frame00, Frame & frame90, bool loop, PixStoring ps_, bool pSim_) {
	scene.o[PIXEL_PATCHES].updatePixelPatches_Sinusoid(scene, frame00, frame90, loop, ps_, pSim_);
}
void Object3D::updatePixelPatches_Simulation(Info & info, Scene & scene, Frame & frame00, Frame & frame90, bool loop, PixStoring ps_, bool pSim_) {
	
	// Setting constant elements
	CalibrationMatrix cmx(info);
	Scene sceneCopy(scene);
	Frame frameSim00, frameSim90;
	Point camC = scene.o[CAMERA].s[0].c;
	Point camN = scene.o[CAMERA].normalQUAD();
	Object3D screenFoVmeasNs;
	screenFoVmeasNs.setScreenFoVmeasNs(camC, camN, ps_, pSim_);
	// Parameters for the WithTilt function
	float alphaRadRes = 3.0f * PI / 180.0f;							// 3.0f * PI / 180.0f
	float alphaRadMin = -45.0f * PI / 180.0f;							// -45.0f * PI / 180.0f
	float alphaRadMax = 45.0f * PI / 180.0f + alphaRadRes / 2.0f;	// 45.0f * PI / 180.0f
	int alphaNum = (int)((alphaRadMax - alphaRadMin) / alphaRadRes) + 1;
	float alphaRad = alphaRadMin;
	float angleFoVRad = atan(CAMERA_FOV_X_METERS / (2.0f * CAMERA_DIST_FOV_MEAS));
	std::vector<float> sinAG(alphaNum);
	for (size_t i = 0; i < sinAG.size(); i++) {
		sinAG[i] = sin(alphaRad) / sin((PI/2.0f)+angleFoVRad-alphaRad);
		alphaRad += alphaRadRes;
	}
	Object3D NsMod(screenFoVmeasNs);	// this is a artificial object, it stores in each x the corresponding module of screenFoVmeasNs
	for (size_t i = 0; i < NsMod.s.size(); i++) {
		NsMod.s[i].p[0].x = NsMod.s[i].p[0].mod();
		NsMod.s[i].p[1].x = NsMod.s[i].p[1].mod();
		NsMod.s[i].p[2].x = NsMod.s[i].p[2].mod();
		NsMod.s[i].p[3].x = NsMod.s[i].p[3].mod();
		NsMod.s[i].c.   x = NsMod.s[i].c.mod();
	}

	// Syncronization
	std::unique_lock<std::mutex> locker_frame_object;	// Create a defered locker (a locker not locked yet)
	locker_frame_object = std::unique_lock<std::mutex>(mutex_frame_object, std::defer_lock);

	// --- LOOP ------------------------------------------------------------------------------------------------
	bool first_iter = true;
	while (loop || first_iter) {

		if (!PMD_LOOP_ENABLE && !first_iter)
			break;
		first_iter = false;

		// Syncronization
		locker_frame_object.lock();		// Lock mutex_frame_object, any thread which used mutex_frame_object can NOT continue until unlock()
		while (!UPDATED_NEW_FRAME)	//std::cout << "Waiting in Object to finish the UPDATED_NEW_Frame. This is OK!\n";
			cv_frame_object.wait(locker_frame_object);
		
		const clock_t begin_time = clock();

		// Update pixel patches, setting the Best Fit
		//updatePixelPatches_Simulation_BestFit(cmx, sceneCopy, frameSim00, frameSim90, frame00, frame90, camC, camN, screenFoVmeasNs, ps_, pSim_);
		updatePixelPatches_Simulation_BestFit_WithTilt(cmx, sceneCopy, NsMod, sinAG, frameSim00, frameSim90, frame00, frame90, camC, camN, screenFoVmeasNs, ps_, pSim_);
		//for (int i = 0; i < 1000; ++i)
		//updatePixelPatches_Simulation_BestFit_Optim(cmx, sceneCopy, frameSim00, frameSim90, frame00, frame90, camC, camN, screenFoVmeasNs, ps_, pSim_);
		scene.o[PIXEL_PATCHES] = sceneCopy.o[PIXEL_PATCHES];
		//frameSim.plot(1, false, "Frame Sim Dir");

		// Syncronization
		//std::cout << ",    UPDATED_NEW_SCENE\n";
		UPDATED_NEW_FRAME = false;
		UPDATED_NEW_SCENE = true;
		cv_frame_object.notify_all();	// Notify all cv_frame_object. All threads waiting for cv_frame_object will break the wait after waking up
		locker_frame_object.unlock();	// Unlock mutex_frame_object, now threads which used mutex_frame_object can continue

		const clock_t end_time = clock();
		float ms_time = 1000.0f * float(end_time - begin_time) / (float)CLOCKS_PER_SEC;
		float fps_time = 1000.0f / ms_time;
		std::cout << "\ntime = " << ms_time << " ms,    fps = " << fps_time <<  " fps";
		std::cout << "\ndist(cam,wall) = " << dist(camC, (scene.o[PIXEL_PATCHES].s[0].c + scene.o[PIXEL_PATCHES].s[rc2idx(0, cols(ps_, pSim_)-1, ps_, pSim_)].c + scene.o[PIXEL_PATCHES].s[rc2idx(rows(ps_, pSim_)-1, 0, ps_, pSim_)].c + scene.o[PIXEL_PATCHES].s[numPix(ps_, pSim_)-1].c) / 4.0f);
	}
	// --- END OF LOOP -----------------------------------------------------------------------------------------
}
void updatePixelPatches_Simulation_antiBugThread(Info & info, Scene & scene, Frame & frame00, Frame & frame90, bool loop, PixStoring ps_, bool pSim_) {
	scene.o[PIXEL_PATCHES].updatePixelPatches_Simulation(info, scene, frame00, frame90, loop, ps_, pSim_);
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
	return normalTo3P(s[0].p[0], s[0].p[1], s[0].p[2]);
}
Point Object3D::normalQUAD() {
	return normalTo3P(s[0].p[0], s[0].p[1], s[0].p[2]);
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
void Object3D::rot(Point & axisN, float rad) {
	float r11, r12, r13, r21, r22, r23, r31, r32, r33;
	setRotationMatrix(r11, r12, r13, r21, r22, r23, r31, r32, r33, axisN.x, axisN.y, axisN.z, rad);
	for (size_t j = 0; j < s.size(); j++) {
		for (size_t i = 0; i < s[j].p.size(); i++)
			s[j].p[i].rotOpt(r11, r12, r13, r21, r22, r23, r31, r32, r33);
		s[j].c.rotOpt(r11, r12, r13, r21, r22, r23, r31, r32, r33);
	}
}
void Object3D::rotXYZ(float radX, float radY, float radZ, bool inverted) {
	float cosX = cos(radX); float sinX = sin(radX);
	float cosY = cos(radY); float sinY = sin(radY);
	float cosZ = cos(radZ); float sinZ = sin(radZ);
	if (inverted) {
		for (size_t j = 0; j < s.size(); j++) {
			for (size_t i = 0; i < s[j].p.size(); i++)
				s[j].p[i].rotXYZoptInverted(cosX, sinX, cosY, sinY, cosZ, sinZ);
			s[j].c.rotXYZoptInverted(cosX, sinX, cosY, sinY, cosZ, sinZ);
		}
	} else {
		for (size_t j = 0; j < s.size(); j++) {
			for (size_t i = 0; i < s[j].p.size(); i++)
				s[j].p[i].rotXYZopt(cosX, sinX, cosY, sinY, cosZ, sinZ);
			s[j].c.rotXYZopt(cosX, sinX, cosY, sinY, cosZ, sinZ);
		}
	}
}
void Object3D::rotX(float rad) {
	float cosT = cos(rad);
	float sinT = sin(rad);
	for (size_t j = 0; j < s.size(); j++) {
		for (size_t i = 0; i < s[j].p.size(); i++)
			s[j].p[i].rotXopt(cosT, sinT);
		s[j].c.rotXopt(cosT, sinT);
	}
}
void Object3D::rotY(float rad) {
	float cosT = cos(rad);
	float sinT = sin(rad);
	for (size_t j = 0; j < s.size(); j++) {
		for (size_t i = 0; i < s[j].p.size(); i++)
			s[j].p[i].rotYopt(cosT, sinT);
		s[j].c.rotYopt(cosT, sinT);
	}
}
void Object3D::rotZ(float rad) {
	float cosT = cos(rad);
	float sinT = sin(rad);
	for (size_t j = 0; j < s.size(); j++) {
		for (size_t i = 0; i < s[j].p.size(); i++)
			s[j].p[i].rotZopt(cosT, sinT);
		s[j].c.rotZopt(cosT, sinT);
	}
}
// rotation absolute (from (0,0,0)), degrees (setters)
void Object3D::rotDeg(Point & axisN, float deg) {
	rot(axisN, deg * PI / 180.0f);
}
void Object3D::rotXYZdeg(float degX, float degY, float degZ, bool inverted) {
	rotXYZ(degX * PI / 180.0f, degY * PI / 180.0f, degZ * PI / 180.0f, inverted);
}
void Object3D::rotXdeg(float deg) {
	rotX(deg * PI / 180.0f);
}
void Object3D::rotYdeg(float deg) {
	rotY(deg * PI / 180.0f);
}
void Object3D::rotZdeg(float deg) {
	rotZ(deg * PI / 180.0f);
}
// rotation relative (from pr), radians (setters)
void Object3D::rotFromP(Point & axisN, float rad, Point & pr) {
	float r11, r12, r13, r21, r22, r23, r31, r32, r33;
	setRotationMatrix(r11, r12, r13, r21, r22, r23, r31, r32, r33, axisN.x, axisN.y, axisN.z, rad);
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
void Object3D::rotXYZfromP(float radX, float radY, float radZ, Point & pr, bool inverted) {
	float cosX = cos(radX); float sinX = sin(radX);
	float cosY = cos(radY); float sinY = sin(radY);
	float cosZ = cos(radZ); float sinZ = sin(radZ);
	if (inverted) {
		for (size_t j = 0; j < s.size(); j++) {
			for (size_t i = 0; i < s[j].p.size(); i++) {
				s[j].p[i].x -= pr.x; s[j].p[i].y -= pr.y; s[j].p[i].z -= pr.z;
				s[j].p[i].rotXYZoptInverted(cosX, sinX, cosY, sinY, cosZ, sinZ);
				s[j].p[i].x += pr.x; s[j].p[i].y += pr.y; s[j].p[i].z += pr.z;
			}
			s[j].c.x -= pr.x; s[j].c.y -= pr.y; s[j].c.z -= pr.z;
			s[j].c.rotXYZoptInverted(cosX, sinX, cosY, sinY, cosZ, sinZ);
			s[j].c.x += pr.x; s[j].c.y += pr.y; s[j].c.z += pr.z;
		}
	} else {
		for (size_t j = 0; j < s.size(); j++) {
			for (size_t i = 0; i < s[j].p.size(); i++) {
				s[j].p[i].x -= pr.x; s[j].p[i].y -= pr.y; s[j].p[i].z -= pr.z;
				s[j].p[i].rotXYZopt(cosX, sinX, cosY, sinY, cosZ, sinZ);
				s[j].p[i].x += pr.x; s[j].p[i].y += pr.y; s[j].p[i].z += pr.z;
			}
			s[j].c.x -= pr.x; s[j].c.y -= pr.y; s[j].c.z -= pr.z;
			s[j].c.rotXYZopt(cosX, sinX, cosY, sinY, cosZ, sinZ);
			s[j].c.x += pr.x; s[j].c.y += pr.y; s[j].c.z += pr.z;
		}
	}
}
void Object3D::rotXfromP(float rad, Point & pr) {
	float cosT = cos(rad);
	float sinT = sin(rad);
	for (size_t j = 0; j < s.size(); j++) {
		for (size_t i = 0; i < s[j].p.size(); i++) {
			s[j].p[i].x -= pr.x; s[j].p[i].y -= pr.y; s[j].p[i].z -= pr.z;
			s[j].p[i].rotXopt(cosT, sinT);
			s[j].p[i].x += pr.x; s[j].p[i].y += pr.y; s[j].p[i].z += pr.z;
		}
		s[j].c.x -= pr.x; s[j].c.y -= pr.y; s[j].c.z -= pr.z;
		s[j].c.rotXopt(cosT, sinT);
		s[j].c.x += pr.x; s[j].c.y += pr.y; s[j].c.z += pr.z;
	}
}
void Object3D::rotYfromP(float rad, Point & pr) {
	float cosT = cos(rad);
	float sinT = sin(rad);
	for (size_t j = 0; j < s.size(); j++) {
		for (size_t i = 0; i < s[j].p.size(); i++) {
			s[j].p[i].x -= pr.x; s[j].p[i].y -= pr.y; s[j].p[i].z -= pr.z;
			s[j].p[i].rotYopt(cosT, sinT);
			s[j].p[i].x += pr.x; s[j].p[i].y += pr.y; s[j].p[i].z += pr.z;
		}
		s[j].c.x -= pr.x; s[j].c.y -= pr.y; s[j].c.z -= pr.z;
		s[j].c.rotYopt(cosT, sinT);
		s[j].c.x += pr.x; s[j].c.y += pr.y; s[j].c.z += pr.z;
	}
}
void Object3D::rotZfromP(float rad, Point & pr) {
	float cosT = cos(rad);
	float sinT = sin(rad);
	for (size_t j = 0; j < s.size(); j++) {
		for (size_t i = 0; i < s[j].p.size(); i++) {
			s[j].p[i].x -= pr.x; s[j].p[i].y -= pr.y; s[j].p[i].z -= pr.z;
			s[j].p[i].rotZopt(cosT, sinT);
			s[j].p[i].x += pr.x; s[j].p[i].y += pr.y; s[j].p[i].z += pr.z;
		}
		s[j].c.x -= pr.x; s[j].c.y -= pr.y; s[j].c.z -= pr.z;
		s[j].c.rotZopt(cosT, sinT);
		s[j].c.x += pr.x; s[j].c.y += pr.y; s[j].c.z += pr.z;
	}
}
// rotation relative (from pr), degrees (setters)
void Object3D::rotDegFromP(Point & axisN, float deg, Point & pr) {
	rotFromP(axisN, deg * PI / 180.0f, pr);
}
void Object3D::rotXYZdegFromP(float degX, float degY, float degZ, Point & pr, bool inverted) {
	rotXYZfromP(degX * PI / 180.0f, degY * PI / 180.0f, degZ * PI / 180.0f, pr, inverted);
}
void Object3D::rotXdegFromP(float deg, Point & pr) {
	rotXfromP(deg * PI / 180.0f, pr);
}
void Object3D::rotYdegFromP(float deg, Point & pr) {
	rotYfromP(deg * PI / 180.0f, pr);
}
void Object3D::rotZdegFromP(float deg, Point & pr) {
	rotZfromP(deg * PI / 180.0f, pr);
}
// rotation relative (from c), radians (setters)
void Object3D::rotFromC(Point & axisN, float rad) {
	float r11, r12, r13, r21, r22, r23, r31, r32, r33;
	setRotationMatrix(r11, r12, r13, r21, r22, r23, r31, r32, r33, axisN.x, axisN.y, axisN.z, rad);
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
void Object3D::rotXYZfromC(float radX, float radY, float radZ, bool inverted) {
	float cosX = cos(radX); float sinX = sin(radX);
	float cosY = cos(radY); float sinY = sin(radY);
	float cosZ = cos(radZ); float sinZ = sin(radZ);
	if (inverted) {
		for (size_t j = 0; j < s.size(); j++) {
			for (size_t i = 0; i < s[j].p.size(); i++) {
				s[j].p[i].x -= s[0].c.x; s[j].p[i].y -= s[0].c.y; s[j].p[i].z -= s[0].c.z;
				s[j].p[i].rotXYZoptInverted(cosX, sinX, cosY, sinY, cosZ, sinZ);
				s[j].p[i].x += s[0].c.x; s[j].p[i].y += s[0].c.y; s[j].p[i].z += s[0].c.z;
			}
			if (j >= 1) {
				s[j].c.x -= s[0].c.x; s[j].c.y -= s[0].c.y; s[j].c.z -= s[0].c.z;
				s[j].c.rotXYZoptInverted(cosX, sinX, cosY, sinY, cosZ, sinZ);
				s[j].c.x += s[0].c.x; s[j].c.y += s[0].c.y; s[j].c.z += s[0].c.z;
			}
		}
	} else {
		for (size_t j = 0; j < s.size(); j++) {
			for (size_t i = 0; i < s[j].p.size(); i++) {
				s[j].p[i].x -= s[0].c.x; s[j].p[i].y -= s[0].c.y; s[j].p[i].z -= s[0].c.z;
				s[j].p[i].rotXYZopt(cosX, sinX, cosY, sinY, cosZ, sinZ);
				s[j].p[i].x += s[0].c.x; s[j].p[i].y += s[0].c.y; s[j].p[i].z += s[0].c.z;
			}
			if (j >= 1) {
				s[j].c.x -= s[0].c.x; s[j].c.y -= s[0].c.y; s[j].c.z -= s[0].c.z;
				s[j].c.rotXYZopt(cosX, sinX, cosY, sinY, cosZ, sinZ);
				s[j].c.x += s[0].c.x; s[j].c.y += s[0].c.y; s[j].c.z += s[0].c.z;
			}
		}
	}
}
void Object3D::rotXfromC(float rad) {
	float cosT = cos(rad);
	float sinT = sin(rad);
	for (size_t j = 0; j < s.size(); j++) {
		for (size_t i = 0; i < s[j].p.size(); i++) {
			s[j].p[i].x -= s[0].c.x; s[j].p[i].y -= s[0].c.y; s[j].p[i].z -= s[0].c.z;
			s[j].p[i].rotXopt(cosT, sinT);
			s[j].p[i].x += s[0].c.x; s[j].p[i].y += s[0].c.y; s[j].p[i].z += s[0].c.z;
		}
		if (j >= 1) {
			s[j].c.x -= s[0].c.x; s[j].c.y -= s[0].c.y; s[j].c.z -= s[0].c.z;
			s[j].c.rotXopt(cosT, sinT);
			s[j].c.x += s[0].c.x; s[j].c.y += s[0].c.y; s[j].c.z += s[0].c.z;
		}
	}
}
void Object3D::rotYfromC(float rad) {
	float cosT = cos(rad);
	float sinT = sin(rad);
	for (size_t j = 0; j < s.size(); j++) {
		for (size_t i = 0; i < s[j].p.size(); i++) {
			s[j].p[i].x -= s[0].c.x; s[j].p[i].y -= s[0].c.y; s[j].p[i].z -= s[0].c.z;
			s[j].p[i].rotYopt(cosT, sinT);
			s[j].p[i].x += s[0].c.x; s[j].p[i].y += s[0].c.y; s[j].p[i].z += s[0].c.z;
		}
		if (j >= 1) {
			s[j].c.x -= s[0].c.x; s[j].c.y -= s[0].c.y; s[j].c.z -= s[0].c.z;
			s[j].c.rotYopt(cosT, sinT);
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
			s[j].p[i].rotZopt(cosT, sinT);
			s[j].p[i].x += s[0].c.x; s[j].p[i].y += s[0].c.y; s[j].p[i].z += s[0].c.z;
		}
		if (j >= 1) {
			s[j].c.x -= s[0].c.x; s[j].c.y -= s[0].c.y; s[j].c.z -= s[0].c.z;
			s[j].c.rotZopt(cosT, sinT);
			s[j].c.x += s[0].c.x; s[j].c.y += s[0].c.y; s[j].c.z += s[0].c.z;
		}
	}
}
// rotation relative (from c), degrees (setters)
void Object3D::rotDegFromC(Point & axisN, float deg) {
	rotFromC(axisN, deg * PI / 180.0f);
}
void Object3D::rotXYZdegFromC(float degX, float degY, float degZ, bool inverted) {
	rotXYZfromC(degX * PI / 180.0f, degY * PI / 180.0f, degZ * PI / 180.0f, inverted);
}
void Object3D::rotXdegFromC(float deg) {
	rotXfromC(deg * PI / 180.0f);
}
void Object3D::rotYdegFromC(float deg) {
	rotYfromC(deg * PI / 180.0f);
}
void Object3D::rotZdegFromC(float deg) {
	rotzFromC(deg * PI / 180.0f);
}

// clear, add Shape to Object3D
void Object3D::clear(bool clearAll) {
	s.clear();
	if (clearAll) {
		ot = UNKOWN_OBT;
		ps = UNKNOWN_PIS;
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
Scene::Scene(SceneType sceneType_) {
	set(sceneType_);
}
// Constructor Copy
Scene::Scene(Scene & scene) {
	set(scene);
}


// ----- SETTERS --------------------------------- // Note that Constructors just call their corresponding Setter
	
// Setter Default
void Scene::set(SceneType sceneType_) {
	o.clear();
	o.resize(SCENE_SIZE);
	sceneType = sceneType_;
}
// Setter Copy
void Scene::set(Scene & scene) {
	o = scene.o;
	sceneType = scene.sceneType;
}
	

// ----- FUNCITONS -------------------------------

// clear (auto resizes to SCENE_SIZE), add Object3D to Scene
void Scene::clear() {
	o.clear();
	o.resize(SCENE_SIZE);
	sceneType = UNKNOWN_SCT;
}
void Scene::add(Object3D & o0) {
	if (o0.ot == UNKOWN_OBT) {
		if (o.size() < SCENE_SIZE + 1) {
			o.resize(SCENE_SIZE + 1);
			o[SCENE_SIZE] = o0;
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

// Setter Scene Direct Vision
void Scene::setScene_DirectVision(PixStoring ps, bool pSim_) {

	// CAMERA (0)
	Point camPosC(0.0f, 0.763f, 0.0f);	// pos of the center of the camera
	float camDegPhi = 180.0f;			// phi degrees of rotation of the camera.
	float camDegTheta = 0.0f;			// theta degrees of rotation of the camera.
	float camDegRoll = 0.0f;			// roll degrees of rotation of the camera.
	Point camS(0.074f, 0.075f, 0.03f);	// size of the main box of the camera
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
	o[CAMERA].setCamera(camPosC, camDegPhi, camDegTheta, camDegRoll, camS, camC_relToP0, camAlbedoVV, camRVV, camGVV, camBVV, camAVV);

	// LASER (1)
	Point lasPosCrelToCam(-0.140f, 0.0f, -0.05f);
	Point lasPosC = camPosC + lasPosCrelToCam;
	float lasDegPhi = 180.0f;
	float lasDegTheta = 0.0f;
	float lasDegRoll = 0.0f;
	Point lasS(0.102f, 0.064f, 0.097f);
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
	o[LASER].setLaser(lasPosC, lasDegPhi, lasDegTheta, lasDegRoll, lasS, lasC_relToP0, lasAlbedoVV, lasRVV, lasGVV, lasBVV, lasAVV);

	// WALL (2)
	/*
	Point walPosC(0.375f,0.95f, -5.0f);
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
	o[WALL].setBox(walPosC, walAxisN, walDeg, walS, walC_relToP0, walAlbedoV, walRV, walGV, walBV, walAV);
	o[WALL].ot = WALL;
	*/

	// FLOOR (4)
	Point floPosC(-1.625f, 0.0f, 0.7f);
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
	o[FLOOR].setBox(floPosC, floAxisN, floDeg, floS, floC_relToP0, floAlbedoV, floRV, floGV, floBV, floAV);
	o[FLOOR].ot = FLOOR;
	
	// CAMERA_FOV (7)
	float cafR = 0.0f, cafG = 0.0f, cafB = 1.0f, cafA = 1.0f, cafDist = 5.0f;
	o[CAMERA_FOV].setCameraFoV(*this, cafR, cafG, cafB, cafA, ps, cafDist);

	// PIXEL_PATCHES (10)
	o[PIXEL_PATCHES].setPixelPatches(*this, 2.0f, ps, pSim_);
	/*
	char dir_name[1024] = "C:\\Users\\Natalia\\Documents\\Visual Studio 2013\\Projects\\DiffuseMirrors2\\CalibrationMatrix\\test_03";
	char file_name[1024] = "PMD";
	Info info(dir_name, file_name);
	RawData rawData(info);
	Frame frame00(rawData, 0, 0, 0, 0, ps, pSim_);
	Frame frame90(rawData, 0, 0, 0, 1, ps, pSim_);
	o[PIXEL_PATCHES].setPixelPatches(*this, frame00, frame90, ps, pSim_);
	*/
}
// Setter Scene Occlusion
void Scene::setScene_Occlusion(std::vector<int> & rowsPerFaceV, std::vector<int> & colsPerFaceV, PixStoring ps, bool pSim_) {
	
	// WALL (2)		// before CAMERA and LASER, because CAMERA camDegPhi, camDegTheta and LASER lasAxisN, lasDeg are WALL-dependent
	Point walPosC(-0.75f, 0.0f, -1.50f);
	Point walS(3.25f, 2.5f, 0.1f);
	Point walAxisN(0.0f, 1.0f, 0.0f);
	float walDeg = 0.0f;
	Point walC_relToP0(0.0f, 0.0f, 0.0f);
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
	o[WALL].setBox(walPosC, walAxisN, walDeg, walS, walC_relToP0, walAlbedoV, walRV, walGV, walBV, walAV);
	o[WALL].ot = WALL;

	// CAMERA (0)
	Point camPosC(0.0f, 0.763f, 0.0f);	// pos of the center of the camera
	Point walN = o[WALL].normalQUAD();	// this must be (0,0,1) (unused)
	Point walCamFloor(camPosC.x, 0.0f, walPosC.z);				// projection of the base of the camera into the wall, used for both CAMERA and LASER
	Point camPosPCrelTowalCamFloor(0.000f, camPosC.y, 0.0f);	// manual measurement (0.000f, camPosC.y, 0.0f), relative position of the projection of the normal of the camera into the wall from the projection of the base of the camera into the wall
	Point camPosPC = walCamFloor + camPosPCrelTowalCamFloor;	// position of the projection of the normal of the camera into the wall
	Point camV = camPosPC - camPosC;
	float camDegPhi = degP(camV);		// phi degrees of rotation of the camera. (usually = 180.0f)
	float camDegTheta = degT(camV);		// theta degrees of rotation of the camera. (usually = 0.0f)
	float camDegRoll = 0.0f;			// roll degrees of rotation of the camera. (usually = 0.0f)
	Point camS(0.074f, 0.075f, 0.03f);	// size of the main box of the camera
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
		}
	}
	o[CAMERA].setCamera(camPosC, camDegPhi, camDegTheta, camDegRoll, camS, camC_relToP0, camAlbedoVV, camRVV, camGVV, camBVV, camAVV);
	
	// LASER (1)	// after WALL, because LASER lasAxisN, lasDeg are WALL-dependent
	Point lasPosCrelToCam(0.405f, 0.0f, -0.05f);				// manual measurement
	//Point lasPosCrelToCam(0.2f, 0.0f, 0.0f);					// for testing...
	Point lasPosC = camPosC + lasPosCrelToCam;
	// measurements WALL-dependent for lasAxisN, lasDeg
	Point lasPosWLrelTowalCamFloor(0.763f, 0.797f, 0.0f);		// manual measurement (0.763f, 0.797f, 0.0f), relative position of WL from the projection of the base of the camera into the wall
	//Point lasPosWLrelTowalCamFloor(0.0f, camPosC.y, 0.0f);		// for testing...
	Point lasPosWL = walCamFloor + lasPosWLrelTowalCamFloor;	// WL position of the projection of the normal of the laser into the wall
	Point lasV = lasPosWL - lasPosC;
	float lasDegPhi = degP(lasV);
	float lasDegTheta = degT(lasV);
	float lasDegRoll = 0.0f;
	Point lasS(0.102f, 0.064f, 0.097f);
	Point lasC_relToP0(lasS.x / 2.0f, lasS.y / 2.0f, 0.0f);
	// print for testing
	/*
	walN.print("\nwalN = ", "\n");
	walCamFloor.print("walCamFloor = ", "\n");
	lasPosLPrelTowalCamFloor.print("lasPosLPrelTowalCamFloor = ", "\n");
	lasPosLP.print("lasPosLP = ", "\n");
	lasV.print("lasV = ", "\n");
	lasAxisN.print("lasAxisN = ", "\n");
	std::cout << "lasDeg = " << lasDeg << "\n";
	lasS.print("lasS = ", "\n");
	lasC_relToP0.print("lasC_relToP0 = ", "\n");
	*/
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
		}
	}
	o[LASER].setLaser(lasPosC, lasDegPhi, lasDegTheta, lasDegRoll, lasS, lasC_relToP0, lasAlbedoVV, lasRVV, lasGVV, lasBVV, lasAVV);

	// OCCLUDER (3)
	Point occPosC(0.6f, 0.0f, 0.7f);
	Point occS(1.0f, walS.y, 0.05f);
	Point occAxisN(0.0f, 1.0f, 0.0f);
	float occDeg = 90.0f;
	Point occC_relToP0(0.0f, 0.0f, 0.0f);
	//std::vector<float> occAlbedoV(0), occRVV(0), occGV(0), occBV(0), occAV(0);
	int occ_s_per_box = 6;
	std::vector<float> occAlbedoV(occ_s_per_box);
	std::vector<float> occRV(occ_s_per_box);
	std::vector<float> occGV(occ_s_per_box);
	std::vector<float> occBV(occ_s_per_box);
	std::vector<float> occAV(occ_s_per_box);
	occRV[FRONT] = 0.5f;  occRV[RIGHT] = 0.8f;  occRV[BACK] = 0.6f;  occRV[LEFT] = 0.3f;  occRV[BOTTOM] = 0.25f;  occRV[TOP] = 0.65f;
	occGV[FRONT] = 0.5f;  occGV[RIGHT] = 0.8f;  occGV[BACK] = 0.6f;  occGV[LEFT] = 0.3f;  occGV[BOTTOM] = 0.25f;  occGV[TOP] = 0.65f;
	occBV[FRONT] = 0.5f;  occBV[RIGHT] = 0.8f;  occBV[BACK] = 0.6f;  occBV[LEFT] = 0.3f;  occBV[BOTTOM] = 0.25f;  occBV[TOP] = 0.65f;
	for (int s = FRONT; s <= TOP; s++) {
		occAlbedoV[s] = 1.0f;
		occAV[s] = 1.0f;
	}
	o[OCCLUDER].setBox(occPosC, occAxisN, occDeg, occS, occC_relToP0, occAlbedoV, occRV, occGV, occBV, occAV);
	o[OCCLUDER].ot = OCCLUDER;

	// FLOOR (4)
	Point floPosC(walPosC.x, 0.0f, occPosC.z);
	Point floS(walS.x, floPosC.z - walPosC.z + walS.z, walS.z);
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
	o[FLOOR].setBox(floPosC, floAxisN, floDeg, floS, floC_relToP0, floAlbedoV, floRV, floGV, floBV, floAV);
	o[FLOOR].ot = FLOOR;

	// WALL_PATCHES (6)
	o[WALL_PATCHES].setWallPatches(*this, ps, pSim_);

	// CAMERA_FOV (7)
	float cafR = 0.0f, cafG = 0.0f, cafB = 1.0f, cafA = 1.0f;
	o[CAMERA_FOV].setCameraFoV(*this, cafR, cafG, cafB, cafA, ps);

	// LASER_RAY (8)
	float larR = 1.0f, larG = 0.2f, larB = 0.2f, larA = 1.0f;
	o[LASER_RAY].setLaserRay(*this, larR, larG, larB, larA, ps);

	// VOLUME_PATCHES (9)
	Point vopPosC(3.0f, 1.0f, 0.0f);	// doesn't matter (it's centered to (0,0,0) in updateVolumePatches(...))
	Point vopAxisN(0.0f, 1.0f, 0.0f);	// doesn't matter (new assignation in updateVolumePatches(...))
	float vopDeg = 0.0f;				// must be 0.0f (this are the reference degrees used along all updateVolumePatches(...))
	Point vopS(0.584f, 0.505f, 0.399f);	// manual measurement
	// albedo, R, G, B, A
	int const vop_faces = rowsPerFaceV.size();
	std::vector<std::vector<float>> vopAlbedoVV(vop_faces);
	std::vector<std::vector<float>> vopRVV(vop_faces);
	std::vector<std::vector<float>> vopGVV(vop_faces);
	std::vector<std::vector<float>> vopBVV(vop_faces);
	std::vector<std::vector<float>> vopAVV(vop_faces);
	int numShapesInFace;
	for (int f = FRONT; f < vop_faces; ++f) {
		numShapesInFace = rowsPerFaceV[f] * colsPerFaceV[f];
		vopAlbedoVV[f].resize(numShapesInFace);
		vopRVV[f].resize(numShapesInFace);
		vopGVV[f].resize(numShapesInFace);
		vopBVV[f].resize(numShapesInFace);
		vopAVV[f].resize(numShapesInFace);
		for (int i = 0; i < numShapesInFace; ++i) {
			vopAlbedoVV[f][i] = 1.0f;
			vopRVV[f][i] = 1.0f;
			vopGVV[f][i] = 1.0f;
			vopBVV[f][i] = 1.0f;
			vopAVV[f][i] = 1.0f;
	}	}
	o[VOLUME_PATCHES].setVolumePatchesBox(vopPosC, vopAxisN, vopDeg, vopS, rowsPerFaceV, colsPerFaceV, vopAlbedoVV, vopRVV, vopGVV, vopBVV, vopAVV);
}
// Setter Scene Calibration Matrix
void Scene::setScene_CalibrationMatrix(float laser_to_cam_offset_x, float laser_to_cam_offset_y, float laser_to_cam_offset_z, float dist_wall_cam,
	PixStoring ps, bool pSim_) {
	
	// CAMERA (0)
	Point camPosC(0.0f, 0.763f, 0.0f);	// pos of the center of the camera
	float camDegPhi = 180.0f;			// phi degrees of rotation of the camera.
	float camDegTheta = 0.0f;			// theta degrees of rotation of the camera.
	float camDegRoll = 0.0f;			// roll degrees of rotation of the camera.
	Point camS(0.074f, 0.075f, 0.03f);	// size of the main box of the camera
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
	o[CAMERA].setCamera(camPosC, camDegPhi, camDegTheta, camDegRoll, camS, camC_relToP0, camAlbedoVV, camRVV, camGVV, camBVV, camAVV);

	// LASER (1)
	Point lasPosCrelToCam(laser_to_cam_offset_x, laser_to_cam_offset_y, laser_to_cam_offset_z);
	Point lasPosC = camPosC + lasPosCrelToCam;
	float lasDegPhi = 180.0f;
	float lasDegTheta = 0.0f;
	float lasDegRoll = 0.0f;
	Point lasS(0.102f, 0.064f, 0.097f);
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
	o[LASER].setLaser(lasPosC, lasDegPhi, lasDegTheta, lasDegRoll, lasS, lasC_relToP0, lasAlbedoVV, lasRVV, lasGVV, lasBVV, lasAVV);

	// WALL (2)
	Point walPosC(0.375f,0.95f, camPosC.z - dist_wall_cam);
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
	o[WALL].setBox(walPosC, walAxisN, walDeg, walS, walC_relToP0, walAlbedoV, walRV, walGV, walBV, walAV);
	o[WALL].ot = WALL;

	// FLOOR (4)
	Point floPosC(-1.625f, 0.0f, 0.7f);
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
	o[FLOOR].setBox(floPosC, floAxisN, floDeg, floS, floC_relToP0, floAlbedoV, floRV, floGV, floBV, floAV);
	o[FLOOR].ot = FLOOR;
	
	// WALL_PATCHES (6)
	o[WALL_PATCHES].setWallPatches(*this, ps, pSim_);
}


// ----------------------------------------------------------------------------------------------------------------------------------------
// ----- OTHER NON-MEMBER FUNCTIONS -------------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------------------

// mean
/*
float mean(std::vector<float> & vf) {		// already defined in data.h/.cpp
	float meanf = vf[0];
	for (int i = 1; i < vf.size(); i++)
		meanf += vf[i];
	return meanf /= vf.size();
}
*/
Point mean(std::vector<Point> & vp) {
	Point mean (0.0f,0.0f,0.0f);
	for (size_t i = 0; i < vp.size(); i++)
		mean += vp[i];
	return mean / vp.size();
}
// Rotation Matrix
void setRotationMatrix(float & r11, float & r12, float & r13,
	float & r21, float & r22, float & r23,
	float & r31, float & r32, float & r33,
	float const & wx, float const & wy, float const & wz, float const & rad) {
	// constants
	float cosT = cos(rad);
	float l_cosT = 1.0f - cosT;
	float sinT = sin(rad);
	// Rodrigues' Rotation Matrix Formula: http://mathworld.wolfram.com/RodriguesRotationFormula.html
	r11 = cosT + wx * wx * l_cosT;			r12 = wx * wy * l_cosT - wz * sinT;		r13 = wy * sinT + wx * wz * l_cosT;
	r21 = wz * sinT + wx * wy * l_cosT;		r22 = cosT + wy * wy * l_cosT;			r23 = -wx * sinT + wy * wz * l_cosT;
	r31 = -wy * sinT + wx * wz * l_cosT;	r32 = wx * sinT + wy * wz * l_cosT;		r33 = cosT + wz * wz * l_cosT;
}
// dist
float dist(Point & v0, Point & v1) {
	Point dif = v1 - v0;
	return dif.mod();
}
float distPow2(Point & v0, Point & v1) {
	Point dif = v1 - v0;
	return dif.modPow2();
}
// dot, cross
Point cross(Point & v0, Point & v1) {
	return v0.cross(v1);
}
float dot(Point & v0, Point & v1) {
	return v0.dot(v1);
}
// cos / rad / deg (rad, deg are dependent of the corresponding cos functions)
float cosVV(Point & v0, Point & v1) {
	return (v0.normal()).dot(v1.normal());
}
float cosVN(Point & v, Point & n) {
	return (v.normal()).dot(n);
}
float cosNN(Point & n0, Point & n1) {
	return n0.dot(n1);
}
float radVV(Point & v0, Point & v1) {
	return acos(cosVV(v0,v1));
}
float radVN(Point & v, Point & n) {
	return acos(cosVN(v,n));
}
float radNN(Point & n0, Point & n1) {
	return acos(cosNN(n0,n1));
}
float degV(Point & v0, Point & v1) {
	return radVV(v0, v1) * 180.0f / PI;
}
float degVN(Point & v, Point & n) {
	return radVN(v, n) * 180.0f / PI;
}
float degNN(Point & n0, Point & n1) {
	return radNN(n0, n1) * 180.0f / PI;
}
Point axisNVV(Point & v0, Point & v1) {
	Point axisN = v0.cross(v1);
	float modPow2_ = axisN.modPow2();
	if (modPow2_ > 1E-12)	// v0 and v1 are NOT parallel
		return (axisN / sqrt(modPow2_));
	else					// v0 and v1 are parallel
		return Point(0.0f, 1.0f, 0.0f);	// (0,1,0) is the arbitrary axisN vector when v0 and v1 are parallel
}
// cos / rad / deg, with the XYZ/PhiTheta rotation system. (rad, deg are dependent of the corresponding cos functions)
// Note that a rotation in Phi and Theta has the same effect as a rotation in rotX(-Theta) and rotY(Phi) IN THIS ORDER.
// Phi (P) is rotation around axis OY, CW starting at OZ.
// Theta (T) is roation around axis OX', which is the axis OX rotated Phi previously. CW starting at OZ'
float cosP(Point & v) {
	float aux = v.x * v.x + v.z * v.z;
	if (aux == 0.0f)
		return 1.0f;
	return v.z / sqrt(aux);
}
float cosPN(Point & vN) {	// does the same as cosP
	return cosP(vN);
}
float cosT(Point & v) {
	float aux = v.x * v.x + v.z * v.z;
	float aux2 = aux + v.y * v.y;
	if (aux2 == 0.0f)
		return 1.0f;
	return sqrt(aux / aux2);
}
float cosTN(Point & vN) {
	return sqrt(vN.x * vN.x + vN.z * vN.z);
}
float radP(Point & v) {	
	if (v.x < 0.0f)
		return -acos(cosP(v));
	return acos(cosP(v));
}
float radPN(Point & vN) {
	if (vN.x < 0.0f)
		return -acos(cosPN(vN));
	return acos(cosPN(vN));
}
float radT(Point & v) {
	if (v.y < 0.0f)
		return -acos(cosT(v));
	return acos(cosT(v));
}
float radTN(Point & vN) {
	if (vN.y < 0.0f)
		return -acos(cosT(vN));
	return acos(cosTN(vN));
}
float degP(Point & v) {
	return radP(v) * 180.0f / PI;
}
float degPN(Point & vN) {
	return radPN(vN) * 180.0f / PI;
}
float degT(Point & v) {
	return radT(v) * 180.0f / PI;
}
float degTN(Point & vN) {
	return radTN(vN) * 180.0f / PI;
}
// set depth map
void setDepthMap(std::vector<float> & depthMap, Frame & frame00, Frame & frame90) {

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

	float pathDist = 0.0f;
	//std::cout << "\nFrame_00_cap.rows, cols = " << Frame_00_cap.rows << ", " << Frame_00_cap.cols;
	for (int r = 0; r < frame00.rows; r++) {
		for (int c = 0; c < frame00.cols; c++) {
			pathDist = (atan2(-frame90.at(r, c), frame00.at(r, c)) + PI) * C_LIGHT_AIR / (2 * PI * frame00.freq * 1000000.0f) + delay_m_100MHz;
			depthMap[rc2idx(r, c, frame00.ps, frame00.pSim)] = pathDist / 2.0f;	// this is an approximation that supposes camera and laser close enough
	}	}
}
// simulation.cpp uses
float geometryTerm(Point & p0, Point & n0, Point & p1, Point & n1) {
	Point nX = (p1 - p0);
	float distPow2_ = nX.modPow2();
	nX.normalize();
	return -cosNN(nX, n0) * cosNN(nX, n1) / distPow2_;
}
// distPath
float distPath2(Point & p0, Point & p1) {
	return dist(p0, p1);
}
float distPath3(Point & p0, Point & p1, Point & p2) {
	return dist(p0, p1) + dist(p1, p2);
}
float distPath4(Point & p0, Point & p1, Point & p2, Point & p3) {
	return dist(p0, p1) + dist(p1, p2) + dist(p2, p3);
}
float distPath5(Point & p0, Point & p1, Point & p2, Point & p3, Point & p4) {
	return dist(p0, p1) + dist(p1, p2) + dist(p2, p3) + dist(p3, p4);
}
// ad-hoc function to deal with p and pAll (see void Object3D::updateVolumePatches_Occlusion(...))
float getPiAll(int iAll, float* p, float* pAll, bool* pUse, int* idxOfpInpAll) {
	if (pUse[iAll])						// if the paramter iAll is stored in p (variable for levmar)
		return p[idxOfpInpAll[iAll]];		// return it from p 
	else								// if the paramter iAll is NOT stored in p (constant for levmar)
		return pAll[iAll];					// return it from pAll
}
float getPiAll(int iAll, float* p, struct OCCLUSION_ADATA* ad) {
	return getPiAll(iAll, p, ad->pAll, ad->pUse, ad->idxOfpInpAll);
}