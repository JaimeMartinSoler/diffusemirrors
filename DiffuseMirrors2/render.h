
#ifndef __RENDER_H
#define __RENDER_H

#include <windows.h>  // for MS Windows
#include <GL/glut.h>  // GLUT, include glu.h and gl.h
#include "shapes.h"


// Initialize OpenGL Graphics 
void initGL();

// Resets position for all 3D-objects
void reset_position();

// Sets initial pose reference for all 3D-objects
void set_initial_pose();

// Sets the transformation depending on UI for all 3D-objects
void transform_UI();



// It renders all the object3D of OBJECT3D_SET
void render_OBJECT3D_SET();

// It renders all the PointMesh of the Object3D obj
void render_Object3D(Object3D* obj, int OBJECT3D_IDX);

// It renders all the shapes of the PointMesh pm
void render_PointMesh(PointMesh* pm, int OBJECT3D_IDX);

// It renders a rectangle from 4 points
void render_rectangle(Point* p0, Point* p1, Point* p2, Point* p3, float* gray_color, int OBJECT3D_IDX);

// It renders a triangle from 3 points
void render_triangle(Point* p0, Point* p1, Point* p2, float* gray_color, int OBJECT3D_IDX);

// It renders a line from 2 points
void render_line(Point* p0, Point* p1, Point* color);

// It renders a point from 1 point
void render_point(Point* p0);




// Handler for window-repaint event. Called back when the window first appears and
// whenever the window needs to be re-painted
void display();

// Handler for window re-size event. Called back when the window first appears and
// whenever the window is re-sized with its new width and height
void reshape(GLsizei width, GLsizei height);

// specialKeys() Callback Function
void specialKeys(int key, int x, int y);

// keyboardKeys() Callback Function
void keyboardKeys(unsigned char key, int x, int y);

// idle(): executed when render is free (not drawing)
void idle();

// calculates the frames per second
void calculateFPS();

// draws FPS
void drawFPS();

// draws a string at the specified coordinates
void printw (float x, float y, float z, char* format, ...);

// mouse(...): to deal with th mouse
void mouse(int button, int state, int x, int y);

// mouseMotion(...): to deal with th mouse
void mouseMotion(int x, int y);

// rendering MAIN function
int render(int argc, char** argv);
// there's a weird bug when calling directly to PMD_params_to_file from thread constructor. With this re-calling functtion the bug is avoided
int render_anti_bug_thread (int argc, char** argv);

#endif

