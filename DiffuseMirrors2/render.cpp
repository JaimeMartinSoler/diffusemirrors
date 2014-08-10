/*

graph_engine.cpp

From the building parameters it renders the building

*/

#include <windows.h>		// for MS Windows
//#include <GL/glut.h>		// GLUT, include glu.h and gl.h, not compatible with x64...
#include <GL/freeglut.h>	// FREEGLUT, needed to run x64
#include "render.h"
#include "shapes.h"
#include "global.h"
#include "scene.h"

// Global variables
// position, rotation, mouse
char* title = "Master Thesis";
float rotate_x = 0.0f;
float rotate_y = 0.0f;
float rotate_z = 0.0f;
float rotate_incr = 5.0f;
float rotate_x_mouse_diff = 0.0f;
float rotate_y_mouse_diff = 0.0f;
bool mouseDown = false;
float translate_x = 0.0f;
float translate_y = 0.0f;
float translate_z = 0.0f;
float translate_incr = 0.1f;
float scale_x = 1.0f;
float scale_y = 1.0f;
float scale_z = 1.0f;
float scale_incr = 1.05f;
// fps
int frameCount = 0;
float fps = 0.0f;
int currentTime = 0;
int previousTime = 0;

// Initialize OpenGL Graphics 
void initGL() {
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f); // Set background color to black and opaque
	glClearDepth(1.0f);                   // Set background depth to farthest
	glEnable(GL_DEPTH_TEST);   // Enable depth testing for z-culling
	glDepthFunc(GL_LEQUAL);    // Set the type of depth-test
	glShadeModel(GL_SMOOTH);   // Enable smooth shading
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);  // Nice perspective corrections
	// Anti-aliasing
	if (true) {
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glEnable(GL_LINE_SMOOTH);
		glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
	}
}


// Resets position for all 3D-objects
void reset_position() {
	rotate_x = 0.0f;
	rotate_y = 0.0f;
	rotate_z = 0.0f;
	translate_x = 0.0f;
	translate_y = 0.0f;
	translate_z = 0.0f;
	scale_x = 1.0f;
	scale_y = 1.0f;
	scale_z = 1.0f;
}


// Sets initial pose reference for all 3D-objects
void set_initial_pose() {
	glTranslatef(-2.0f, -1.8f, -6.0f);
	glRotatef(0, 1.0f, 0.0f, 0.0f);
	glRotatef(15, 0.0f, 1.0f, 0.0f);
	glRotatef(0, 0.0f, 0.0f, 1.0f);
}

// Sets the transformation depending on UI for all 3D-objects
void transform_UI() {
	glRotatef(rotate_x, 1.0f, 0.0f, 0.0f);
	glRotatef(rotate_y, 0.0f, 1.0f, 0.0f);
	glRotatef(rotate_z, 0.0f, 0.0f, 1.0f);
	glTranslatef(translate_x, translate_y, translate_z);
	glScalef(scale_x, scale_y, scale_z);
}

// It renders all the object3D of OBJECT3D_SET
void render_OBJECT3D_SET() {

	bool render_wall_patches = true;

	for (std::size_t i = 0; i < OBJECT3D_SET.size(); i++) {
		if ((i == WALL_PATCHES) && !render_wall_patches)
			continue;
		render_Object3D(OBJECT3D_SET[i], i);
	}
}

// It renders all the PointMesh of the Object3D obj
void render_Object3D(Object3D* obj, int OBJECT3D_IDX) {
	for (std::size_t i = 0; i < (*obj).size(); i++)
		render_PointMesh((*obj)[i], OBJECT3D_IDX);
	// it renders the center of the Object3D obj
	if ((*obj).size() > 0)
		render_point((*obj)[0]->c);
}

// It renders all the shapes of the PointMesh pm
void render_PointMesh(PointMesh* pm, int OBJECT3D_IDX) {
	float gray_color = 0.2f;	// better colors if setted to 0.2f
	if (OBJECT3D_IDX == WALL)
		gray_color = 0.7f;
	else if (OBJECT3D_IDX == PIXEL_PATCHES)
		gray_color = 0.7f;
	if ((*pm).shape == RECTANGLE) {
		for (std::size_t i = 0; i < pm->p.size(); i += 4) {
			render_rectangle(pm->p[i], pm->p[i + 1], pm->p[i + 2], pm->p[i + 3], &gray_color, OBJECT3D_IDX);
		}
	}
	if ((*pm).shape == TRIANGLE) {
		for (std::size_t i = 0; i < pm->p.size(); i += 3) {
			render_triangle(pm->p[i], pm->p[i + 1], pm->p[i + 2], &gray_color, OBJECT3D_IDX);
		}
	}
	if ((*pm).shape == LINE) {
		for (std::size_t i = 0; i < pm->p.size(); i += 2) {
			render_line(pm->p[i], pm->p[i + 1], new Point(0.0f, 0.0f, 1.0f));
		}
	}
	if ((*pm).shape == PT) {
		for (std::size_t i = 0; i < pm->p.size(); i++) {
			render_point(pm->p[i]);
		}
	}
	
}

// It renders a rectangle from 4 points
void render_rectangle(Point* p0, Point* p1, Point* p2, Point* p3, float* gray_color, int OBJECT3D_IDX) {
	glLoadIdentity();
	set_initial_pose();
	transform_UI();
	glBegin(GL_QUADS);
	// Define vertices in counter-clockwise (CCW) order with normal pointing out
	// Color
	(*gray_color) += 0.1f;
	if ((*gray_color) > 0.85f)
		(*gray_color) = 0.2f;
	//glColor3f((*gray_color), (*gray_color), (*gray_color));
	glColor4f((*gray_color), (*gray_color), (*gray_color), 1.0f);
	// Vertex
	glVertex3f((*p0)[0], (*p0)[1], (*p0)[2]);
	glVertex3f((*p1)[0], (*p1)[1], (*p1)[2]);
	glVertex3f((*p2)[0], (*p2)[1], (*p2)[2]);
	glVertex3f((*p3)[0], (*p3)[1], (*p3)[2]);
	glEnd();
	// it renders the sides of the rectangle
	if (OBJECT3D_IDX == PIXEL_PATCHES)
		return;
	Point color_black(0.0f, 0.0f, 0.0f);
	render_line(p0, p1, &color_black);
	render_line(p1, p2, &color_black);
	render_line(p2, p3, &color_black);
	render_line(p3, p0, &color_black);
}

// It renders a triangle from 3 points
void render_triangle(Point* p0, Point* p1, Point* p2, float* gray_color, int OBJECT3D_IDX) {
	glLoadIdentity();
	set_initial_pose();
	transform_UI();
	glBegin(GL_TRIANGLES);
	// Define vertices in counter-clockwise (CCW) order with normal pointing out
	// Color
	(*gray_color) += 0.1f;
	if ((*gray_color) > 0.85f)
		(*gray_color) = 0.2f;
	glColor3f((*gray_color), (*gray_color), (*gray_color));
	// Vertex
	glVertex3f((*p0)[0], (*p0)[1], (*p0)[2]);
	glVertex3f((*p1)[0], (*p1)[1], (*p1)[2]);
	glVertex3f((*p2)[0], (*p2)[1], (*p2)[2]);
	glEnd();
	// it renders the sides of the rectangle
	Point color_black(0.0f, 0.0f, 0.0f);
	render_line(p0, p1, &color_black);
	render_line(p1, p2, &color_black);
	render_line(p2, p0, &color_black);
}

// It renders a line from 2 points
void render_line(Point* p0, Point* p1, Point* color) {
	glLoadIdentity();
	set_initial_pose();
	transform_UI();
	glLineWidth(1.0);	// line width = 1.0
	glBegin(GL_LINES);
	// Color
	glColor3f((*color).x(), (*color).y(), (*color).z());
	// Vertex
	glVertex3f((*p0)[0], (*p0)[1], (*p0)[2]);
	glVertex3f((*p1)[0], (*p1)[1], (*p1)[2]);
	glEnd();  // End of drawing lines

}

// It renders a point from 1 point
void render_point(Point* p0) {
	glLoadIdentity();
	set_initial_pose();
	transform_UI();
	glPointSize(3.0);	// point size = 3.0
	glBegin(GL_POINTS);
	// Define vertices in counter-clockwise (CCW) order with normal pointing out
	// Color
	glColor3f(1.0f, 0.0f, 0.0f);	// Red
	// Vertex
	glVertex3f((*p0)[0], (*p0)[1], (*p0)[2]);
	glEnd();
}


// Handler for window-repaint event. Called back when the window first appears and
// whenever the window needs to be re-painted
void display() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear color and depth buffers
	glMatrixMode(GL_MODELVIEW);     // To operate on model-view matrix
	
	render_OBJECT3D_SET();
	drawFPS();

	glutSwapBuffers();  // Swap the front and back frame buffers (double buffering)
}

// Handler for window re-size event. Called back when the window first appears and
// whenever the window is re-sized with its new width and height
void reshape(GLsizei width, GLsizei height) {  // GLsizei for non-negative integer
	// Compute aspect ratio of the new window
	if (height == 0) height = 1;                // To prevent divide by 0
	GLfloat aspect = (GLfloat)width / (GLfloat)height;

	// Set the viewport to cover the new window
	glViewport(0, 0, width, height);

	// Set the aspect ratio of the clipping volume to match the viewport
	glMatrixMode(GL_PROJECTION);  // To operate on the Projection matrix
	glLoadIdentity();             // Reset
	// Enable perspective projection with fovy, aspect, zNear and zFar
	gluPerspective(45.0f, aspect, 0.1f, 100.0f);
}


// specialKeys() Callback Function
void specialKeys(int key, int x, int y) {
	switch (key) {
	case GLUT_KEY_RIGHT: translate_x -= translate_incr; break;
	case GLUT_KEY_LEFT:	 translate_x += translate_incr; break;
	case GLUT_KEY_UP:    translate_z += translate_incr; break;
	case GLUT_KEY_DOWN:  translate_z -= translate_incr; break;
	}
	//  Request display update
	glutPostRedisplay();
}

// keyboardKeys() Callback Function
void keyboardKeys(unsigned char key, int x, int y) {
	switch (key) {
	case 'a': rotate_x += rotate_incr; break;
	case 'z': rotate_x -= rotate_incr; break;
	case 'y': rotate_x -= rotate_incr; break;	// german keyboard
	case 's': rotate_y += rotate_incr; break;
	case 'x': rotate_y -= rotate_incr; break;
	//case 's': rot_from_c(OBJECT3D_SET[LASER], new Point(0.0f, rotate_incr, 0.0f), true); set_laser_ray(); break;
	//case 'x': rot_from_c(OBJECT3D_SET[LASER], new Point(0.0f, -rotate_incr, 0.0f), true); set_laser_ray(); break;
	case 'd': rotate_z += rotate_incr; break;
	case 'c': rotate_z -= rotate_incr; break;
	case 'f': scale_x *= scale_incr;
		scale_y *= scale_incr;
		scale_z *= scale_incr; break;
	case 'v': scale_x /= scale_incr;
		scale_y /= scale_incr;
		scale_z /= scale_incr; break;
	case 'g': scale_x *= scale_incr; break;
	case 'b': scale_x /= scale_incr; break;
	case 'h': scale_y *= scale_incr; break;
	case 'n': scale_y /= scale_incr; break;
	case 'j': scale_z *= scale_incr; break;
	case 'm': scale_z /= scale_incr; break;
	case 'o': translate_y -= translate_incr; break;
	case 'l': translate_y += translate_incr; break;
	case 'r': reset_position(); break;
	}
	//  Request display update
	glutPostRedisplay();
}

// idle(): executed when render is free (not drawing)
void idle() {
	// calculate FPS
    calculateFPS();
	// draw the current frame
	glutPostRedisplay();
}

// calculates the frames per second
void calculateFPS() {

    //  Increase frame count
    frameCount++;

    //  Get the number of milliseconds since glutInit called 
    //  (or first call to glutGet(GLUT_ELAPSED_TIME)).
    currentTime = glutGet(GLUT_ELAPSED_TIME);

    //  Calculate time passed
    int timeInterval = currentTime - previousTime;

    if (timeInterval > 1000) {
        //  calculate the number of frames per second
        fps = ((float)frameCount) / (((float)timeInterval) / 1000.0f);
        previousTime = currentTime;
        frameCount = 0;
    }
}

// draws FPS
void drawFPS() {

	glLoadIdentity ();
	
	printw (-0.9, -0.9, 0, "FPS: %4.2f", fps);
	printw (0.9, 0.9, 0, "FPS: %4.2f", fps);
}

// draws a string at the specified coordinates
void printw (float x, float y, float z, char* format, ...) {

	va_list args;	//  Variable argument list
	int len;		//	String length
	int i;			//  Iterator
	char * text;	//	Text

	//  Initialize a variable argument list
	va_start(args, format);

	//  Return the number of characters in the string referenced the list of arguments.
	//  _vscprintf doesn't count terminating '\0' (that's why +1)
	len = _vscprintf(format, args) + 1; 

	//  Allocate memory for a string of the specified size
	text = (char *)malloc(len * sizeof(char));

	//  Write formatted output using a pointer to the list of arguments
	vsprintf_s(text, len, format, args);

	//  End using variable argument list 
	va_end(args);

	//  Specify the raster position for pixel operations.
	glRasterPos3f (x, y, z);
	//glRasterPos2i( 20, 20 );

	//  Draw the characters one by one
	//std::cout << text << ",    fps = " << fps << "\n";
	//glColor3f(1.0f, 0.0f, 0.0f);
	GLvoid *font_style = GLUT_BITMAP_TIMES_ROMAN_24;
    for (i = 0; text[i] != '\0'; i++)
        glutBitmapCharacter(font_style, text[i]);
	//const unsigned char test[3] = {'W', 'E', '\0'};
	//glutBitmapString(GLUT_BITMAP_TIMES_ROMAN_24, test); 

	//  Free the allocated memory for the string
	free(text);
}

// mouse(...): to deal with th mouse
void mouse(int button, int state, int x, int y) {
	if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
		mouseDown = true;
		rotate_x_mouse_diff = x - rotate_y;
		rotate_y_mouse_diff = -y + rotate_x;
	}
	else
		mouseDown = false;
}

// mouseMotion(...): to deal with th mouse
void mouseMotion(int x, int y) {
	if (mouseDown) {
		rotate_y = x - rotate_x_mouse_diff;
		rotate_x = y + rotate_y_mouse_diff;
		glutPostRedisplay();
	}
}

// rendering MAIN function
int render(int argc, char** argv) {
	glutInit(&argc, argv);            // Initialize GLUT
	glutInitDisplayMode(GLUT_DOUBLE); // Enable double buffered mode
	glutInitWindowSize(640, 480);	  // Set the window's initial width & height
	glutInitWindowPosition(50, 50);   // Position the window's initial top-left corner
	glutCreateWindow(title);          // Create window with the given title
	glutDisplayFunc(display);       // Register callback handler for window re-paint event
	glutReshapeFunc(reshape);       // Register callback handler for window re-size event
	glutSpecialFunc(specialKeys);	// Register callback handler for rotation via UI (arrows, etc)
	glutKeyboardFunc(keyboardKeys);	// Register callback handler for rotation via UI (WASD, etc)
	glutMouseFunc(mouse);
	glutMotionFunc(mouseMotion);
	glutIdleFunc (idle);
	initGL();                       // Our own OpenGL initialization
	glutMainLoop();                 // Enter the infinite event-processing loop
	return 0;
}

