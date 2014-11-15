/*

graph_engine.cpp

From the building parameters it renders the building

*/

#include <windows.h>		// for MS Windows
//#include <GL/glut.h>		// GLUT, include glu.h and gl.h, not compatible with x64...
#include <GL/freeglut.h>	// FREEGLUT, needed to run x64
#include "render.h"
#include "scene.h"
#include "global.h"

// Global variables
// position, rotation, mouse
char* title = "Master Thesis";
float rotate_x = 0.0f;
float rotate_y = 0.0f;
float rotate_z = 0.0f;
float rotate_incr = 0.1f;
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




// It renders all the object3D's of a Scene
void render_Scene(Scene & scene) {
	float lineWidth = 1.0f;
	float centerPointSize = 3.0f;
	for (std::size_t i = 0; i < scene.o.size(); i++) {
		switch (i) {
			case CAMERA			: render_Object3D(scene.o[i], true, lineWidth, 0.0f);	break;
			case LASER			: render_Object3D(scene.o[i], true, lineWidth, centerPointSize);	break;
			case WALL			: render_Object3D(scene.o[i], true, lineWidth, 0.0f);	break;
			//case OCCLUDER		: render_Object3D(scene.o[i], true, lineWidth, 0.0f);	break;	// comment out
			case FLOOR			: render_Object3D(scene.o[i], true, lineWidth, 0.0f);	break;
			case VOLUME			: render_Object3D(scene.o[i], true, lineWidth, 0.0f);	break;
			case WALL_PATCHES	: render_Object3D(scene.o[i], false, lineWidth * 0.0f, 0.0f);	break;
			case CAMERA_FOV		: render_Object3D(scene.o[i], true, lineWidth * 1.5f, 0.0f);	break;
			case LASER_RAY		: render_Object3D(scene.o[i], true, lineWidth * 1.5f, 0.0f);	break;
			case VOLUME_PATCHES	: render_Object3D(scene.o[i], true, lineWidth * 1.5f, 0.0f);	break;
			case PIXEL_PATCHES	: render_Object3D(scene.o[i], true, lineWidth * 1.0f, 0.0f);	break;
			case UNKOWN_OBT		: render_Object3D(scene.o[i], true, lineWidth * 0.0f, 0.0f);	break;
		}
	}
	// render the occluder in last place to take care of transparency
	if (scene.o.size() > OCCLUDER);
		render_Object3D(scene.o[OCCLUDER], true, lineWidth, 0.0f);	
}

// It renders all the Shapes of an object3D
void render_Object3D(Object3D & o, bool renderShape, float lineWidth, float centerPointSize) {
	for (std::size_t i = 0; i < o.s.size(); i++) {
		render_Shape(o.s[i], renderShape, lineWidth, centerPointSize);
	}
	if ((centerPointSize > 0.0f) && (o.s.size() > 0)) {
		if ((o.s[0].st != PT) || ((o.s[0].st == PT) && (o.s.size() > 1))) 
			render_Point(o.s[0].c, 1.0f, 0.0f, 0.0f, 1.0f, centerPointSize);
	}
}

// It renders a Shape
void render_Shape(Shape & s, bool renderShape, float lineWidth, float pointSize) {
	if (s.st == QUAD)
		render_Rectangle(s.p[0], s.p[1], s.p[2], s.p[3], s.R, s.G, s.B, s.A, renderShape, lineWidth);
	else if (s.st == TRIANGLE)
		render_Triangle(s.p[0], s.p[1], s.p[2], s.R, s.G, s.B, s.A, renderShape, lineWidth);
	else if (s.st == LINE)
		render_Line(s.p[0], s.p[1], s.R, s.G, s.B, s.A, lineWidth);
	else if (s.st == PT)
		render_Point(s.p[0], s.R, s.G, s.B, s.A, pointSize);
}

// It renders a rectangle from 4 points
void render_Rectangle(Point & p0, Point & p1, Point & p2, Point & p3, float R, float G, float B, float A, bool renderShape, float lineWidth) {
	if (renderShape) {
		// OpenGL
		glLoadIdentity();
		set_initial_pose();
		transform_UI();
		glBegin(GL_QUADS);
		// Color
		glColor4f(R, G, B, A);
		// Vertex: counter-clockwise (CCW) order with normal pointing out
		glVertex3f(p0.x, p0.y, p0.z);
		glVertex3f(p1.x, p1.y, p1.z);
		glVertex3f(p2.x, p2.y, p2.z);
		glVertex3f(p3.x, p3.y, p3.z);
		glEnd();
	}
	// Render the edges of the Rectangle (black)
	if (lineWidth > 0.0f) {
		render_Line(p0, p1, 0.0f, 0.0f, 0.0f, 1.0f, lineWidth);
		render_Line(p1, p2, 0.0f, 0.0f, 0.0f, 1.0f, lineWidth);
		render_Line(p2, p3, 0.0f, 0.0f, 0.0f, 1.0f, lineWidth);
		render_Line(p3, p0, 0.0f, 0.0f, 0.0f, 1.0f, lineWidth);
	}
}

// It renders a triangle from 3 points
void render_Triangle(Point & p0, Point & p1, Point & p2, float R, float G, float B, float A, bool renderShape, float lineWidth) {
	if (renderShape) {
		// OpenGL
		glLoadIdentity();
		set_initial_pose();
		transform_UI();
		glBegin(GL_TRIANGLES);
		// Color
		glColor4f(R, G, B, A);
		// Vertex: counter-clockwise (CCW) order with normal pointing out
		glVertex3f(p0.x, p0.y, p0.z);
		glVertex3f(p1.x, p1.y, p1.z);
		glVertex3f(p2.x, p2.y, p2.z);
		glEnd();
	}
	// Render the edges of the Triangle (black)
	if (lineWidth > 0.0f) {
		render_Line(p0, p1, 0.0f, 0.0f, 0.0f, 1.0f, lineWidth);
		render_Line(p1, p2, 0.0f, 0.0f, 0.0f, 1.0f, lineWidth);
		render_Line(p2, p0, 0.0f, 0.0f, 0.0f, 1.0f, lineWidth);
	}
}

// It renders a line from 2 points
void render_Line(Point & p0, Point & p1, float R, float G, float B, float A, float lineWidth) {
	if (lineWidth > 0.0f) {
		// OpenGL
		glLoadIdentity();
		set_initial_pose();
		transform_UI();
		glLineWidth(lineWidth);	// def = 1.0f
		glBegin(GL_LINES);
		// Color
		glColor4f(R, G, B, A);
		// Vertex
		glVertex3f(p0.x, p0.y, p0.z);
		glVertex3f(p1.x, p1.y, p1.z);
		glEnd();
	}
}

// It renders a point from 1 point
void render_Point(Point & p0, float R, float G, float B, float A, float pointSize) {
	if (pointSize > 0.0f) {
		// OpenGL
		glLoadIdentity();
		set_initial_pose();
		transform_UI();
		glPointSize(pointSize);	// def = 3.0f
		glBegin(GL_POINTS);
		// Color
		glColor4f(R, G, B, A);
		// Vertex
		glVertex3f(p0.x, p0.y, p0.z);
		glEnd();
	}
}




// Handler for window-repaint event. Called back when the window first appears and
// whenever the window needs to be re-painted
void display() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear color and depth buffers
	glMatrixMode(GL_MODELVIEW);     // To operate on model-view matrix
	
	render_Scene(SCENEMAIN);
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

// draws a std::string at the specified coordinates
void printw (float x, float y, float z, char* format, ...) {

	va_list args;	//  Variable argument list
	int len;		//	String length
	int i;			//  Iterator
	char * text;	//	Text

	//  Initialize a variable argument list
	va_start(args, format);

	//  Return the number of characters in the std::string referenced the list of arguments.
	//  _vscprintf doesn't count terminating '\0' (that's why +1)
	len = _vscprintf(format, args) + 1; 

	//  Allocate memory for a std::string of the specified size
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

	//  Free the allocated memory for the std::string
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
	glutInitWindowSize(800, 600);	  // Set the window's initial width & height
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
// there's a weird bug when calling directly to PMD_params_to_file from thread constructor. With this re-calling functtion the bug is avoided
int render_anti_bug_thread (int argc, char** argv) {
	return render(argc, argv);
}

