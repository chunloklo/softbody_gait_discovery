//Jeff Chastine
#include <Windows.h>
#include <GL\glew.h>
#include <GL\freeglut.h>
#include <iostream>

#include "Simulation.h"
#include "VX_Voxel.h"
#include <vector>
#include <Eigen/Dense>

#include "VX_MeshRender.h"

#include "Camera.h"

#include "cmaes.h"
#include <iostream>
#include "CMATest.h"
#include "JsonIO.h"
#include "Text.h"

#include "CMADebug.h"
#include "cmaesNewLibTest.h"
#include"Main.h"


using namespace std;

bool pause = false;

Eigen::Vector3d camCenter(0, 0, 0);
Eigen::Vector3d camOffset(0, 1, .1);

Camera camera(camCenter, camOffset);

enum mode {
	Optimize,	
	Display,
	Replay
};

const mode MODE = mode::Replay;

const int msec_replay = 1000 / 60;
double frameTime = 1.0 / 480;
double recordTime = 2;
string saveFile = "./replay/dummy.txt";
string loadFile = "./replay/dummy.txt";
bool fullScreen = true;

void drawGrid() {

	//dx, dy are thichness parameters of grid
	double xmin = -1, xmax = 1, dx = .02;
	double ymin = -1, ymax = 1, dy = .02;

	//glBegin(GL_QUADS);
	//glColor3d(1.0, 1.0, 1.0);
	//glVertex3d(xmin, ymin, -0.00001);
	//glVertex3d(xmin, ymax, -0.00001);
	//glVertex3d(xmax, ymax, -0.00001);
	//glVertex3d(xmax, ymin, -0.00001);
	//glEnd();

	glBegin(GL_LINES);
	glColor3d(0.0, 0.0, 0.0);
	for (double x = xmin; x < xmax; x += dx) {
		glVertex3d(x, ymin, 0.0);
		glVertex3d(x, ymax, 0.0);
	}

	for (double y = ymin; y < ymax; y += dy) {
		glVertex3d(xmin, y, 0.0);
		glVertex3d(xmax, y, 0.0);
	}
	glEnd();


	//glBegin(GL_LINES);
	//glColor3d(1.0, 0.0, 0.0);

	//glVertex3d(-1, 0, 0.0);
	//glVertex3d(1, 0, 0.0);

	//glVertex3d(-1, .1, 0.0);
	//glVertex3d(1, .1, 0.0);

	//glVertex3d(-1, -.1, 0.0);
	//glVertex3d(1, -.1, 0.0);

	//glVertex3d(0, 1, 0.0);
	//glVertex3d(0, -1, 0.0);

	//glVertex3d(-.01, 1, 0.0);
	//glVertex3d(-.01, -1, 0.0);

	//glVertex3d(.01, 1, 0.0);
	//glVertex3d(.01, -1, 0.0);

	//glEnd();

}

void changeViewPort(int w, int h)
{
	// Prevent a divide by zero, when window is too short
	// (you cant make a window of zero width).
	if (h == 0)
		h = 1;
	float ratio = (float)w / h;

	// Use the Projection Matrix
	glMatrixMode(GL_PROJECTION);

	// Reset Matrix
	glLoadIdentity();

	// Set the viewport to be the entire window
	glViewport(0, 0, w, h);

	// Set the correct perspective.
	gluPerspective(10, ratio, .001, 3);
	//gluOrtho2D(-10.0, 10.0, -10.0, 10.0);
	Vector3d camLoc = camera.camOffset;
	Vector3d camcenter = camera.camCenter;
	gluLookAt(camera.CameraLocation()(0), camera.CameraLocation()(1), camera.CameraLocation()(2),
		camera.camCenter(0), camera.camCenter(1), camera.camCenter(2),
		0.0f, 0.0f, 1.0f);
	glMatrixMode(GL_MODELVIEW);
	// Get Back to the Modelview
}

double rotSpeed = 0.02;
double moveSpeed = 0.01;
Simulation simulation;
int replayPosition = 0;


void processNormalKeys(unsigned char key, int x, int y) {

	if (key == 'q') {
		camera.Move(Vector3d(0, moveSpeed, 0));
	}
	else if (key == 'e') {
		camera.Move(Vector3d(0, -1 * moveSpeed, 0));
	}
	if (key == 'w') {

		camera.Move(Vector3d(0, 0, moveSpeed));
	}
	else if (key == 's') {

		camera.Move(Vector3d(0, 0, -1 * moveSpeed));
	}
	else if (key == 'a') {
		camera.Move(Vector3d(moveSpeed, 0, 0));
		
	}
	else if (key == 'd') {
		camera.Move(Vector3d(-1 * moveSpeed, 0, 0));
		
	}
	else if (key == 'p') {
		pause = !pause;
	}
	else if (key == 'r') {
		//simulation.dt = 0;
		simulation.reset();
		resetColor();
		replayPosition = 0;
	}
	else if (key == 'u') {
		simulation.writePosition(saveFile);
	}
	changeViewPort(glutGet(GLUT_WINDOW_WIDTH), glutGet(GLUT_WINDOW_HEIGHT));
}

//test material oscillations
#ifndef M_PI
#define M_PI (3.1415926535)
#endif


CVX_MeshRender meshRender(&simulation.Vx);
CMATest cmaTest;


using namespace std;
void drawSphere(double x, double y, double z, float radius) {
	glPushMatrix();
	glTranslated(x, y, z);
	glutSolidSphere(radius, 50, 50);
	glPopMatrix();
}

ifstream infile;
string line;
vector<Simulation::voxelState> states;

bool wrote = false;
unsigned int msec_simulate = 16;

vector<vector<Simulation::voxelState>> statesHistory;
void replayInit() {
	infile = ifstream(loadFile);
	while (getline(infile, line))
	{
		states.clear();
		istringstream iss(line);
		while (getline(iss, line, ' ')) {
			//getline(iss, line, ' ');
			double x = strtof(line.c_str(), 0);
			getline(iss, line, ' ');
			double y = strtof(line.c_str(), 0);
			getline(iss, line, ' ');
			double z = strtof(line.c_str(), 0);

			getline(iss, line, ' ');
			double qw = strtof(line.c_str(), 0);
			getline(iss, line, ' ');
			double qx = strtof(line.c_str(), 0);
			getline(iss, line, ' ');
			double qy = strtof(line.c_str(), 0);
			getline(iss, line, ' ');
			double qz = strtof(line.c_str(), 0);

			getline(iss, line, ' ');
			double temp = strtof(line.c_str(), 0);

			//printf("%f, %f, %f, %f\n", x, y, z, temp);
			Vec3D<double> p(x, y, z);
			Quat3D<double> o(qw, qx, qy, qz);
			Simulation::voxelState s;
			s.position = p;
			s.orientation = o;
			s.temperature = temp;
			states.push_back(s);
		}
		statesHistory.push_back(states);
	}
}

void replayStep() {
	if (pause) {
		return;
	}
	simulation.setState(&statesHistory[replayPosition]);
	replayPosition += 1;
	replayPosition %= statesHistory.size();
	
}

void replay(int value) {
	replayStep();
	glutTimerFunc(msec_replay, replay, 0);

	simulation.storeLocation();
	if (simulation.locationHistory.size() > 0) {
		Vector3d camLoc = camera.CameraLocation();
		Vector3d camcenter = camera.camCenter;
		camera.camCenter[0] = simulation.locationHistory[simulation.locationHistory.size() - 1][0];
		camera.camCenter[1] = simulation.locationHistory[simulation.locationHistory.size() - 1][1];
		//printf("%f, %f\n", simulation.locationHistory[simulation.locationHistory.size() - 1][0], simulation.locationHistory[simulation.locationHistory.size() - 1][1]);
		changeViewPort(glutGet(GLUT_WINDOW_WIDTH), glutGet(GLUT_WINDOW_HEIGHT));
		//gluLookAt(camLoc(0), camLoc(1), camLoc(2),
		//	camCenter(0), camCenter(1), camCenter(2),
		//	0.0f, 0.0f, 1.0f);	
	}


}

void simulate(int value) {
	if (pause) {
		//printf("%i\n", pause);
		glutTimerFunc(msec_simulate, simulate, 0);
		return;
	}
	simulation.timestep();
	simulation.record(frameTime);
	simulation.storeLocation();
	printf("Time: %f\n", simulation.t);

	if (simulation.t > recordTime && wrote == false) {
		wrote = true;
		simulation.writePosition(saveFile);
	}
	if (simulation.locationHistory.size() > 0) {
		Vector3d camLoc = camera.CameraLocation();
		Vector3d camcenter = camera.camCenter;
		camera.camCenter[0] = simulation.locationHistory[simulation.locationHistory.size() - 1][0];
		camera.camCenter[1] = simulation.locationHistory[simulation.locationHistory.size() - 1][1];
		//printf("%f, %f\n", simulation.locationHistory[simulation.locationHistory.size() - 1][0], simulation.locationHistory[simulation.locationHistory.size() - 1][1]);
		changeViewPort(glutGet(GLUT_WINDOW_WIDTH), glutGet(GLUT_WINDOW_HEIGHT));
		//gluLookAt(camLoc(0), camLoc(1), camLoc(2),
		//	camCenter(0), camCenter(1), camCenter(2),
		//	0.0f, 0.0f, 1.0f);	
	}
	

	glutTimerFunc(msec_simulate, simulate, 0);

	//glutTimerFunc(16, simulate, 0);
}

void render()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	drawGrid();
	meshRender.generateMesh();
	meshRender.glDraw();

	//glDisable(GL_DEPTH_TEST);
	//glColor3d(0., 0., 0.);
	//unsigned char string[] = "The quick god jumps over the lazy brown fox.";
	//int w;
	//w = glutBitmapLength(GLUT_BITMAP_8_BY_13, string);
	//float x = .5; /* Centre in the middle of the window */
	//glRasterPos2f(0., 0.);


	glutSwapBuffers();
}



int main(int argc, char* argv[]) {

	// Initialize GLUT
	glutInit(&argc, argv);
	// Set up some memory buffers for our display
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	// Set the window size
	glutInitWindowSize(800, 600);
	// Create the window with the title "Hello,GL"
	glutCreateWindow("Hello, GL");
	if (fullScreen) {
		glutFullScreen();
	}
	
	// Bind the two functions (above) to respond when necessary
	glutReshapeFunc(changeViewPort);
	glutDisplayFunc(render);

	glutIdleFunc(render);
	glutKeyboardFunc(processNormalKeys);

	// Very important!  This initializes the entry points in the OpenGL driver so we can 
	// call all the functions in the API.
	GLenum err = glewInit();
	if (GLEW_OK != err) {
		fprintf(stderr, "GLEW error");
		return 1;
	}

	if (MODE == mode::Optimize) {
		testLibcmaes();
	}

	//round to nearest integer
	if (MODE == mode::Display) {
		//msec_simulate = (int)(simulation.Vx.recommendedTimeStep() * 1000.0 + 0.5);
		msec_simulate = 5;
		printf("timstep %d\n", msec_simulate);

		glutTimerFunc(msec_simulate, simulate, 0);
	}

	if (MODE == mode::Replay) {
		replayInit();
		glutTimerFunc(msec_simulate, replay, 0);
	}

	glEnable(GL_DEPTH_TEST);
	//glDepthFunc(GL_LESS);
	//glDepthMask(GL_TRUE);
	//glDepthRange(-10.0f, 10.0f);
	glClearColor(1.0, 1.0, 1.0, 1.);
	//glClearDepth(1.0f);

	resetColor();

	meshRender.updateMesh(CVX_MeshRender::viewColoring::MATERIAL);
	

	glutMainLoop();
	return 0;
}

void resetColor() {
	//for (int i = 0; i < simulation.Vx.materialCount(); i++) {
	//	simulation.Vx.material(i)->setColor(255, 255, 255);
	//}
	//simulation.Vx.material(0)->setColor(250, 0, 0);
	//simulation.Vx.material(1)->setColor(250, 0, 0);
	//simulation.Vx.material(2)->setColor(0, 250, 0);
	//simulation.Vx.material(3)->setColor(0, 250, 0);
	//simulation.Vx.material(4)->setColor(0, 100, 100);
}