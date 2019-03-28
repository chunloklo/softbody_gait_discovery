//#define USE_OPEN_GL
//#include "VX_MeshRender.h"
//#include "Voxelyze.h"

//int main() {
//	CVoxelyze Vx(0.005); //5mm voxels
//	CVX_Material* pMaterial = Vx.addMaterial(1000000, 1000); //A material with stiffness E=1MPa and density 1000Kg/m^3
//	CVX_Voxel* Voxel1 = Vx.setVoxel(pMaterial, 0, 0, 0); //Voxel at index x=0, y=0. z=0
//	CVX_Voxel* Voxel2 = Vx.setVoxel(pMaterial, 1, 0, 0);
//	CVX_Voxel* Voxel3 = Vx.setVoxel(pMaterial, 2, 0, 0); //Beam extends in the +X direction
//
//	Voxel1->external()->setFixedAll(); //Fixes all 6 degrees of freedom with an external condition on Voxel 1
//	Voxel3->external()->setForce(0, 0, -1); //pulls Voxel 3 downward with 1 Newton of force.
//	CVX_MeshRender Render(&Vx);
//	Render.generateMesh();
//	Render.saveObj("testSave");
//	for (int i = 0; i < 100; i++) Vx.doTimeStep(); //simulate  100 timesteps.
//}

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

#include "CMADebug.h"
#include "cmaesNewLibTest.h"



using namespace std;


Eigen::Vector3d camCenter(0, 0, 0);
Eigen::Vector3d camOffset(0, 1, .1);

Camera camera(camCenter, camOffset);

void drawGrid() {
	//dx, dy are thichness parameters of grid
	float xmin = -50.0, xmax = 50.0, dx = 5.0, x;
	float ymin = -50.0, ymax = 50.0, dy = 5.0, y;

	glBegin(GL_LINES);
	glColor3f(1.0, 0.0, 0.0);

	glVertex3f(-1, 0, 0.0);
	glVertex3f(1, 0, 0.0);

	glVertex3f(-1, .1, 0.0);
	glVertex3f(1, .1, 0.0);

	glVertex3f(-1, -.1, 0.0);
	glVertex3f(1, -.1, 0.0);

	glVertex3f(0, 1, 0.0);
	glVertex3f(0, -1, 0.0);

	glVertex3f(-.01, 1, 0.0);
	glVertex3f(-.01, -1, 0.0);

	glVertex3f(.01, 1, 0.0);
	glVertex3f(.01, -1, 0.0);

	glEnd();

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
	gluPerspective(10, ratio, 0, 10000000);
	//gluOrtho2D(-10.0, 10.0, -10.0, 10.0);
	Vector3d camLoc = camera.CameraLocation();
	Vector3d camcenter = camera.camCenter;
	gluLookAt(camLoc(0), camLoc(1), camLoc(2),
		camCenter(0), camCenter(1), camCenter(2),
		0.0f, 0.0f, 1.0f);
	glMatrixMode(GL_MODELVIEW);
	// Get Back to the Modelview
}

double rotSpeed = 0.02;
double moveSpeed = 0.01;
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
	changeViewPort(glutGet(GLUT_WINDOW_WIDTH), glutGet(GLUT_WINDOW_HEIGHT));
}

VectorXd params(4);
//test material oscillations
#ifndef M_PI
#define M_PI (3.1415926535)
#endif

Simulation simulation;	
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


void replayInit() {
	infile = ifstream("./locationSave.txt");
}

void replay() {
	if (getline(infile, line))
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
			double temp = strtof(line.c_str(), 0);

			printf("%f, %f, %f, %f\n", x, y, z, temp);
			Simulation::voxelState s;
			s.x = x;
			s.y = y;
			s.z = z;
			s.temp = temp;
			states.push_back(s);

		}
		simulation.setState(&states);

	}
	else {
		infile.close();
		printf("Finished replaying\n");
	}
	
}

bool wrote = false;
unsigned int msec_simulate = 16;
void simulate(int value) {
	//replay();
	//camera.Rotate(0, .1);
	//changeViewPort(glutGet(GLUT_WINDOW_WIDTH), glutGet(GLUT_WINDOW_HEIGHT));
	simulation.timestep();
	printf("%f\n", simulation.t);
	//printf("%f\n", msec_simulate);
	glutTimerFunc(msec_simulate, simulate, 0);
	if (simulation.t > 0.1 && wrote == false) {
		wrote = true;
		simulation.writePosition("locationSave.txt");
	}
}

void render()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	drawGrid();

	const std::vector< CVX_Voxel * > *list = simulation.Vx.voxelList();
	

	meshRender.generateMesh();
	meshRender.glDraw();

	glBegin(GL_POINTS);
	//printf("Positions\n");
	for (int i = 0; i < list->size(); i++) {
		Vec3D<double> position = (*list)[i]->position();
		(*list)[i]->enableFloor(true);
		//printf("x: %f, y: %f, z: %f\n", position.getX(), position.getY(), position.getZ());
		glVertex3d(position.getX(), position.getZ(), position.getY());
	}
	glEnd();
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
	// Bind the two functions (above) to respond when necessary
	glutReshapeFunc(changeViewPort);
	glutDisplayFunc(render);
	glutIdleFunc(render);
	glutTimerFunc(msec_simulate, simulate, 0);
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glutKeyboardFunc(processNormalKeys);

	// Very important!  This initializes the entry points in the OpenGL driver so we can 
	// call all the functions in the API.
	GLenum err = glewInit();
	if (GLEW_OK != err) {
		fprintf(stderr, "GLEW error");
		return 1;
	}

	//testLibcmaes();
	//while (true);
	//debug();
	//cmaTest.test();		
	//CMATest();

	//round to nearest integer
	msec_simulate = (int) (simulation.Vx.recommendedTimeStep() * 1000.0 + 0.5);
	printf("timstep %d\n", msec_simulate);

	//JsonIO::save("testSave.json", &simulation);
	//replayInit();

	glutMainLoop();
	return 0;
}
