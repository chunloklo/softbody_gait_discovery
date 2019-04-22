#include <GL\glew.h>
#include <GL\freeglut.h>

void RenderBitmapString(float x, float y, void *font, char *string) {
	char *c;
	::glRasterPos2f(x, y);
	for (c = string; *c != '\0'; c++) {
		::glutBitmapCharacter(font, *c);
	}
	::glRasterPos2f(x + 1, y);
	for (c = string; *c != '\0'; c++) {
		::glutBitmapCharacter(font, *c);
	}
}


void ShowUIText() {
	int* pFont = (int*)GLUT_BITMAP_8_BY_13;

	GLint viewport[4];
	::glGetIntegerv(GL_VIEWPORT, viewport);
	const int win_w = viewport[2];
	const int win_h = viewport[3];

	glPushMatrix();
	glLoadIdentity();
	glScalef(1, -1, 1);
	glTranslatef(0, -win_h, 0);

	char s_tmp[256];
	int interval = 14;
	glColor3d(0.0, 0.0, 0.0);
	strcpy_s(s_tmp, "Left Click: add point");
	RenderBitmapString(10, 20, pFont, s_tmp);

	strcpy_s(s_tmp, "Right Click: remove last point");
	RenderBitmapString(10, 20 + interval, pFont, s_tmp);

	strcpy_s(s_tmp, "\'C\': Remove all points");
	RenderBitmapString(10, 20 + interval * 2, pFont, s_tmp);
	glPopMatrix();
}