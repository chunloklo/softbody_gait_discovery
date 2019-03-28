#include "Camera.h"
#include <Eigen/Dense>
#include <Eigen/Geometry> 

Camera::Camera(Eigen::Vector3d center, Eigen::Vector3d offset) : camCenter(center), camOffset(offset) {};

void Camera::Translate(Eigen::Vector3d move) {
	camOffset += move;

}

using namespace Eigen;

double M_PI = 3.14159;

void Camera::Rotate(double degX, double degY) {
	Matrix3d m;
	m = AngleAxisd(degX*M_PI, Vector3d::UnitX())
		* AngleAxisd(degY*M_PI, Vector3d::UnitY())
		* AngleAxisd(0*M_PI, Vector3d::UnitZ());
	camOffset = m * camOffset;
}	

void Camera::Move(Vector3d move) {
	camOffset += move;
}

Vector3d Camera::CameraLocation() {
	return camCenter + camOffset;
}