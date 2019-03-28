#pragma once
#include <Eigen/Dense>

using namespace Eigen;
class Camera {
public:
	Camera(Eigen::Vector3d center, Eigen::Vector3d offset);
	Eigen::Vector3d camCenter;
	Eigen::Vector3d camOffset;
	Eigen::Vector3d camUp;

	void Translate(Vector3d move);
	void Rotate(double degX, double degY);
	void Move(Vector3d move);

	Vector3d CameraLocation();
};