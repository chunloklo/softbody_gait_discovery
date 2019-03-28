#pragma once
#include <Eigen/Dense>
#include "Simulation.h"


class CMATest {
public:
	double T;
	Eigen::VectorXd params;
	
	double fitfun(double const *x, int size, double * arrLoc);

	void test();
};