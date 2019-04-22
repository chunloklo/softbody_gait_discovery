#pragma once
#include <Eigen/Dense>

#include "Voxelyze.h"
#include "VX_MeshRender.h"
#include "SinActivation.h"
#define M_PI (3.1415926535)

typedef struct {
	int numMaterials;
	bool optAmp;
	bool optFreq;
	bool optOff;
}FlexConfig;


class Simulation {
public:

	Simulation();
	Simulation(Eigen::VectorXd &params);
	
	void initParams(Eigen::VectorXd &params);

	double fitness();

	CVoxelyze Vx;
	CVX_MeshRender Render;
	double t;
	double dt;
	int numStep;
	std::vector<SinActivation> matOscillation;
	std::vector<SinActivation> frictionOscillation;
	Eigen::VectorXd controlParams;

	void initOffsetOnly(Eigen::VectorXd &params);

	void timestep();
	void varySize();

	bool readJSON(rapidjson::Value& o);
	void writeJSON(rapidjson::PrettyWriter<rapidjson::StringBuffer>& w);

	std::vector<Eigen::Vector2d> locationHistory;

	double fitnessLinearRegression();

	void storeLocation();

	void initParamsFlex(Eigen::VectorXd &params, int numMaterials, bool amplitude, bool frequency, bool offset);

	std::vector<std::string> history;
	void savePosition();
	void writePosition(std::string filename);

	typedef struct {
		Vec3D<double> position;
		Quat3D<double> orientation;
		double temperature;
	} voxelState;

	void setState(std::vector<voxelState> *li);

	FlexConfig flexConfig;
	void flexConfigInit(Eigen::VectorXd &params);

	double fitnessStraight();

	void record(double frameTime = 1.0 / 240.0);

	double lastT = 0.0;

	//std::vector<double> linearController 

	void reset();
};
