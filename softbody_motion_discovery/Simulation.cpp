#include "Simulation.h"
#include "Voxelyze.h"
#include "VX_MeshRender.h"
#include <cmath>

#include "rapidjson/prettywriter.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/document.h"

#include <stdio.h>
#include <fstream>
#include <sstream>
#include <assert.h>
#include "JsonIO.h"
#include <iostream>
#include <math.h> 

Simulation::Simulation() : Vx(0.005), Render(&Vx) {

	//Vx.loadJSON("prop.vxl.json");
	JsonIO::loadJSON("prop.vxl.json", this);
	t = 0;
	numStep = 0;
	dt = Vx.recommendedTimeStep() * 1.0;

	locationHistory.clear();


	const std::vector< CVX_Voxel * > *list = Vx.voxelList();
	for (int i = 0; i < list->size(); i++) {
		//enable floor
		(*list)[i]->enableFloor(true);
	}

	for (int i = 0; i < Vx.materialCount(); i++) {
		Vx.material(i)->setKineticFriction(.8);
		Vx.material(i)->setStaticFriction(2);
		Vx.material(i)->setCollisionDamping(.1);
	}

	//enable self collision
	//Vx.enableCollisions(true);
	Vx.setGravity(1);

	////initialize material oscillation frequencies
	//matOscillation.resize(Vx.materialCount());

	////test material oscillations

	//matOscillation[0] = SinActivation(60, .5, 0);
	//matOscillation[1] = SinActivation(60, .5, M_PI * 1 / 5);
	//matOscillation[2] = SinActivation(60, .5, M_PI * 2 / 5);
	//matOscillation[3] = SinActivation(60, .5, M_PI * 3 / 5);
	//matOscillation[4] = SinActivation(60, .5, M_PI * 4 / 5);
	//matOscillation[5] = SinActivation(60, .5, M_PI * 5 / 5);
}	

Simulation::Simulation(Eigen::VectorXd &params) : Simulation() {
	initParams(params);
}

void Simulation::initParams(Eigen::VectorXd &params) {
	int paramPerMaterial = 3;
	//int defAmp = 60;
	//int defFreq = 1;
	assert(params.size() == Vx.materialCount() * paramPerMaterial);
	controlParams = params;
	for (int i = 0; i < matOscillation.size(); i++) {
		//matOscillation[i] = SinActivation(controlParams[i * paramPerMaterial], controlParams[i * paramPerMaterial + 1], controlParams[i * paramPerMaterial + 2]);
		matOscillation[i] = SinActivation(matOscillation[i].amplitude, matOscillation[i].frequency, controlParams[i * paramPerMaterial + 2]);

	}
	//testSaveJSON("testSave_prop.vxl.json", this);
}

void Simulation::initParamsFlex(Eigen::VectorXd &params, int numMaterials, bool amplitude, bool frequency, bool offset) {
	int numParams = amplitude + frequency + offset;
	//printf("params[0] %f\n", params[0]);
	for (int i = 0; i < numMaterials; i++) {
		int incr = 0;
		double ampVal;
		double freqVal;
		double offVal;
		if (amplitude) {
			ampVal = params[i * numParams + incr];
			incr += 1;
		}
		else {
			ampVal = matOscillation[i].amplitude;
		}

		if (frequency) {
			freqVal = params[i * numParams + incr];
			incr += 1;
		}
		else {
			freqVal = matOscillation[i].frequency;
		}
		if (offset) {
			offVal = params[i * numParams + incr];
			incr += 1;
		}
		else {
			offVal = matOscillation[i].offset;
		}
		//printf("%f, %f, %f\n", ampVal, freqVal, offVal);
		matOscillation[i] = SinActivation(ampVal, freqVal, offVal);
	}
	//testSaveJSON("testSave_prop.vxl.json", this);
}



void Simulation::initOffsetOnly(Eigen::VectorXd &params) {
	controlParams = params;
	for (int i = 0; i < controlParams.size(); i++) {
		matOscillation[i] = SinActivation(matOscillation[i].amplitude, matOscillation[i].frequency, controlParams[i]);
	}
}

void Simulation::storeLocation() {
	const std::vector< CVX_Voxel * > *list = Vx.voxelList();
	Eigen::Vector2d sumValue;
	sumValue << 0, 0;
	for (int i = 0; i < list->size(); i++) {
		Eigen::Vector2d curVal;
		curVal << 0, 0;

		curVal[0] = list->at(i)->position()[0];
		curVal[1] = list->at(i)->position()[1];
		sumValue = sumValue + curVal;
	}
	sumValue /= list->size();

	locationHistory.push_back(sumValue);
	//printf("size: %d\n", locationHistory.size());

}

double Simulation::fitnessLinearRegression() {
	//Eigen::MatrixXd A(locationHistory.size(), 2);
	//Eigen::VectorXd b(locationHistory.size());
	//for (int i = 0; i < locationHistory.size(); i++) {
	//	/*printf("Point\n");
	//	printf("x:%f, y:%f\n", locationHistory[i](0), locationHistory[i](1));*/
	//	A(i, 0) = locationHistory[i](0);
	//	A(i, 1) = 1;
	//	b(i) = locationHistory[i](1);
	//}

	//Eigen::VectorXd x = A.colPivHouseholderQr().solve(b);
	//Eigen::VectorXd diff = A * x - b;
	//double error = diff.transpose() * diff;
	//error /= locationHistory.size();
	Eigen::Vector2d d = locationHistory[locationHistory.size() -1] - locationHistory[0];
	double distance = sqrt(d.transpose() * d);
	//printf("error %f\n", error);
	std::string sep = "\n----------------------------------------\n";	
	Eigen::IOFormat HeavyFmt(Eigen::FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");
	//std::cout << locationHistory[locationHistory.size() - 1] << sep;
	//std::cout << locationHistory[0] << sep;
	//printf("fitness: %f\n", distance + error * -1);
	//printf("distance: %f\n", distance);
	//printf("error: %f\n", error * -1);

	return distance;
	//return distance + error * -1;
}


double Simulation::fitness() {
	const std::vector< CVX_Voxel * > *list = Vx.voxelList();
	double sumValue = 0;
	for (int i = 0; i < list->size(); i++) {
		sumValue += (*list)[i]->displacement()[0];
	}

	sumValue /= list->size();
	return sumValue;
}


void Simulation::timestep() {
	t += dt;
	numStep += 1;
	if (t > .05) {
		storeLocation();
		//if (numStep % 100 == 99) {
		//printf("%f\n", t);
		//printf("%f\n", fitnessLinearRegression());
		//}
	}
	savePosition();
	Vx.doTimeStep(dt);
	varySize();
}

void Simulation::varySize() {
	const std::vector< CVX_Voxel * > *list = Vx.voxelList();
	for (int i = 0; i < list->size(); i++) {
		for (int j = 0; j < matOscillation.size(); j++) {
			for (int k = 0; k < matOscillation[j].materials.size(); k++) {
				if (matOscillation[j].amplitude == 0.0) {
					continue;
				}
				if (matOscillation[j].materials[k].compare((*list)[i]->material()->name()) ==0) {
					(*list)[i]->setTemperature(matOscillation[j].value(t));
				}
			}
		}
	}
	/*for (int i = 0; i < matOscillation.size(); i++) {
		for (int j = 0; j < (int)Vx.materialCount(); j++) {
			for (int k = 0; k < list->size(); k++) {

			}
		}
	}*/
	//for (int i = 0; i < list->size(); i++) {
	//	for (int j = 0; j < (int)Vx.materialCount(); j++) {
	//		for (
	//	}
	//}	

	//for (int i = 0; i < list->size(); i++) {
	//	for (int j = 0; j < (int)Vx.materialCount(); j++) {

	//		if ((*list)[i]->material() == Vx.material(j)) {
	//			if (matOscillation[j].amplitude == 0) {
	//				continue;
	//			}
	//			(*list)[i]->setTemperature(matOscillation[j].value(t));
	//			(*list)[i]->setTemperature(matOscillation[j].value(t));
	//		
	//		}
	//	}
	//}	
}


void Simulation::writeJSON(rapidjson::PrettyWriter<rapidjson::StringBuffer>& w)
{
	w.Key("Voxelyze");
	Vx.writeJSON(w);
	w.Key("materialOscillation");
	w.StartArray();
	for (int i = 0; i < matOscillation.size(); i++) {
		matOscillation[i].writeJSON(w);
	}
	w.EndArray();
}

bool Simulation::readJSON(rapidjson::Value& o)
{
	//clear();

	if (!o.IsObject()) { return false; }

	if (o.HasMember("Voxelyze")) {
		rapidjson::Value& m = o["Voxelyze"];
		Vx.readJSON(m);
	}

	if (o.HasMember("materialOscillation")) {
		rapidjson::Value& m = o["materialOscillation"];
		for (int i = 0; i < (int)m.Size(); i++) {
			matOscillation.push_back(SinActivation(m[i]));
		}
	}
}

void Simulation::savePosition() {
	const std::vector< CVX_Voxel * > *list = Vx.voxelList();
	std::vector<double> info;
	for (int i = 0; i < list->size(); i++) {
		Vec3D<double> p = list->at(i)->position();
		info.push_back(p.getX());
		info.push_back(p.getY());
		info.push_back(p.getZ());
		info.push_back(list->at(i)->temperature());
	}
	std::stringstream ss;
	for (size_t i = 0; i < info.size(); ++i)
	{
		if (i != 0)
			ss << " ";
		ss << std::to_string(info[i]);
	}
	//ss << "\n";
	std::string s = ss.str();
	history.push_back(s);
}

void Simulation::writePosition(std::string filename) {
	std::ofstream file(filename);

	for (unsigned int i = 0; i < history.size(); i++)
		file << history[i] << "\n";
	file.close();
}

void Simulation::setState(std::vector<voxelState> *li) {
	const std::vector< CVX_Voxel * > *list = Vx.voxelList();
	for (int i = 0; i < li->size(); i++) {
		CVX_Voxel * curVoxel = list->at(i);
		Vec3D<double> pos = Vec3D<double>(li->at(i).x, li->at(i).y, li->at(i).z);
		curVoxel->setPos(pos);
		curVoxel->setTemperature(li->at(i).temp);
	}
}