#pragma once
#define M_PI (3.1415926535)

#include "rapidjson/prettywriter.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/document.h"
#include <vector>
#include <string>

class SinActivation {
public:
	double amplitude;
	//number of cycles per timestep
	double frequency;
	//where in the wave to start. Offset in radians
	double offset;

	std::vector<std::string> materials;

	SinActivation() {}

	SinActivation(double amp, double freq, double off)
		: amplitude{ amp }, frequency{ freq }, offset{ off } {}

	SinActivation(rapidjson::Value& m) {
		readJSON(m);
	}

	double value(double t);
	void writeJSON(rapidjson::PrettyWriter<rapidjson::StringBuffer>& w);

	bool readJSON(rapidjson::Value& m);
};