#include "SinActivation.h"

void SinActivation::writeJSON(rapidjson::PrettyWriter<rapidjson::StringBuffer>& w) {
	w.StartObject();
	w.Key("amplitude");
	w.Double(amplitude);
	w.Key("frequency");
	w.Double(frequency);
	w.Key("offset");
	w.Double(offset);

	w.Key("materialNames");
	w.StartArray();
	for (unsigned int i = 0; i < materials.size(); i++) {
		w.String(materials[i].c_str());
	}
	w.EndArray();

	w.EndObject();
}	


bool SinActivation::readJSON(rapidjson::Value& m) {
	if (m.HasMember("amplitude") && m["amplitude"].IsDouble()) {
		amplitude = m["amplitude"].GetDouble();
	}
	if (m.HasMember("frequency") && m["frequency"].IsDouble()) {
		frequency = m["frequency"].GetDouble();
	}
	if (m.HasMember("offset") && m["offset"].IsDouble()) {
		offset = m["offset"].GetDouble();
	}

	materials.clear();
	if (m.HasMember("materialNames") && m["materialNames"].IsArray() && m["materialNames"].Size() >= 0) {
		for (int i = 0; i < (int)m["materialNames"].Size(); i++) {
			materials.push_back(m["materialNames"][i].GetString());
		}
	}

	return true;
}

double SinActivation::value(double t) {
	double out = sin(2 * M_PI * frequency * t + offset) * amplitude;
	if (out > 0) {
		return out;
	}
	return 0;
	//return out;
}