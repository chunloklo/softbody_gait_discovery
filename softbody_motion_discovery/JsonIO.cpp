#include "JsonIO.h"

bool JsonIO::save(const char* jsonFilePath, Simulation *simulation){
	std::ofstream t(jsonFilePath);
	if (t) {
		rapidjson::StringBuffer s;
		rapidjson::PrettyWriter<rapidjson::StringBuffer> w(s);
		w.StartObject();

		simulation->writeJSON(w);
		w.EndObject();
		t << s.GetString();
		t.close();
		return true;
	}
	return false;
}

bool JsonIO::loadJSON(const char* jsonFilePath, Simulation *simulation) {
	std::ifstream t(jsonFilePath);
	if (t) {
		std::stringstream buffer;
		buffer << t.rdbuf();

		rapidjson::Document doc;
		doc.Parse(buffer.str().c_str());
		simulation->readJSON(doc);

		t.close();
		return true;
	}
	return false;
	//else error!
}