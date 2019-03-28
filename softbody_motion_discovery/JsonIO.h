#include <stdio.h>
#include <fstream>
#include <sstream>
#include <assert.h>

#include "rapidjson/prettywriter.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/document.h"

#include "Voxelyze.h"
#include "Simulation.h"

namespace JsonIO {

	bool save(const char* jsonFilePath, Simulation *simulation);
	bool loadJSON(const char* jsonFilePath, Simulation *simulation);
}