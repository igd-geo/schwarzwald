
#ifndef POINTWRITER_H
#define POINTWRITER_H

#include <string>
#include <iostream>

#include "Point.h"

using std::string;
class Tileset;

namespace Potree{

class PointWriter{

public:
	string file;
	int numPoints = 0;

	virtual ~PointWriter(){};

	virtual void write(const Point &point) = 0;

	virtual bool writeJSON(const string& workDir, const Tileset& tileset) { return false; }

	virtual void close() = 0;

};

}

#endif


