#pragma once

#include <iostream>
#include <fstream>
#include <iostream>
#include <vector>
#include <algorithm>
#include <iterator>

#include "PointWriter.hpp"
#include "Point.h"
#include "PointAttributes.hpp"
#include "AABB.h"


using Potree::Point;
using Potree::AABB;

using std::ios;
struct BatchTable
{

};

struct FeatureTable
{
	FeatureTable(float_t position[3]){ // you can define both but have to define one
		for (int i = 0; i < 3; i++) {
			POSITION[i] = position[i];
		}
	}
	FeatureTable(uint16_t position_quantized[3]) {
		for (int i = 0; i < 3; i++) {
			POSITION_QUANTIZED[i] = position_quantized[i];
		}
	}
	float_t POSITION[3] = {}; // x, y, z;
	uint16_t POSITION_QUANTIZED[3] = {}; // x, y, z;
	uint8_t RGBA[4] = {};
	uint8_t RGB[3] = {};
	uint16_t RGB565 = 0;
	float_t NORMAL[3] = {};
	uint8_t NORMAL_OCT16P[2] = {};
	uint16_t BATCH_ID = 0;
};

static_assert(sizeof(char) == 1, "char has to be 1 byte");
static_assert(sizeof(float) == 4, "float has to be 4 bytes");

/*
Writes a .pnt file
*/
class PNTWriter : public Potree::PointWriter
{
public:
	string file;
	std::ofstream *writer;
	AABB aabb;
	double scale;


	// header 
	const char magic[4] = {'p','n','t','s'};
	const uint32_t version = 1;
	uint32_t t_byteLength = 0;
	uint32_t ft_byteLength = 0;
	uint32_t ftJSON_byteLength = 0;
	uint32_t bt_byteLength = 0;
	uint32_t btJSON_byteLength = 0;

	

	// feature table JSON
	int32_t points_length = 0; // The number of points to render. The length of each array value for a point semantic should be equal to this.
	float rtc_center[3] = {}; // A 3-component array of numbers defining the center position when point positions are defined relative-to-center.
	float quantized_volume_offset[3] = {}; // A 3-component array of numbers defining the offset for the quantized volume. - not required, unless POSITION_QUANTIZED is defined.
	float quantized_volume_scale[3] = {}; // A 3-component array of numbers defining the scale for the quantized volume. - not required, unless POSITION_QUANTIZED is defined.
	uint8_t constant_rgba[4];  // A 4-component array of values defining a constant RGBA color for all points in the tile.
	uint32_t batch_length = -1; // The number of unique BATCH_ID values. - not required, unless BATCH_ID is defined.

	PNTWriter(string file, AABB, double scale, BatchTable bt, FeatureTable ft);
	~PNTWriter();

	void write(const Point &point);

	

	

	void writePNT();
	
	void close();

private:
	// header components	
	uint32_t getbyteLength() const; //returns total length of the tile includung the header length
	uint32_t getfeatureTableJSONByteLength() const;
	uint32_t getfeatureTableBINByteLength() const;
	uint32_t getbatchTableJSONByteLength() const;
	uint32_t getbatchTableBinaryByteLength() const;

	std::vector<char> createHeader() const;


};

