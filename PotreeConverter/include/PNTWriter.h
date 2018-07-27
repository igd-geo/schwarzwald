#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <iterator>
#include <optional>
#include <variant>

#include "PointWriter.hpp"
#include "Point.h"
#include "PointAttributes.hpp"
#include "AABB.h"
#include "Vector3.h"

#include "rapidjson/document.h"
#include <rapidjson/filewritestream.h>
#include <rapidjson/writer.h>
#include "rapidjson/prettywriter.h"
#include "rapidjson/stringbuffer.h"

using std::optional;
using std::nullopt;
using std::variant;

using Potree::Point;
using Potree::AABB;
using Potree::Vector3;
using rapidjson::Document;
using rapidjson::FileWriteStream;

using rapidjson::Value;
using rapidjson::kObjectType;

using std::ios;
struct BatchTable
{
	// caontains metadata for points
};

// https://github.com/AnalyticalGraphicsInc/3d-tiles/blob/master/TileFormats/PointCloud/README.md

struct RGBA
{
	RGBA(uint8_t r, uint8_t g, uint8_t b, uint8_t a)
		:R(r), G(g), B(b), A(a) {}
	uint8_t R;
	uint8_t G;
	uint8_t B;
	uint8_t A;
};
struct RGB
{
	RGB(uint8_t r, uint8_t g, uint8_t b)
		: R(r), G(g), B(b) {}
	uint8_t R;
	uint8_t G;
	uint8_t B;
};
struct NORMAL_OCT16P {
	NORMAL_OCT16P(uint8_t x, uint8_t y) : x(x), y(y) {}
	uint8_t x;
	uint8_t y;
};

// create this struct as soon as all points are written
struct Features
{
	// Global semantics
	uint32_t POINTS_LENGTH = 0; // total number of points

	optional<Vector3<float>> RTC_CENTER; // float or float_t
	optional<Vector3<float>> QUANTIZED_VOLUME_OFFSET;
	optional<Vector3<float>> QUANTIZED_VOLUME_SCALE;
	optional<RGBA> CONSTANT_RGBA; 
	optional<uint32_t> BATCH_LENGTH; // The number of unique batch_id values

	// Point semantics
	/*
	optional<Vector3<float>> POSITION; // x, y, z;
	optional<Vector3<uint16_t>> POSITION_QUANTIZED; // x, y, z;
	optional<RGBA> RGBA;
	optional<RGB> RGB;
	optional<uint16_t> RGB565;
	optional<Vector3<float>> NORMAL;
	optional<NORMAL_OCT16P> NORMAL_OCT16P;
	optional<uint16_t> BATCH_ID;
	*/
	optional<int> POSITION_byteOffset; // x, y, z;
	optional<int> POSITION_QUANTIZED_byteOffset; // x, y, z;
	optional<int> RGBA_byteOffset;
	optional<int> RGB_byteOffset;
	optional<int> RGB565_byteOffset;
	optional<int> NORMAL_byteOffset;
	optional<int> NORMAL_OCT16P_byteOffset;
	optional<int> BATCH_ID_byteOffset;
};

static_assert(sizeof(char) == 1, "char has to be 1 byte");
static_assert(sizeof(float) == 4, "float has to be 4 bytes");

/*
Create a .pnt file / add points to it with write() / create it with writePNT()
*/
class PNTWriter : public Potree::PointWriter
{
public:
	string file;
	std::ofstream *writer;
	AABB aabb;
	double scale;
	Features featuretable;
	BatchTable batchtable;


	// header 
	
	const char magic[4] = {'p','n','t','s'};
	const uint32_t version = 1;
	uint32_t t_byteLength = 0; // length of the entire tile including header
	uint32_t ftJSON_byteLength = 0; //if this equals zero the tile does not need to be rendered
	uint32_t ft_byteLength = 0;
	uint32_t btJSON_byteLength = 0;
	uint32_t bt_byteLength = 0;
	
	int position_byteLength = 0; // This is necessary for byte offset in feature table
	int position_quantized_byteLength = 0;
	int rgba_byteLength = 0;
	int rgb_byteLength = 0;
	int rgb565_byteLength = 0;
	int normal_byteLength = 0;
	int normal_oct_byteLength = 0;
	int batchid_byteLength = 0;

	int number_of_features = 0;

	vector<float> positions;
	vector<uint8_t> colors;

	PNTWriter(string file, AABB aabb, double scale);
	~PNTWriter();

	void write(const Point &point);

	void writePNT();
	
	void close();

private:
	
	std::vector<std::byte> createFeatureBIN(); // with this function we have the right size for ft_byteLenght
	
	int seekfeatureTableJSONByteSize();

	void writeFeatureTableJSON();

	void writeHeader();

	int get_number_of_digits(int integer);

};

