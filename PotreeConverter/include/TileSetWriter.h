#ifndef TILESETWRITER_H
#define TILESETWRITER_H

#include "rapidjson/document.h"
#include <rapidjson/filewritestream.h>
#include <rapidjson/writer.h>
#include "rapidjson/prettywriter.h"
#include "rapidjson/stringbuffer.h"
#include <cstdio>
#include <iostream>
#include <fstream>

#include "PointWriter.hpp"
#include "PointAttributes.hpp"

#include "Tileset.h"
#include "Point.h"

using rapidjson::Document;
using rapidjson::FileWriteStream;
using rapidjson::Writer;
using rapidjson::Value;
using rapidjson::kObjectType;
using rapidjson::kArrayType;
using rapidjson::StringBuffer;
using rapidjson::PrettyWriter;
using Potree::AABB;
using Potree::Point;
using std::ofstream;
using std::ios;
using Potree::PointAttributes;

class TileSetWriter : public Potree::PointWriter
{
public:
	Document document;
	string file;
	ofstream *writer;
	PointAttributes p_attributes;
	AABB aabb;
	double scale;

	TileSetWriter(string file, AABB aabb, double scale, PointAttributes pointattributes);
	~TileSetWriter();

	void write(const Point &point);
	

	void close();

	bool writeJSON(const string& workDir, const Tileset& tileset); // add working dir
};

#endif