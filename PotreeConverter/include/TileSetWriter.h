#ifndef TILESETWRITER_H
#define TILESETWRITER_H

#include "rapidjson/document.h"
#include <rapidjson/filewritestream.h>
#include <rapidjson/writer.h>
#include "rapidjson/prettywriter.h"
#include "rapidjson/stringbuffer.h"
#include <cstdio>

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

class TileSetWriter
{
public:
	Document document;
	
	

	TileSetWriter(string workDir, AABB aabb, float spacing, int maxDepth, double scale);
	~TileSetWriter();

	bool createTileset(string workDir, Tileset); // add working dir
};

#endif