#ifndef TILEMETAWRITER_H
#define TILEMETAWRITER_H

#include "rapidjson/document.h"
#include <rapidjson/filewritestream.h>
#include <rapidjson/writer.h>
#include <cstdio>

using rapidjson::Document;
using rapidjson::FileWriteStream;
using rapidjson::Writer;
using rapidjson::Value;
using rapidjson::kObjectType;
using rapidjson::kArrayType;

class TileMetaWriter
{
public:
	TileMetaWriter();
	~TileMetaWriter();
private:
	Document document;
	bool createTileset();
};

#endif