#include "TileMetaWriter.h"

TileMetaWriter::TileMetaWriter()
{
	document.SetObject();
	
	
	createTileset();
	
	
}

bool TileMetaWriter::createTileset()
{
	rapidjson::Document::AllocatorType& alloc = document.GetAllocator();
	Value assetobj(kObjectType); 
	Value propertiesobj(kObjectType);
	Value heightobj(kObjectType);
	Value rootobj(kObjectType);
	Value regionarr(kArrayType);
	Value rootboundingvolumregionarr(kArrayType);
	Value contentobj(kObjectType);
	Value rootcontentboundingvolumeobj(kObjectType);
	Value rootcontentboundingvolumeregionarr(kArrayType);
	Value childrenarr(kArrayType);

	Value val(kObjectType);

	//asset
	assetobj.AddMember("version", "0.0", alloc); // defines the JSON schema for tileset.json and the base set of tile formats
	assetobj.AddMember("tilesetVersion", "12345", alloc); // optional string that defines an application-specific version of a tileset, e.g., for when an existing tileset is updated
	assetobj.AddMember("gltfUpAxis", "Y", alloc); // optional string that specifies the up-axis of glTF models contained in the tileset
	document.AddMember("asset", assetobj, alloc);

	//properties
	heightobj.AddMember("minimum", 1, alloc);
	heightobj.AddMember("maximum", 241.6, alloc);
	propertiesobj.AddMember("Height", heightobj, alloc);
	document.AddMember("properties", propertiesobj, alloc);
	
	//geometricError
	document.AddMember("geometricError", 494.345678, alloc); // error when the entire tileset is not rendered

	//root
	//root.boundingVolume
	regionarr.PushBack(3, alloc); // rootboundingvolumeregionarr
	regionarr.PushBack(4, alloc);
	rootobj.AddMember("boundingVolume", regionarr, alloc);

	rootobj.AddMember("geometricError", 283.5678, alloc);


	// Add .content

	// Add .children


	document.AddMember("root", rootobj, alloc);

	FILE* fp = fopen("tileset.json", "wb");
	char writeBuffer[65536];
	FileWriteStream os(fp, writeBuffer, sizeof(writeBuffer));

	Writer<FileWriteStream> writer(os);
	document.Accept(writer);

	fclose(fp);
	return true;
}

TileMetaWriter::~TileMetaWriter()
{
}
