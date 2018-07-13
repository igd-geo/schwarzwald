#include "TileSetWriter.h"

TileSetWriter::TileSetWriter(string file, AABB aabb, double scale, PointAttributes pointattributes) 
	: file(file), aabb(aabb), scale(scale), p_attributes(pointattributes)
{
	writer = new ofstream(file, ios::out | ios::binary);
}



/*
Write to workDir + url of Tileset
*/
bool TileSetWriter::writeJSON(const string& WorkDir, const Tileset& ts)
{
	document.SetObject();
	
	rapidjson::Document::AllocatorType& alloc = document.GetAllocator();


	Value assetobj(kObjectType); 
	Value propertiesobj(kObjectType);
	Value heightobj(kObjectType);
	Value rootobj(kObjectType);
	Value boxarr(kArrayType);
	Value rootboundingvolumeobj(kObjectType);
	Value rootboundingvolumregionarr(kArrayType);

	
	
	// Tileset https://github.com/AnalyticalGraphicsInc/3d-tiles/blob/master/schema/tileset.schema.json
	/*
	-asset required
	-properties
	-geometricError required
	-root required
	*/
	


	//asset https://github.com/AnalyticalGraphicsInc/3d-tiles/blob/master/schema/asset.schema.json
	/*
	Metadata about the entire Tileset
	-version required
	-tilsetVersion
	-gltUpAxis
	*/
	Value version(ts.version.c_str(), (rapidjson::SizeType)ts.version.size());
	assetobj.AddMember("version", version, alloc); // defines the JSON schema for tileset.json and the base set of tile formats
	if (!ts.tilesetVersion.empty())
	{
		Value v(ts.tilesetVersion.c_str(), (rapidjson::SizeType)ts.version.size());
		assetobj.AddMember("tilesetVersion", v, alloc); 
	}
	if (ts.gltfUpAxis == X || ts.gltfUpAxis == Z) //Y is deafult in schema
	{
		if (ts.gltfUpAxis == X)
		{
			assetobj.AddMember("gltUpAxis", "X", alloc);
		}
		if (ts.gltfUpAxis == Z)
		{
			assetobj.AddMember("gltUpAxis", "Z", alloc);
		}
	}

	document.AddMember("asset", assetobj, alloc);
	



	//properties https://github.com/AnalyticalGraphicsInc/3d-tiles/blob/master/schema/properties.schema.json
	/*
	A dictionary object of metadata about per-feature properties.
	-maximum required
	-minimum required
	*/
	if (ts.height_max != 0 && ts.height_min != 0)
	{
		heightobj.AddMember("minimum", ts.height_min, alloc);
		heightobj.AddMember("maximum", ts.height_max, alloc);
		propertiesobj.AddMember("Height", heightobj, alloc);

		document.AddMember("properties", propertiesobj, alloc);
	}
	
	//geometricError
	/*
	The error, in meters, introduced if this tileset is not rendered. 
	At runtime, the geometric error is used to compute screen space error (SSE), i.e., the error measured in pixels.
	minimum = 0
	*/
	document.AddMember("geometricError", ts.geometricError, alloc); // error when the entire tileset is not rendered
	

	// bounding box: An array of 12 numbers that define an oriented bounding box.  
	// The first three elements define the x, y, and z values for the center of the box.  
	// The next three elements (with indices 3, 4, and 5) define the x axis direction and half-length.  
	// The next three elements (indices 6, 7, and 8) define the y axis direction and half-length. 
	// The last three elements (indices 9, 10, and 11) define the z axis direction and half-length."
	// root https://github.com/AnalyticalGraphicsInc/3d-tiles/blob/master/schema/tile.schema.json
	// A tile in a 3D Tiles tileset.
	/*
	-boundingVolume required
	-viewerRequestVolume
	-geometricError required
	-refine
	-transform
	-content required for leaf tiles
	-children ->with external Tilesets: must be undefined or an empty array
	*/
	Potree::Vector3<double> center = ts.box.getCenter();
	boxarr.PushBack(center.x, alloc);
	boxarr.PushBack(center.y, alloc); 
	boxarr.PushBack(center.z, alloc);

	// its an AABB to an OBB // what is meant by "half-lenght"?
	// x axis direction
	boxarr.PushBack(1, alloc); 
	boxarr.PushBack(0, alloc); 
	boxarr.PushBack(0, alloc); 
					 
	// y axis direction	 
	boxarr.PushBack(0, alloc); 
	boxarr.PushBack(1, alloc);
	boxarr.PushBack(0, alloc);
					 
	// z axis direction	 
	boxarr.PushBack(0, alloc); 
	boxarr.PushBack(0, alloc);
	boxarr.PushBack(1, alloc);

	rootboundingvolumeobj.AddMember("box", boxarr, alloc);
	rootobj.AddMember("boundingVolume", rootboundingvolumeobj, alloc);

	
	if (ts.requestVolumeIsSet())
	{
		Value reqBoxObj(kObjectType);
		Value reqBoxArr(kArrayType);
		Value reqObj(kObjectType);

		AABB rv = ts.getRequestVolume();
		Potree::Vector3<double> center = rv.getCenter();

		// center
		reqBoxArr.PushBack(center.x, alloc);
		reqBoxArr.PushBack(center.y, alloc);
		reqBoxArr.PushBack(center.z, alloc);

		// x axis
		reqBoxArr.PushBack(1, alloc);
		reqBoxArr.PushBack(0, alloc);
		reqBoxArr.PushBack(0, alloc);

		// y axis	 
		reqBoxArr.PushBack(0, alloc);
		reqBoxArr.PushBack(1, alloc);
		reqBoxArr.PushBack(0, alloc);

		// z axis	 
		reqBoxArr.PushBack(0, alloc);
		reqBoxArr.PushBack(0, alloc);
		reqBoxArr.PushBack(1, alloc);

		reqBoxObj.AddMember("box", reqBoxArr, alloc);
		rootobj.AddMember("viewerRequestVolume", reqBoxObj, alloc);
	}

	
	rootobj.AddMember("geometricError", ts.r_geometricError, alloc);

	if (ts.writeRefine == true)
	{
		switch (ts.refine)
		{
		case ADD:
			rootobj.AddMember("refine", "ADD", alloc);
			break;
		case REFINE:
			rootobj.AddMember("refine", "REFINE", alloc);
			break;
		}
	}

	// Add transform


	// content
	Value contentObj(kObjectType);
	// optional: add bounding box for that object
	Value co_url(ts.content_url.c_str(), (rapidjson::SizeType)ts.content_url.size());
	contentObj.AddMember("url", co_url, alloc);

	rootobj.AddMember("content", contentObj, alloc);


	// Add up to 8 children with bounding voulume(box), geometricError and content (url to pnt file)
	Value childrenArr(kArrayType);
	Value childObj(kObjectType);
	Value val;
	Value o(kObjectType);
	Value a(kArrayType);
	for (auto const& child : ts.children)
	{
		//Create Children Tile with boundingVolume geometric error and content-url pointing to extrernal tileset
		Value box(kArrayType);
		Value bvObj(kObjectType);

		Potree::Vector3<double> center = child->box.getCenter();
		// center
		box.PushBack(center.x, alloc);
		box.PushBack(center.y, alloc);
		box.PushBack(center.z, alloc);

		// x axis
		box.PushBack(1, alloc);
		box.PushBack(0, alloc);
		box.PushBack(0, alloc);
					 
		// y axis	 
		box.PushBack(0, alloc);
		box.PushBack(1, alloc);
		box.PushBack(0, alloc);
					 
		// z axis	 
		box.PushBack(0, alloc);
		box.PushBack(0, alloc);
		box.PushBack(1, alloc);
		
		bvObj.AddMember("box", box, alloc); // bounding volume object
		childObj.SetObject();
		assert(childObj.IsObject());
		
		childObj.AddMember("boundingVolume", bvObj, alloc);

		childObj.AddMember("geometricError", child->geometricError, alloc);

		Value cObj(kObjectType);
		Value ch_url(child->url.c_str(), (rapidjson::SizeType)child->url.size());
		cObj.AddMember("url", ch_url, alloc);
		childObj.AddMember("content", cObj, alloc);

		childrenArr.PushBack(childObj, alloc);
	}


	rootobj.AddMember("children", childrenArr, alloc);



	document.AddMember("root", rootobj, alloc);
	
	/*
	External Tilesets:
		root.geometricError === tile.geometricError
		root.refine === tile.refine
		root.boundingVolume === tile.content.boundingVolume
		root.viewerRequestVolume === tile.viewerRequestVolume
		geometricError from child is generally less than its parent tiles geometricError
		tile.children must be undefined or empty array
	*/

	FILE* fp = fopen(WorkDir.c_str(), "wb");
	char writeBuffer[65536];
	
	FileWriteStream os(fp, writeBuffer, sizeof(writeBuffer));

	Writer<FileWriteStream> writer(os);
	document.Accept(writer);

	fclose(fp);
	
	return true;
}

TileSetWriter::~TileSetWriter()
{
}

// Point Cloud tile format
void TileSetWriter::write(const Point & point)
{


}

// Content of the pnt file:
/*

*/

// Header of pnt file:
/*
magic 
version
byteLenght
featureTableJSONByteLength
featureTableBinaryByteLength
batchTableJSONByteLength
batchTableBinaryByteLength
*/


void TileSetWriter::close()
{
	
	if (writer != NULL) {
		writer->close();
		delete writer;
		writer = NULL;
	}
	
}
