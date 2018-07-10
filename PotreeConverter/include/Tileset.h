#ifndef TILESET_H
#define TILESET_H

#include <string>
#include <thread>
#include <vector>
#include <functional>
#include <sstream>
#include <list>
#include "SparseGrid.h"
#include "PointAttributes.hpp"
#include "Vector3.h"

#include "AABB.h"

using Potree::Vector3;
using std::string;
using std::vector;
using std::stringstream;
using Potree::AABB;

enum GltAxis { X, Y, Z };
enum Refine { ADD, REFINE };

class Tileset {
	
public:
	string url = ""; //url of this Tileset e.g r/tileset.json
	string name = ""; // e.g tileset.json
	//asset - required
	string version = "0.0";
	string tilesetVersion = ""; // not required
	GltAxis gltfUpAxis = Y;

	//properties
	double height_min = 0; // weakness if its really 0 optionals would be great
	double height_max = 0;

	//geometricError -required
	double geometricError = 500; // This should be set up and be less or eq in child tilesets

	//root - required
	AABB box; // root.boundingVolume
	double r_geometricError = 500;
	
	Refine refine = ADD;
	bool writeRefine = true;

	AABB contend_box; // is this undefined or == box? //not required
	string content_url; // Refers to the pnt-file
	
	vector<Tileset*> children; // Get aabb geometricError and url
	

	string contentUrl; // Points to another Tileset File

	Tileset(AABB box, string name) 
		: box(box), name(name)
	{
		
	}

	void setRequestVolume(AABB rv)
	{
		viewerRequestVolume = rv;
		this->requestVolumeSet = true;
	}

	AABB getRequestVolume() { return viewerRequestVolume; }
	bool requestVolumeIsSet() { return requestVolumeSet; }

private:
	AABB viewerRequestVolume; // Viewer must be inside of it before the tiles content will be refined based on geometric error
	bool requestVolumeSet = false;

};

#endif