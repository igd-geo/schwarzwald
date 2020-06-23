# Pointcloud Tiler

Command-line tool to generate acceleration structures for point cloud data. Can be used to convert raw point cloud data in LAS/LAZ format into [3D Tiles format](https://github.com/AnalyticalGraphicsInc/3d-tiles). 
Built upon the [potree-converter](https://github.com/potree/PotreeConverter) tool. 

This tool generates a multi-resolution [octree](https://en.wikipedia.org/wiki/Octree) from raw point cloud data. The octree can be used to speed up rendering or analysis by quickly identifying points that lie at a given location or in a given region in space. The Pointcloud Tiler stores the octree structure by writing one file for each node in the octree. Files are named according to the path from the root node to that specific node. The root node is always named `r`, every level in the octree is identified by a number between 0 and 7 (inclusive) indicating the octant that a node belongs to. 

**Supported input formats**
*  LAS/LAZ

**Supported output formats**
*  LAS/LAZ
*  3D Tiles
*  Binary dump

## Build

**Windows build is not supported currently!**

Requirements:
*  [CMake >= 3.11](https://cmake.org/)
*  [gcc >= 7.5](https://gcc.gnu.org/) or another C++17-compliant compiler
*  (Optional) [Docker >= 18](https://www.docker.com/) 

### Dependencies

The Pointcloud Tiler uses `LASzip` from the [lastools](https://github.com/m-schuetz/LAStools.git). Before building the Pointcloud Tiler, build `LASzip` like this:

```
cd ~/dev/lastools
git clone https://github.com/m-schuetz/LAStools.git
cd LASzip
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release .. && make
```

This should produce `liblaszip.so` in `~/dev/lastools/LASzip/build/src`. 

### Building on Linux

```
cd ~/dev/path-to-this-repository
mkdir -p build/release
cd build/release
cmake -DCMAKE_BUILD_TYPE=Release -DLASZIP_INCLUDE_DIRS=~/dev/lastools/LASzip/dll -DLASZIP_LIBRARY=~/dev/lastools/LASzip/build/src/liblaszip.so ../../
make
```

This should produce the `PointcloudTiler` executable in `~/dev/path-to-this-repository/build/Release`.

### Building with Docker

Run `sudo docker build -t pointcloud-tiler:latest .` from the root folder. 

## Usage

The Pointcloud Tiler supports two different modes:
*  `tiler` for creating an optimized point cloud structure
*  (**Currently not fully supported**) `converter` to convert the result of the `tiler` mode into different output formats

To get information about the possible arguments, call `PointcloudTiler` without any arguments. 

### Generating 3D Tiles directly from LAS/LAZ

The easiest way to generate 3D Tiles is to call the Pointcloud Tiler like this:

```
PointcloudTiler --tiler -i /path/to/your/LAS/files -o /output/path/for/3D/tiles --output-format 3DTILES
```

This starts a tiling process that generates 3D Tiles and stores them in the specified output folder. 

### Specifying output attributes for 3D Tiles

You can specify which of the attributes in the LAS/LAZ files you want to write to the 3D Tiles files. This is done through the `-a` option:

```
PointcloudTiler --tiler -i /path/to/your/LAS/files -o /output/path/for/3D/tiles --output-format 3DTILES -a RGB
```

This writes RGB colors to the 3D Tiles files by reading the RGB values from LAS/LAZ. Possible attributes are:

*  `RGB`: Write 8-bit per channel RGB values by reading the RGB fields of LAS/LAZ
*  `INTENSITY`: Write 16-bit intensity values by reading the intensity field of LAS/LAZ
*  `RGB_FROM_INTENSITY`: Write 8-bit per channel RGB values by reading the intensity field of LAS/LAZ and converting it to greyscale
*  `CLASSIFICATION`: Write 8-bit classification values by readin the classification field of LAS/LAZ
*  There is also `NORMALS` which would write point normals, however LAS/LAZ does not support point normals so this option is reserved for future versions

### Transforming the points into EPSG:4978 (the world coordinate system used by 3D Tiles)

If you call the Pointcloud Tiler without any further options, point positions are written as-is, that is in the same coordinate system as they were in the input files.
If you want to convert the positions into EPSG:4978 (the world coordinate system of 3D Tiles and Cesium), you have to specify the spatial reference system of your input files in [proj format](https://en.wikipedia.org/wiki/PROJ) like so:

```
PointcloudTiler --tiler -i /path/to/your/LAS/files -o /output/path/for/3D/tiles --output-format 3DTILES --source-projection "+proj=utm +zone=32 +ellps=WGS84 +datum=WGS84 +units=m +no_defs"
```

This example uses the proj string for the UTM32N spatial reference system. 

### Storing tiling results as LAS/LAZ

If you want to generate LAS/LAZ instead of 3D Tiles, you have to run the Pointcloud Tiler twice, once to generate the octree structure in an intermediate format, and then again to convert this structure into LAS/LAZ:

```
PointcloudTiler --tiler -i /path/to/your/LAS/files -o /output/path/for/octree
PointcloudTiler --converter -i /output/path/for/octree -o /output/path/for/LAS/files --output-format LAS //or LAZ
```