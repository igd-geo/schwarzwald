# Potree to 3D Tiles converter

This is a fork of the official [potree converter](https://github.com/potree/PotreeConverter) repository, adapted for converting pointclouds into the [3D Tiles format](https://github.com/AnalyticalGraphicsInc/3d-tiles).

## Dependencies

* [lastools(LASzip)](https://github.com/LAStools/LAStools) or [fork of lastools with cmake for LASzip](https://github.com/m-schuetz/LAStools)
* [terminalpp](https://github.com/KazDragon/terminalpp), which requires the [conan package manager](https://conan.io/)

## Build

Official support only for Linux build at the moment! You can also build using Docker for deployment. 

### linux / gcc

lastools (from fork with cmake)

```
cd ~/dev/workspaces/lastools
git clone https://github.com/m-schuetz/LAStools.git master
cd master/LASzip
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make

```

terminalpp

```
cd ~/dev/workspaces/
git clone https://github.com/KazDragon/terminalpp.git terminalpp
cd terminalpp
conan install .
cmake -DCMAKE_BUILD_TYPE=release .
make
```

PotreeConverter

```
cd ~/dev/workspaces/PotreeConverter
git clone https://github.com/potree/PotreeConverter.git master
cd master
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DLASZIP_INCLUDE_DIRS=~/dev/workspaces/lastools/master/LASzip/dll -DLASZIP_LIBRARY=~/dev/workspaces/lastools/master/LASzip/build/src/liblaszip.so -DTERMINALPP_INCLUDE_DIRS=~/dev/workspaces/terminalpp/include -DTERMINALPP_LIBRARY=~/dev/workspaces/terminalpp.a ..
make

# copy ./PotreeConverter/resources/page_template to your binary working directory.

```

## Usage

Converts las/laz files into 3D Tiles files. Run with `-h`/`--help` to get help on the supported command line arguments. Most arguments are equal to the regular potree converter, but there are some new ones too:

*  `-a` has new parameters: 
    *  `RGB_FROM_INTENSITY` Adds greyscale colors to the pointcloud based on the intensity values in the las/laz files
    *  `NORMAL` Adds normals to the pointcloud based on the normals in the las/laz files
*  `--projection` is now `--source-projection`. It represents the projection of the source files, conversion always transforms the pointcloud into [WGS84 (geocentric)](https://epsg.io/4328) as this is what [CesiumJS](https://cesiumjs.org) uses
*  `--max-memory-usage` Defines a rough upper limit on the amount of memory that the tool can allocate during conversion. This is experimental.
